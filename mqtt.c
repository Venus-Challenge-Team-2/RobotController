#include <libpynq.h>
#include <math.h>
#include <stepper.h>
#include <stdlib.h>
#include <stdio.h>
#include "mqtt.h"

/* ------------------------------------------------------------------ */
/* Constants                                                           */
/* ------------------------------------------------------------------ */
#define PI 3.14159265358979323846f

/* ------------------------------------------------------------------ */
/* State                                                               */
/* ------------------------------------------------------------------ */
static float steps_cm  = 64.2f;          /* steps per cm              */
static float steps_rad = 408.709874761f; /* steps per radian          */

static float angle = 0.0f;
static float x     = 0.0f;
static float y     = 0.0f;

static int target_x = 0;
static int target_y = 0;

static int   update = 1;      /* non-zero → position changed, send it */
static char *message = NULL;
static int   cursor  = 0;

/* Active stepper command bookkeeping */
static int16_t active_left_cmd  = 0;
static int16_t active_right_cmd = 0;
static int     prev_completed_left  = 0;
static int     prev_completed_right = 0;

/* ------------------------------------------------------------------ */
/* Private helpers                                                     */
/* ------------------------------------------------------------------ */
static void set_stepper_command(int16_t left, int16_t right)
{
    stepper_steps(left, right);
    active_left_cmd  = left;
    active_right_cmd = right;
    prev_completed_left  = 0;
    prev_completed_right = 0;
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

void mqtt_init(void)
{
    uart_init(UART0);
    uart_reset_fifos(UART0);

    switchbox_set_pin(IO_AR0, SWB_UART0_RX);
    switchbox_set_pin(IO_AR1, SWB_UART0_TX);

    stepper_init();
    stepper_enable();
    stepper_set_speed(15000, 15000);

    message = malloc(sizeof(char) * 1000);
}

void mqtt_read(void)
{
    while (uart_has_data(UART0)) {
        char in = uart_recv(UART0);
        message[cursor++] = in;
    }

    /* A complete message starts with byte value 7 and is 11 bytes long */
    if (message[0] == 7 && cursor == 11) {
        char type = message[cursor - 7];
        if (type == 0) {
            target_x = 100 * message[cursor - 6]
                     +  10 * message[cursor - 5]
                     +       message[cursor - 4];
            target_y = 100 * message[cursor - 3]
                     +  10 * message[cursor - 2]
                     +       message[cursor - 1];
            printf("MQTT target → X: %d  Y: %d\n", target_x, target_y);
        }
        cursor = 0;
    }
}

void mqtt_update_position(void)
{
    int16_t curr_left_rem, curr_right_rem;
    stepper_get_steps(&curr_left_rem, &curr_right_rem);

    int completed_left  = abs(active_left_cmd)  - abs(curr_left_rem);
    int completed_right = abs(active_right_cmd) - abs(curr_right_rem);

    if (completed_left  < prev_completed_left)  completed_left  = prev_completed_left;
    if (completed_right < prev_completed_right) completed_right = prev_completed_right;

    int tick_left  = completed_left  - prev_completed_left;
    int tick_right = completed_right - prev_completed_right;

    prev_completed_left  = completed_left;
    prev_completed_right = completed_right;

    if (tick_left == 0 && tick_right == 0) return;

    float delta_left  = tick_left  * (active_left_cmd  >= 0 ? 1.0f : -1.0f);
    float delta_right = tick_right * (active_right_cmd >= 0 ? 1.0f : -1.0f);

    float delta_angle = (delta_left - delta_right) / (2.0f * steps_rad);
    float delta_dist  = (delta_left + delta_right) / (2.0f * steps_cm);

    angle += delta_angle;

    float avg_angle = angle - (delta_angle / 2.0f);
    x += delta_dist * sinf(avg_angle);
    y += delta_dist * cosf(avg_angle);

    update = 1;
}

void mqtt_navigation_control(void)
{
    float goal_angle  = atan2f((float)(target_x - x), (float)(target_y - y));
    float dx          = (float)(target_x) - x;
    float dy          = (float)(target_y) - y;
    float distance    = sqrtf(dx * dx + dy * dy);
    float angle_diff  = goal_angle - angle;

    while (angle_diff >  PI) angle_diff -= 2.0f * PI;
    while (angle_diff < -PI) angle_diff += 2.0f * PI;

    if (!stepper_steps_done()) return;

    if (fabsf(angle_diff) > 0.1f && distance > 1.0f) {
        int req_steps = (int)(steps_rad * angle_diff);
        printf("MQTT nav: rotating %.2f rad\n", angle_diff);
        set_stepper_command((int16_t)req_steps, (int16_t)(-req_steps));
    } else if (distance > 1.0f) {
        int req_steps = (int)(steps_cm * distance);
        printf("MQTT nav: moving %.2f cm\n", distance);
        set_stepper_command((int16_t)req_steps, (int16_t)req_steps);
    }
}

int mqtt_needs_update(void)
{
    return update;
}

void mqtt_send_coords(void)
{
    uint32_t length = (uint32_t)snprintf(NULL, 0,
        "x = %.2f y = %.2f angle = %.2f\n", x, y, angle);

    char *buf = malloc(length + 1);
    snprintf(buf, length + 1, "x = %.2f y = %.2f angle = %.2f\n", x, y, angle);

    /* Length as little-endian 4-byte prefix */
    uart_send(UART0, (uint8_t)(length        & 0xFF));
    uart_send(UART0, (uint8_t)((length >> 8)  & 0xFF));
    uart_send(UART0, (uint8_t)((length >> 16) & 0xFF));
    uart_send(UART0, (uint8_t)((length >> 24) & 0xFF));

    for (uint32_t i = 0; i < length; i++) {
        uart_send(UART0, (uint8_t)buf[i]);
    }

    free(buf);
    update = 0;
}

void mqtt_destroy(void)
{
    stepper_destroy();
    free(message);
    message = NULL;
}
