extern "C" {
#include <libpynq.h>
#include <stepper.h>
#include <uart.h>
#include <switchbox.h>
}
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include "mqtt.h"
#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>

#define PI 3.14159265358979323846f

extern bool scanning;
extern double robot_temperature;

static const float WHEELBASE     = 12.3f;
static const float CASTER_OFFSET = 10.0f;
static const float CASTER_TRAIL  = 2.5f;

static const float STEPS_PER_ROTATION = 1600.0f;
static const float WHEEL_DIAMETER     = 7.7f;
static const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;
const float steps_cm  = STEPS_PER_ROTATION / WHEEL_CIRCUMFERENCE;
const float steps_rad = (steps_cm * WHEELBASE) / 2.0f;

float robot_angle = 0.0f;
float robot_x     = 50.0f;
float robot_y     = 50.0f;
float robot_caster_phi = PI; // Initialized to trailing position

// Estimated slip factors (tunable)
static const float CASTER_DRAG_S  = 0.005f; // 0.5% loss per cm of scrub
static const float CASTER_DRAG_TH = 0.010f; // 1.0% loss per cm of scrub on rotation

static int target_x = 50;
static int target_y = 50;

static int   update = 1;
static char *message = nullptr;
static int   cursor  = 0;

static int16_t active_left_cmd  = 0;
static int16_t active_right_cmd = 0;
static int     prev_completed_left  = 0;
static int     prev_completed_right = 0;

void set_stepper_command(int16_t left, int16_t right)
{
    stepper_steps(left, right);
    active_left_cmd  = left;
    active_right_cmd = right;
    prev_completed_left  = 0;
    prev_completed_right = 0;
}

void mqtt_init(void)
{
    uart_init(UART0);
    uart_reset_fifos(UART0);

    switchbox_set_pin(IO_AR0, SWB_UART0_RX);
    switchbox_set_pin(IO_AR1, SWB_UART0_TX);

    stepper_init();
    stepper_enable();
    stepper_set_speed(15000, 15000);

    message = new char[1000];
}

void mqtt_read(void)
{
    while (uart_has_data(UART0)) {
        char in = uart_recv(UART0);
        message[cursor++] = in;
        int out = in;
        std::cout << "aa" << out << std::endl;
    }

    if (cursor > 4 && message[4] == 0) {
        char type = message[cursor - 7];
        if (type == 0) {
            target_x = 100 * message[cursor - 6]
                     +  10 * message[cursor - 5]
                     +       message[cursor - 4];
            target_y = 100 * message[cursor - 3]
                     +  10 * message[cursor - 2]
                     +       message[cursor - 1];
            printf("MQTT target -> X: %d  Y: %d\n", target_x, target_y);
        }
        cursor = 0;
    } else if (cursor > 4 && message[4] == 1) {
        scanning = true;
        printf("MQTT scan command received\n");
        cursor = 0;
    }
}

void mqtt_update_position(void)
{
    int16_t curr_left_rem, curr_right_rem;
    stepper_get_steps(&curr_left_rem, &curr_right_rem);

    int completed_left  = std::abs(active_left_cmd)  - std::abs(curr_left_rem);
    int completed_right = std::abs(active_right_cmd) - std::abs(curr_right_rem);

    if (completed_left  < prev_completed_left)  completed_left  = prev_completed_left;
    if (completed_right < prev_completed_right) completed_right = prev_completed_right;

    int tick_left  = completed_left  - prev_completed_left;
    int tick_right = completed_right - prev_completed_right;

    prev_completed_left  = completed_left;
    prev_completed_right = completed_right;

    if (tick_left == 0 && tick_right == 0) return;

    float delta_left  = tick_left  * (active_left_cmd  >= 0 ? 1.0f : -1.0f);
    float delta_right = tick_right * (active_right_cmd >= 0 ? 1.0f : -1.0f);

    float delta_angle = (delta_right - delta_left) / (2.0f * steps_rad);
    float delta_dist  = (delta_left + delta_right) / (2.0f * steps_cm);

    // Kinematic update for caster
    // v_px is forward velocity at pivot, v_py is lateral velocity at pivot
    float v_px = delta_dist;
    float v_py = -delta_angle * CASTER_OFFSET;

    // Change in caster angle relative to robot body
    // dphi/dt = (v_px*sin(phi) - v_py*cos(phi))/L - dtheta/dt
    float dphi = (v_px * std::sin(robot_caster_phi) - v_py * std::cos(robot_caster_phi)) / CASTER_TRAIL - delta_angle;
    robot_caster_phi += dphi;

    // Keep phi in [-PI, PI]
    while (robot_caster_phi >  PI) robot_caster_phi -= 2.0f * PI;
    while (robot_caster_phi < -PI) robot_caster_phi += 2.0f * PI;

    // Apply drag correction based on scrubbing
    // Scrub is the velocity component perpendicular to the caster wheel
    float scrub = std::abs(-v_px * std::sin(robot_caster_phi) + v_py * std::cos(robot_caster_phi));
    delta_dist  *= (1.0f - CASTER_DRAG_S  * scrub);
    delta_angle *= (1.0f - CASTER_DRAG_TH * scrub);

    robot_angle += delta_angle;

    float avg_angle = robot_angle - (delta_angle / 2.0f);
    robot_x += delta_dist * std::sin(avg_angle) / 3.0f;
    robot_y += delta_dist * std::cos(avg_angle) / 3.0f;

    update = 1;
}

void mqtt_navigation_control(void)
{
    float goal_angle  = std::atan2((float)(target_x - robot_x), (float)(target_y - robot_y));
    float dx          = (float)(target_x) - robot_x;
    float dy          = (float)(target_y) - robot_y;
    float distance    = std::sqrt(dx * dx + dy * dy);
    float angle_diff  = goal_angle - robot_angle;

    while (angle_diff >  PI) angle_diff -= 2.0f * PI;
    while (angle_diff < -PI) angle_diff += 2.0f * PI;

    if (!stepper_steps_done()) return;

    if (std::abs(angle_diff) > 0.1f && distance > 1.0f) {
        int req_steps = (int)(steps_rad * angle_diff);
        printf("MQTT nav: rotating %.2f rad\n", angle_diff);
        set_stepper_command((int16_t)(-req_steps), (int16_t)req_steps);
    } else if (distance > 1.0f) {
        int req_steps = (int)(steps_cm * distance * 3.0f);
        printf("MQTT nav: moving %.2f cm\n", distance * 3.0f);
        set_stepper_command((int16_t)req_steps, (int16_t)req_steps);
    }
}

void mqtt_cancel_navigation(void)
{
    target_x = (int)robot_x;
    target_y = (int)robot_y;
    stepper_reset();
    active_left_cmd = 0;
    active_right_cmd = 0;
    prev_completed_left = 0;
    prev_completed_right = 0;
    printf("MQTT nav: CANCELLED (detected black line)\n");
}

int mqtt_needs_update(void)
{
    return update;
}

void uart_send_string(const std::string& str) {
    if (str.empty()) return;
    uint32_t length = static_cast<uint32_t>(str.length());

    uart_send(UART0, static_cast<uint8_t>(length        & 0xFF));
    uart_send(UART0, static_cast<uint8_t>((length >> 8)  & 0xFF));
    uart_send(UART0, static_cast<uint8_t>((length >> 16) & 0xFF));
    uart_send(UART0, static_cast<uint8_t>((length >> 24) & 0xFF));

    for (char c : str) {
        uart_send(UART0, static_cast<uint8_t>(c));
    }
}

void mqtt_send_coords(void) {
    std::ostringstream oss;
    int gx = (int)(robot_x);
    int gy = (int)(robot_y);
    int temp = (int)robot_temperature;

    if (gx >= 0 && gx < 333 && gy >= 0 && gy < 333) {
        oss << "3" << std::setfill('0') << std::setw(3) << gx
            << std::setfill('0') << std::setw(3) << gy
            << std::setfill('0') << std::setw(3) << temp << "\n";
    }

    if (!oss.str().empty()) {
        uart_send_string(oss.str());
    }
    update = 0;
}

void mqtt_destroy(void)
{
    stepper_destroy();
    delete[] message;
    message = nullptr;
}
