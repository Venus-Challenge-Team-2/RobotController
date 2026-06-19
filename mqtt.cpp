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
#include "exploration.h"

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

const float CM_PER_GRID_UNIT = 3.0f;

static int target_x = 50;
static int target_y = 50;

static int   update = 1;
static char *message = nullptr;
static int   cursor  = 0;

static int16_t active_left_cmd  = 0;
static int16_t active_right_cmd = 0;
static int     prev_completed_left  = 0;
static int     prev_completed_right = 0;

static bool    is_retreating = false;

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

        // Safety: prevent overflow
        if (cursor >= 1000) {
            cursor = 0;
            continue;
        }

        // We need at least 4 bytes for the header
        if (cursor >= 4) {
            // Read 4-byte little-endian length
            uint32_t payload_len = (uint8_t)message[0] |
                                  ((uint8_t)message[1] << 8) |
                                  ((uint8_t)message[2] << 16) |
                                  ((uint8_t)message[3] << 24);

            // Total message length is 4 bytes header + payload_len
            uint32_t total_len = 4 + payload_len;

            if (cursor >= total_len) {
                // We have a full message.
                // message[4] is the type
                int type = message[4];
                bool handled = false;

                if (type == 0 && payload_len == 7) { // Target: type(1) + Y(3) + X(3)
                    target_y = 100 * (message[5]) + 10 * (message[6]) + (message[7]);
                    target_x = 100 * (message[8]) + 10 * (message[9]) + (message[10]);
                    printf("MQTT target received -> X: %d  Y: %d\n", target_x, target_y);
                    handled = true;
                }
                else if (type == 3 && payload_len == 8) { // Map Update: type(1) + obj(1) + X(3) + Y(3)
                    int obj_type = message[5];
                    int x = 100 * (message[6]) + 10 * (message[7]) + (message[8]);
                    int y = 100 * (message[9]) + 10 * (message[10]) + (message[11]);
                    exploration_add_obstacle(x, y, (ObjectData)obj_type);
                    printf("MQTT map update -> X: %d  Y: %d  Type: %d\n", x, y, obj_type);
                    exploration_start();
                    handled = true;
                }
                else if (type == 4 && payload_len == 1) { // Exploration Start
                    printf("MQTT exploration start command received\n");
                    //exploration_start();
                    handled = true;
                }
                else if (type == 1 && payload_len == 1) { // Scan Request
                    scanning = true;
                    printf("MQTT scan command received\n");
                    handled = true;
                }

                // Shift remaining bytes in buffer
                size_t remaining = cursor - total_len;
                for (size_t i = 0; i < remaining; i++) {
                    message[i] = message[i + total_len];
                }
                cursor = remaining;
            }
        }
    }
    fflush(stdout);
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
    robot_x += delta_dist * std::sin(avg_angle) / CM_PER_GRID_UNIT;
    robot_y += delta_dist * std::cos(avg_angle) / CM_PER_GRID_UNIT;

    update = 1;
}

void mqtt_navigation_control(void)
{
    if (is_retreating) return;
    float goal_angle  = std::atan2((float)(target_x - robot_x), (float)(target_y - robot_y));
    float dx          = (float)(target_x) - robot_x;
    float dy          = (float)(target_y) - robot_y;
    float distance    = std::sqrt(dx * dx + dy * dy);
    float angle_diff  = goal_angle - robot_angle;

    while (angle_diff >  PI) angle_diff -= 2.0f * PI;
    while (angle_diff < -PI) angle_diff += 2.0f * PI;

    if (!stepper_steps_done()) return;

    active_left_cmd = 0;
    active_right_cmd = 0;

    if (std::abs(angle_diff) > 0.1f && distance > 1.0f) {
        int req_steps = (int)(steps_rad * angle_diff);
        printf("MQTT nav: rotating %.2f rad\n", angle_diff);
        set_stepper_command((int16_t)(-req_steps), (int16_t)req_steps);
    } else if (distance > 1.0f) {
        float move_cm = distance * CM_PER_GRID_UNIT;
        // Safety cap: Never move more than 50cm in a single command to prevent runaway
        if (move_cm > 50.0f) {
            printf("MQTT nav: distance %.2f cm exceeds safety cap, capping to 50cm\n", move_cm);
            move_cm = 50.0f;
        }
        int req_steps = (int)(steps_cm * move_cm);
        printf("MQTT nav: moving %.2f cm (grid dist: %.2f)\n", move_cm, distance);
        set_stepper_command((int16_t)req_steps, (int16_t)req_steps);
    }
}

void mqtt_cancel_navigation(void)
{
    target_x = (int)robot_x;
    target_y = (int)robot_y;
    stepper_reset();
    stepper_enable();
    active_left_cmd = 0;
    active_right_cmd = 0;
    prev_completed_left = 0;
    prev_completed_right = 0;
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

    if (gx >= 0 && gx < MAP_SIZE_X && gy >= 0 && gy < MAP_SIZE_Y) {
        oss << "3" << std::setfill('0') << std::setw(3) << gx
            << std::setfill('0') << std::setw(3) << gy
            << std::setfill('0') << std::setw(3) << temp << "\n";
    }

    if (!oss.str().empty()) {
        uart_send_string(oss.str());
    }
    update = 0;
}

bool mqtt_is_idle(void) {
    if (is_retreating) {
        if (stepper_steps_done()) {
            is_retreating = false;
            target_x = (int)robot_x;
            target_y = (int)robot_y;
            return true;
        }
        return false;
    }
    return (active_left_cmd == 0 && active_right_cmd == 0 && stepper_steps_done());
}

bool mqtt_is_retreating(void) {
    return is_retreating;
}

void mqtt_send_idle_msg(void) {
    uart_send_string("5\n");
}

void mqtt_send_work_msg(void) {
    uart_send_string("6\n");
}

void mqtt_send_hole() {
    float total_dist_coords = 5.0f / 3.0f;
    float gx_coords = robot_x + std::sin(robot_angle) * total_dist_coords;
    float gy_coords = robot_y + std::cos(robot_angle) * total_dist_coords;
    int gx = (int)gx_coords;
    int gy = (int)gy_coords;

    if (gx >= 0 && gx < MAP_SIZE_X && gy >= 0 && gy < MAP_SIZE_Y) {
        exploration_add_obstacle(gx, gy, HOLE);
        std::ostringstream oss;
        oss << "214" << std::setfill('0') << std::setw(3) << gx
            << std::setfill('0') << std::setw(3) << gy << "\n";
        uart_send_string(oss.str());
    }
}

void mqtt_send_wall(uint32_t dist_mm) {
    float dist_cm = dist_mm / 10.0f;
    float total_dist_coords = (dist_cm + 5.0f) / 3.0f;
    float gx_coords = robot_x + std::sin(robot_angle) * total_dist_coords;
    float gy_coords = robot_y + std::cos(robot_angle) * total_dist_coords;
    int gx = (int)gx_coords;
    int gy = (int)gy_coords;

    if (gx >= 0 && gx < MAP_SIZE_X && gy >= 0 && gy < MAP_SIZE_Y) {
        exploration_add_obstacle(gx, gy, MOUNTAIN); // Using MOUNTAIN as wall/obstacle
        std::ostringstream oss;
        oss << "212" << std::setfill('0') << std::setw(3) << gx
            << std::setfill('0') << std::setw(3) << gy << "\n";
        uart_send_string(oss.str());
    }
}

void mqtt_evasive_move_back() {
    if (is_retreating) return;
    printf("EVASIVE ACTION: Moving back 10cm\n");
    mqtt_cancel_navigation();
    is_retreating = true;
    int req_steps = (int)(steps_cm * 10.0f);
    set_stepper_command((int16_t)(-req_steps), (int16_t)(-req_steps));
}

void mqtt_set_target(int x, int y) {
    target_x = x;
    target_y = y;
}
