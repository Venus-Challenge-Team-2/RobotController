extern "C" {
#include <libpynq.h>
#include <stepper.h>
}
#include <iostream>
#include <cmath>
#include <vector>
#include <string>
#include <cstdio>
#include <sstream>
#include <iomanip>
#include <chrono>

extern uint32_t read_distance(void);
#include "mqtt.h"

#ifndef PI
#define PI 3.14159265358979323846f
#endif

bool scanning = false;
extern void camera_run();

void scan() {
    printf("Starting smooth scan...\n");

    stepper_set_speed(65535, 65535);

    int total_steps = (int)(steps_rad * 2.0f * PI);
    set_stepper_command((int16_t)total_steps, (int16_t)-total_steps);
    std::ostringstream oss;

    auto last_update = std::chrono::steady_clock::now();

    while (!stepper_steps_done()) {
        printf("Scan loop \n");
        fflush(stdout);
        mqtt_update_position();

        uint32_t dist_mm = read_distance();

        if (dist_mm > 0 && dist_mm < 400) {
            float dist_cm = dist_mm / 10.0f;

            float total_dist_coords = (dist_cm + 5.0f) / 3.0f;

            float gx_coords = robot_x + std::sin(robot_angle) * total_dist_coords;
            float gy_coords = robot_y + std::cos(robot_angle) * total_dist_coords;

            int gx = (int)(gx_coords);
            int gy = (int)(gy_coords);

            if (gx >= 0 && gx < 333 && gy >= 0 && gy < 333) {
                oss << "212" << std::setfill('0') << std::setw(3) << gx
                    << std::setfill('0') << std::setw(3) << gy << "\n";              
            }
        }

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update).count() >= 300) {
            camera_run();
            last_update = std::chrono::steady_clock::now();
        }
    }
    uart_send_string(oss.str());
    stepper_set_speed(15000, 15000);
    printf("Scan completed.\n");
}
