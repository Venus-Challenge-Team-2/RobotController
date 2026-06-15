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

    while (!stepper_steps_done()) {
        mqtt_update_position();

        // Optional: camera_run() could be called here if needed,
        // but it might slow down the loop too much.
        camera_run();

        uint32_t dist_mm = read_distance();

        if (dist_mm > 0 && dist_mm < 400) {
            float dist_cm = dist_mm / 10.0f;

            float total_dist_coords = (dist_cm + 5.0f) / 3.0f;

            float gx_coords = robot_x + std::sin(robot_angle) * total_dist_coords;
            float gy_coords = robot_y + std::cos(robot_angle) * total_dist_coords;

            int gx = (int)(gx_coords);
            int gy = (int)(gy_coords);

            // Boundary check for the 333x333 visualization grid (1000/3)
            if (gx >= 0 && gx < 333 && gy >= 0 && gy < 333) {
                std::ostringstream oss;
                oss << "222" << std::setfill('0') << std::setw(3) << gx
                    << std::setfill('0') << std::setw(3) << gy << "\n";
                uart_send_string(oss.str());
            }
        }
    }
    stepper_set_speed(15000, 15000);
    printf("Scan completed.\n");
}
