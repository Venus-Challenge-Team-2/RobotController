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
void uart_send_string(const std::string& str);

#include "mqtt.h"

#define PI 3.14159265358979323846
#define map_size 100

bool scanning = false;
extern void camera_run();

void scan() {
    int map[map_size][map_size] = {0};
    int offset = map_size/2;

    mqtt_update_position();
    float start_x = robot_x;
    float start_y = robot_y;

    int last_gx = 0;
    int last_gy = 0;
    double number = 16.0;
    float steps_per_sample = (2.0f * (float)PI / (float)number) * 408.709874761f;

    for (int i = 0; i < (int)number; i++) {
        mqtt_update_position();
        camera_run();

        uint32_t dist_mm = read_distance();
        double distance = dist_mm / 10.0;

        int gx = (int)(robot_x + std::sin(robot_angle) * distance);
        int gy = (int)(robot_y + std::cos(robot_angle) * distance);

        int map_x = gx - (int)start_x + offset;
        int map_y = gy - (int)start_y + offset;

        if (map_x >= 0 && map_x < map_size && map_y >= 0 && map_y < map_size) {
            map[map_x][map_y] = 1;
        }

        if (!(last_gx == 0 && last_gy == 0)) {
            int vector_x = gx - last_gx;
            int vector_y = gy - last_gy;
            int d = (int)std::sqrt(vector_x*vector_x + vector_y*vector_y);
            if (d > 0) {
                for (int j = 0; j <= d; j++) {
                    int px = last_gx + vector_x * j / d - (int)start_x + offset;
                    int py = last_gy + vector_y * j / d - (int)start_y + offset;
                    if (px >= 0 && px < map_size && py >= 0 && py < map_size) {
                        map[px][py] = 1;
                    }
                }
            }
        }

        last_gx = gx;
        last_gy = gy;

        stepper_steps((int16_t)steps_per_sample, (int16_t)-steps_per_sample);
        while (!stepper_steps_done());
    }

    std::ostringstream accumulated_oss;

    for (int i = 0; i < map_size; ++i) {
        for (int j = 0; j < map_size; ++j) {
            if (map[i][j] == 1) {
                int gx = i - offset + (int)start_x;
                int gy = j - offset + (int)start_y;
                if (gx >= 0 && gx < 1000 && gy >= 0 && gy < 1000) {
                    accumulated_oss << "222"
                                    << std::setfill('0') << std::setw(3) << gx
                                    << std::setfill('0') << std::setw(3) << gy
                                    << "\n";
                }
            }
        }
    }

    std::string final_payload = accumulated_oss.str();
    if (!final_payload.empty()) {
        uart_send_string(final_payload);
    }
}
