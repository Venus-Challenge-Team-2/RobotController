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

#define PI 3.14159265358979323846
#define map_size 40

bool scanning = false;

void scan() {
    int map[map_size][map_size] = {0};
    map[map_size/2][map_size/2] = 9;
    int last_x = 0;
    int last_y = 0;
    int offset = map_size/2;
    double number = 16.0;
    float steps_per_sample = (2.0f * PI / number) * 408.709874761f;

    for (int i = 0; i < (int)number; i++) {
        uint32_t dist_mm = read_distance();
        double distance = dist_mm / 10.0;

        double rotation = i * 2.0 * PI / number;
        int x = std::sin(rotation) * distance;
        int y = std::cos(rotation) * distance;

        if (x + offset >= 0 && x + offset < map_size && y + offset >= 0 && y + offset < map_size) {
            map[x+offset][y+offset] = 1;
        }

        if (!(last_x == 0 && last_y == 0) && !(x == 0 && y == 0)) {
            int vector_x = last_x - x;
            int vector_y = last_y - y;
            int d = std::sqrt(vector_x*vector_x + vector_y*vector_y);
            if (d > 0) {
                for (int j = 0; j < d; j++) {
                    int px = x + vector_x * j / d + offset;
                    int py = y + vector_y * j / d + offset;
                    if (px >= 0 && px < map_size && py >= 0 && py < map_size) {
                        map[px][py] = 1;
                    }
                }
            }
        }

        last_x = x;
        last_y = y;

        stepper_steps((int16_t)steps_per_sample, (int16_t)-steps_per_sample);
        while (!stepper_steps_done());
    }

    std::ostringstream accumulated_oss;

    for (int i = 0; i < map_size; ++i) {
        for (int j = 0; j < map_size; ++j) {
            std::cout << map[i][j];
            if (map[i][j] == 1) {
                accumulated_oss << "222" 
                                << std::setfill('0') << std::setw(3) << i
                                << std::setfill('0') << std::setw(3) << j
                                << "\n"; 
            }
        }
        std::cout << std::endl;
    }

    std::string final_payload = accumulated_oss.str();
    if (!final_payload.empty()) {
        uart_send_string(final_payload);
    }
}
