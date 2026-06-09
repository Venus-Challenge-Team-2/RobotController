extern "C" {
#include <libpynq.h>
#include <iic.h>
}
#include "vl53l0x.h"
#include "mqtt.h"
#include "ntc_temperature.h"
#include <cstdio>
#include <iostream>
#include <chrono>

extern void camera_init();
extern void camera_run();
extern void scan();
extern bool scanning;

extern void init_distance();
extern uint32_t read_distance(void);

double robot_temperature = 0.0;

int main() {
    pynq_init();

    switchbox_set_pin(IO_AR_SCL, SWB_IIC0_SCL);
    switchbox_set_pin(IO_AR_SDA, SWB_IIC0_SDA);
    iic_init(IIC0);

    init_distance();
    ntc_temperature_init(NULL);
    mqtt_init();
    camera_init();

    auto last_update = std::chrono::steady_clock::now();
    while (true) {
        mqtt_read();
        mqtt_update_position();
        mqtt_navigation_control();

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update).count() >= 1000) {
            camera_run();
            ntc_temperature_read_celsius(&robot_temperature, NULL, NULL);
            mqtt_send_coords();

            if (scanning) {
                scan();
                scanning = false;
            }
            last_update = std::chrono::steady_clock::now();
        }
    }

    mqtt_destroy();
    iic_destroy(IIC0);
    pynq_destroy();
    return 0;
}
