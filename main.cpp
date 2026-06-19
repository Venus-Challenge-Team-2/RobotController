extern "C" {
#include <libpynq.h>
#include <iic.h>
}
#include "vl53l0x.h"
#include "mqtt.h"
#include "exploration.h"
#include "ntc_temperature.h"
#include "tcs3200.h"
#include <cstdio>
#include <iostream>
#include <chrono>

extern "C" {
#include <stepper.h>
}

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

    tcs3200_t color_sensor;
    tcs3200_config_t color_config = {
        .out_pin = IO_AR6,
        .s0_pin = IO_AR5,
        .s1_pin = IO_AR4,
        .s2_pin = IO_AR7,
        .s3_pin = IO_AR8,
        .black_threshold_hz = 0.0,
        .white_threshold_hz = 3000.0,
        .measure_window_ms = 15,
        .pulsecounter = PULSECOUNTER0,
    };
    tcs3200_init(&color_sensor, &color_config);

    mqtt_init();
    exploration_init();
    //exploration_generate_hole_border();
    camera_init();
    sleep_msec(300);

    //exploration_start();

    auto last_update = std::chrono::steady_clock::now();
    while (true) {
        mqtt_read();
        mqtt_update_position();
        exploration_run();
        mqtt_navigation_control();

        double freq = tcs3200_read_frequency_hz(&color_sensor);
        uint32_t dist = read_distance();

        if(!mqtt_is_retreating()) {
            if (!tcs3200_is_white(&color_sensor, freq)) {
                printf("HOLE DETECTED by ground sensor!\n");
                mqtt_send_hole();
                mqtt_evasive_move_back();
                if (is_exploration_active) exploration_request_scan();
            } else if (dist > 0 && dist < 50) {
                printf("MQTT nav: CANCELLED (distance too close) %d\n", dist);
                mqtt_send_wall(dist);
                mqtt_evasive_move_back();
                if (is_exploration_active) exploration_request_scan();
            }
        }

        if (scanning) {
            scan();
            scanning = false;
        }

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update).count() >= 1000) {
            //exploration_print_map();
            camera_run();
            ntc_temperature_read_celsius(&robot_temperature, NULL, NULL);
            mqtt_send_coords();

            if (mqtt_is_idle()) {
                mqtt_send_idle_msg();
                std::cout << "Sending idle\n";
            } else {
                mqtt_send_work_msg();
            }
            last_update = std::chrono::steady_clock::now();
        }
    }

    mqtt_destroy();
    iic_destroy(IIC0);
    pynq_destroy();
    return 0;
}
