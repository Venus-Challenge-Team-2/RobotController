extern "C" {
#include <libpynq.h>
#include <iic.h>
}
#include "vl53l0x.h"
#include "mqtt.h"
#include <cstdio>
#include <iostream>

extern void camera_init();
extern void camera_run();
extern void scan();
extern bool scanning;

extern void init_distance();
extern uint32_t read_distance(void);

int main() {
    pynq_init();

    switchbox_set_pin(IO_AR_SCL, SWB_IIC0_SCL);
    switchbox_set_pin(IO_AR_SDA, SWB_IIC0_SDA);
    iic_init(IIC0);

    init_distance();
    mqtt_init();
    camera_init();

    while (true) {
        mqtt_read();
        mqtt_update_position();
        mqtt_navigation_control();

        if (mqtt_needs_update()) {
            mqtt_send_coords();
        }

        if (scanning) {
            scan();
            scanning = false;
        }
    }

    mqtt_destroy();
    iic_destroy(IIC0);
    pynq_destroy();
    return 0;
}
