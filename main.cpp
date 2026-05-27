#include <libpynq.h>
#include <iic.h>
#include "vl53l0x.h"
#include "mqtt.h"
#include <stdio.h>
#include <iostream>
#include <camera.cpp>

extern "C" {
    void init_distance();
    uint32_t read_distance(void);
}

int main() {
    pynq_init();

    /* IIC bus for the VL53L0X distance sensor */
    switchbox_set_pin(IO_AR_SCL, SWB_IIC0_SCL);
    switchbox_set_pin(IO_AR_SDA, SWB_IIC0_SDA);
    iic_init(IIC0);

    init_distance();

    /* UART0 + stepper motors for MQTT navigation */
    mqtt_init();

    /* Camera / OpenCV */
    camera_init();

    while (true) {
        /* 1. Process incoming MQTT commands */
        mqtt_read();

        /* 2. Update dead-reckoning position from stepper feedback */
        mqtt_update_position();

        /* 3. Issue next movement command toward the current target */
        mqtt_navigation_control();

        /* 4. Publish position whenever it has changed */
        if (mqtt_needs_update()) {
            mqtt_send_coords();
        }

        /* 5. Run one frame of camera processing */
        camera_run();
    }

    mqtt_destroy();
    iic_destroy(IIC0);
    pynq_destroy();
    return 0;
}
