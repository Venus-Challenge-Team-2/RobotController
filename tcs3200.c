#include "tcs3200.h"

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <gpio.h>
#include <switchbox.h>
#include <util.h>

static void gpio_write(io_t pin, bool value) {
    gpio_set_level(pin, value ? GPIO_LEVEL_HIGH : GPIO_LEVEL_LOW);
}

static bool is_valid_sensor(const tcs3200_t *sensor) {
    if (sensor == NULL) {
        return false;
    }

    return sensor->config.measure_window_ms > 0 &&
           sensor->config.white_threshold_hz >= sensor->config.black_threshold_hz;
}

static io_configuration_t pulsecounter_swb(const tcs3200_t *sensor) {
    if (sensor == NULL) {
        return SWB_GPIO;
    }

    return (sensor->config.pulsecounter == PULSECOUNTER1) ? SWB_TIMER_IC1 : SWB_TIMER_IC0;
}

static void tcs3200_set_output_scaling_20pct(const tcs3200_t *sensor) {
    gpio_write(sensor->config.s0_pin, true);
    gpio_write(sensor->config.s1_pin, false);
}

static void tcs3200_select_clear_filter(const tcs3200_t *sensor) {
    gpio_write(sensor->config.s2_pin, false);
    gpio_write(sensor->config.s3_pin, true);
}

bool tcs3200_init(tcs3200_t *sensor, const tcs3200_config_t *config) {
    if (sensor == NULL || config == NULL) {
        return false;
    }

    sensor->config = *config;
    if (!is_valid_sensor(sensor)) {
        return false;
    }

    switchbox_init();
    switchbox_set_pin(sensor->config.out_pin, pulsecounter_swb(sensor));
    switchbox_set_pin(sensor->config.s0_pin, SWB_GPIO);
    switchbox_set_pin(sensor->config.s1_pin, SWB_GPIO);
    switchbox_set_pin(sensor->config.s2_pin, SWB_GPIO);
    switchbox_set_pin(sensor->config.s3_pin, SWB_GPIO);

    gpio_init();
    gpio_set_direction(sensor->config.s0_pin, GPIO_DIR_OUTPUT);
    gpio_set_direction(sensor->config.s1_pin, GPIO_DIR_OUTPUT);
    gpio_set_direction(sensor->config.s2_pin, GPIO_DIR_OUTPUT);
    gpio_set_direction(sensor->config.s3_pin, GPIO_DIR_OUTPUT);

    tcs3200_set_output_scaling_20pct(sensor);
    tcs3200_select_clear_filter(sensor);

    pulsecounter_init(sensor->config.pulsecounter);
    pulsecounter_set_edge(sensor->config.pulsecounter, GPIO_LEVEL_HIGH);
    pulsecounter_reset_count(sensor->config.pulsecounter);

    return true;
}

double tcs3200_read_frequency_hz(const tcs3200_t *sensor) {
    uint32_t start_ts = 0;
    uint32_t end_ts = 0;
    uint32_t start_count;
    uint32_t end_count;
    uint32_t dt_ticks;
    uint32_t dcount;
    double dt_sec;

    if (!is_valid_sensor(sensor)) {
        return 0.0;
    }

    start_count = pulsecounter_get_count(sensor->config.pulsecounter, &start_ts);
    sleep_msec(sensor->config.measure_window_ms);
    end_count = pulsecounter_get_count(sensor->config.pulsecounter, &end_ts);

    dcount = (uint32_t)(end_count - start_count);
    dt_ticks = (uint32_t)(end_ts - start_ts);

    if (dt_ticks == 0U) {
        return 0.0;
    }

    // libpynq pulsecounter runs at ~100MHz.
    dt_sec = (double)dt_ticks / 100000000.0;
    if (dt_sec <= 0.0) {
        return 0.0;
    }

    return (double)dcount / dt_sec;
}

bool tcs3200_is_white(const tcs3200_t *sensor, double frequency_hz) {
    if (!is_valid_sensor(sensor)) {
        return false;
    }

    return frequency_hz >= sensor->config.white_threshold_hz;
}

bool tcs3200_is_white_hysteresis(const tcs3200_t *sensor,
                                 double frequency_hz,
                                 bool previous_is_white) {
    if (!is_valid_sensor(sensor)) {
        return false;
    }

    if (frequency_hz <= sensor->config.black_threshold_hz) {
        return false;
    }

    if (frequency_hz >= sensor->config.white_threshold_hz) {
        return true;
    }

    return previous_is_white;
}

const char *tcs3200_classify(const tcs3200_t *sensor, double frequency_hz) {
    return tcs3200_is_white(sensor, frequency_hz) ? "WHITE" : "BLACK";
}

const char *tcs3200_classify_hysteresis(const tcs3200_t *sensor,
                                        double frequency_hz,
                                        bool previous_is_white) {
    return tcs3200_is_white_hysteresis(sensor, frequency_hz, previous_is_white)
               ? "WHITE"
               : "BLACK";
}
