#ifndef TCS3200_H
#define TCS3200_H

#include <stdbool.h>
#include <stdint.h>
#include <gpio.h>
#include <pulsecounter.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    io_t out_pin;
    io_t s0_pin;
    io_t s1_pin;
    io_t s2_pin;
    io_t s3_pin;
    double black_threshold_hz;
    double white_threshold_hz;
    int measure_window_ms;
    pulsecounter_index_t pulsecounter;
} tcs3200_config_t;

typedef struct {
    tcs3200_config_t config;
} tcs3200_t;

bool tcs3200_init(tcs3200_t *sensor, const tcs3200_config_t *config);
double tcs3200_read_frequency_hz(const tcs3200_t *sensor);
bool tcs3200_is_white(const tcs3200_t *sensor, double frequency_hz);
bool tcs3200_is_white_hysteresis(const tcs3200_t *sensor,
                                 double frequency_hz,
                                 bool previous_is_white);
const char *tcs3200_classify(const tcs3200_t *sensor, double frequency_hz);
const char *tcs3200_classify_hysteresis(const tcs3200_t *sensor,
                                        double frequency_hz,
                                        bool previous_is_white);

#ifdef __cplusplus
}
#endif

#endif // TCS3200_H
