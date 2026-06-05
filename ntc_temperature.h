#ifndef NTC_TEMPERATURE_H
#define NTC_TEMPERATURE_H

#include <stdbool.h>

typedef struct {
    double vcc;
    double r_fixed_ohm;
    double r0_ohm;
    double t0_kelvin;
    double beta;
    double cal_vout;
    double cal_rntc_ohm;
} ntc_temperature_config_t;

#ifdef __cplusplus
extern "C" {
#endif

bool ntc_temperature_init(const ntc_temperature_config_t *config);
bool ntc_temperature_read_celsius(double *temp_c, double *v_out, double *r_ntc);

#ifdef __cplusplus
}
#endif

#endif // NTC_TEMPERATURE_H
