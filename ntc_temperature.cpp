#include "ntc_temperature.h"
#include <stddef.h>
#include <math.h>
extern "C" {
#include <libpynq.h>
}

static ntc_temperature_config_t active_config = {
    .vcc = 3.3,
    .r_fixed_ohm = 10000.0,
    .r0_ohm = 10000.0,
    .t0_kelvin = 298.15,
    .beta = 4050.0,
    .cal_vout = 0.643,
    .cal_rntc_ohm = 9600.0,
};

static double r_load_ohm = -1.0;
static bool initialized = false;

static double parallel_from_voltage(double v_out) {
    if (v_out <= 0.0 || v_out >= active_config.vcc) {
        return -1.0;
    }
    return active_config.r_fixed_ohm * v_out / (active_config.vcc - v_out);
}

static double calculate_r_load(double v_cal, double r_ntc_cal) {
    double r_parallel = parallel_from_voltage(v_cal);
    double denom;

    if (r_parallel <= 0.0 || r_ntc_cal <= 0.0) {
        return -1.0;
    }

    denom = (1.0 / r_parallel) - (1.0 / r_ntc_cal);
    if (denom <= 0.0) {
        return -1.0;
    }

    return 1.0 / denom;
}

static double ntc_from_parallel(double r_parallel, double r_load) {
    double denom;

    if (r_parallel <= 0.0 || r_load <= 0.0) {
        return -1.0;
    }

    denom = (1.0 / r_parallel) - (1.0 / r_load);
    if (denom <= 0.0) {
        return -1.0;
    }

    return 1.0 / denom;
}

static double resistance_to_temp_c(double r_ntc) {
    double temp_k;
    temp_k = 1.0 / ((1.0 / active_config.t0_kelvin) +
                    (log(r_ntc / active_config.r0_ohm) / active_config.beta));
    return temp_k - 273.15;
}

extern "C" bool ntc_temperature_init(const ntc_temperature_config_t *config) {
    if (config != NULL) {
        active_config = *config;
    }

    adc_init();
    r_load_ohm = calculate_r_load(active_config.cal_vout, active_config.cal_rntc_ohm);
    initialized = (r_load_ohm > 0.0);
    return initialized;
}

extern "C" bool ntc_temperature_read_celsius(double *temp_c, double *v_out, double *r_ntc) {
    double local_v_out;
    double r_parallel;
    double local_r_ntc;

    if (!initialized || temp_c == NULL) {
        return false;
    }

    local_v_out = adc_read_channel(ADC0);
    r_parallel = parallel_from_voltage(local_v_out);
    local_r_ntc = ntc_from_parallel(r_parallel, r_load_ohm);

    if (local_r_ntc <= 0.0) {
        return false;
    }

    *temp_c = resistance_to_temp_c(local_r_ntc);

    if (v_out != NULL) {
        *v_out = local_v_out;
    }
    if (r_ntc != NULL) {
        *r_ntc = local_r_ntc;
    }

    return true;
}
