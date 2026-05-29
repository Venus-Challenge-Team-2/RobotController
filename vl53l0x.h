#ifndef _TOFLIB_H_
#define _TOFLIB_H_

#include <cstdint>

extern "C" {
#include <libpynq.h>
}

typedef struct _vl53_sensor_ {
    iic_index_t iic_index;
    uint8_t baseAddr;
    uint8_t stop_variable;
    uint32_t measurement_timing_budget_us;
} vl53x;

int tofSetAddress(iic_index_t iic, uint8_t addr, uint8_t newAddr);
int tofPing(iic_index_t iic, uint8_t addr);
int tofInit(vl53x *sensor, iic_index_t iic, uint8_t addr, int bLongRange);
int tofGetModel(vl53x *sensor, uint8_t *model, uint8_t *revision);
uint32_t tofReadDistance(vl53x *sensor);

#endif
