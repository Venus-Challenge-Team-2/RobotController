extern "C" {
#include <libpynq.h>
}
#include "vl53l0x.h"
#include <unistd.h>
#include <cstdio>
#include <cstdint>
#include <cstring>

static uint8_t readReg(vl53x *ptr_s, uint8_t ucAddr);
static unsigned short readReg16(vl53x *ptr_s, uint8_t ucAddr);
static void writeReg16(vl53x *ptr_s, uint8_t ucAddr, unsigned short usValue);
static void writeReg(vl53x *ptr_s, uint8_t ucAddr, uint8_t ucValue);
static void writeRegList(vl53x *ptr_s, uint8_t *ucList);
static int initSensor(vl53x *ptr_s, int);
static int performSingleRefCalibration(vl53x *ptr_s, uint8_t vhv_init_byte);
static int setMeasurementTimingBudget(vl53x *ptr_s, uint32_t budget_us);

#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

#define VL53L0X_SEQUENCE_ENABLE_FINAL_RANGE 0x80
#define VL53L0X_SEQUENCE_ENABLE_PRE_RANGE   0x40
#define VL53L0X_SEQUENCE_ENABLE_TCC         0x10
#define VL53L0X_SEQUENCE_ENABLE_DSS         0x08
#define VL53L0X_SEQUENCE_ENABLE_MSRC        0x04

typedef enum vcselperiodtype { VcselPeriodPreRange, VcselPeriodFinalRange } vcselPeriodType;
static int setVcselPulsePeriod(vl53x *ptr_s, vcselPeriodType type, uint8_t period_pclks);

typedef struct tagSequenceStepTimeouts
{
  uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;
  uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
  uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
} SequenceStepTimeouts;

#define VL53L0X_REG_IDENTIFICATION_MODEL_ID		0xc0
#define VL53L0X_EXPECTED_MODEL_ID 0xEE
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID		0xc2
#define VL53L0X_REG_SYSRANGE_START			0x00
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS 		0x13
#define VL53L0X_RESULT_RANGE_STATUS      		0x14
#define VL53L0X_ALGO_PHASECAL_LIM                       0x30
#define VL53L0X_ALGO_PHASECAL_CONFIG_TIMEOUT            0x30
#define VL53L0X_GLOBAL_CONFIG_VCSEL_WIDTH               0x32
#define VL53L0X_FINAL_RANGE_CONFIG_VALID_PHASE_LOW      0x47
#define VL53L0X_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH     0x48
#define VL53L0X_PRE_RANGE_CONFIG_VCSEL_PERIOD           0x50
#define VL53L0X_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI      0x51
#define VL53L0X_PRE_RANGE_CONFIG_VALID_PHASE_LOW        0x56
#define VL53L0X_PRE_RANGE_CONFIG_VALID_PHASE_HIGH       0x57
#define VL53L0X_REG_MSRC_CONFIG_CONTROL                 0x60
#define VL53L0X_FINAL_RANGE_CONFIG_VCSEL_PERIOD         0x70
#define VL53L0X_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI    0x71
#define VL53L0X_MSRC_CONFIG_TIMEOUT_MACROP              0x46
#define VL53L0X_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT  0x44
#define VL53L0X_SYSRANGE_START                          0x00
#define VL53L0X_SYSTEM_SEQUENCE_CONFIG                  0x01
#define VL53L0X_SYSTEM_INTERRUPT_CONFIG_GPIO            0x0A
#define VL53L0X_RESULT_INTERRUPT_STATUS                 0x13
#define VL53L0X_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV       0x89
#define VL53L0X_GLOBAL_CONFIG_SPAD_ENABLES_REF_0        0xB0
#define VL53L0X_GPIO_HV_MUX_ACTIVE_HIGH                 0x84
#define VL53L0X_SYSTEM_INTERRUPT_CLEAR                  0x0B
#define VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS 0x8A

int tofSetAddress(iic_index_t iic, uint8_t addr, uint8_t newAddr)
{
  return iic_write_register(iic, addr, VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS, &newAddr, 1);
}

int tofPing(iic_index_t iic, uint8_t addr)
{
  uint8_t model = 0;
  iic_read_register(iic, addr, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &model, 1);
  return (model != VL53L0X_EXPECTED_MODEL_ID);
}

int tofInit(vl53x *ptr_s, iic_index_t iic, uint8_t addr, int bLongRange)
{
  ptr_s->iic_index = iic;
  ptr_s->baseAddr = addr;
  return initSensor(ptr_s, bLongRange);
}

static unsigned short readReg16(vl53x *ptr_s, uint8_t ucAddr)
{
  uint8_t ucTemp[2];
  iic_read_register(ptr_s->iic_index, ptr_s->baseAddr, ucAddr, ucTemp, 2);
  return (unsigned short)((ucTemp[0]<<8) + ucTemp[1]);
}

static uint8_t readReg(vl53x *ptr_s, uint8_t ucAddr)
{
  uint8_t ucTemp;
  iic_read_register(ptr_s->iic_index, ptr_s->baseAddr, ucAddr, &ucTemp, 1);
  return ucTemp;
}

static void readMulti(vl53x *ptr_s, uint8_t ucAddr, uint8_t *pBuf, int iCount)
{
  iic_read_register(ptr_s->iic_index, ptr_s->baseAddr, ucAddr, pBuf, iCount);
}

static void writeMulti(vl53x *ptr_s, uint8_t ucAddr, uint8_t *pBuf, int iCount)
{
  iic_write_register(ptr_s->iic_index, ptr_s->baseAddr, ucAddr, pBuf, iCount);
}

static void writeReg16(vl53x *ptr_s, uint8_t ucAddr, unsigned short usValue)
{
  uint8_t pBuf[2];
  pBuf[0] = (uint8_t)(usValue >> 8);
  pBuf[1] = (uint8_t) usValue;
  iic_write_register(ptr_s->iic_index, ptr_s->baseAddr, ucAddr, pBuf, 2);
}

static void writeReg(vl53x *ptr_s, uint8_t ucAddr, uint8_t ucValue)
{
  iic_write_register(ptr_s->iic_index, ptr_s->baseAddr, ucAddr, &ucValue, 1);
}

static void writeRegList(vl53x *ptr_s, uint8_t *ucList)
{
  uint8_t ucCount = *ucList++;
  while (ucCount)
  {
    iic_write_register(ptr_s->iic_index, ptr_s->baseAddr, ucList[0], &(ucList[1]), 1);
    ucList += 2;
    ucCount--;
  }
}

uint8_t ucI2CMode[] = {4, 0x88,0x00, 0x80,0x01, 0xff,0x01, 0x00,0x00};
uint8_t ucI2CMode2[] = {3, 0x00,0x01, 0xff,0x00, 0x80,0x00};
uint8_t ucSPAD0[] = {4, 0x80,0x01, 0xff,0x01, 0x00,0x00, 0xff,0x06};
uint8_t ucSPAD1[] = {5, 0xff,0x07, 0x81,0x01, 0x80,0x01, 0x94,0x6b, 0x83,0x00};
uint8_t ucSPAD2[] = {4, 0xff,0x01, 0x00,0x01, 0xff,0x00, 0x80,0x00};
uint8_t ucSPAD[] = {5, 0xff,0x01, 0x4f,0x00, 0x4e,0x2c, 0xff,0x00, 0xb6,0xb4};
uint8_t ucDefTuning[] = {80, 0xff,0x01, 0x00,0x00, 0xff,0x00, 0x09,0x00,
0x10,0x00, 0x11,0x00, 0x24,0x01, 0x25,0xff, 0x75,0x00, 0xff,0x01, 0x4e,0x2c,
0x48,0x00, 0x30,0x20, 0xff,0x00, 0x30,0x09, 0x54,0x00, 0x31,0x04, 0x32,0x03,
0x40,0x83, 0x46,0x25, 0x60,0x00, 0x27,0x00, 0x50,0x06, 0x51,0x00, 0x52,0x96,
0x56,0x08, 0x57,0x30, 0x61,0x00, 0x62,0x00, 0x64,0x00, 0x65,0x00, 0x66,0xa0,
0xff,0x01, 0x22,0x32, 0x47,0x14, 0x49,0xff, 0x4a,0x00, 0xff,0x00, 0x7a,0x0a,
0x7b,0x00, 0x78,0x21, 0xff,0x01, 0x23,0x34, 0x42,0x00, 0x44,0xff, 0x45,0x26,
0x46,0x05, 0x40,0x40, 0x0e,0x06, 0x20,0x1a, 0x43,0x40, 0xff,0x00, 0x34,0x03,
0x35,0x44, 0xff,0x01, 0x31,0x04, 0x4b,0x09, 0x4c,0x05, 0x4d,0x04, 0xff,0x00,
0x44,0x00, 0x45,0x20, 0x47,0x08, 0x48,0x28, 0x67,0x00, 0x70,0x04, 0x71,0x01,
0x72,0xfe, 0x76,0x00, 0x77,0x00, 0xff,0x01, 0x0d,0x01, 0xff,0x00, 0x80,0x01,
0x01,0xf8, 0xff,0x01, 0x8e,0x01, 0x00,0x01, 0xff,0x00, 0x80,0x00};

int getSpadInfo(vl53x *ptr_s, uint8_t *pCount, uint8_t *pTypeIsAperture)
{
  int iTimeout;
  uint8_t ucTemp;
  #define VL53L0X_SPAD_MAX_TIMEOUT 50
  writeRegList(ptr_s, ucSPAD0);
  writeReg(ptr_s, 0x83, readReg(ptr_s, 0x83) | 0x04);
  writeRegList(ptr_s, ucSPAD1);
  iTimeout = 0;
  while(iTimeout < VL53L0X_SPAD_MAX_TIMEOUT)
  {
    if (readReg(ptr_s, 0x83) != 0x00) break;
    iTimeout++;
    sleep_msec(5);
  }
  if (iTimeout == VL53L0X_SPAD_MAX_TIMEOUT) return 0;
  writeReg(ptr_s, 0x83,0x01);
  ucTemp = readReg(ptr_s, 0x92);
  *pCount = (ucTemp & 0x7f);
  *pTypeIsAperture = (ucTemp & 0x80);
  writeReg(ptr_s, 0x81,0x00);
  writeReg(ptr_s, 0xff,0x06);
  writeReg(ptr_s, 0x83, readReg(ptr_s, 0x83) & ~0x04);
  writeRegList(ptr_s, ucSPAD2);
  return 1;
}

static uint16_t decodeTimeout(uint16_t reg_val)
{
  return (uint16_t)((reg_val & 0x00FF) << (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

static uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
  return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

static uint16_t encodeTimeout(uint16_t timeout_mclks)
{
  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;
  if (timeout_mclks > 0)
  {
    ls_byte = timeout_mclks - 1;
    while ((ls_byte & 0xFFFFFF00) > 0)
    {
      ls_byte >>= 1;
      ms_byte++;
    }
    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  return 0;
}

static void getSequenceStepTimeouts(vl53x *ptr_s, uint8_t enables, SequenceStepTimeouts * timeouts)
{
  timeouts->pre_range_vcsel_period_pclks = ((readReg(ptr_s, VL53L0X_PRE_RANGE_CONFIG_VCSEL_PERIOD) +1) << 1);
  timeouts->msrc_dss_tcc_mclks = readReg(ptr_s, VL53L0X_MSRC_CONFIG_TIMEOUT_MACROP) + 1;
  timeouts->msrc_dss_tcc_us = timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks, timeouts->pre_range_vcsel_period_pclks);
  timeouts->pre_range_mclks = decodeTimeout(readReg16(ptr_s, VL53L0X_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
  timeouts->pre_range_us = timeoutMclksToMicroseconds(timeouts->pre_range_mclks, timeouts->pre_range_vcsel_period_pclks);
  timeouts->final_range_vcsel_period_pclks = ((readReg(ptr_s, VL53L0X_FINAL_RANGE_CONFIG_VCSEL_PERIOD) +1) << 1);
  timeouts->final_range_mclks = decodeTimeout(readReg16(ptr_s, VL53L0X_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));
  if (enables & VL53L0X_SEQUENCE_ENABLE_PRE_RANGE) timeouts->final_range_mclks -= timeouts->pre_range_mclks;
  timeouts->final_range_us = timeoutMclksToMicroseconds(timeouts->final_range_mclks, timeouts->final_range_vcsel_period_pclks);
}

static int setVcselPulsePeriod(vl53x *ptr_s, vcselPeriodType type, uint8_t period_pclks)
{
  uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);
  uint8_t enables;
  SequenceStepTimeouts timeouts;
  enables = readReg(ptr_s, VL53L0X_SYSTEM_SEQUENCE_CONFIG);
  getSequenceStepTimeouts(ptr_s, enables, &timeouts);
  if (type == VcselPeriodPreRange)
  {
    switch (period_pclks)
    {
      case 12: writeReg(ptr_s, VL53L0X_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18); break;
      case 14: writeReg(ptr_s, VL53L0X_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30); break;
      case 16: writeReg(ptr_s, VL53L0X_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40); break;
      case 18: writeReg(ptr_s, VL53L0X_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50); break;
      default: return 0;
    }
    writeReg(ptr_s, VL53L0X_PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
    writeReg(ptr_s, VL53L0X_PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);
    uint16_t new_pre_range_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);
    writeReg16(ptr_s, VL53L0X_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(new_pre_range_timeout_mclks));
    uint16_t new_msrc_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);
    writeReg(ptr_s, VL53L0X_MSRC_CONFIG_TIMEOUT_MACROP, (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));
  }
  else if (type == VcselPeriodFinalRange)
  {
    switch (period_pclks)
    {
      case 8:
        writeReg(ptr_s, VL53L0X_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
        writeReg(ptr_s, VL53L0X_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(ptr_s, VL53L0X_GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
        writeReg(ptr_s, VL53L0X_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
        writeReg(ptr_s, 0xFF, 0x01);
        writeReg(ptr_s, VL53L0X_ALGO_PHASECAL_LIM, 0x30);
        writeReg(ptr_s, 0xFF, 0x00);
        break;
      case 10:
        writeReg(ptr_s, VL53L0X_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
        writeReg(ptr_s, VL53L0X_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(ptr_s, VL53L0X_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        writeReg(ptr_s, VL53L0X_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
        writeReg(ptr_s, 0xFF, 0x01);
        writeReg(ptr_s, VL53L0X_ALGO_PHASECAL_LIM, 0x20);
        writeReg(ptr_s, 0xFF, 0x00);
        break;
      case 12:
        writeReg(ptr_s, VL53L0X_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
        writeReg(ptr_s, VL53L0X_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(ptr_s, VL53L0X_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        writeReg(ptr_s, VL53L0X_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
        writeReg(ptr_s, 0xFF, 0x01);
        writeReg(ptr_s, VL53L0X_ALGO_PHASECAL_LIM, 0x20);
        writeReg(ptr_s, 0xFF, 0x00);
        break;
      case 14:
        writeReg(ptr_s, VL53L0X_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
        writeReg(ptr_s, VL53L0X_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(ptr_s, VL53L0X_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        writeReg(ptr_s, VL53L0X_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
        writeReg(ptr_s, 0xFF, 0x01);
        writeReg(ptr_s, VL53L0X_ALGO_PHASECAL_LIM, 0x20);
        writeReg(ptr_s, 0xFF, 0x00);
        break;
      default: return 0;
    }
    writeReg(ptr_s, VL53L0X_FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);
    uint16_t new_final_range_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);
    if (enables & VL53L0X_SEQUENCE_ENABLE_PRE_RANGE) new_final_range_timeout_mclks += timeouts.pre_range_mclks;
    writeReg16(ptr_s, VL53L0X_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(new_final_range_timeout_mclks));
  }
  else return 0;
  setMeasurementTimingBudget(ptr_s, ptr_s->measurement_timing_budget_us);
  uint8_t sequence_config = readReg(ptr_s, VL53L0X_SYSTEM_SEQUENCE_CONFIG);
  writeReg(ptr_s, VL53L0X_SYSTEM_SEQUENCE_CONFIG, 0x02);
  performSingleRefCalibration(ptr_s, 0x0);
  writeReg(ptr_s, VL53L0X_SYSTEM_SEQUENCE_CONFIG, sequence_config);
  return 1;
}

int setMeasurementTimingBudget(vl53x *ptr_s, uint32_t budget_us)
{
  uint32_t used_budget_us;
  uint32_t final_range_timeout_us;
  uint16_t final_range_timeout_mclks;
  uint8_t enables;
  SequenceStepTimeouts timeouts;
  const uint16_t StartOverhead      = 1320;
  const uint16_t EndOverhead        = 960;
  const uint16_t MsrcOverhead       = 660;
  const uint16_t TccOverhead        = 590;
  const uint16_t DssOverhead        = 690;
  const uint16_t PreRangeOverhead   = 660;
  const uint16_t FinalRangeOverhead = 550;
  const uint32_t MinTimingBudget = 20000;
  if (budget_us < MinTimingBudget) return 0;
  used_budget_us = StartOverhead + EndOverhead;
  enables = readReg(ptr_s, VL53L0X_SYSTEM_SEQUENCE_CONFIG);
  getSequenceStepTimeouts(ptr_s, enables, &timeouts);
  if (enables & VL53L0X_SEQUENCE_ENABLE_TCC) used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  if (enables & VL53L0X_SEQUENCE_ENABLE_DSS) used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  else if (enables & VL53L0X_SEQUENCE_ENABLE_MSRC) used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  if (enables & VL53L0X_SEQUENCE_ENABLE_PRE_RANGE) used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  if (enables & VL53L0X_SEQUENCE_ENABLE_FINAL_RANGE)
  {
    used_budget_us += FinalRangeOverhead;
    if (used_budget_us > budget_us) return 0;
    final_range_timeout_us = budget_us - used_budget_us;
    final_range_timeout_mclks = timeoutMicrosecondsToMclks(final_range_timeout_us, timeouts.final_range_vcsel_period_pclks);
    if (enables & VL53L0X_SEQUENCE_ENABLE_PRE_RANGE) final_range_timeout_mclks += timeouts.pre_range_mclks;
    writeReg16(ptr_s, VL53L0X_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(final_range_timeout_mclks));
    ptr_s->measurement_timing_budget_us = budget_us;
  }
  return 1;
}

uint32_t getMeasurementTimingBudget(vl53x *ptr_s)
{
  uint8_t enables;
  SequenceStepTimeouts timeouts;
  const uint16_t StartOverhead     = 1910;
  const uint16_t EndOverhead        = 960;
  const uint16_t MsrcOverhead       = 660;
  const uint16_t TccOverhead        = 590;
  const uint16_t DssOverhead        = 690;
  const uint16_t PreRangeOverhead   = 660;
  const uint16_t FinalRangeOverhead = 550;
  uint32_t budget_us = StartOverhead + EndOverhead;
  enables = readReg(ptr_s, VL53L0X_SYSTEM_SEQUENCE_CONFIG);
  getSequenceStepTimeouts(ptr_s, enables, &timeouts);
  if (enables & VL53L0X_SEQUENCE_ENABLE_TCC) budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  if (enables & VL53L0X_SEQUENCE_ENABLE_DSS) budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  else if (enables & VL53L0X_SEQUENCE_ENABLE_MSRC) budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  if (enables & VL53L0X_SEQUENCE_ENABLE_PRE_RANGE) budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  if (enables & VL53L0X_SEQUENCE_ENABLE_FINAL_RANGE) budget_us += (timeouts.final_range_us + FinalRangeOverhead);
  ptr_s->measurement_timing_budget_us = budget_us;
  return budget_us;
}

int performSingleRefCalibration(vl53x *ptr_s, uint8_t vhv_init_byte)
{
  writeReg(ptr_s, VL53L0X_SYSRANGE_START, 0x01 | vhv_init_byte);
  int iTimeout = 0;
  while ((readReg(ptr_s, VL53L0X_RESULT_INTERRUPT_STATUS) & 0x07) == 0)
  {
    iTimeout++;
    sleep_msec(5);
    if (iTimeout > 100) return 0;
  }
  writeReg(ptr_s, VL53L0X_SYSTEM_INTERRUPT_CLEAR, 0x01);
  writeReg(ptr_s, VL53L0X_SYSRANGE_START, 0x00);
  return 1;
}

int initSensor(vl53x *ptr_s, int bLongRangeMode)
{
  uint8_t spad_count=0, spad_type_is_aperture=0, ref_spad_map[6];
  uint8_t ucFirstSPAD, ucSPADsEnabled;
  int i;
  writeReg(ptr_s, VL53L0X_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, readReg(ptr_s, VL53L0X_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01);
  writeRegList(ptr_s, ucI2CMode);
  ptr_s->stop_variable = readReg(ptr_s, 0x91);
  writeRegList(ptr_s, ucI2CMode2);
  writeReg(ptr_s, VL53L0X_REG_MSRC_CONFIG_CONTROL, readReg(ptr_s, VL53L0X_REG_MSRC_CONFIG_CONTROL) | 0x12);
  writeReg16(ptr_s, VL53L0X_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 32);
  writeReg(ptr_s, VL53L0X_SYSTEM_SEQUENCE_CONFIG, 0xFF);
  getSpadInfo(ptr_s, &spad_count, &spad_type_is_aperture);
  readMulti(ptr_s, VL53L0X_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);
  writeRegList(ptr_s, ucSPAD);
  ucFirstSPAD = (spad_type_is_aperture) ? 12: 0;
  ucSPADsEnabled = 0;
  for (i=0; i<48; i++)
  {
    if (i < ucFirstSPAD || ucSPADsEnabled == spad_count) ref_spad_map[i>>3] &= ~(1<<(i & 7));
    else if (ref_spad_map[i>>3] & (1<< (i & 7))) ucSPADsEnabled++;
  }
  writeMulti(ptr_s, VL53L0X_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);
  writeRegList(ptr_s, ucDefTuning);
  if (bLongRangeMode)
  {
    writeReg16(ptr_s, VL53L0X_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 13);
    setVcselPulsePeriod(ptr_s, VcselPeriodPreRange, 18);
    setVcselPulsePeriod(ptr_s, VcselPeriodFinalRange, 14);
  }
  writeReg(ptr_s, VL53L0X_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
  writeReg(ptr_s, VL53L0X_GPIO_HV_MUX_ACTIVE_HIGH, readReg(ptr_s, VL53L0X_GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10);
  writeReg(ptr_s, VL53L0X_SYSTEM_INTERRUPT_CLEAR, 0x01);
  ptr_s->measurement_timing_budget_us = getMeasurementTimingBudget(ptr_s);
  writeReg(ptr_s, VL53L0X_SYSTEM_SEQUENCE_CONFIG, 0xe8);
  setMeasurementTimingBudget(ptr_s, ptr_s->measurement_timing_budget_us);
  writeReg(ptr_s, VL53L0X_SYSTEM_SEQUENCE_CONFIG, 0x01);
  if (!performSingleRefCalibration(ptr_s, 0x40)) return 1;
  writeReg(ptr_s, VL53L0X_SYSTEM_SEQUENCE_CONFIG, 0x02);
  if (!performSingleRefCalibration(ptr_s, 0x00)) return 1;
  writeReg(ptr_s, VL53L0X_SYSTEM_SEQUENCE_CONFIG, 0xe8);
  return 0;
}

uint16_t readRangeContinuousMillimeters(vl53x *ptr_s)
{
  int iTimeout = 0;
  uint16_t range;
  while ((readReg(ptr_s, VL53L0X_RESULT_INTERRUPT_STATUS) & 0x07) == 0)
  {
    iTimeout++;
    sleep_msec(50);
    if (iTimeout > 50) {
      writeReg(ptr_s, VL53L0X_SYSTEM_INTERRUPT_CLEAR, 0x01);
      return 0;
    }
  }
  range = readReg16(ptr_s, VL53L0X_RESULT_RANGE_STATUS + 10);
  writeReg(ptr_s, VL53L0X_SYSTEM_INTERRUPT_CLEAR, 0x01);
  return range;
}

uint32_t tofReadDistance(vl53x *sensor)
{
  int iTimeout;
  writeReg(sensor, 0x80, 0x01);
  writeReg(sensor, 0xFF, 0x01);
  writeReg(sensor, 0x00, 0x00);
  writeReg(sensor, 0x91, sensor->stop_variable);
  writeReg(sensor, 0x00, 0x01);
  writeReg(sensor, 0xFF, 0x00);
  writeReg(sensor, 0x80, 0x00);
  writeReg(sensor, VL53L0X_SYSRANGE_START, 0x01);
  iTimeout = 0;
  while (readReg(sensor, VL53L0X_SYSRANGE_START) & 0x01)
  {
    iTimeout++;
    sleep_msec(50);
    if (iTimeout > 50) {
      writeReg(sensor, VL53L0X_SYSRANGE_START, 0x00);
      writeReg(sensor, VL53L0X_SYSTEM_INTERRUPT_CLEAR, 0x01);
      return 0;
    }
  }
  return readRangeContinuousMillimeters(sensor);
}

int tofGetModel(vl53x *sensor, uint8_t *model, uint8_t *revision)
{
  uint8_t ucTemp[2];
  if (model)
  {
    if (iic_read_register(sensor->iic_index, sensor->baseAddr, VL53L0X_REG_IDENTIFICATION_MODEL_ID, ucTemp, 1) == 0)
      *model = ucTemp[0];
    else return 1;
  }
  if (revision)
  {
    if (iic_read_register(sensor->iic_index, sensor->baseAddr, VL53L0X_REG_IDENTIFICATION_REVISION_ID, ucTemp, 1) == 0)
      *revision = ucTemp[0];
    else return 1;
  }
  return 0;
}
