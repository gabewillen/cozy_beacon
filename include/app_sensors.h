//
// Created by gwillen on 1/15/21.
//

#ifndef COZY_BEACON_MAKE_APP_SENSORS_H
#define COZY_BEACON_MAKE_APP_SENSORS_H

#include "nrfx_twi.h"

#ifdef __cplusplus
extern "C" {
#endif

extern nrfx_twi_t app_twi;


#define SPL06_007_RATE(rate) \
    rate == 128 ? 0b111 :               \
    rate == 64 ? 0b110 :                \
    rate == 32 ? 0b101 :                \
    rate == 16 ? 0b100 :                \
    rate == 8 ? 0b011 :                 \
    rate == 4 ? 0b010 :                 \
    rate == 2 ? 0b001 : 0

#define SPL06_007_ADDR 0x76
#define SPL06_007_PSR_B2 0
#define SPL06_007_TMP_B2_REG 0x03
// Pressure Configuration (PRS_CFG)
#define SPL06_007_PRS_CFG_REG 0x06
#define SPL06_007_PRS_CFG_PM_RATE(rate) (SPL06_007_RATE(rate) << 4)
#define SPL06_007_PRS_CFG_PM_PRC(rate) SPL06_007_RATE(rate)
// Temperature configuration (TMP_CFG)
#define SPL06_007_TMP_CFG_REG 0x07
#define SPL06_007_TMP_CFG_TMP_EXT BIT_7
#define SPL06_007_TMP_CFG_TMP_RATE(rate) (SPL06_007_RATE(rate) << 4)
#define SPL06_007_TMP_CFG_TMP_PRC(rate) (SPL06_007_RATE(rate))
// Sensor Operating Mode and Status (MEAS_CFG)
#define SPL06_007_MEAS_CFG_REG 0x08
#define SPL06_007_MEAS_CFG_COEF_RDY(byte) IS_SET(byte, 7)
#define SPL06_007_MEAS_CFG_SENSOR_RDY(byte) IS_SET(byte, 6)
#define SPL06_007_MEAS_CFG_TMP_RDY(byte) IS_SET(byte, 5)

#define SPL06_007_MEAS_CTRL_IDLE  0
#define SPL06_007_MEAS_CTRL_PRS  0b1
#define SPL06_007_MEAS_CTRL_TMP  0b010
#define SPL06_007_MEAS_CTRL_CONT_PRS  0b101
#define SPL06_007_MEAS_CTRL_CONT_TMP  0b110
#define SPL06_007_MEAS_CTRL_CONT_BOTH SPL06_007_MEAS_CTRL_CONT_PRS | SPL06_007_MEAS_CTRL_CONT_TMP

// Interrupt and FIFO configuration (CFG_REG)
#define SPL06_007_CFG_REG 0x09
#define SPL06_007_CFG_INT_HIGH BIT_7
#define SPL06_007_CFG_INT_FIFO BIT_6
#define SPL06_007_CFG_INT_PRS BIT_5
#define SPL06_007_CFG_INT_TMP BIT_4
#define SPL06_007_CFG_INT_T_SHIFT BIT_3
#define SPL06_007_CFG_INT_P_SHIFT BIT_2
#define SPL06_007_CFG_INT_FIFO_ENABLE BIT_1
#define SPL06_007_CFG_SPI BIT_0

// Interrupt Status (INT_STS)
#define SPL06_007_INT_STS_REG 0x0A
#define SPL06_007_INT_STS_INT_FIFO_FULL(byte) IS_SET(byte, 2)
#define SPL06_007_INT_STS_INT_TMP(byte) IS_SET(byte, 1)
#define SPL06_007_INT_STS_INT_PRS(byte) IS_SET(byte, 0)

// FIFO Status (FIFO_STS)
#define SPL06_007_FIFO_STS_REG 0x0B
#define SPL06_007_FIFO_STS_FIFO_FULL(byte) IS_SET(byte, 1)
#define SPL06_007_FIFO_STS_FIFO_EMPTY(byte) IS_SET(byte, 0)

// Soft Reset and FIFO flush (RESET)
#define SPL06_007_RESET_REG 0x0c
#define SPL06_007_RESET_FIFO_FLUSH BIT_7
#define SPL06_007_RESET_SOFT_RST 0b1001

typedef struct {
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;
} spl06_007_coef_t;



typedef struct  {
    int32_t raw_temperature;
    int32_t raw_pressure;
    uint8_t meas_cfg;
    spl06_007_coef_t coef;
} spl06_007_t ;



#define SPL06_007_SOFT_RESET 0b00001001
#define SPL06_007_FIFO_FLUSH 0b10000000

#define ICM_20600_ADDR 0x69
#define ICM_20600_PWR_MGMT_1 0x6b
#define ICM_20600_PWR_MGMT_2 0x6c
#define ICM_20600_ACCEL_CONFIG2 0x1d
#define ICM_20600_INT_ENABLE 0x38
#define ICM_20600_ACCEL_WOM_X_THR 0x20
#define ICM_20600_ACCEL_WOM_Y_THR 0x21
#define ICM_20600_ACCEL_WOM_Z_THR 0x22
#define ICM_20600_ACCEL_INTEL_CTRL 0x69
#define ICM_20600_SMPLRT_DIV 0x19

#define AHT20_ADDRESS 0x38

void app_sensors_init();

void ICM_20600_int_clear();



#ifdef __cplusplus
}
#endif

#endif //COZY_BEACON_MAKE_APP_SENSORS_H
