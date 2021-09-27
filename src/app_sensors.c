//
// Created by gwillen on 1/15/21.
//

#include <nrfx_twi.h>
#include <nrfx_gpiote.h>
#include "nrfx_log.h"
#include "app_sensors.h"
#include "app_util_bds.h"
#include "nrfx_temp.h"
#include "nrf_bitmask.h"
#include "app_advertising.h"
#include "app_timer.h"
#include "app_scheduler.h"
#include "custom_board.h"
APP_TIMER_DEF(ICM_20600_motion_timout);
APP_TIMER_DEF(spl06_007_temperature_measurement_timeout);
APP_TIMER_DEF(spl06_007_pressure_measurement_timeout);
APP_TIMER_DEF(aht20_measurement_timeout);
APP_TIMER_DEF(app_sensor_timer);
nrfx_twi_t app_twi = NRFX_TWI_INSTANCE(0);

spl06_007_t spl06_007;

void SPL06_007_read_reg(uint8_t reg, void *buffer, uint32_t size) {
    APP_ERROR_CHECK(nrfx_twi_tx(&app_twi, SPL06_007_ADDR, &reg, 1, false));
    APP_ERROR_CHECK(nrfx_twi_rx(&app_twi, SPL06_007_ADDR, buffer, size));
}

void SPL06_007_write_reg(uint8_t reg, uint8_t byte) {
    uint8_t data[2] = {reg, byte};
    APP_ERROR_CHECK(nrfx_twi_tx(&app_twi, SPL06_007_ADDR, data, 2, true));
}

void SPL06_007_stop() {
    SPL06_007_write_reg(SPL06_007_MEAS_CFG_REG, 0);
}

void SPL06_007_start() {
    SPL06_007_write_reg(SPL06_007_MEAS_CFG_REG, SPL06_007_MEAS_CTRL_TMP);
    app_timer_start(spl06_007_temperature_measurement_timeout, APP_TIMER_TICKS(20), NULL);
}

//
//void SPL06_007_event_handler(void *event, uint16_t event_size) {
//    static uint8_t data[3];
////    struct  {
////        int32_t value: 24;
////    } measurement;
//    SPL06_007_read_reg(SPL06_007_MEAS_CFG_REG
//    for (int m=0; m < 8; m++) {
//        SPL06_007_read_reg(SPL06_007_PSR_B2, data, 3);
//        for (int i=0; i < 3; i++) {
//            NRF_LOG_INFO("data[%d] = %d", i, data[i]);
//        }
//    }
//
////    if (IS_SET(data[2], 0)) {
////         temperature
////    } else {
////         pressure
////    }
//    SPL06_007_write_reg(SPL06_007_RESET_REG, SPL06_007_RESET_FIFO_FLUSH);
//    SPL06_007_read_reg(SPL06_007_INT_STS_REG, data, 1);
//}

//void SPL06_007_gpiote_evt_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
//    if (nrf_gpio_pin_read(SPL06_007_INT_PIN)) {
//        app_sched_event_put(NULL, 0, SPL06_007_event_handler);
//    }
//}

void spl06_007_calculate_event(void *event, uint16_t event_size) {
    float t_sc = spl06_007.raw_temperature / 524288.0;
    float p_sc = spl06_007.raw_pressure / 524288.0;

    p_sc = (spl06_007.coef.c00 + p_sc * (spl06_007.coef.c10 + p_sc * ((int32_t)spl06_007.coef.c20 + p_sc * (int32_t)spl06_007.coef.c30)) +
           t_sc * spl06_007.coef.c01 + t_sc * p_sc * (spl06_007.coef.c11 + p_sc * spl06_007.coef.c21)) / 100;
    t_sc = ((spl06_007.coef.c0 >> 1) + spl06_007.coef.c1 * t_sc) * 100;
    app_manuf_data.spl06_007_temperature = t_sc;
    app_manuf_data.spl06_007_pressure = p_sc;
    app_sched_event_put(NULL, 0, app_advertising_update);
}

void spl06_007_temperature_measurement_timeout_handler(void *context) {
    uint8_t bytes[3] = {0, 0, 0};
    SPL06_007_read_reg(SPL06_007_TMP_B2_REG, bytes, 3);
//    spl06_007.raw_temperature = ((int8_t) bytes[0] << 24 | (int8_t) bytes[1] << 16 | (int8_t) bytes[2] << 8) >> 8;
    spl06_007.raw_temperature = (int32_t)bytes[0] << 16 | (int32_t)bytes[1] << 8 | (int32_t)bytes[2];
    spl06_007.raw_temperature = (spl06_007.raw_temperature & 0x800000) ? (0xFF000000 | spl06_007.raw_temperature) : spl06_007.raw_temperature;
    SPL06_007_write_reg(SPL06_007_MEAS_CFG_REG, SPL06_007_MEAS_CTRL_PRS);
    app_timer_start(spl06_007_pressure_measurement_timeout, APP_TIMER_TICKS(20), NULL);
}

void spl06_007_pressure_measurement_timeout_handler(void *context) {
    uint8_t bytes[3] = {0, 0, 0};
    SPL06_007_read_reg(SPL06_007_PSR_B2, bytes, 3);
    spl06_007.raw_pressure = (int32_t)bytes[0] << 16 | (int32_t)bytes[1] << 8 | (int32_t)bytes[2];
    spl06_007.raw_pressure = (spl06_007.raw_pressure & 0x800000) ? (0xFF000000 | spl06_007.raw_pressure) : spl06_007.raw_pressure;
    app_sched_event_put(NULL, 0, spl06_007_calculate_event);
}

void SPL06_007_read_coef() {
    static uint8_t buffer[18];
    SPL06_007_read_reg(0x10, buffer, 18);
    spl06_007.coef.c0 = (int16_t) buffer[0] << 4 | (buffer[1] & 0xF0) >> 4;
    spl06_007.coef.c0 = (spl06_007.coef.c0 & 0x0800) ? (0xF000 | spl06_007.coef.c0) : spl06_007.coef.c0;

    spl06_007.coef.c1 = (buffer[1] & ~0xf0) << 8 | buffer[2];
    spl06_007.coef.c1 = (spl06_007.coef.c1 & 0x0800) ? (0xF000 | spl06_007.coef.c1) : spl06_007.coef.c1;

    spl06_007.coef.c00 = buffer[3] << 12 | buffer[4] << 4 | buffer[5] >> 4;
    spl06_007.coef.c00 = (spl06_007.coef.c00 & 0x080000) ? (0xFFF00000 | spl06_007.coef.c00) : spl06_007.coef.c00;

    spl06_007.coef.c10 = (buffer[5] & ~0xf0) << 16 | buffer[6] << 8 | buffer[7];
    spl06_007.coef.c10 = (spl06_007.coef.c10 & 0x080000) ? (0xFFF00000 | spl06_007.coef.c10) : spl06_007.coef.c10;

    spl06_007.coef.c01 = (int16_t)buffer[8] << 8 | buffer[9];
    spl06_007.coef.c11 = (int16_t)buffer[10] << 8 | buffer[11];
    spl06_007.coef.c20 = (int16_t)buffer[12] << 8 | buffer[13];
    spl06_007.coef.c21 = (int16_t)buffer[14] << 8 | buffer[15];
    spl06_007.coef.c30 = (int16_t)buffer[16] << 8 | buffer[17];
}


void SPL06_007_reset(uint8_t flags) {
    uint8_t data[2] = {SPL06_007_RESET_REG, flags};
    APP_ERROR_CHECK(nrfx_twi_tx(&app_twi, SPL06_007_ADDR, data, 2, true));
    nrfx_coredep_delay_us(50000);
}

void SPL06_007_init() {

    uint8_t rx;
    SPL06_007_reset(SPL06_007_FIFO_FLUSH | SPL06_007_SOFT_RESET);
    SPL06_007_read_coef();
    SPL06_007_read_reg(SPL06_007_MEAS_CFG_REG, &rx, 1);
    if (IS_SET(rx, 6)) {
//        nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
//        APP_ERROR_CHECK(nrfx_gpiote_in_init(SPL06_007_INT_PIN, &in_config, SPL06_007_gpiote_evt_handler));
//        nrfx_gpiote_in_event_enable(SPL06_007_INT_PIN, true);
        SPL06_007_write_reg(SPL06_007_PRS_CFG_REG, SPL06_007_PRS_CFG_PM_PRC(1));
        SPL06_007_write_reg(SPL06_007_TMP_CFG_REG, SPL06_007_TMP_CFG_TMP_EXT | SPL06_007_TMP_CFG_TMP_PRC(1));
        app_timer_create(&spl06_007_pressure_measurement_timeout, APP_TIMER_MODE_SINGLE_SHOT,
                         spl06_007_pressure_measurement_timeout_handler);
        app_timer_create(&spl06_007_temperature_measurement_timeout, APP_TIMER_MODE_SINGLE_SHOT,
                         spl06_007_temperature_measurement_timeout_handler);
//        SPL06_007_write_reg(SPL06_007_CFG_REG, SPL06_007_CFG_INT_HIGH | SPL06_007_CFG_INT_TMP | SPL06_007_CFG_INT_PRS);
    }

}

void ICM_20600_read_reg(uint8_t reg, uint8_t *buffer, uint8_t length) {
    APP_ERROR_CHECK(nrfx_twi_tx(&app_twi, ICM_20600_ADDR, &reg, 1, false));
    APP_ERROR_CHECK(nrfx_twi_rx(&app_twi, ICM_20600_ADDR, buffer, length));
}

void ICM_20600_write_reg(uint8_t reg, uint8_t byte) {
    uint8_t data[2] = {reg, byte};
    APP_ERROR_CHECK(nrfx_twi_tx(&app_twi, ICM_20600_ADDR, data, 2, false));
}

void ICM_20600_int_clear() {
    static uint8_t buffer;
    ICM_20600_read_reg(0x3A, &buffer, 1);
}


void ICM_20600_gpiote_event_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
//    NRF_LOG_INFO("motion event %d", app_manuf_data.icm_20600_motion);
    if (nrf_gpio_pin_read(ICM_20600_INT1)) {
        if (app_manuf_data.icm_20600_motion == 0) {
            app_manuf_data.icm_20600_motion = 1;
            app_sched_event_put(NULL, 0, app_advertising_update);
            app_sched_event_put(NULL, 0, app_advertise_event_handler);
            app_timer_start(ICM_20600_motion_timout, APP_TIMER_TICKS(APP_ADV_PERIOD - 500), NULL);
        }
    }
}

void ICM_20600_motion_timeout_handler(void *context) {
    ICM_20600_int_clear();
    app_manuf_data.icm_20600_motion = 0;
    app_sched_event_put(NULL, 0, app_advertising_update);
}

void ICM_20600_init() {
    static uint8_t buffer;
    ICM_20600_read_reg(0x6b, &buffer, 1);
    buffer &= ~(1 << 6) & ~(1 << 5) & ~(1 << 4);
    ICM_20600_write_reg(0x6b, buffer);
    ICM_20600_write_reg(0x6c, 0b00000111);
    ICM_20600_write_reg(0x1d, 0b00001001);
    ICM_20600_write_reg(0x38, 0b11100000);
    ICM_20600_write_reg(0x37, 0b00100000);
    ICM_20600_write_reg(0x20, 100u);
    ICM_20600_write_reg(0x21, 100u);
    ICM_20600_write_reg(0x22, 100u);
    ICM_20600_write_reg(0x69, 0b11000000);
    ICM_20600_write_reg(0x19, 255U);
    ICM_20600_write_reg(0x6b, buffer | (1 << 5));
    ICM_20600_read_reg(0x3A, &buffer, 1);
    APP_ERROR_CHECK(
            app_timer_create(&ICM_20600_motion_timout, APP_TIMER_MODE_SINGLE_SHOT, ICM_20600_motion_timeout_handler));
    nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
    APP_ERROR_CHECK(nrfx_gpiote_in_init(ICM_20600_INT1, &in_config, ICM_20600_gpiote_event_handler));
    nrfx_gpiote_in_event_enable(ICM_20600_INT1, true);
}

void AHT20_cycle_mode() {
    static uint8_t data[3] = {0xBE, 0x20 | 0x08, 0x0};
    APP_ERROR_CHECK(nrfx_twi_tx(&app_twi, AHT20_ADDRESS, data, 3, false));
}

void AHT20_normal_mode() {
    static uint8_t data[3] = {0xA8, 0, 0};
    APP_ERROR_CHECK(nrfx_twi_tx(&app_twi, AHT20_ADDRESS, data, 3, false));
    APP_DELAY_MS(350);
}

void AHT20_enable_cal_coeff() {
    static uint8_t data[3] = {0xbe, 0x08, 0};
    APP_ERROR_CHECK(nrfx_twi_tx(&app_twi, AHT20_ADDRESS, data, 3, false));
    APP_DELAY_MS(350);
}

uint8_t AHT20_status() {
    uint8_t data;
    APP_ERROR_CHECK(nrfx_twi_rx(&app_twi, AHT20_ADDRESS, &data, 1));
    return data;
}


void AHT20_measurement_timeout_handler(void *context) {
    uint8_t buffer[6];
    uint32_t measurement;
    if (IS_SET(AHT20_status(), 7)) {
        app_timer_start(aht20_measurement_timeout, APP_TIMER_TICKS(100), NULL);
    } else {
            APP_ERROR_CHECK(nrfx_twi_rx(&app_twi, AHT20_ADDRESS, buffer, 6));
        if (buffer[0] != 0xff) {
            measurement = buffer[1];
            measurement <<= 8;
            measurement |= buffer[2];
            measurement <<= 4;
            measurement |= buffer[3] >> 4;
            app_manuf_data.aht20_humidity = (((float)measurement * 100) / 0x100000) * 100;
            measurement = buffer[3] & 0x0F;
            measurement  <<= 8;
            measurement |= buffer[4];
            measurement <<= 8;
            measurement |= buffer[5];
            app_manuf_data.aht20_temperature = (((float)measurement * 200 / 0x100000) - 50) * 1000;
            app_sched_event_put(NULL, 0, app_advertising_update);

        }
    }



//    app_manuf_data.aht20_temperature
}


void AHT20_start_measurement() {
    uint8_t data[3] = {0xAC, 0x33, 0x00};
    APP_ERROR_CHECK(nrfx_twi_tx(&app_twi, AHT20_ADDRESS, data, 3, false));
    app_timer_start(aht20_measurement_timeout, APP_TIMER_TICKS(100), NULL);
}


void AHT20_config() {
    AHT20_normal_mode();
    AHT20_enable_cal_coeff();
}


void AHT20_init() {
    nrfx_coredep_delay_us(40000); // power on delay
    AHT20_config();
    app_timer_create(&aht20_measurement_timeout, APP_TIMER_MODE_SINGLE_SHOT, AHT20_measurement_timeout_handler);
}

void AHT20_soft_reset() {
    static uint8_t data = 0xBA;
    APP_ERROR_CHECK(nrfx_twi_tx(&app_twi, AHT20_ADDRESS, &data, 1, false));
    APP_DELAY_MS(20);
    AHT20_config();
}



void nrf52811_temp_data_handler(int32_t measurement) {
    app_manuf_data.nrf52811_temperature = nrfx_temp_calculate(measurement) & 0xffff;
//    NRF_LOG_INFO("nrf52811_temperature = %lu", app_manuf_data.nrf52811_temperature);
    app_sched_event_put(NULL, 0, app_advertising_update);
}

static void nrf_temp_sensor_init(void) {
    static nrfx_temp_config_t temp_config = NRFX_TEMP_DEFAULT_CONFIG;
    nrfx_temp_init(&temp_config, nrf52811_temp_data_handler);
}

void app_sensor_timer_timeout_handler(void *context) {
    uint8_t buffer[2];
//    nrfx_temp_measure();
    SPL06_007_start();
    AHT20_start_measurement();
    ICM_20600_read_reg(0x41, buffer, 2);
    app_manuf_data.icm_20600_temperature = ((((uint16_t)buffer[0] << 8) | buffer[1]) / 326.8 + 25) * 100;
    app_sched_event_put(NULL, 0, app_advertising_update);
}


void app_sensors_init() {
    nrfx_twi_config_t twi_config = {
            .scl                = SCL_PIN,
            .sda                = SDA_PIN,
            .frequency          = NRFX_TWIM_DEFAULT_CONFIG_FREQUENCY,
            .interrupt_priority = APP_IRQ_PRIORITY_LOWEST,
            .hold_bus_uninit    = false
    };
    APP_ERROR_CHECK(nrfx_twi_init(&app_twi, &twi_config, NULL, NULL));
    nrfx_twi_enable(&app_twi);
//    nrf_temp_sensor_init();
    SPL06_007_init();
    ICM_20600_init();
    AHT20_init();
    APP_ERROR_CHECK(app_timer_create(&app_sensor_timer, APP_TIMER_MODE_REPEATED, app_sensor_timer_timeout_handler));
    APP_ERROR_CHECK(app_timer_start(app_sensor_timer, APP_TIMER_TICKS(APP_ADV_PERIOD - 500), NULL));
}
