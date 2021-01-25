//
// Created by gwillen on 1/15/21.
//

#ifndef COZY_BEACON_MAKE_APP_ADVERTISING_H
#define COZY_BEACON_MAKE_APP_ADVERTISING_H

#include "app_config.h"
#include "ble_advdata.h"


#ifdef __cplusplus
extern "C" {
#endif

typedef struct __attribute__((packed))  {
    uint32_t uuid_lsb;
    uint32_t uuid_msb;
    uint8_t icm_20600_motion;
    uint16_t icm_20600_temperature;
    uint16_t aht20_temperature;
    uint16_t aht20_humidity;
    uint16_t spl06_007_pressure;
    uint16_t spl06_007_temperature;
    uint16_t nrf52811_temperature;
    uint8_t smd0805_20_voltage;
    uint8_t mic_voltage;
    uint8_t battery_voltage;
} app_manuf_data_t;

extern volatile app_manuf_data_t app_manuf_data;
void app_advertise_event_handler(void *event, uint16_t event_size);
void app_advertising_init(void);
void app_advertising_start(void);
void app_advertising_stop(void);
void app_advertising_update(void *event, uint16_t event_size);
#ifdef __cplusplus
}
#endif
#endif //COZY_BEACON_MAKE_APP_ADVERTISING_H
