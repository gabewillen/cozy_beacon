//
// Created by gwillen on 1/15/21.
//

#ifndef COZY_BEACON_MAKE_APP_CONFIG_H
#define COZY_BEACON_MAKE_APP_CONFIG_H


#define APP_DELAY_MS(ms) nrfx_coredep_delay_us(ms * 1000)
#define APP_SCHED_QUEUE_SIZE            10
#define APP_SCHED_MAX_EVENT_DATA_SIZE   APP_TIMER_SCHED_EVENT_DATA_SIZE
#define APP_DELAY_US(us) nrfx_coredep_delay_us(us)
#define APP_TIMER_CONFIG_USE_SCHEDULER 1


/*********************************** Sensor Offsets ******************************************/
// Motion & Temperature sensor
#define ICM_20600_MOTION_DATA_INDEX sizeof(uint8_t) + (sizeof(uint32_t) * 2) + 1 // 1
#define ICM_20600_TEMPERATURE_DATA_INDEX ICM20600_MOTION_DATA_INDEX + 1 // 2
#define ICM_20600_TEMPERATURE_DATA_SIZE sizeof(uint16_t)
#define ICM_20600_MOTION_TIMEOUT 10

// Temperature and Humidity sensor
#define AHT20_TEMPERATURE_DATA_INDEX ICM_20600_TEMPERATURE_DATA_INDEX + ICM_20600_TEMPERATURE_DATA_SIZE // 4
#define AHT20_DATA_SIZE sizeof(uint16_t)
#define AHT20_HUMIDITY_DATA_INDEX AHT20_TEMPERATURE_DATA_INDEX + AHT20_DATA_SIZE // 6


#define SPL06_007_DATA_SIZE sizeof(uint16_t)
#define SPL06_007_PRESSURE_DATA_INDEX AHT20_HUMIDITY_DATA_INDEX + AHT20_DATA_SIZE // 8
#define SPL06_007_TEMPERATURE_DATA_INDEX SPL06_007_PRESSURE_DATA_INDEX + SPL06_007_DATA_SIZE // 10
// NRF52811 internal temperature sensor
#define NRF52811_TEMPERATURE_DATA_INDEX SPL06_007_TEMPERATURE_DATA_INDEX + SPL06_007_DATA_SIZE // 12
#define NRF52811_TEMPERATURE_DATA_SIZE sizeof(uint16_t);
// Ambient light sensor
#define SMD0805_20_DATA_OFFSET NRF52811_TEMPERATURE_DATA_INDEX + NRF52811_TEMPERATURE_DATA_SIZE // 14
// Microphone
#define MIC_DATA_OFFSET SMD0805_20_DATA_OFFSET + 1 // 15
// Battery
#define BATTERY_DATA_OFFSET MIC_DATA_OFFSET + 1 // 16

/**********************************************************************************************************************/


#define NRFX_TEMP_ENABLED 1
#define NRFX_GPIOTE_ENABLED 1
#define NRFX_SAADC_CONFIG_LP_MODE 1

#define APP_SAADC_CHANNEL_COUNT 3
#define APP_SAADC_BUFFER_SIZE 3




#define APP_BLE_CONN_CFG_TAG            1                                  /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_ADV_INTERVAL    MSEC_TO_UNITS(100, UNIT_0_625_MS)  /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */
#define APP_ADV_DURATION        MSEC_TO_UNITS(1000, UNIT_10_MS)
#define APP_ADV_PERIOD 5000
#define APP_MANUF_DATA_LENGTH          0x18                              /**< Total length of information advertised by the Beacon. */
#define APP_ADV_DATA_LENGTH             0x15                               /**< Length of manufacturer specific data in the advertisement. */
#define APP_DEVICE_TYPE                 0x02                               /**< 0x02 refers to Beacon. */
#define APP_MEASURED_RSSI               0xC3                               /**< The Beacon's measured RSSI at 1 meter distance in dBm. */
#define APP_COMPANY_IDENTIFIER          0x0059                             /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
#define APP_BEACON_UUID                 0x13, 0x31, 0xDD, 0xEE, \
                                        0x13, 0x06, 0x29, 0x06, \
                                        0x82, 0x83, 0x28, 0x07            /**< UUID for Beacon. */


#endif //COZY_BEACON_MAKE_APP_CONFIG_H
