//
// Created by gwillen on 1/15/21.
//

#include <ble_advertising.h>
#include "app_advertising.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "nrf_log.h"
#include "app_scheduler.h"
#include "app_error.h"
#include "nrf_soc.h"
#include "nrf_sdh.h"
#include "nrf_ficr.h"


volatile bool app_advertising;
//
void app_ble_evt_handler(ble_evt_t const *evt, void *context) {
    if (evt->header.evt_id == BLE_GAP_EVT_ADV_SET_TERMINATED) {
//            memset(&(&app_manuf_data)[sizeof(uint32_t) * 2], 0, sizeof(app_manuf_data) - (sizeof(uint32_t) * 2));
            app_advertising = false;
    }
}


static ble_gap_adv_params_t app_adv_params;                                  /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t              app_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET; /**< Advertising handle used to identify an advertising set. */
static uint8_t              app_enc_advdata[2][BLE_GAP_ADV_SET_DATA_SIZE_MAX];  /**< Buffer for storing an encoded advertising set. */
volatile app_manuf_data_t app_manuf_data;
//volatile app_manuf_data_t *app_enc_manuf_data;


ble_advdata_manuf_data_t app_advdata_manuf_data = {
        .company_identifier = APP_COMPANY_IDENTIFIER,
        .data = {
                .size = APP_MANUF_DATA_LENGTH,
                .p_data = (uint8_t *)&app_manuf_data
        }
};
//
static ble_advdata_t app_advdata = {
        .p_manuf_specific_data = &app_advdata_manuf_data
};

APP_TIMER_DEF(app_advertising_timer);

/**@brief Struct that contains pointers to the encoded advertising data. */
ble_gap_adv_data_t app_gap_adv_data =
        {
                .adv_data =
                        {
                                .p_data = app_enc_advdata[0],
                                .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
                        },
                .scan_rsp_data =
                        {
                                .p_data = NULL,
                                .len    = 0

                        }
        };



//
NRF_SDH_BLE_OBSERVER(app_advertising_observer,
                     BLE_ADV_BLE_OBSERVER_PRIO,
                     app_ble_evt_handler,&app_gap_adv_data);


void app_advertising_timer_timeout_handler(void *context) {
    app_advertising_start();
}


static void app_ble_enable(void)
{
    uint32_t ram_start = 0;
    APP_ERROR_CHECK(nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start));
    APP_ERROR_CHECK(nrf_sdh_ble_enable(&ram_start));
}


void app_advertise_event_handler(void *event, uint16_t event_size) {
    app_advertising_stop();
    app_advertising_start();
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
void app_advertising_init(void)
{
    app_ble_enable();
    app_manuf_data.uuid_lsb = NRF_FICR->DEVICEID[0];
    app_manuf_data.uuid_msb = NRF_FICR->DEVICEID[1];
//    // Build and set advertising data.
    memset(&app_advdata, 0, sizeof(app_advdata));
    app_advdata.name_type             = BLE_ADVDATA_NO_NAME;
    app_advdata.flags                 = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    app_advdata.p_manuf_specific_data = &app_advdata_manuf_data;
    // Initialize advertising parameters (used when starting advertising).
    memset(&app_adv_params, 0, sizeof(app_adv_params));

    app_adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
    app_adv_params.p_peer_addr     = NULL;    // Undirected advertisement.
    app_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    app_adv_params.interval        = APP_ADV_INTERVAL;
    app_adv_params.duration        = APP_ADV_DURATION;       // Never time out.
    APP_ERROR_CHECK(ble_advdata_encode(&app_advdata, app_gap_adv_data.adv_data.p_data, &app_gap_adv_data.adv_data.len));
    APP_ERROR_CHECK(sd_ble_gap_adv_set_configure(&app_adv_handle, NULL, &app_adv_params));
    APP_ERROR_CHECK(sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, app_adv_handle, 4));
    APP_ERROR_CHECK(app_timer_create(&app_advertising_timer, APP_TIMER_MODE_REPEATED, app_advertising_timer_timeout_handler));
}

void app_advertising_update(void *event, uint16_t event_size) {

    ble_gap_adv_data_t new_app_gap_adv_data;
    // Build and set advertising data.
    memset(&new_app_gap_adv_data, 0, sizeof(new_app_gap_adv_data));
    new_app_gap_adv_data.adv_data.p_data =
            (app_gap_adv_data.adv_data.p_data != app_enc_advdata[0]) ?
            app_enc_advdata[0] : app_enc_advdata[1];
    new_app_gap_adv_data.adv_data.len = BLE_GAP_ADV_SET_DATA_SIZE_MAX;
    APP_ERROR_CHECK(ble_advdata_encode(&app_advdata, new_app_gap_adv_data.adv_data.p_data, &new_app_gap_adv_data.adv_data.len));
    memcpy(&app_gap_adv_data, &new_app_gap_adv_data, sizeof(app_gap_adv_data));

    APP_ERROR_CHECK(sd_ble_gap_adv_set_configure(&app_adv_handle, &app_gap_adv_data, NULL));
}

void app_advertising_stop(void) {

    if (app_advertising) {
        APP_ERROR_CHECK(sd_ble_gap_adv_stop(app_adv_handle));
        app_advertising = false;
    }
    APP_ERROR_CHECK(app_timer_stop(app_advertising_timer));
//    NRF_LOG_INFO("app advertising stopped");
}


/**@brief Function for starting advertising.
 */
void app_advertising_start(void)
{
    if (!app_advertising) {
        APP_ERROR_CHECK(sd_ble_gap_adv_start(app_adv_handle, APP_BLE_CONN_CFG_TAG));
        app_advertising = true;
//        NRF_LOG_INFO("app advertising started");
    }
    if (!app_advertising_timer->active) {
        APP_ERROR_CHECK(app_timer_start(app_advertising_timer, APP_TIMER_TICKS(APP_ADV_PERIOD), NULL));
//        NRF_LOG_INFO("app advertising timer started");
    }
}


