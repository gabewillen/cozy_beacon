//
// Created by gwillen on 1/15/21.
//

#include "app_timer.h"
#include "app_scheduler.h"
#include "app_saadc.h"
#include "nrf_gpio.h"
#include "nrf_saadc.h"
#include "app_advertising.h"
#include "app_timer.h"
#include "nrfx_ppi.h"
#include "nrfx_rtc.h"
#include "nrfx_log.h"
#include "bsp.h"

static nrf_saadc_value_t app_sadc_buffer[2][APP_SAADC_BUFFER_SIZE];
static uint16_t app_saadc_samples[APP_SAADC_CHANNEL_COUNT];
//static app_button_cfg_t app_button_cfg = {};
static bool app_saadc_initialized = false;
APP_TIMER_DEF(app_saadc_timer);

static void app_saadc_sample_event_handler(void *event, uint16_t event_size) {
    app_manuf_data.battery_voltage = battery_level_in_percent(app_saadc_samples[0]);
    uint8_t mic_voltage = app_saadc_samples[1] ;
    if (mic_voltage > app_manuf_data.mic_voltage) {
        app_manuf_data.mic_voltage = mic_voltage;
    }
    app_manuf_data.smd0805_20_voltage = app_saadc_samples[2] & 0xff;
    app_sched_event_put(NULL, 0, app_advertising_update);
//    NRF_LOG_INFO("mic_voltage= %lu, battery_voltage=%lu", app_saadc_samples[1], app_saadc_samples[0]);
}

static void app_saadc_event_handler(nrfx_saadc_evt_t const * p_event)
{
    static uint8_t i = 0;
    if (p_event->type == NRFX_SAADC_EVT_DONE)
    {
        APP_ERROR_CHECK(nrfx_saadc_buffer_convert(p_event->data.done.p_buffer, APP_SAADC_BUFFER_SIZE));
        for (i = 0; i < APP_SAADC_BUFFER_SIZE; i++)
        {
            app_saadc_samples[i] = APP_SAADC_MILLI_VOLTS(p_event->data.done.p_buffer[i]);
        }
//        nrf_gpio_pin_set(MIC_SWITCH);
        nrfx_saadc_uninit();
        app_saadc_initialized = false;
        app_sched_event_put(NULL, 0, app_saadc_sample_event_handler);
    }

}

/**@brief Function for configuring ADC to do battery level conversion.
 */
void app_nrfx_saadc_init()
{
    /*config adc*/
    nrfx_saadc_config_t saadc_config = NRFX_SAADC_DEFAULT_CONFIG;

    APP_ERROR_CHECK(nrfx_saadc_init(&saadc_config, app_saadc_event_handler));
    app_saadc_initialized = true;

    /*battery*/
    nrf_saadc_channel_config_t channel_1_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);
    APP_ERROR_CHECK(nrfx_saadc_channel_init(0, &channel_1_config));

    /*microphone*/
    nrf_saadc_channel_config_t channel_2_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);
//    channel_2_config.acq_time = NRF_SAADC_ACQTIME_40US;
    APP_ERROR_CHECK(nrfx_saadc_channel_init(1, &channel_2_config));

    /*smd0*/
    nrf_saadc_channel_config_t channel_3_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
    APP_ERROR_CHECK(nrfx_saadc_channel_init(2, &channel_3_config));

    //Set SAADC buffer 1. The SAADC will start to write to this buffer
    APP_ERROR_CHECK(nrfx_saadc_buffer_convert(app_sadc_buffer[0], 3));
    //Set SAADC buffer 2. The SAADC will write to this buffer when buffer 1 is full. This will give the applicaiton time to process data in buffer 1.
    APP_ERROR_CHECK(nrfx_saadc_buffer_convert(app_sadc_buffer[1], 3));

    nrf_gpio_pin_clear(MIC_SWITCH);
}

void app_saadc_timer_timeout_handler(void *context) {
    app_saadc_sample();
}

void app_saadc_init() {
//    nrf_gpio_cfg(MIC_SWITCH, NRF_GPIO_PIN_DIR_OUTPUT,NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE);
    nrf_gpio_cfg_output(MIC_SWITCH);
    nrf_gpio_pin_set(MIC_SWITCH);
    app_nrfx_saadc_init();
    app_saadc_sample();
    APP_ERROR_CHECK(app_timer_create(&app_saadc_timer, APP_TIMER_MODE_REPEATED, app_saadc_timer_timeout_handler));
    APP_ERROR_CHECK(app_timer_start(app_saadc_timer, APP_TIMER_TICKS(10000), NULL));
}

void app_saadc_sample() {
    if (!app_saadc_initialized) {
        app_nrfx_saadc_init();
    }
    nrfx_saadc_sample();
}
