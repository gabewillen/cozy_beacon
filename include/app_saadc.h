//
// Created by gwillen on 1/15/21.
//

#ifndef COZY_BEACON_MAKE_APP_SAADC_H
#define COZY_BEACON_MAKE_APP_SAADC_H

#include "nrfx_saadc.h"



#ifdef __cplusplus
extern "C" {
#endif

#define APP_SAADC_MILLI_VOLTS(ADC_VALUE) \
    ((((ADC_VALUE) *600) / 4096) * 6)

void app_saadc_init();
void app_saadc_sample();

#ifdef __cplusplus
}
#endif


#endif //COZY_BEACON_MAKE_APP_SAADC_H
