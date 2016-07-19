/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file rtc_blinky.c (changed by pcbreflux)
 * @brief Blinky Example Application main file.
 *
 */
#include <stdlib.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_rtc.h"
#include "app_error.h"

// see nrf_drv_config.h for defining RTC1
#define RTC1_CC_VALUE           8   // 125ms*8*1=1s
// Softdevice S110, S120, S130 blocks RTC0
const nrf_drv_rtc_t rtc1 = NRF_DRV_RTC_INSTANCE(1); /**< Declaring an instance of nrf_drv_rtc for RTC1. */

const uint32_t led_pin1 = 19;

/** @brief Function initialization and configuration of RTC driver instance.
 */
static void gpio_config(void) {
    // Configure LED-pin as outputs and clear.
    nrf_gpio_cfg_output(led_pin1);
    nrf_gpio_pin_clear(led_pin1);
}

static void rtc1_handler(nrf_drv_rtc_int_type_t int_type) {
//    uint32_t err_code;

    if (int_type == NRF_DRV_RTC_INT_COMPARE0) {   // Interrupt from COMPARE0 event.
        nrf_gpio_pin_set(led_pin1);
        nrf_drv_rtc_int_enable(&rtc1, RTC_CHANNEL_INT_MASK(0));
    } else if (int_type == NRF_DRV_RTC_INT_COMPARE1) {   // Interrupt from COMPARE1 event.
        nrf_gpio_pin_clear(led_pin1);
        nrf_drv_rtc_int_enable(&rtc1, RTC_CHANNEL_INT_MASK(1));
    } else if (int_type == NRF_DRV_RTC_INT_COMPARE2) {   // Interrupt from COMPARE2 event.
        nrf_drv_rtc_counter_clear(&rtc1);
        nrf_drv_rtc_int_enable(&rtc1, RTC_CHANNEL_INT_MASK(2));
    } else if (int_type == NRF_DRV_RTC_INT_TICK) {       // Tick off
        nrf_gpio_pin_toggle(led_pin1);
    }
}

/** @brief Function starting the internal LFCLK XTAL oscillator.
 */
static void lfclk_config(void) {
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}

/** @brief Function initialization and configuration of RTC driver instance.
 */
static void rtc_config(void) {
    uint32_t err_code;

    //Initialize RTC instance
    err_code = nrf_drv_rtc_init(&rtc1, NULL, rtc1_handler);
    APP_ERROR_CHECK(err_code);

    //Enable tick event & interrupt
    nrf_drv_rtc_tick_enable(&rtc1,false);

    /*
            |----|                              |----|
            |    |                              |    |
	____|    |______________________..._____|    |____...
         1s   1s             8s              1s   1s
    */

    //Set compare channel 0 to trigger interrupt after RTC1_CC_VALUE*125ms
    err_code = nrf_drv_rtc_cc_set(&rtc1,0,RTC1_CC_VALUE,true);
    APP_ERROR_CHECK(err_code);

    //Set compare channel 1 to trigger interrupt after RTC1_CC_VALUE*125ms*2
    err_code = nrf_drv_rtc_cc_set(&rtc1,1,RTC1_CC_VALUE*2,true);
    APP_ERROR_CHECK(err_code);

    //Set compare channel 2 to trigger interrupt after RTC1_CC_VALUE*125ms*9
    err_code = nrf_drv_rtc_cc_set(&rtc1,2,RTC1_CC_VALUE*8,true);
    APP_ERROR_CHECK(err_code);

    //Power on RTC instance
    nrf_drv_rtc_enable(&rtc1);
}

/**
 * @brief Function for application main entry.
 */
int main(void) {

    // setup
    gpio_config();
    lfclk_config();
    rtc_config();

    // loop
    while (true) {
            // Enter System ON sleep mode
            __WFE();
            // Make sure any pending events are cleared
            __SEV();
            __WFE();
    }
}


/** @} */
