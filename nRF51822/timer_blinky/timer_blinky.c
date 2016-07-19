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

/** @file timer_blinky.c (changed by pcbreflux)
 * @brief Blinky Example Application main file.
 *
 */
#include <stdlib.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_timer.h"
#include "app_error.h"

// see nrf_drv_config.h for defining TIMER1
// Softdevice S110, S120, S130 blocks TIMER0
const nrf_drv_timer_t mytimer1 = NRF_DRV_TIMER_INSTANCE(1); /**< Declaring an instance of nrf_drv_timer for TIMER1. */

const uint32_t led_pin1 = 19;
static uint32_t tick_pos = 0;

/** @brief Function initialization and configuration of GPIO.
 */
static void gpio_config(void) {
    // Configure LED-pin as outputs and clear.
    nrf_gpio_cfg_output(led_pin1);
    nrf_gpio_pin_clear(led_pin1);
}

/** @brief Function for timer events.
 */
static void timer1_handler(nrf_timer_event_t event_type, void* p_context) {
//    uint32_t err_code;

    if (event_type == NRF_TIMER_EVENT_COMPARE0) {   // Interrupt from COMPARE0 event.
        tick_pos++;
        if (tick_pos==1) {
        	nrf_gpio_pin_set(led_pin1);
        } else if (tick_pos==2) {
        	nrf_gpio_pin_clear(led_pin1);
        } else if (tick_pos==11) {
        	tick_pos=0;
        }
    }
}

/** @brief Function starting the HFCLK oscillator.
 */
static void hfclk_config(void) {
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_hfclk_request(NULL);
}

/** @brief Function initialization and configuration of timer driver instance.
 */
static void timer_config(void) {
    uint32_t time_in_ms = 1000;   //Time(in miliseconds) between consecutive compare events.
                                  // see 
    uint32_t time2ticks;
    uint32_t err_code;

    err_code = nrf_drv_timer_init(&mytimer1, NULL, timer1_handler);
    APP_ERROR_CHECK(err_code);
    
    time2ticks = nrf_drv_timer_ms_to_ticks(&mytimer1, time_in_ms);
    
    /*
        |----|                         |----|
        |    |                         |    |
	|    |______________________...|    |____...
          1s             9s              1s
    */
    nrf_drv_timer_extended_compare(&mytimer1, NRF_TIMER_CC_CHANNEL0, time2ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
    
    nrf_drv_timer_enable(&mytimer1);
}

/**
 * @brief Function for application main entry.
 */
int main(void) {

    // setup
    gpio_config();
    hfclk_config();
    timer_config();

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
