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

/** @file blinky.c (changed by pcbreflux)
 * @brief Blinky Example Application main file.
 *
 */
#include <stdlib.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"

const uint32_t led_pin1 = 31;
const uint32_t led_pin2 = 30;

/**
 * @brief Function for application main entry.
 */
int main(void) {

    // setup
    // Configure LED-pin as outputs and clear.
    nrf_gpio_cfg_output(led_pin1);
    nrf_gpio_pin_clear(led_pin1);
    nrf_gpio_cfg_output(led_pin2);
    nrf_gpio_pin_set(led_pin2);

    // loop
    // Toggle LED.
    while (true) {
        nrf_gpio_pin_toggle(led_pin1);
        nrf_gpio_pin_toggle(led_pin2);
	nrf_delay_ms(1000);
    }
}


/** @} */
