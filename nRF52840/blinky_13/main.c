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
#include <stdint.h>
#include <stdbool.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"

const uint32_t led_pin[] = { 13,14,15,16 };
#define LEDPINS 4
/**
 * @brief Function for application main entry.
 */
int main(void) {

    uint8_t pos=0;
    // setup
    // Configure LED-pin as outputs and clear.
    for (pos=0;pos<LEDPINS;pos++) {
    	nrf_gpio_cfg_output(led_pin[pos]);
    	nrf_gpio_pin_clear(led_pin[pos]);
    }

    // loop
    // Toggle LED.
    pos=0;
    while (true) {
        nrf_gpio_pin_toggle(led_pin[pos]);
        nrf_delay_ms(1000);
        pos++;
		if (pos>=LEDPINS) {
			pos=0;
		}
    }
}


/** @} */
