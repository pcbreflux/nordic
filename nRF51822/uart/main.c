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

/** @file
 * @defgroup uart_example_main main.c (changed by pcbreflux)
 * @{
 * @ingroup uart_example
 * @brief UART Example Application main file.
 *
 * This file contains the source code for a sample application using UART.
 * 
 */
#include <stdio.h>
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "uart.h"

const uint32_t led_pin1 = 19;

/** @brief Function initialization and configuration of RTC driver instance.
 */
static void gpio_config(void) {
    // Configure LED-pin as outputs and clear.
    nrf_gpio_cfg_output(led_pin1);
    nrf_gpio_pin_clear(led_pin1);
}

/**
 * @brief Function for main application entry.
 */
int main(void) {

    gpio_config();
    uart_init();
	
    uart_printf("\n\rStart: \n\r");

    uint32_t pos=0xFFFFFF00;  // -> next to 0
    while (true) {
        nrf_gpio_pin_toggle(led_pin1);
        uart_printf("Counter: %u\n\r",pos);
	nrf_delay_ms(500);
	pos++;
    }
}


/** @} */
