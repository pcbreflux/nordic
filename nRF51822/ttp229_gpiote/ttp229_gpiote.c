/* Copyright (c) 2016 pcbreflux. All Rights Reserved.
 *
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>. *
 */

/** @file tt229_gpiote.c (changed by pcbreflux)
 * @brief Blinky Example Application main file.
 *
 */
#include <stdlib.h>
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_drv_gpiote.h"
#include "uart.h"

const uint32_t led_pin1 = 19;
const uint32_t ttp229_sdo_pin = 24;
const uint32_t ttp229_scl_pin = 25;
static uint32_t keypress=0;

uint32_t ttp229_getbit() {
	uint32_t retVal;

	nrf_gpio_pin_clear(ttp229_scl_pin);  // CMOS output active-low when TP1=1
	nrf_delay_us(1);  // 1us -> 1000kHz -> F_SCL max 512KHz first half
	retVal=nrf_gpio_pin_read(ttp229_sdo_pin); 	
	nrf_gpio_pin_set(ttp229_scl_pin);
	nrf_delay_us(1);  // 1us -> 1000kHz -> F_SCL max 512KHz second half

	return retVal;
}

uint8_t ttp229_getKey8() {
	uint8_t retVal=0;
	uint8_t i;

	nrf_delay_us(10); // Tw 10us

	for (i=0; i<8; i++) {
		if (!ttp229_getbit()) {
			retVal |= (1 << i);
		}
	}
	nrf_delay_ms(2); // Tout 2ms

	return retVal;
}

uint16_t ttp229_getKey16() {
	uint16_t retVal=0;
	uint8_t i;

	nrf_delay_us(10); // Tw 10us

  	for (i=0; i<16; i++) {
		if (!ttp229_getbit()) {
			retVal |= (1 << i);
		}
	}
	nrf_delay_ms(2); // Tout 2ms

	return retVal;
}


/** @brief Function initialization and configuration of RTC driver instance.
 */
static void gpio_config(void) {
	// Configure LED-pin as outputs and clear.
	nrf_gpio_cfg_output(led_pin1);
	nrf_gpio_pin_clear(led_pin1);
}

static void gpiote_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
//    uint32_t err_code;
	uint16_t keys;

	if (pin == ttp229_sdo_pin) {
		if (action == NRF_GPIOTE_POLARITY_LOTOHI) {	// yes it's me
			nrf_drv_gpiote_in_event_disable(ttp229_sdo_pin); // do not disturbe

        		nrf_gpio_pin_toggle(led_pin1);

			keypress++;
			keys = ttp229_getKey16();
			uart_printf("keypress k=%u p=%u a=%u tt229=%u\n\r",keypress,pin,action,keys);
			nrf_drv_gpiote_in_event_enable(ttp229_sdo_pin,true); // Event and corresponding interrupt are enabled.

		}
	}
}

static void gpiote_config(void) {
	uint32_t err_code;

        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);

        nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

        err_code = nrf_drv_gpiote_out_init(ttp229_scl_pin, &out_config);
	nrf_gpio_pin_set(ttp229_scl_pin);  // active low
        APP_ERROR_CHECK(err_code);

	//nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);  // hi_accuracy off -> low power
	nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);  // hi_accuracy off -> low power
        in_config.pull = NRF_GPIO_PIN_PULLUP;
	err_code = nrf_drv_gpiote_in_init(ttp229_sdo_pin,&in_config,gpiote_handler);
	uart_printf("nrf_drv_gpiote_in_init %u\n\r",keypress);

        APP_ERROR_CHECK(err_code);

	// ppi_event_addr = nrf_drv_gpiote_in_event_addr_get(ttp229_sdo_pin);	// Configure PPI using ppi_event_address.
	nrf_drv_gpiote_in_event_enable(ttp229_sdo_pin,true); // Event and corresponding interrupt are enabled.
	uart_printf("nrf_drv_gpiote_in_event_enable %u\n\r",keypress);
}

/**
 * @brief Function for application main entry.
 */
int main(void) {
    // uint32_t retVal;
    uint32_t pos=0;

    uart_init();
    uart_printf("\n\rstart tt229 gpiote\n\r");

    // setup
    gpio_config(); // Configure GPIO pins.
    gpiote_config(); // Configure GPIOTE Events.

    // loop
    while (true) {
	// retVal=nrf_gpio_pin_read(ttp229_sdo_pin); 	
	nrf_delay_ms(10000);
        uart_printf("tick (%u)\n\r",pos++);
        
/*
            // Enter System ON sleep mode
            __WFE();
            // Make sure any pending events are cleared
            __SEV();
            __WFE();
*/
    }
}


/** @} */
