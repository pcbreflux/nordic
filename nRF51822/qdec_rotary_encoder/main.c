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

/** @file main.c (changed by pcbreflux)
 * @brief QDEC and Rotary Encoder Example Application main file.
 *
 */
#include <stdlib.h>
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_drv_qdec.h"
#include "uart.h"

static volatile uint32_t m_accdblread;
static volatile int32_t m_accread;
static volatile int32_t m_value=0;
static volatile int32_t m_last_value=0;


/** @brief Function initialization and configuration GPIO pins.
 */
static void gpio_config(void) {
	// Configure Pins as inputs with pull-up.
	nrf_gpio_cfg_input(QDEC_CONFIG_PIO_A,NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_input(QDEC_CONFIG_PIO_B,NRF_GPIO_PIN_PULLUP);
}

/** @brief Function handle qdec events.
 */
static void qdec_event_handler(nrf_drv_qdec_event_t event) {
    if (event.type == NRF_QDEC_EVENT_REPORTRDY) {
        m_accdblread = event.data.report.accdbl;
        m_accread = event.data.report.acc;
        if (m_accdblread==0) {
        	m_value += m_accread;
	}
        if (m_value<0) {
		m_value = 0;
	} else if (m_value>100) {
		m_value=100;
	}
        if (m_value != m_last_value) {
        	uart_printf("report dbl=%u acc=%d",m_accdblread,m_accread);
                if (m_accread>0) {
			uart_printf("\x1B[1;32m"); // GREEN
                } else {
			uart_printf("\x1B[1;31m"); // RED
		}
        	uart_printf(" val=%d\n\r",m_value);
		m_last_value = m_value;
		uart_printf("\x1B[0m"); // DEFAULT color
	}
    }
}

/** @brief Function initialization and configuration of QDEC driver instance.
 */
static void qdec_config(void) {
	uint32_t err_code;

        nrf_drv_qdec_config_t qdec_config = NRF_DRV_QDEC_DEFAULT_CONFIG;
	// Initialize hardware
	err_code = nrf_drv_qdec_init(&qdec_config, qdec_event_handler);
	APP_ERROR_CHECK(err_code);
    
	printf("QDEC testing started\n");
	uart_printf("nrf_drv_qdec_init\n\r");

	nrf_drv_qdec_enable(); // Event and corresponding interrupt are enabled.
	uart_printf("nrf_drv_qdec_enable \n\r");
}

/**
 * @brief Function for application main entry.
 */
int main(void) {
    uart_init();
    uart_printf("\n\rstart qdec\n\r");

    // setup
    gpio_config(); // Configure GPIO pins.
    qdec_config(); // Configure QDEC

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
