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


/** 
 * @brief UART header file.
 *
 * This file contains the source code for a application using UART.
 * 
 */
#include <stdio.h>
#include "app_uart.h"
#include "app_error.h"
#include "nrf.h"
//#include "nrf_drv_config.h"
#include "bsp.h"
#include "uart.h"

#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 1                           /**< UART RX buffer size. */

/**
 * @brief Function for uart_error_handle uart error handling.
 */
void uart_error_handle(app_uart_evt_t * p_event) {
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR) {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    } else if (p_event->evt_type == APP_UART_FIFO_ERROR) {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}


/**
 * @brief Function for uart_write write string to uart function.
 * @param out output string
 */
void uart_write(const char* out) {
    while(*out) {
        while(app_uart_put(*out++) != NRF_SUCCESS);
    }
}

/**
 * @brief Function for uart_printf print formated to uart function.
 * @param format_out output format
 * @param ... output variables
 */
void uart_printf(const char * format_out, ...) {
    static char outbuffer[256];
    
    va_list p_args;
    va_start(p_args, format_out);
    vsprintf(outbuffer, format_out, p_args);
    va_end(p_args);
    uart_write(outbuffer);
}

/**
 * @brief Function for uart_init initialise my uart function.
 */
void uart_init(void) {
   uint32_t err_code;

    const app_uart_comm_params_t comm_params = {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          HWFC?APP_UART_FLOW_CONTROL_ENABLED:APP_UART_FLOW_CONTROL_DISABLED,
          false,  // use_parity
          UART0_CONFIG_BAUDRATE
      };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOW,
                         err_code);

    APP_ERROR_CHECK(err_code);
}

