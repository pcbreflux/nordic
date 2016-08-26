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


#ifndef UART_H
#define UART_H
/** 
 * @brief UART header file.
 *
 * This file contains the defining code for a application using UART.
 * 
 */

/**
 * @brief Function for uart_error_handle uart error handling.
 */
void uart_error_handle(app_uart_evt_t * p_event);

/**
 * @brief Function for uart_write write string to uart function.
 * @param out output string
 */
void uart_write(const char* out);

/**
 * @brief Function for uart_printf print formated to uart function.
 * @param format_out output format
 * @param ... output variables
 */
void uart_printf(const char * format_out, ...);

/**
 * @brief Function for uart_init initialise my uart function.
 */
void uart_init(void);

#endif // UART_H
