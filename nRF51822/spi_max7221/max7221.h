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

#ifndef MAX7221_H
#define MAX7221_H
/** 
 * @brief UART header file.
 *
 * This file contains the defining code for a application using UART.
 * 
 */

#define MAX7221_CS_PIN 28                                /**< SPI Slave Select GPIO pin number. */

#define MAX7221_DEFAULT_CONFIG { \
    .mosi_pin     = SPIM0_MOSI_PIN, \
    .sck_pin      = SPIM0_SCK_PIN, \
    .cs_pin       = SPIM0_SS_PIN \
}

/**@brief MAX7221 configuration structure.*/
typedef struct {
    uint32_t               mosi_pin;           /**< Pin number for MOSI output. */
    uint32_t               sck_pin;            /**< Pin number for CLK output. */
    uint32_t               cs_pin;             /**< Pin number for client select/Load output. */
} max7221_config_t;

// see MAX7219/MAX7221 Datasheet Table 2. Register Address Map
enum  	max7221_op_t {
  MAX7221_CON_NOOP = 0,
  MAX7221_CON_DIGIT0 = 1,
  MAX7221_CON_DIGIT1 = 2,
  MAX7221_CON_DIGIT2 = 3,
  MAX7221_CON_DIGIT3 = 4,
  MAX7221_CON_DIGIT4 = 5,
  MAX7221_CON_DIGIT5 = 6,
  MAX7221_CON_DIGIT6 = 7,
  MAX7221_CON_DIGIT7 = 8,
  MAX7221_CON_DECODEMODE = 9,
  MAX7221_CON_INTENSITY = 10,
  MAX7221_CON_SCANLIMIT = 11,
  MAX7221_CON_SHUTDOWN = 12,
  MAX7221_CON_DISPLAYTEST = 15
};

/**
 * @brief Function for uart_error_handle uart error handling.
 */
void max7221_init(nrf_drv_spi_t const * const spi_instance,uint8_t devnum);

void max7221_set(nrf_drv_spi_t const * const spi_instance,uint8_t devnum,uint8_t pos,uint32_t value);

void max7221_clear(nrf_drv_spi_t const * const spi_instance,uint8_t devnum);

void max7221_writenumber(nrf_drv_spi_t const * const spi_instance,uint8_t devnum,uint8_t number);

void max7221_writechar(nrf_drv_spi_t const * const spi_instance,uint8_t devnum,uint8_t ascii);

void max7221_transbuffer(nrf_drv_spi_t const * const spi_instance,uint8_t devnum,uint8_t code,uint8_t value);

#endif // MAX7221_H
