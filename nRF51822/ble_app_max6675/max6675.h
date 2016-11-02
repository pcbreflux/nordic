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

#ifndef MAX6675_H
#define MAX6675_H
/** 
 * @brief UART header file.
 *
 * This file contains the defining code for a application using UART.
 * 
 */

#define MAX6675_DEFAULT_CONFIG { \
    .miso_pin     = SPIM0_MISO_PIN, \
    .sck_pin      = SPIM0_SCK_PIN, \
    .cs_pin       = SPIM0_SS_PIN, \
	.spi_instance = NULL \
}

/**@brief MAX6675 configuration structure.*/
typedef struct {
    uint32_t               miso_pin;           /* Pin number for MISO input. */
    uint32_t               sck_pin;            /* Pin number for CLK output. */
    uint32_t               cs_pin;             /* Pin number for client select/Load output. */
    nrf_drv_spi_t         const *spi_instance;  /* ptr to SPI instance */
} max6675_config_t;

/**@brief Function for initialize max6675.
 *
 * @param[in]  max6675_config  Pointer to MAX6675 configuration structure.
 */
void max6675_init(max6675_config_t const * const max6675_config);

/**@brief Function for prepare reading max6675.
 *
 * @param[in]  max6675_config  Pointer to MAX6675 configuration structure.
 */
void max6675_prepare(max6675_config_t const * const max6675_config);

/**@brief Function for reading max6675 value in 0.25 times degrees celsius.
 *
 * @param[in]  max6675_config  Pointer to MAX6675 configuration structure.
 *
 * @return     value in 0.25 times degrees celsius.
 */
uint16_t max6675_readcelsius(max6675_config_t const * const max6675_config);


/**@brief Function for prepare reading max6675.
 *
 * @param[in]  max6675_config  Pointer to MAX6675 configuration structure.
 */
void max6675_unprepare(max6675_config_t const * const max6675_config);

#endif // MAX6675_H
