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

#ifndef MCP3008_H
#define MCP3008_H
/** 
 * @brief UART header file.
 *
 * This file contains the defining code for a application using UART.
 * 
 */

#define MCP3008_DEFAULT_CONFIG { \
    .miso_pin     = SPIM0_MISO_PIN, \
    .sck_pin      = SPIM0_SCK_PIN, \
    .cs_pin       = SPIM0_SS_PIN, \
	.spi_instance = NULL \
}

/**@brief MCP3008 configuration structure.*/
typedef struct {
    uint32_t               miso_pin;           /* Pin number for MISO input. */
    uint32_t               sck_pin;            /* Pin number for CLK output. */
    uint32_t               cs_pin;             /* Pin number for client select/Load output. */
    nrf_drv_spi_t         const *spi_instance;  /* ptr to SPI instance */
} mcp3008_config_t;

/**@brief Function for initialize mcp3008.
 *
 * @param[in]  mcp3008_config  Pointer to MCP3008 configuration structure.
 */
void mcp3008_init(mcp3008_config_t const * const mcp3008_config);

/**@brief Function for prepare reading mcp3008.
 *
 * @param[in]  mcp3008_config  Pointer to MCP3008 configuration structure.
 */
void mcp3008_prepare(mcp3008_config_t const * const mcp3008_config);

/**@brief Function for reading mcp3008 value in 0.25 times degrees celsius.
 *
 * @param[in]  mcp3008_config  Pointer to MCP3008 configuration structure.
 * @param[in]  adcnum  MCP3008 chip has 8 adc's (0 thru 7)
 *
 * @return     value in 0.25 times degrees celsius.
 */
uint16_t mcp3008_readanalog(mcp3008_config_t const * const mcp3008_config,uint8_t adcnum);


/**@brief Function for prepare reading mcp3008.
 *
 * @param[in]  mcp3008_config  Pointer to MCP3008 configuration structure.
 */
void mcp3008_unprepare(mcp3008_config_t const * const mcp3008_config);

#endif // MCP3008_H
