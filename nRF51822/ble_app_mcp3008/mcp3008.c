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
#include <stdint.h>
#include "app_error.h"
#include "nrf_drv_spi.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "boards.h"
#include "mcp3008.h"

void mcp3008_init(mcp3008_config_t const * const mcp3008_config) {
    NRF_LOG_DEBUG("mcp3008_init %d\n\r",mcp3008_config->cs_pin);
	nrf_gpio_cfg_output(mcp3008_config->cs_pin);
	nrf_gpio_pin_set(mcp3008_config->cs_pin);
}

void mcp3008_prepare(mcp3008_config_t const * const mcp3008_config) {
    NRF_LOG_DEBUG("mcp3008_prepare\n\r");
	nrf_gpio_pin_clear(mcp3008_config->cs_pin);
}

uint16_t mcp3008_readanalog(mcp3008_config_t const * const mcp3008_config,uint8_t adcnum) {
    ret_code_t err_code;
	uint8_t inbuffer[1];
	uint8_t outbuffer[2];
	uint16_t adcout;

    //NRF_LOG_INFO("mcp3008_readanalog\n\r");
	inbuffer[1]=0x08|adcnum; // single-ended
	inbuffer[1]=inbuffer[1]<<4;

    err_code = nrf_drv_spi_transfer(mcp3008_config->spi_instance,inbuffer,1,outbuffer,2);
    APP_ERROR_CHECK(err_code);
	nrf_gpio_pin_set(mcp3008_config->cs_pin);

    //NRF_LOG_INFO("mcp3008_readanalog out=%d %d\n\r",outbuffer[0],outbuffer[1]);
	adcout = ((uint16_t)outbuffer[0]<<8)|((uint16_t)outbuffer[1]);

    NRF_LOG_DEBUG("mcp3008_readanalog out=%d\n\r",adcout);

    return adcout;
}

void mcp3008_unprepare(mcp3008_config_t const * const mcp3008_config) {
    NRF_LOG_DEBUG("mcp3008_prepare\n\r");
	nrf_gpio_pin_set(mcp3008_config->cs_pin);
}



