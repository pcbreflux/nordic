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
#include "max6675.h"

void max6675_init(max6675_config_t const * const max6675_config) {
    NRF_LOG_DEBUG("max6675_init %d\n\r",max6675_config->cs_pin);
	nrf_gpio_cfg_output(max6675_config->cs_pin);
	nrf_gpio_pin_set(max6675_config->cs_pin);
}

void max6675_prepare(max6675_config_t const * const max6675_config) {
    NRF_LOG_DEBUG("max6675_prepare\n\r");
	nrf_gpio_pin_clear(max6675_config->cs_pin);
}

uint16_t max6675_readcelsius(max6675_config_t const * const max6675_config) {
    ret_code_t err_code;
	uint8_t outbuffer[2];
	uint16_t temp;

    //NRF_LOG_INFO("max6675_readcelsius\n\r");
    err_code = nrf_drv_spi_transfer(max6675_config->spi_instance,NULL,0,outbuffer,2);
    APP_ERROR_CHECK(err_code);
	nrf_gpio_pin_set(max6675_config->cs_pin);

    //NRF_LOG_INFO("max6675_readcelsius out=%d %d\n\r",outbuffer[0],outbuffer[1]);
    temp = ((uint16_t)outbuffer[0]<<8)|((uint16_t)outbuffer[1]);
    temp = temp>>3;

    NRF_LOG_DEBUG("max6675_readcelsius out=%d\n\r",temp);

    return temp;
}



