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

#include "app_error.h"
#include "nrf_drv_spi.h"
#include "max7221.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "boards.h"
#include "glcdfont.h"

#define MAX7221_OUT_BUF_SIZE 256                         /**< SPI out buffer size. */
static uint8_t max7221_outbuffer[MAX7221_OUT_BUF_SIZE];
static uint8_t max7221_device_cnt=1;

/**
 * @brief Function for uart_error_handle uart error handling.
 */
void max7221_init(nrf_drv_spi_t const * const spi_instance, uint8_t device_cnt) {
	uint8_t devpos;

	if (device_cnt<(MAX7221_OUT_BUF_SIZE/2)) {
		max7221_device_cnt = device_cnt;
	}

	for (devpos=0;devpos<max7221_device_cnt;devpos++) {
		max7221_transbuffer(spi_instance,devpos,MAX7221_CON_DISPLAYTEST,0);
		max7221_transbuffer(spi_instance,devpos,MAX7221_CON_SCANLIMIT,7);
		max7221_transbuffer(spi_instance,devpos,MAX7221_CON_DECODEMODE,0);
		max7221_transbuffer(spi_instance,devpos,MAX7221_CON_SHUTDOWN,0);
		max7221_transbuffer(spi_instance,devpos,MAX7221_CON_INTENSITY,8);
		max7221_transbuffer(spi_instance,devpos,MAX7221_CON_SHUTDOWN,1);
		max7221_clear(spi_instance,devpos);
	}
}

/**
 * @brief Function for uart_error_handle uart error handling.
 */
void max7221_clear(nrf_drv_spi_t const * const spi_instance,uint8_t devnum) {
    uint8_t bitpos;

    for(bitpos=0;bitpos<8;bitpos++) {
        max7221_transbuffer(spi_instance,devnum,MAX7221_CON_DIGIT0+bitpos,0);
    }
}
/**
 * @brief Function for uart_error_handle uart error handling.
 */
void max7221_set(nrf_drv_spi_t const * const spi_instance,uint8_t devnum,uint8_t pos,uint32_t value) {

    uint8_t bitpos=pos;

    max7221_transbuffer(spi_instance,devnum,MAX7221_CON_DIGIT0+bitpos,value&0xFF);
}

/**
 * @brief Function for uart_error_handle uart error handling.
 */
void max7221_writenumber(nrf_drv_spi_t const * const spi_instance,uint8_t devnum,uint8_t number) {
	if (number>9) {
		number='*';
	}
	max7221_writechar(spi_instance,devnum,number+'0');
}

/**
 * @brief Function for uart_error_handle uart error handling.
 */
void max7221_writechar(nrf_drv_spi_t const * const spi_instance,uint8_t devnum,uint8_t ascii) {
    uint8_t bitpos=0;
    uint16_t fontpos=0;

    fontpos = ascii*5;
    max7221_clear(spi_instance,devnum);
    for(bitpos=0;bitpos<5;bitpos++) {
        max7221_transbuffer(spi_instance,devnum,MAX7221_CON_DIGIT0+bitpos+2,font[fontpos+bitpos]<<1);
    }
}


void max7221_transbuffer(nrf_drv_spi_t const * const spi_instance,uint8_t devnum,uint8_t code,uint8_t value) {
    ret_code_t err_code;
	uint8_t devpos;
	uint8_t device;
	uint32_t bufsize;

	if (devnum<=max7221_device_cnt) {
		device = devnum;
	} else {
		device = 1;
	}
	bufsize=max7221_device_cnt*2;
	for (devpos=0;devpos<max7221_device_cnt;devpos++) {
		if (devpos==device) {
		    max7221_outbuffer[devpos*2]=code;
		    max7221_outbuffer[devpos*2+1]=value;
		} else {
		    max7221_outbuffer[devpos*2]=MAX7221_CON_NOOP;
		    max7221_outbuffer[devpos*2+1]=0;
		}

	}
    err_code = nrf_drv_spi_transfer(spi_instance,max7221_outbuffer,bufsize,NULL,0);

    APP_ERROR_CHECK(err_code);
}



