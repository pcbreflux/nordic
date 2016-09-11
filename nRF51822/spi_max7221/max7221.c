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

/**
 * @brief Function for uart_error_handle uart error handling.
 */
void max7221_init(nrf_drv_spi_t const * const spi_instance) {

    //uint8_t *pstart = max7221_outbuffer;
    //uint8_t *pbuf = max7221_outbuffer;
    // uint8_t bitpos;
    //uint32_t size;
    //ret_code_t err_code;

    max7221_config_t max7221_config = MAX7221_DEFAULT_CONFIG;

    NRF_LOG_INFO("spi config %d %d %d\n\r",max7221_config.mosi_pin,max7221_config.sck_pin,max7221_config.cs_pin);

    max7221_transbuffer(spi_instance,MAX7221_CON_DISPLAYTEST,0);
    max7221_transbuffer(spi_instance,MAX7221_CON_SCANLIMIT,7);
    max7221_transbuffer(spi_instance,MAX7221_CON_DECODEMODE,0);
    max7221_transbuffer(spi_instance,MAX7221_CON_SHUTDOWN,0);
    max7221_transbuffer(spi_instance,MAX7221_CON_INTENSITY,8);
    max7221_transbuffer(spi_instance,MAX7221_CON_SHUTDOWN,1);
    max7221_clear(spi_instance);
}

/**
 * @brief Function for uart_error_handle uart error handling.
 */
void max7221_clear(nrf_drv_spi_t const * const spi_instance) {
    uint8_t bitpos;

    for(bitpos=0;bitpos<8;bitpos++) {
        max7221_transbuffer(spi_instance,MAX7221_CON_DIGIT0+bitpos,0);
    }
}
/**
 * @brief Function for uart_error_handle uart error handling.
 */
void max7221_set(nrf_drv_spi_t const * const spi_instance,uint8_t pos,uint32_t value) {

    uint8_t bitpos=pos;

    //for(bitpos=0;bitpos<8;bitpos++) {
        max7221_transbuffer(spi_instance,MAX7221_CON_DIGIT0+bitpos,value&0xFF);
    //}
}

/**
 * @brief Function for uart_error_handle uart error handling.
 */
void max7221_writenumber(nrf_drv_spi_t const * const spi_instance,uint8_t number) {
	if (number>9) {
		number='*';
	}
	max7221_writechar(spi_instance,number+'0');
}

/**
 * @brief Function for uart_error_handle uart error handling.
 */
void max7221_writechar(nrf_drv_spi_t const * const spi_instance,uint8_t ascii) {
    uint8_t bitpos=0;
    uint16_t fontpos=0;

    fontpos = ascii*5;
    max7221_clear(spi_instance);
    for(bitpos=0;bitpos<5;bitpos++) {
        max7221_transbuffer(spi_instance,MAX7221_CON_DIGIT0+bitpos+2,font[fontpos+bitpos]<<1);
    }
}


void max7221_transbuffer(nrf_drv_spi_t const * const spi_instance,uint8_t code,uint8_t value) {
    ret_code_t err_code;

    max7221_outbuffer[0]=code;
    max7221_outbuffer[1]=value;

    //nrf_gpio_pin_clear(MAX7221_CS_PIN); // real active low !
    err_code = nrf_drv_spi_transfer(spi_instance,max7221_outbuffer,2,NULL,0);
    //nrf_gpio_pin_set(MAX7221_CS_PIN); // real active low !

    APP_ERROR_CHECK(err_code);
}

uint32_t max7221_setbuffer(uint8_t *pbuf,uint8_t code,uint8_t value) {
	*pbuf = code;
        pbuf++;
	*pbuf = value;
        pbuf++;
        
	return (uint32_t)pbuf;
}


