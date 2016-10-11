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
 * @brief SPI Example Application main file.
 *
 */
#include <stdlib.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "sdk_config.h"
#define NRF_LOG_MODULE_NAME "PCBREFLUX SPI"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_drv_spi.h"
#include "app_uart.h"
#include "app_error.h"
#include "tm_stm32f4_fonts.h"
#include "ili9341.h"
#include "xprintf.h"

#define MAX7221_DEVCNT 5

// see nrf_drv_config.h for defining TIMER1
// Softdevice S110, S120, S130 blocks TIMER0
const nrf_drv_spi_t my_spi_0 = NRF_DRV_SPI_INSTANCE(0); /**< Declaring an instance of nrf_drv_spi for SPI0. */

/** @brief Function initialization and configuration of RTC driver instance.
 */
static void gpio_config(ili9341_config_t const * const ili9341_config) {
	// Configure ILI9341-pin as outputs and clear.
	nrf_gpio_cfg_output(ili9341_config->cs_pin);
	nrf_gpio_pin_set(ili9341_config->cs_pin);
	nrf_gpio_cfg_output(ili9341_config->dc_pin);
	nrf_gpio_pin_clear(ili9341_config->dc_pin);
	nrf_gpio_cfg_output(ili9341_config->rst_pin);
	nrf_gpio_pin_clear(ili9341_config->rst_pin);
	nrf_gpio_cfg_output(ili9341_config->bkl_pin);
	nrf_gpio_pin_set(ili9341_config->bkl_pin);


}

/** @brief Function initialization and configuration of RTC driver instance.
 */
static void spi_config(ili9341_config_t const * const ili9341_config) {
    ret_code_t err_code;

    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;

    NRF_LOG_INFO("spi config %d %d %d\n\r",ili9341_config->mosi_pin,ili9341_config->sck_pin,ili9341_config->cs_pin);
    spi_config.sck_pin = ili9341_config->sck_pin;
    spi_config.mosi_pin = ili9341_config->mosi_pin;
    spi_config.ss_pin = 0xFF;  // external usage
    //spi_config.frequency = NRF_DRV_SPI_FREQ_125K; // tCP = 100ns see MAX7219/MAX7221 Datasheet TIMING CHARACTERISTICS
    spi_config.frequency = NRF_DRV_SPI_FREQ_8M; // tCP = 100ns see MAX7219/MAX7221 Datasheet TIMING CHARACTERISTICS
    spi_config.mode = NRF_DRV_SPI_MODE_0; // SCK active high, sample on leading edge of clock.
    spi_config.bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;

    err_code = nrf_drv_spi_init(&my_spi_0,&spi_config,NULL);
    APP_ERROR_CHECK(err_code);

}


uint8_t TxBuffer[513];

/**
 * @brief Function for application main entry.
 */
int main(void) {
    // uint32_t retVal;
    uint32_t pos=0;
    int i,iLine;

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));

    NRF_LOG_INFO("main spi_config\n\r");
    ili9341_config_t ili9341_config = ILI9341_DEFAULT_CONFIG;
	ili9341_config.dc_pin = 19;
    ili9341_config.rst_pin = 20;
    ili9341_config.bkl_pin = 21;

    // setup
    gpio_config(&ili9341_config); // Configure GPIO pins.
    spi_config(&ili9341_config);

    NRF_LOG_INFO("main ILI9341_init\n\r");
	NRF_LOG_FLUSH();
    ILI9341_init(&my_spi_0,&ili9341_config);
    ILI9341_backLight(1);

	// ILI9341_fillscreen(0); // yes this sucks

    NRF_LOG_INFO("main loop\n\r");
    // loop
    while (true) {
		for (i=0;i<3;i++) {
		    for (iLine=0;iLine<3;iLine++) {
		    		pos++;
		    		xsprintf((char *)TxBuffer,"nRF51822 (16MHz) + ILI9341 via SPI 8MHz %d         ",pos);
		    		ILI9341_Puts(0,  iLine*10, (char *)TxBuffer, &TM_Font_7x10, 0xFFFF, pos);
			}
		    for (iLine=4;iLine<8;iLine++) {
		    		pos++;
		    		xsprintf((char *)TxBuffer,"nRF51822 + ILI9341 %d       ",pos);
		    		ILI9341_Puts(0,  iLine*18-40, (char *)TxBuffer, &TM_Font_11x18, 0xFFFF, pos);
			}
		    for (iLine=8;iLine<13;iLine++) {
		    		pos++;
		    		xsprintf((char *)TxBuffer,"nRF51822+TTF  %d    ",pos);
		    		ILI9341_Puts(0,  iLine*26-100, (char *)TxBuffer, &TM_Font_16x26, 0xFFFF, pos);
			}
		}
		NRF_LOG_INFO("tick (%u)\n\r",pos);

    }
}


/** @} */
