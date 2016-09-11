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

/** @file tt229_gpiote.c (changed by pcbreflux)
 * @brief Blinky Example Application main file.
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
#include "max7221.h"

#define MAX7221_DEVCNT 4

// see nrf_drv_config.h for defining TIMER1
// Softdevice S110, S120, S130 blocks TIMER0
const nrf_drv_spi_t my_spi_0 = NRF_DRV_SPI_INSTANCE(0); /**< Declaring an instance of nrf_drv_spi for SPI0. */

const uint32_t led_pin1 = 19;

/** @brief Function initialization and configuration of RTC driver instance.
 */
static void gpio_config(void) {
	// Configure LED-pin as outputs and clear.
    NRF_LOG_INFO("nrf_gpio_cfg_output %d\n\r",led_pin1);
	nrf_gpio_cfg_output(led_pin1);
	nrf_gpio_pin_clear(led_pin1);
}

/** @brief Function initialization and configuration of RTC driver instance.
 */
static void spi_config(void) {
    ret_code_t err_code;

    max7221_config_t max7221_config = MAX7221_DEFAULT_CONFIG;

    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;

    NRF_LOG_INFO("spi config %d %d %d\n\r",max7221_config.mosi_pin,max7221_config.sck_pin,max7221_config.cs_pin);
    spi_config.sck_pin = max7221_config.sck_pin;
    spi_config.mosi_pin = max7221_config.mosi_pin;
    spi_config.ss_pin = max7221_config.cs_pin;
    //spi_config.frequency = NRF_DRV_SPI_FREQ_125K; // tCP = 100ns see MAX7219/MAX7221 Datasheet TIMING CHARACTERISTICS
    spi_config.frequency = NRF_DRV_SPI_FREQ_8M; // tCP = 100ns see MAX7219/MAX7221 Datasheet TIMING CHARACTERISTICS
    spi_config.mode = NRF_DRV_SPI_MODE_0; // SCK active high, sample on leading edge of clock.
    spi_config.bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;

    err_code = nrf_drv_spi_init(&my_spi_0,&spi_config,NULL);
    APP_ERROR_CHECK(err_code);

}

/** @brief Function output secret message :)
 */
void writeNRF51822(uint32_t delay_ms) {
	max7221_writechar(&my_spi_0,0,'n');
	nrf_delay_ms(delay_ms);
	max7221_writechar(&my_spi_0,1,'n');
	max7221_writechar(&my_spi_0,0,'R');
	nrf_delay_ms(delay_ms);
	max7221_writechar(&my_spi_0,2,'n');
	max7221_writechar(&my_spi_0,1,'R');
	max7221_writechar(&my_spi_0,0,'F');
	nrf_delay_ms(delay_ms);
	max7221_writechar(&my_spi_0,3,'n');
	max7221_writechar(&my_spi_0,2,'R');
	max7221_writechar(&my_spi_0,1,'F');
	max7221_writechar(&my_spi_0,0,'5');
	nrf_delay_ms(delay_ms);
	max7221_writechar(&my_spi_0,3,'R');
	max7221_writechar(&my_spi_0,2,'F');
	max7221_writechar(&my_spi_0,1,'5');
	max7221_writechar(&my_spi_0,0,'1');
	nrf_delay_ms(delay_ms);
	max7221_writechar(&my_spi_0,3,'F');
	max7221_writechar(&my_spi_0,2,'5');
	max7221_writechar(&my_spi_0,1,'1');
	max7221_writechar(&my_spi_0,0,'8');
	nrf_delay_ms(delay_ms);
	max7221_writechar(&my_spi_0,3,'5');
	max7221_writechar(&my_spi_0,2,'1');
	max7221_writechar(&my_spi_0,1,'8');
	max7221_writechar(&my_spi_0,0,'2');
	nrf_delay_ms(delay_ms);
	max7221_writechar(&my_spi_0,3,'1');
	max7221_writechar(&my_spi_0,2,'8');
	max7221_writechar(&my_spi_0,1,'2');
	max7221_writechar(&my_spi_0,0,'2');
	nrf_delay_ms(delay_ms);
	max7221_writechar(&my_spi_0,3,'8');
	max7221_writechar(&my_spi_0,2,'2');
	max7221_writechar(&my_spi_0,1,'2');
	max7221_writechar(&my_spi_0,0,' ');
	nrf_delay_ms(delay_ms);
	max7221_writechar(&my_spi_0,3,'2');
	max7221_writechar(&my_spi_0,2,'2');
	max7221_writechar(&my_spi_0,1,' ');
	max7221_writechar(&my_spi_0,0,' ');
	nrf_delay_ms(delay_ms);
	max7221_writechar(&my_spi_0,3,'2');
	max7221_writechar(&my_spi_0,2,' ');
	max7221_writechar(&my_spi_0,1,' ');
	max7221_writechar(&my_spi_0,0,' ');
	nrf_delay_ms(delay_ms);
	max7221_writechar(&my_spi_0,3,' ');
	max7221_writechar(&my_spi_0,2,' ');
	max7221_writechar(&my_spi_0,1,' ');
	max7221_writechar(&my_spi_0,0,' ');
	nrf_delay_ms(delay_ms);
}

void writeNumbers(uint32_t delay_ms) {
    uint8_t numpos=0;
    uint8_t devpos=0;
    for(devpos=0;devpos<MAX7221_DEVCNT;devpos++) {
        max7221_writenumber(&my_spi_0,devpos,0);
    }
    for(devpos=0;devpos<MAX7221_DEVCNT;devpos++) {
		for(numpos=0;numpos<10;numpos++) {
			max7221_writenumber(&my_spi_0,devpos,numpos);
			nrf_delay_ms(delay_ms);
		}
		max7221_writenumber(&my_spi_0,devpos,0);
    }
}

void writeChars(uint32_t delay_ms) {
    uint8_t charpos=0;
    uint8_t devpos=0;
    for(devpos=0;devpos<MAX7221_DEVCNT;devpos++) {
		for(charpos=1;charpos<255;charpos++) {
			max7221_writechar(&my_spi_0,devpos,charpos);
			NRF_LOG_INFO("spi writeChars %d=%c\n\r",devpos,charpos);
			NRF_LOG_FLUSH();
			nrf_delay_ms(delay_ms);
		}
    }
}

void writeDot(uint32_t delay_ms) {
    uint8_t bitpos=0;
    uint8_t value=0;
    uint8_t devpos=0;

    for(devpos=0;devpos<MAX7221_DEVCNT;devpos++) {
		max7221_clear(&my_spi_0,devpos);
		for(value=0;value<8;value++) {
			for(bitpos=0;bitpos<8;bitpos++) {
				max7221_set(&my_spi_0,devpos,bitpos,1<<value);
				nrf_delay_ms(delay_ms);
				max7221_set(&my_spi_0,devpos,bitpos,0);
			}
			max7221_clear(&my_spi_0,devpos);
		}
		max7221_clear(&my_spi_0,devpos);
		for(bitpos=0;bitpos<8;bitpos++) {
			for(value=0;value<8;value++) {
				max7221_set(&my_spi_0,devpos,bitpos,1<<value);
				nrf_delay_ms(delay_ms);
			}
			max7221_set(&my_spi_0,devpos,bitpos,0);
			max7221_clear(&my_spi_0,devpos);
		}
    }
}

/**
 * @brief Function for application main entry.
 */
int main(void) {
    // uint32_t retVal;
    uint32_t pos=0;

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));

    NRF_LOG_INFO("start spi MAX7221\n\r");

    // setup
    gpio_config(); // Configure GPIO pins.
    spi_config();

    // loop
    while (true) {
        max7221_init(&my_spi_0,MAX7221_DEVCNT);

        //nrf_gpio_pin_toggle(led_pin1);
        //nrf_gpio_pin_toggle(MAX7221_CS_PIN);
        //writeDot(10);
        writeNRF51822(200);
		writeNumbers(100);
		//writeChars(100);
		pos++;
		NRF_LOG_INFO("tick (%u)\n\r",pos);
		NRF_LOG_FLUSH();
       
/*
            // Enter System ON sleep mode
            __WFE();
            // Make sure any pending events are cleared
            __SEV();
            __WFE();
*/
    }
}


/** @} */
