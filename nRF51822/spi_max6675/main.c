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
#include "nrf_drv_clock.h"
#include "nrf_drv_rtc.h"
#include "app_uart.h"
#include "app_error.h"
#include "max6675.h"

// see nrf_drv_config.h for defining RTC1
#define RTC1_PRESCALE           8

// Softdevice S110, S120, S130 blocks RTC0
const nrf_drv_rtc_t rtc1 = NRF_DRV_RTC_INSTANCE(1); /**< Declaring an instance of nrf_drv_rtc for RTC1. */

// defining SPI0
const nrf_drv_spi_t my_spi_0 = NRF_DRV_SPI_INSTANCE(0); /**< Declaring an instance of nrf_drv_spi for SPI0. */

const uint32_t led_pin1 = 19;

static max6675_config_t max6675_config = MAX6675_DEFAULT_CONFIG;
static uint32_t pos=0;

/** @brief Function initialization and configuration of RTC driver instance.
 */
static void gpio_config() {
	// Configure LED-pin as outputs and clear.
    NRF_LOG_DEBUG("nrf_gpio_cfg_output %d\n\r",led_pin1);
	nrf_gpio_cfg_output(led_pin1);
	nrf_gpio_pin_clear(led_pin1);
}

/** @brief Function initialization and configuration of RTC driver instance.
 */
static void spi_config(max6675_config_t *max6675_config) {
    ret_code_t err_code;

    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;

    NRF_LOG_DEBUG("spi config %d %d %d\n\r",max6675_config->miso_pin,max6675_config->sck_pin,max6675_config->cs_pin);
    spi_config.sck_pin = max6675_config->sck_pin;
    spi_config.mosi_pin = 0xFF; // not used
    spi_config.miso_pin = max6675_config->miso_pin;
    spi_config.ss_pin = 0xFF; // max6675 need Conversion Time 220ms, so we set it external
    //spi_config.frequency = NRF_DRV_SPI_FREQ_125K; // tCP = 100ns see MAX7219/MAX7221 Datasheet TIMING CHARACTERISTICS
    spi_config.frequency = NRF_DRV_SPI_FREQ_8M; // tCP = 100ns see MAX7219/MAX7221 Datasheet TIMING CHARACTERISTICS
    spi_config.mode = NRF_DRV_SPI_MODE_0; // SCK active high, sample on leading edge of clock.
    spi_config.bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;

    err_code = nrf_drv_spi_init(&my_spi_0,&spi_config,NULL);
    APP_ERROR_CHECK(err_code);

}

static void rtc1_handler(nrf_drv_rtc_int_type_t int_type) {
//    uint32_t err_code;

    NRF_LOG_DEBUG("rtc1_handler %d %u\n\r",int_type,pos);
    if (int_type == NRF_DRV_RTC_INT_COMPARE0) {   // Interrupt from COMPARE0 event.
    	uint16_t temp;
        nrf_gpio_pin_toggle(led_pin1);
        if (pos%2==0) {
        	max6675_prepare(&max6675_config);
        } else {
        	temp=max6675_readcelsius(&max6675_config);
            NRF_LOG_INFO("rtc1_handler temp \x1B[1;32m%u\x1B[0m,\x1B[1;31m%d\x1B[0m    \r",temp>>2,(temp&0x0003)*25);
        }
        nrf_drv_rtc_int_enable(&rtc1, RTC_CHANNEL_INT_MASK(0));
        nrf_drv_rtc_counter_clear(&rtc1);


    }
    pos++;
}

/** @brief Function starting the internal LFCLK XTAL oscillator.
 */
static void lfclk_config(void) {
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}

/** @brief Function initialization and configuration of RTC driver instance.
 */
static void rtc_config(void) {
    uint32_t err_code;

    NRF_LOG_DEBUG("rtc config %d\n\r",RTC1_PRESCALE);
    //Initialize RTC instance
    nrf_drv_rtc_config_t rtc_config = NRF_DRV_RTC_DEFAULT_CONFIG;
    rtc_config.prescaler = RTC1_PRESCALE;

    err_code = nrf_drv_rtc_init(&rtc1, &rtc_config, rtc1_handler);
    APP_ERROR_CHECK(err_code);

    //Disable tick event & interrupt
    nrf_drv_rtc_tick_enable(&rtc1,false);

   //Set compare channel 0 to trigger interrupt after 0.5s
    err_code = nrf_drv_rtc_cc_set(&rtc1,0,RTC_DEFAULT_CONFIG_FREQUENCY/RTC1_PRESCALE/4,true); // 500ms
    APP_ERROR_CHECK(err_code);

    //Power on RTC instance
    nrf_drv_rtc_enable(&rtc1);
}

/**
 * @brief Function for application main entry.
 */
int main(void) {

    max6675_config.spi_instance = &my_spi_0;

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));

    NRF_LOG_INFO("start spi MAX6675\n\r");

    // setup
    gpio_config(max6675_config.cs_pin); // Configure GPIO pins.
    spi_config(&max6675_config);
    max6675_init(&max6675_config);
    lfclk_config();
    rtc_config();

    // loop
    while (true) {
        // Enter System ON sleep mode
        __WFE();
        // Make sure any pending events are cleared
        __SEV();
        __WFE();
    }
}


/** @} */
