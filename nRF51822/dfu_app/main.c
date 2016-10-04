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
 * @brief dfu Example Application main file.
 *
 */
#include <stdlib.h>
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "app_error.h"
#include "app_error_weak.h"
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_rtc.h"
#include "bsp.h"

// see nrf_drv_config.h for defining RTC1
#define RTC1_CC_VALUE           8   // 125ms*8*1=1s
// Softdevice S110, S120, S130 blocks RTC0
const nrf_drv_rtc_t rtc1 = NRF_DRV_RTC_INSTANCE(1); /**< Declaring an instance of nrf_drv_rtc for RTC1. */

static uint32_t tick_pos = 0;

static void rtc1_handler(nrf_drv_rtc_int_type_t int_type) {
//    uint32_t err_code;

    if (int_type == NRF_DRV_RTC_INT_COMPARE0) {   // Interrupt from COMPARE0 event.
        NRF_LOG_INFO("test app main loop %d\r\n",tick_pos);
	tick_pos++;
        nrf_drv_rtc_counter_clear(&rtc1);
        nrf_drv_rtc_int_enable(&rtc1, RTC_CHANNEL_INT_MASK(0));
    }
}

/** @brief Function initialization and configuration of RTC driver instance.
 */
static void rtc_config(void) {
    uint32_t err_code;

    //Initialize RTC instance
    err_code = nrf_drv_rtc_init(&rtc1, NULL, rtc1_handler);
    APP_ERROR_CHECK(err_code);

    //Disable tick event & interrupt
    nrf_drv_rtc_tick_enable(&rtc1,false);

    //Set compare channel 0 to trigger interrupt after RTC1_CC_VALUE*125ms
    err_code = nrf_drv_rtc_cc_set(&rtc1,0,RTC1_CC_VALUE,true);
    APP_ERROR_CHECK(err_code);

    //Power on RTC instance
    nrf_drv_rtc_enable(&rtc1);
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    
    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);
    
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(0,0,&ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
    
    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for doing power management.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for application main entry.
 */
int main(void) {
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));

    NRF_LOG_INFO("test app start\r\n");

    // Enable BLE stack.
    ble_stack_init();
    rtc_config();

    while (true) {
        power_manage();
    }
}


/** @} */
