#include <stdio.h>
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
#include "ucg_nrf51822.h"

extern nrf_drv_spi_t *nrf51822_spi_instance;


int16_t ucg_com_nrf51822_generic_HW_SPI(ucg_t *ucg, int16_t msg, uint16_t arg, uint8_t *data) {

  switch(msg) {
    case UCG_COM_MSG_POWER_UP:
      /* "data" is a pointer to ucg_com_info_t structure with the following information: */
      /*	((ucg_com_info_t *)data)->serial_clk_speed value in nanoseconds */
      /*	((ucg_com_info_t *)data)->parallel_clk_speed value in nanoseconds */

      /* setup pins */
    	nrf_gpio_cfg_output(ucg->pin_list[UCG_PIN_CD]);
        nrf_gpio_pin_write(ucg->pin_list[UCG_PIN_CD], 1);  // dc 1 = data, 0 = control
      if ( ucg->pin_list[UCG_PIN_CS] != UCG_PIN_VAL_NONE ) {
      	nrf_gpio_cfg_output(ucg->pin_list[UCG_PIN_CS]);
        nrf_gpio_pin_write(ucg->pin_list[UCG_PIN_CS], 1);  // cs off
      }
      if ( ucg->pin_list[UCG_PIN_RST] != UCG_PIN_VAL_NONE ) {
        	nrf_gpio_cfg_output(ucg->pin_list[UCG_PIN_RST]);
            nrf_gpio_pin_write(ucg->pin_list[UCG_PIN_RST], 1);  // no reset
      }

      break;
    case UCG_COM_MSG_POWER_DOWN:
      break;
    case UCG_COM_MSG_DELAY:
      nrf_delay_us(arg);
      break;
    case UCG_COM_MSG_CHANGE_RESET_LINE:
      if ( ucg->pin_list[UCG_PIN_RST] != UCG_PIN_VAL_NONE )
        nrf_gpio_pin_write(ucg->pin_list[UCG_PIN_RST], arg);
      break;
    case UCG_COM_MSG_CHANGE_CS_LINE:
      if ( ucg->pin_list[UCG_PIN_CS] != UCG_PIN_VAL_NONE )
        nrf_gpio_pin_write(ucg->pin_list[UCG_PIN_CS], arg);
      break;
    case UCG_COM_MSG_CHANGE_CD_LINE:
      nrf_gpio_pin_write(ucg->pin_list[UCG_PIN_CD], arg);
      break;
    case UCG_COM_MSG_SEND_BYTE:
      nrf_drv_spi_transfer(nrf51822_spi_instance,(uint8_t *)&arg,1,NULL,0);
      break;
    case UCG_COM_MSG_REPEAT_1_BYTE:
      while( arg > 0 ) {
        nrf_drv_spi_transfer(nrf51822_spi_instance,data,1,NULL,0);
	arg--;
      }
      break;
    case UCG_COM_MSG_REPEAT_2_BYTES:
      while( arg > 0 ) {
        nrf_drv_spi_transfer(nrf51822_spi_instance,data,2,NULL,0);
	arg--;
      }
      break;
    case UCG_COM_MSG_REPEAT_3_BYTES:
      while( arg > 0 ) {
        nrf_drv_spi_transfer(nrf51822_spi_instance,data,3,NULL,0);
	arg--;
      }
      break;
    case UCG_COM_MSG_SEND_STR:
      nrf_drv_spi_transfer(nrf51822_spi_instance,data,arg,NULL,0);
      break;
    case UCG_COM_MSG_SEND_CD_DATA_SEQUENCE:
      while(arg > 0) {
	if ( *data != 0 ) {
	  if ( *data == 1 ) {
            nrf_gpio_pin_write(ucg->pin_list[UCG_PIN_CD], 0);
	  } else {
            nrf_gpio_pin_write(ucg->pin_list[UCG_PIN_CD], 1);
	  }
	}
	data++;
        nrf_drv_spi_transfer(nrf51822_spi_instance,data,1,NULL,0);
	data++;
	arg--;
      }
      break;
  }
  return 1;
}

