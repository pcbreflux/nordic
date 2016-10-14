#ifndef _UCG_NRF51822_H
#define _UCG_NRF51822_H

//adjust this path:
#include "ucg.h"

//main com function. read on...
int16_t ucg_com_nrf51822_generic_HW_SPI(ucg_t *ucg, int16_t msg, uint16_t arg, uint8_t *data);

#endif
