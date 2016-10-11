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

#define NRF_LOG_MODULE_NAME "ILI9341"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_drv_spi.h"
#include "nrf_delay.h"
#include "tm_stm32f4_fonts.h"
#include "ili9341.h"
#include "boards.h"

#define LCD_C 0
#define LCD_D 1

#define ILI9341_CASET 0x2A
#define ILI9341_RASET 0x2B
#define ILI9341_MADCTL 0x36
#define ILI9341_RAMWR 0x2C
#define ILI9341_RAMRD 0x2E
#define ILI9341_COLMOD 0x3A

static ili9341_config_t ili9341_config = ILI9341_DEFAULT_CONFIG;
static const nrf_drv_spi_t *ili9341_spi_instance; /**< Declaring an instance of nrf_drv_spi for SPI0. */

struct ILI9341_cmdBuf {
  uint8_t command;   // ILI9341 command byte
  uint8_t delay;     // ms delay after
  uint8_t len;       // length of parameter data
  uint8_t data[16];  // parameter data
};

/* Pin functions */
uint16_t ILI9341_x;
uint16_t ILI9341_y;

static const struct ILI9341_cmdBuf initseq[] = {
  // SWRESET Software reset 
  { 0x01, 120, 0, {}},
  // SLPOUT Leave sleep mode
  { 0x11,  10, 0, {} },
  // FRMCTR1, FRMCTR2 Frame Rate configuration -- Normal mode, idle
  // frame 
  { 0xB1, 0, 2, { 0x00, 0x1B }}, // 70 Hz default
  { 0xB2, 0, 2, { 0x00, 0x1B }},
  // FRMCTR3 Frame Rate configureation -- partial mode
  { 0xB3, 0, 2, { 0x00, 0x1B }},
  // INVCTR Display inversion (no inversion)
  { 0xB4,  0, 1, { 0x00 }},
  // PWCTR1 Power control -4.6V, Auto mode
  { 0xC0,  0, 1, { 0x23}},
  // PWCTR2 Power control VGH25 2.4C, VGSEL -10, VGH = 3 * AVDD
  { 0xC1,  0, 1, { 0x03}},
  // PWCTR3 Power control, opamp current smal, boost frequency
  { 0xC2,  0, 2, { 0x0A, 0x00 }},
  // PWCTR4 Power control, BLK/2, opamp current small and medium low
  { 0xC3,  0, 2, { 0x8A, 0x2A}},
  // INVOFF Don't invert display
  { 0x20,  0, 0, {}},
  // Memory access directions. row address/col address, bottom to top refesh (10.1.27)
  { ILI9341_MADCTL,  0, 1, { 0xF4 }},
  // Color mode 16 bits / pixel (8.2.33. COLMOD)
  { ILI9341_COLMOD,   0, 1, {0x55}},
  // Column address set 0..239
  // { ILI9341_CASET,   0, 4, {0x00, 0x00, 0x00, 0xEF }},
  // Row address set 0..319
  // { ILI9341_RASET,   0, 4, {0x00, 0x00, 0x01, 0x3F }},
  // GMCTRP1 Gamma correction
  { 0xE0, 0, 15, {0x0F, 0x32, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1,
			    0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00 }},
  // GMCTRP2 Gamma Polarity corrction
  { 0xE1, 0, 15, {0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1,
			    0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F }},
  // DISPON Display on
  { 0x29, 5, 0, {}},
  // NORON Normal on
  { 0x13,  5, 0, {}},
  // End
  { 0, 0, 0, {}}
};

/** @brief write data/command via SPI
 */
static void LcdWrite(char dc, const uint8_t *data, uint16_t cnt) {
	uint16_t len,offset;

  nrf_gpio_pin_write(ili9341_config.dc_pin,dc);  // dc 1 = data, 0 = control
  nrf_gpio_pin_clear(ili9341_config.cs_pin);
  //NRF_LOG_INFO("LcdWrite (%u)\n\r",cnt);
  //NRF_LOG_HEXDUMP_INFO(data, cnt);
  if (cnt>0xFF) {
	  len=0xFF;
	  offset=0;
	  while (offset < cnt) {
		  nrf_drv_spi_transfer(ili9341_spi_instance,data+offset,len,NULL,0);
		  offset+=len;
		  len = cnt-offset;
		  if (len>0XFF) {
			  len=0xFF;
		  }
	  }
  } else {
	  nrf_drv_spi_transfer(ili9341_spi_instance,data,cnt,NULL,0);
  }
  nrf_gpio_pin_set(ili9341_config.cs_pin);
}

/** @brief write 16 Bit buffer
 */
static void LcdWrite16(char dc, uint16_t *data, uint16_t cnt) {
  uint16_t ob;
  int pos;

  for(pos=0;pos<cnt;pos++) {
	  ob = data[pos];
	  data[pos]=(ob<<8)|(ob>>8);
  }
  LcdWrite(dc, (uint8_t *)data, cnt*2);
}

static void ILI9341_writeCmd(uint8_t c) {
  LcdWrite(LCD_C, &c, 1);
}

/** @brief set pixel address window
 */
void ILI9341_setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  ILI9341_writeCmd(ILI9341_CASET);
  LcdWrite16(LCD_D, &x0, 1);
  LcdWrite16(LCD_D, &x1, 1);

  ILI9341_writeCmd(ILI9341_RASET);
  LcdWrite16(LCD_D, &y0, 1);
  LcdWrite16(LCD_D, &y1, 1);

  ILI9341_writeCmd(ILI9341_RAMWR);
}

/** @brief baclight on/off
 */
void ILI9341_backLight(uint8_t on) {
  nrf_gpio_pin_write(ili9341_config.bkl_pin,(on) ? 1 : 0); 
}

/** @brief send one char
 */
void ILI9341_Putc(uint16_t x, uint16_t y, char c, TM_FontDef_t *font, uint32_t foreground, uint32_t background) {
	uint32_t i, b, j;
	uint16_t LCDbuffer[512];
	//uint32_t lcd;

    /* Set coordinates */
	ILI9341_x = x;
	ILI9341_y = y;
	if ((ILI9341_x + font->FontWidth) > ILI9341_width) {
		//If at the end of a line of display, go to new line and set x to 0 position
		ILI9341_y += font->FontHeight;
		ILI9341_x = 0;
	}
	for (i = 0; i < font->FontHeight; i++) {
		b = font->data[(c - 32) * font->FontHeight + i];

		for (j = 0; j < font->FontWidth; j++) {
			if ((b << j) & 0x8000) {
				LCDbuffer[j+(i*font->FontWidth)]=((foreground << 8)|(foreground >> 8));
			} else if ((background & ILI9341_TRANSPARENT) == 0) {
				LCDbuffer[j+(i*font->FontWidth)]=((background << 8)|(background >> 8));
			}
		}
	}

	ILI9341_setAddrWindow((uint16_t) ILI9341_x, (uint16_t) ILI9341_y, (uint16_t) ILI9341_x + font->FontWidth-1, (uint16_t) ILI9341_y+font->FontHeight-1);
    LcdWrite16(LCD_D, LCDbuffer, (font->FontWidth*font->FontHeight));

	ILI9341_x += font->FontWidth;
}

/** @brief send string
 */
void ILI9341_Puts(uint16_t x, uint16_t y, char *str, TM_FontDef_t *font, uint32_t foreground, uint32_t background) {
	uint16_t startX = x;

	/* Set X and Y coordinates */
	ILI9341_x = x;
	ILI9341_y = y;

	while (*str) {
		//New line
		if (*str == '\n') {
			ILI9341_y += font->FontHeight + 1;
			//if after \n is also \r, than go to the left of the screen
			if (*(str + 1) == '\r') {
				ILI9341_x = 0;
				str++;
			} else {
				ILI9341_x = startX;
			}
			str++;
			continue;
		} else if (*str == '\r') {
			str++;
			continue;
		}

		ILI9341_Putc(ILI9341_x, ILI9341_y, *str++, font, foreground, background);
	}
}

/** @brief send colored pixel
 */
void ILI9341_pushColor(uint16_t *color, int cnt) {
  LcdWrite16(LCD_D, color, cnt);
}

/** @brief clear screen lazy version
 */
void ILI9341_fillscreen(uint16_t color) {
	uint16_t x,y;
	ILI9341_setAddrWindow(0,0,319,239);
	for (x=0;x<320;x++) {
		for (y=0;y<240;y++) {
			ILI9341_pushColor(&color,1);
		}
	}

}

/** @brief init display
 */
void ILI9341_init(nrf_drv_spi_t const * const spi_instance,ili9341_config_t const * const pin_config) {
  const struct ILI9341_cmdBuf *cmd;

  ili9341_config.cs_pin = pin_config->cs_pin;
  ili9341_config.dc_pin = pin_config->dc_pin;
  ili9341_config.rst_pin = pin_config->rst_pin;
  ili9341_config.bkl_pin = pin_config->bkl_pin;

  ili9341_spi_instance = spi_instance;

  NRF_LOG_INFO("ili9341 config %d %d %d %d\n\r",ili9341_config.cs_pin,ili9341_config.dc_pin,ili9341_config.rst_pin,ili9341_config.bkl_pin);
  NRF_LOG_FLUSH();
// set cs, reset device

  nrf_gpio_pin_set(ili9341_config.cs_pin);
  nrf_gpio_pin_set(ili9341_config.rst_pin);
  nrf_delay_ms(10);
  nrf_gpio_pin_clear(ili9341_config.cs_pin);
  nrf_gpio_pin_clear(ili9341_config.rst_pin);
  nrf_delay_ms(10);
  nrf_gpio_pin_set(ili9341_config.rst_pin);
  nrf_delay_ms(10);

  // Send initialization commands to ILI9341

  for (cmd = initseq; cmd->command; cmd++) {
      LcdWrite(LCD_C, &(cmd->command), 1);
      if (cmd->len) {
        LcdWrite(LCD_D, cmd->data, cmd->len);
      }
      if (cmd->delay) {
        nrf_delay_ms(cmd->delay);
      }
    }
}

