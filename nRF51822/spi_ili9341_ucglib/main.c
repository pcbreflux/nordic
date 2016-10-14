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
#define NRF_LOG_MODULE_NAME "PCBREFLUX UCGLIB"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_rng.h"
#include "app_uart.h"
#include "app_error.h"
#include "ucg_nrf51822.h"

#define ILI9341_DC 19
#define ILI9341_RST 20
#define ILI9341_BKL 21

#define RANDOM_BUFF_SIZE 3                                                           /**< Random numbers buffer size. */

// see sdk_config.h for defining SPI0
const nrf_drv_spi_t my_spi_0 = NRF_DRV_SPI_INSTANCE(0); /**< Declaring an instance of nrf_drv_spi for SPI0. */
const nrf_drv_spi_t *nrf51822_spi_instance = &my_spi_0;

uint16_t w = 0;
uint16_t v = 0;

/** @brief Function initialization and configuration of RTC driver instance.
 */
void gpio_config() {
	// Configure ILI9341-pin as outputs and clear.
	nrf_gpio_cfg_output(ILI9341_BKL);
	nrf_gpio_pin_set(ILI9341_BKL); // backlight LED on
}

/** @brief Function initialization and configuration of SPI driver instance.
 */
void spi_config() {
    ret_code_t err_code;

    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;

    spi_config.sck_pin = SPIM0_SCK_PIN;
    spi_config.mosi_pin = SPIM0_MOSI_PIN;
    spi_config.ss_pin = 0xFF;  // dummy, external usage
    //spi_config.frequency = NRF_DRV_SPI_FREQ_125K; // tCP = 100ns see MAX7219/MAX7221 Datasheet TIMING CHARACTERISTICS
    spi_config.frequency = NRF_DRV_SPI_FREQ_8M; // tCP = 100ns see MAX7219/MAX7221 Datasheet TIMING CHARACTERISTICS
    spi_config.mode = NRF_DRV_SPI_MODE_0; // SCK active high, sample on leading edge of clock.
    spi_config.bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;

    err_code = nrf_drv_spi_init(&my_spi_0,&spi_config,NULL);
    APP_ERROR_CHECK(err_code);

}

/** @brief Function initialization and configuration of RNG driver instance.
 */
void rng_config() {
    ret_code_t err_code;
    err_code = nrf_drv_rng_init(NULL);
    APP_ERROR_CHECK(err_code);
}

/** @brief copied from ucglib Arduino examples
 */
void upper_pin(ucg_t *ucg, ucg_int_t x, ucg_int_t y) {
  ucg_int_t w = 7;
  ucg_int_t h = 6;
  ucg_SetColor(ucg, 0, 212, 212, 212);
  ucg_SetColor(ucg, 1, 200, 200, 200);
  ucg_SetColor(ucg, 2, 200, 200, 200);
  ucg_SetColor(ucg, 3, 188, 188, 188);
  ucg_DrawGradientBox(ucg, x, y, w, h);

  //ucg_DrawVLine(ucg, x+w, y+1, len);
  ucg_SetColor(ucg, 0, 220, 220, 220);
  ucg_SetColor(ucg, 1, 232, 232, 232);
  ucg_DrawGradientLine(ucg, x+w, y, h, 1);
}

/** @brief copied from ucglib Arduino examples
 */
void lower_pin(ucg_t *ucg, ucg_int_t x, ucg_int_t y) {
  ucg_int_t w = 7;
  ucg_int_t h = 5;
  ucg_SetColor(ucg, 0, 212, 212, 212);
  ucg_SetColor(ucg, 1, 200, 200, 200);
  ucg_SetColor(ucg, 2, 200, 200, 200);
  ucg_SetColor(ucg, 3, 188, 188, 188);
  ucg_DrawGradientBox(ucg, x, y, w, h);

  //ucg_DrawVLine(ucg, x+w, y+1, len);
  ucg_SetColor(ucg, 0, 220, 220, 220);
  ucg_SetColor(ucg, 1, 232, 232, 232);
  ucg_DrawGradientLine(ucg, x+w, y, h, 1);
  ucg_SetColor(ucg, 0, 220, 220, 220);
  ucg_SetColor(ucg, 1, 232, 232, 232);
  ucg_DrawGradientLine(ucg, x, y+h, w, 0);
  ucg_SetColor(ucg, 0, 240, 240, 240);
  ucg_DrawPixel(ucg, x+w, y+h);
}

/** @brief copied from ucglib Arduino examples
 */
void ic_body(ucg_t *ucg, ucg_int_t x, ucg_int_t y) {
  ucg_int_t w = 4*14+4;
  ucg_int_t h = 31;
  ucg_SetColor(ucg, 0, 60, 60, 60);
  ucg_SetColor(ucg, 1, 40, 40, 40);
  ucg_SetColor(ucg, 2, 48, 48, 48);
  ucg_SetColor(ucg, 3, 30, 30, 30);
  ucg_DrawGradientBox(ucg, x, y, w, h);

  ucg_SetColor(ucg, 0, 255, 168, 0);
  //ucg_SetColor(ucg, 0, 225, 168, 30);
  ucg_DrawDisc(ucg, x+w-1, y+h/2-1, 7, UCG_DRAW_UPPER_LEFT|UCG_DRAW_LOWER_LEFT);

  ucg_SetColor(ucg, 0, 60, 30, 0);
  //ucg_DrawDisc(ucg, x+w-1, y+h/2+1, 7, UCG_DRAW_UPPER_LEFT|UCG_DRAW_LOWER_LEFT);

  ucg_SetColor(ucg, 0, 50, 50, 50);
  ucg_SetColor(ucg, 0, 25, 25, 25);
  ucg_DrawDisc(ucg, x+w-1, y+h/2+1, 7, UCG_DRAW_UPPER_LEFT|UCG_DRAW_LOWER_LEFT);


}


/** @brief copied and changed from ucglib Arduino examples
 */
void draw_ucg_logo(ucg_t *ucg) {
  ucg_int_t a,b;

  //ucg_Init(ucg, ucg_sdl_dev_cb, ucg_ext_none, (ucg_com_fnptr)0);
  ucg_SetFont(ucg, ucg_font_ncenB24_tr);

  //ucg_SetRotate270(ucg);
  //ucg_SetClipRange(ucg, 10,5,40,20);



  a = 2;
  b = 3;

  ucg_SetColor(ucg, 0, 135*a/b,206*a/b,250*a/b);
  ucg_SetColor(ucg, 1, 176*a/b,226*a/b,255*a/b);
  ucg_SetColor(ucg, 2, 25*a/b,25*a/b,112*a/b);
  ucg_SetColor(ucg, 3, 	85*a/b,26*a/b,139*a/b);
  ucg_DrawGradientBox(ucg, 0, 0, ucg_GetWidth(ucg)/4, ucg_GetHeight(ucg));

  ucg_SetColor(ucg, 1, 135*a/b,206*a/b,250*a/b);
  ucg_SetColor(ucg, 0, 176*a/b,226*a/b,255*a/b);
  ucg_SetColor(ucg, 3, 25*a/b,25*a/b,112*a/b);
  ucg_SetColor(ucg, 2, 	85*a/b,26*a/b,139*a/b);
  ucg_DrawGradientBox(ucg, ucg_GetWidth(ucg)/4, 0, ucg_GetWidth(ucg)/4, ucg_GetHeight(ucg));

  ucg_SetColor(ucg, 0, 135*a/b,206*a/b,250*a/b);
  ucg_SetColor(ucg, 1, 176*a/b,226*a/b,255*a/b);
  ucg_SetColor(ucg, 2, 25*a/b,25*a/b,112*a/b);
  ucg_SetColor(ucg, 3, 	85*a/b,26*a/b,139*a/b);
  ucg_DrawGradientBox(ucg, ucg_GetWidth(ucg)*2/4, 0, ucg_GetWidth(ucg)/4, ucg_GetHeight(ucg));

  ucg_SetColor(ucg, 1, 135*a/b,206*a/b,250*a/b);
  ucg_SetColor(ucg, 0, 176*a/b,226*a/b,255*a/b);
  ucg_SetColor(ucg, 3, 25*a/b,25*a/b,112*a/b);
  ucg_SetColor(ucg, 2, 	85*a/b,26*a/b,139*a/b);
  ucg_DrawGradientBox(ucg, ucg_GetWidth(ucg)*3/4, 0, ucg_GetWidth(ucg)/4, ucg_GetHeight(ucg));


  upper_pin(ucg, 7+0*14, 4);
  upper_pin(ucg, 7+1*14, 4);
  upper_pin(ucg, 7+2*14, 4);
  upper_pin(ucg, 7+3*14, 4);

  ic_body(ucg, 2, 10);

  lower_pin(ucg, 7+0*14, 41);
  lower_pin(ucg, 7+1*14, 41);
  lower_pin(ucg, 7+2*14, 41);
  lower_pin(ucg, 7+3*14, 41);

  ucg_SetColor(ucg, 0, 135*a/b, 206*a/b, 250*a/b);
  ucg_DrawString(ucg, 63+1, 33+1, 0, "glib");

  ucg_SetColor(ucg, 0, 255, 168, 0);
  ucg_DrawGlyph(ucg, 26, 38, 0, 'U');
  ucg_DrawString(ucg, 63, 33, 0, "glib");

  ucg_SetColor(ucg, 0, 135*a/b, 206*a/b, 250*a/b);
  ucg_SetColor(ucg, 1, 135*a/b, 206*a/b, 250*a/b);
  ucg_SetColor(ucg, 2, 135*a/b, 206*a/b, 250*a/b);
  ucg_SetColor(ucg, 3, 135*a/b, 206*a/b, 250*a/b);
  ucg_DrawGradientBox(ucg, 84+1, 42+1-6, 42, 4);

  ucg_SetColor(ucg, 0, 255, 180, 40);
  ucg_SetColor(ucg, 1, 235, 148, 0);
  //ucg_DrawGradientLine(ucg, 79, 42, 20, 0);
  ucg_SetColor(ucg, 2, 245, 158, 0);
  ucg_SetColor(ucg, 3, 220, 138, 0);
  ucg_DrawGradientBox(ucg, 84, 42-6, 42, 4);

  ucg_SetColor(ucg, 0, 255, 168, 0);
  //ucg_SetFont(ucg, ucg_font_5x8_tr);
  ucg_SetFont(ucg, ucg_font_7x13B_tr);
  //ucg_SetFont(ucg, ucg_font_courB08_tr);
  //ucg_SetFont(ucg, ucg_font_timR08_tr);
  ucg_DrawString(ucg, 2, 54+5, 0, "http://github.com");
  ucg_DrawString(ucg, 2, 61+10, 0, "/olikraus/ucglib");
  //ucg_DrawString(ucg, 1, 61, 0, "code.google.com/p/ucglib/");
}

uint8_t lcg_rnd() {
    uint8_t rnd_buff[1];

    nrf_drv_rng_rand(rnd_buff,1);

	return rnd_buff[0];
}

/** @brief copied and changed from ucglib Arduino examples
 */
void box(ucg_t *ucg) {
  ucg_int_t x, y, w, h;
  uint16_t m=0;

  ucg_SetColor(ucg,0, 0, 40, 80);
  ucg_SetColor(ucg,1, 60, 0, 40);
  ucg_SetColor(ucg,2, 128, 0, 140);
  ucg_SetColor(ucg,3, 0, 128, 140);
  ucg_DrawGradientBox(ucg,0, 0, ucg_GetWidth(ucg), ucg_GetHeight(ucg));

  ucg_SetColor(ucg,0,255, 255, 255);
  ucg_DrawString(ucg, 2, 18, 0, "ucglib nRF51822 Box");

  while(m<50) {
    ucg_SetColor(ucg,0,(lcg_rnd()&127)+127, (lcg_rnd()&127)+64, lcg_rnd() & 31);
    w = lcg_rnd() & 31;
    h = lcg_rnd() & 31;
    w += 10;
    h += 10;
    x = (lcg_rnd()*(ucg_GetWidth(ucg)-w))>>8;
    y = (lcg_rnd()*(ucg_GetHeight(ucg)-h-20))>>8;

    ucg_DrawBox(ucg,x, y+20, w, h);
    m++;
  }

}

/** @brief copied and changed from ucglib Arduino examples
 */
void gradient(ucg_t *ucg) {
	ucg_SetColor(ucg,0, 0, 255, 0);
	ucg_SetColor(ucg,1, 255, 0, 0);
	ucg_SetColor(ucg,2, 255, 0, 255);
	ucg_SetColor(ucg,3, 0, 255, 255);

	ucg_DrawGradientBox(ucg,0, 0, ucg_GetWidth(ucg), ucg_GetHeight(ucg));

	ucg_SetColor(ucg, 0, 255, 255, 255);
	ucg_DrawString(ucg, 2, 18, 0, "ucglib nRF51822 GradientBox");

	ucg_SetColor(ucg, 0, 0, 255, 0);
	ucg_DrawBox(ucg, 2, 25, 8, 8);

	ucg_SetColor(ucg, 0, 255, 0, 0);
	ucg_DrawBox(ucg, 2+10, 25, 8, 8);

	ucg_SetColor(ucg, 0, 255, 0, 255);
	ucg_DrawBox(ucg, 2, 25+10, 8, 8);

	ucg_SetColor(ucg, 0, 0, 255, 255);
	ucg_DrawBox(ucg, 2+10, 25+10, 8, 8);
}

/** @brief copied from ucglib Arduino examples
 */
void pixel_and_lines(ucg_t *ucg) {
  ucg_int_t mx;
  ucg_int_t x, xx;
  mx = ucg_GetWidth(ucg) / 2;
  //my = ucg.getHeight() / 2;

  ucg_SetColor(ucg,0, 0, 0, 150);
  ucg_SetColor(ucg,1, 0, 60, 40);
  ucg_SetColor(ucg,2, 60, 0, 40);
  ucg_SetColor(ucg,3, 120, 120, 200);
  ucg_DrawGradientBox(ucg, 0, 0, ucg_GetWidth(ucg), ucg_GetHeight(ucg));

  ucg_SetColor(ucg,0,255, 255, 255);
  ucg_DrawString(ucg, 2, 18, 0, "ucglib nRF51822 Pix&Line");

  ucg_DrawPixel(ucg,0, 0);
  ucg_DrawPixel(ucg,1, 0);

  ucg_DrawPixel(ucg,ucg_GetWidth(ucg)-1, ucg_GetHeight(ucg)-1);
  ucg_DrawPixel(ucg,ucg_GetWidth(ucg)-1-1, ucg_GetHeight(ucg)-1);


  for( x = 0; x  < mx; x++ ) {
    xx = (((uint16_t)x)*255)/mx;
    ucg_SetColor(ucg,0,255, 255-xx/2, 255-xx);
    ucg_DrawPixel(ucg,x, 24);
    ucg_DrawVLine(ucg,x+7, 26, 13);
  }

}

/** @brief copied from ucglib Arduino examples
 */
void triangle(ucg_t *ucg) {
  uint16_t m=0;

  ucg_SetColor(ucg,0, 0, 80, 20);
  ucg_SetColor(ucg,1, 60, 80, 20);
  ucg_SetColor(ucg,2, 60, 120, 0);
  ucg_SetColor(ucg,3, 0, 140, 30);
  ucg_DrawGradientBox(ucg, 0, 0, ucg_GetWidth(ucg), ucg_GetHeight(ucg));

  ucg_SetColor(ucg,0,255, 255, 255);
  ucg_DrawString(ucg, 2, 18, 0, "ucglib nRF51822 Triangle");

  while(m<100) {
	ucg_SetColor(ucg,0,(lcg_rnd()&127)+127, lcg_rnd() & 31, (lcg_rnd()&127)+64);

    ucg_DrawTriangle(ucg,
      (lcg_rnd()*(ucg_GetWidth(ucg)))>>8,
      ((lcg_rnd()*(ucg_GetHeight(ucg)-20))>>8)+20,
      (lcg_rnd()*(ucg_GetWidth(ucg)))>>8,
      ((lcg_rnd()*(ucg_GetHeight(ucg)-20))>>8)+20,
      (lcg_rnd()*(ucg_GetWidth(ucg)))>>8,
      ((lcg_rnd()*(ucg_GetHeight(ucg)-20))>>8)+20
    );
    m++;
  }
}

/** @brief copied and changed from ucglib Arduino examples
 */
void fonts(ucg_t *ucg) {

  ucg_int_t d = 5;
  ucg_SetColor(ucg,0, 0, 40, 80);
  ucg_SetColor(ucg,1, 150, 0, 200);
  ucg_SetColor(ucg,2, 60, 0, 40);
  ucg_SetColor(ucg,3, 0, 160, 160);
  ucg_DrawGradientBox(ucg, 0, 0, ucg_GetWidth(ucg), ucg_GetHeight(ucg));

  ucg_SetColor(ucg,0,255, 255, 255);
  ucg_DrawString(ucg, 2, 18, 0, "ucglib nRF51822 Fonts");

  ucg_SetFontMode(ucg,UCG_FONT_MODE_TRANSPARENT);

  ucg_SetColor(ucg,0,255, 200, 170);
  ucg_SetFont(ucg,ucg_font_helvB08_hr);
  ucg_DrawString(ucg, 2, 30+d, 0, "ABC abc 123");
  ucg_SetFont(ucg,ucg_font_helvB10_hr);
  ucg_DrawString(ucg, 2, 45+d, 0, "ABC abc 123");
  ucg_SetFont(ucg,ucg_font_helvB12_hr);
  ucg_DrawString(ucg, 2, 62+d, 0, "ABC abc 123");

  ucg_SetFontMode(ucg,UCG_FONT_MODE_SOLID);

  ucg_SetColor(ucg,0,255, 200, 170);
  ucg_SetColor(ucg,1, 0, 100, 120);		// background color in solid mode
  ucg_SetFont(ucg,ucg_font_helvB08_hr);
  ucg_DrawString(ucg, 2, 75+30+d, 0, "ABC abc 123");
  ucg_SetFont(ucg,ucg_font_helvB10_hr);
  ucg_DrawString(ucg, 2, 75+45+d, 0, "ABC abc 123");
  ucg_SetFont(ucg,ucg_font_helvB12_hr);
  ucg_DrawString(ucg, 2, 75+62+d, 0, "ABC abc 123");

  ucg_SetFontMode(ucg,UCG_FONT_MODE_TRANSPARENT);

  ucg_SetFont(ucg,ucg_font_ncenR14_hr);
}

/** @brief copied and changed from ucglib Arduino examples
 */
void clip(ucg_t *ucg) {
	ucg_SetColor(ucg,0, 0x00, 0xd1, 0x5e);		// dark green
	ucg_SetColor(ucg,1, 0xff, 0xf7, 0x61);		// yellow
	ucg_SetColor(ucg,2, 0xd1, 0xc7, 0x00);			// dark yellow
	ucg_SetColor(ucg,3, 0x61, 0xff, 0xa8);		// green
	ucg_DrawGradientBox(ucg, 0, 0, ucg_GetWidth(ucg), ucg_GetHeight(ucg));

	ucg_SetColor(ucg,0,255, 255, 255);
	ucg_DrawString(ucg, 2, 18, 0, "ucglib nRF51822 ClipRange");

	ucg_SetColor(ucg,0,0xd1, 0x00, 0x073);

	ucg_SetFont(ucg,ucg_font_helvB18_hr);

	ucg_DrawString(ucg, 25, 45, 0, "Ucg");
	ucg_DrawString(ucg, 25, 45, 1, "Ucg");
	ucg_DrawString(ucg, 25, 45, 2, "Ucg");
	ucg_DrawString(ucg, 25, 45, 3, "Ucg");

	ucg_SetMaxClipRange(ucg);
	ucg_SetColor(ucg,0,0xff, 0xff, 0xff);
	ucg_DrawFrame(ucg,20-1,30-1,15+2,20+2);
	ucg_SetClipRange(ucg,20, 30, 15, 20);
	ucg_SetColor(ucg,0,0xff, 0x61, 0x0b8);
	ucg_DrawString(ucg, 25, 45, 0, "Ucg");
	ucg_DrawString(ucg, 25, 45, 1, "Ucg");
	ucg_DrawString(ucg, 25, 45, 2, "Ucg");
	ucg_DrawString(ucg, 25, 45, 3, "Ucg");

	ucg_SetMaxClipRange(ucg);
	ucg_SetColor(ucg,0,0xff, 0xff, 0xff);
	ucg_DrawFrame(ucg,60-1,35-1,25+2,18+2);
	ucg_SetClipRange(ucg,60-1,35-1,25+2,18+2);
	ucg_SetColor(ucg,0,0xff, 0x61, 0x0b8);
	ucg_DrawString(ucg, 25, 45, 0, "Ucg");
	ucg_DrawString(ucg, 25, 45, 1, "Ucg");
	ucg_DrawString(ucg, 25, 45, 2, "Ucg");
	ucg_DrawString(ucg, 25, 45, 3, "Ucg");

	ucg_SetMaxClipRange(ucg);
	ucg_SetColor(ucg,0,0xff, 0xff, 0xff);
	ucg_DrawFrame(ucg,7-1,58-1,90+2,4+2);
	ucg_SetClipRange(ucg,7, 58, 90, 4);
	ucg_SetColor(ucg,0,0xff, 0x61, 0x0b8);
	ucg_DrawString(ucg, 25, 45, 0, "Ucg");
	ucg_DrawString(ucg, 25, 45, 1, "Ucg");
	ucg_DrawString(ucg, 25, 45, 2, "Ucg");
	ucg_DrawString(ucg, 25, 45, 3, "Ucg");

	ucg_SetFont(ucg,ucg_font_helvB12_tr);
	ucg_SetMaxClipRange(ucg);
}

// define a 3d point structure
struct pt3d {
  ucg_int_t x, y, z;
};

struct surface {
  uint8_t p[4];
  int16_t z;
};

struct pt2d {
  ucg_int_t x, y;
  unsigned is_visible;
};


// define the point at which the observer looks, 3d box will be centered there
#define MX 160
#define MY 120

// define a value that corresponds to "1"
#define U 64

// eye to screen distance (fixed)
#define ZS U

// cube edge length is 2*U
struct pt3d cube[8] = {
  { -U, -U, U},
  { U, -U, U},
  { U, -U, -U},
  { -U, -U, -U},
  { -U, U, U},
  { U, U, U},
  { U, U, -U},
  { -U, U, -U},
};

// define the surfaces
struct surface cube_surface[6] = {
  { {0, 1, 2, 3}, 0 },	// bottom
  { {4, 5, 6, 7}, 0 },	// top
  { {0, 1, 5, 4}, 0 },	// back

  { {3, 7, 6, 2}, 0 },	// front
  { {1, 2, 6, 5}, 0 },	// right
  { {0, 3, 7, 4}, 0 },	// left
};

// define some structures for the copy of the box, calculation will be done there
struct pt3d cube2[8];
struct pt2d cube_pt[8];

// will contain a rectangle border of the box projection into 2d plane
ucg_int_t x_min, x_max;
ucg_int_t y_min, y_max;

int16_t sin_tbl[65] = {
0,1606,3196,4756,6270,7723,9102,10394,11585,12665,13623,14449,15137,15679,16069,16305,16384,16305,16069,15679,
15137,14449,13623,12665,11585,10394,9102,7723,6270,4756,3196,1606,0,-1605,-3195,-4755,-6269,-7722,-9101,-10393,
-11584,-12664,-13622,-14448,-15136,-15678,-16068,-16304,-16383,-16304,-16068,-15678,-15136,-14448,-13622,-12664,-11584,-10393,-9101,-7722,
-6269,-4755,-3195,-1605,0};

int16_t cos_tbl[65] = {
16384,16305,16069,15679,15137,14449,13623,12665,11585,10394,9102,7723,6270,4756,3196,1606,0,-1605,-3195,-4755,
-6269,-7722,-9101,-10393,-11584,-12664,-13622,-14448,-15136,-15678,-16068,-16304,-16383,-16304,-16068,-15678,-15136,-14448,-13622,-12664,
-11584,-10393,-9101,-7722,-6269,-4755,-3195,-1605,0,1606,3196,4756,6270,7723,9102,10394,11585,12665,13623,14449,
15137,15679,16069,16305,16384};


/** @brief copied from ucglib Arduino examples
 */
void copy_cube(void) {
  uint8_t i;
  for( i = 0; i < 8; i++ )
  {
    cube2[i] = cube[i];
  }
}

/** @brief copied from ucglib Arduino examples
 */
void rotate_cube_y(uint16_t w) {
  uint8_t i;
  int16_t x, z;
  /*
    x' = x * cos(w) + z * sin(w)
    z' = - x * sin(w) + z * cos(w)
  */
  for( i = 0; i < 8; i++ ) {
    x = ((int32_t)cube2[i].x * (int32_t)cos_tbl[w] + (int32_t)cube2[i].z * (int32_t)sin_tbl[w])>>14;
    z = (- (int32_t)cube2[i].x * (int32_t)sin_tbl[w] + (int32_t)cube2[i].z * (int32_t)cos_tbl[w])>>14;
    //printf("%d: %d %d --> %d %d\n", i, cube2[i].x, cube2[i].z, x, z);
    cube2[i].x = x;
    cube2[i].z = z;
  }
}

/** @brief copied from ucglib Arduino examples
 */
void rotate_cube_x(uint16_t w) {
  uint8_t i;
  int16_t y, z;
  for( i = 0; i < 8; i++ ) {
    y = ((int32_t)cube2[i].y * (int32_t)cos_tbl[w] + (int32_t)cube2[i].z * (int32_t)sin_tbl[w])>>14;
    z = (- (int32_t)cube2[i].y * (int32_t)sin_tbl[w] + (int32_t)cube2[i].z * (int32_t)cos_tbl[w])>>14;
    cube2[i].y = y;
    cube2[i].z = z;
  }
}

/** @brief copied from ucglib Arduino examples
 */
void trans_cube(uint16_t z) {
  uint8_t i;
  for( i = 0; i < 8; i++ )
  {
    cube2[i].z += z;
  }
}

/** @brief copied from ucglib Arduino examples
 */
void reset_min_max(void) {
  x_min = 0x07fff;
  y_min = 0x07fff;
  x_max = -0x07fff;
  y_max = -0x07fff;
}

/** @brief copied and changed from ucglib Arduino examples
// calculate xs and ys from a 3d value
 */
void convert_3d_to_2d(struct pt3d *p3, struct pt2d *p2) {
  int32_t t;
  p2->is_visible = 1;
  if ( p3->z >= ZS ) {
    t = ZS;
    t *= p3->x;
    t <<=1;
    t /= p3->z;
    if ( t >= -MX && t <= MX-1 ) {
      t += MX;
      p2->x = t;

      if ( x_min > t )
	    x_min = t;
      if ( x_max < t )
	    x_max = t;

      t = ZS;
      t *= p3->y;
      t <<=1;
      t /= p3->z;
      if ( t >= -MY && t <= MY-1 ) {
		t += MY;
		p2->y = t;

		if ( y_min > t )
		  y_min = t;
		if ( y_max < t )
		  y_max = t;
      } else {
    	  p2->is_visible = 0;
      }
    } else {
      p2->is_visible = 0;
    }
  } else {
    p2->is_visible = 0;
  }
}

/** @brief copied from ucglib Arduino examples
 */
void convert_cube(void) {
  uint8_t i;
  reset_min_max();
  for( i = 0; i < 8; i++ ) {
    convert_3d_to_2d(cube2+i, cube_pt+i);
  }
}

/** @brief copied from ucglib Arduino examples
 */
void calculate_z(void) {
  uint8_t i, j;
  uint16_t z;
  for( i = 0; i < 6; i++ ) {
    z = 0;
    for( j = 0; j < 4; j++ ) {
      z+=cube2[cube_surface[i].p[j]].z;
    }
    z/=4;
    cube_surface[i].z = z;
  }
}

/** @brief copied and changed from ucglib Arduino examples
 */
void draw_cube(ucg_t *ucg) {
  uint8_t i, ii;
  uint8_t skip_cnt = 3;		/* it is known, that the first 3 surfaces are invisible */
  int16_t z, z_upper;


  z_upper = 32767;
  for(;;) {
    ii = 6;
    z = -32767;
    for( i = 0; i < 6; i++ ) {
      if ( cube_surface[i].z <= z_upper ) {
		if ( z < cube_surface[i].z ) {
		  z = cube_surface[i].z;
		  ii = i;
		}
	  }
    }

    if ( ii >= 6 )
      break;
    //printf("%d z=%d upper=%d\n", ii, z, z_upper);
    z_upper = cube_surface[ii].z;
    cube_surface[ii].z++;

    if ( skip_cnt > 0 ) {
      skip_cnt--;
    } else {
      ucg_SetColor(ucg,0, ((ii+1)&1)*255,(((ii+1)>>1)&1)*255,(((ii+1)>>2)&1)*255);
      ucg_DrawTetragon(ucg,
		cube_pt[cube_surface[ii].p[0]].x, cube_pt[cube_surface[ii].p[0]].y,
		cube_pt[cube_surface[ii].p[1]].x, cube_pt[cube_surface[ii].p[1]].y,
		cube_pt[cube_surface[ii].p[2]].x, cube_pt[cube_surface[ii].p[2]].y,
		cube_pt[cube_surface[ii].p[3]].x, cube_pt[cube_surface[ii].p[3]].y);
    }
  }
}

/** @brief copied and changed from ucglib Arduino examples
 */
void calc_and_draw(ucg_t *ucg,uint16_t w, uint16_t v) {
  copy_cube();
  rotate_cube_y(w);
  rotate_cube_x(v);
  trans_cube(U*8);
  convert_cube();
  calculate_z();
  draw_cube(ucg);
}

/** @brief copied and changed from ucglib Arduino examples
 */
void cube3d(ucg_t *ucg) {
	  uint16_t m=0;

	  ucg_SetColor(ucg,0,0,0,0);
	  ucg_DrawBox(ucg,0, 0, ucg_GetWidth(ucg), ucg_GetHeight(ucg));

	  ucg_SetColor(ucg,0,255, 255, 255);
	  ucg_DrawString(ucg, 2, 18, 0, "ucglib nRF51822 cube 3D");

	  while(m<50) {
		  calc_and_draw(ucg,w, v>>3);

		  v+=3;
		  v &= 511;

		  w++;
		  w &= 63;
		  nrf_delay_ms(30);

		  ucg_SetColor(ucg,0,0,0,0);
		  ucg_DrawBox(ucg,x_min, y_min, x_max-x_min+1, y_max-y_min+1);
		  m++;
	  }

}

/** @brief print some color text
 */
void nRF51822_printText(ucg_t *ucg) {
	uint16_t iLine;
    uint8_t rnd_buff[RANDOM_BUFF_SIZE];

    ucg_SetFont(ucg,ucg_font_helvB10_tr);
    for (iLine=0;iLine<3;iLine++) {
    		//ucg_SetColor(&ucg,1, 0xFF, 0, 0);
        	nrf_drv_rng_rand(rnd_buff,3);
    		ucg_SetColor(ucg,0, rnd_buff[0], rnd_buff[1], rnd_buff[2]);
        	ucg_DrawBox(ucg, 240-((iLine+1)*16), 0, 16, 320);
    		ucg_SetColor(ucg,0, 0xFF, 0xFF,0xFF);
    		ucg_DrawString(ucg, 242-((iLine+1)*16), 0, 1, "nRF51822 + ucglib + ILI9341 + SPI 8MHz");
	}
	ucg_SetFont(ucg,ucg_font_courB18_tr);
    for (iLine=0;iLine<3;iLine++) {
        	nrf_drv_rng_rand(rnd_buff,3);
    		ucg_SetColor(ucg,0, rnd_buff[0], rnd_buff[1], rnd_buff[2]);
        	ucg_DrawBox(ucg, 194-((iLine+1)*26), 0, 26, 320);

    		ucg_SetColor(ucg,0, 0xFF, 0xFF,0xFF);
    		ucg_DrawString(ucg, 198-((iLine+1)*26), 0, 1, "nRF51822+ucglib+ILI9341");
	}
	ucg_SetFont(ucg,ucg_font_ncenR24_hr);
    for (iLine=0;iLine<3;iLine++) {
        	nrf_drv_rng_rand(rnd_buff,3);
    		ucg_SetColor(ucg,0, rnd_buff[0], rnd_buff[1], rnd_buff[2]);
        	ucg_DrawBox(ucg, 114-((iLine+1)*36), 0, 36, 320);

    		ucg_SetColor(ucg,0, 0xFF, 0xFF,0xFF);
    		ucg_DrawString(ucg, 120-((iLine+1)*36), 0, 1, "nRF51822 + ucglib");
	}

}

/**
 * @brief Function for application main entry.
 */
int main(void) {
    // uint32_t retVal;
	ucg_t ucg;
    uint32_t pos=0;

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));

    // setup
    NRF_LOG_INFO("main gpio+spi+rng config\n\r");
    gpio_config();
    spi_config();
    rng_config();

    NRF_LOG_INFO("main ucglib config\n\r");
    ucg.pin_list[UCG_PIN_RST] = ILI9341_RST;
    ucg.pin_list[UCG_PIN_CD] = ILI9341_DC;
    ucg.pin_list[UCG_PIN_CS] = SPIM0_SS_PIN;

    ucg_Init(&ucg, ucg_dev_ili9341_18x240x320, ucg_ext_ili9341_18, ucg_com_nrf51822_generic_HW_SPI);
    ucg_SetFontMode(&ucg, UCG_FONT_MODE_TRANSPARENT);
    //ucg_SetFontMode(&ucg, UCG_FONT_MODE_SOLID);

    NRF_LOG_INFO("main loop\n\r");
    // loop
    while (true) {

    	ucg_SetFont(&ucg,ucg_font_helvB12_tr);
    	ucg_SetRotate90(&ucg);

		ucg_SetScale2x2(&ucg);
		draw_ucg_logo(&ucg);
		ucg_UndoScale(&ucg);
		nrf_delay_ms(5000);

    	cube3d(&ucg);
    	nrf_delay_ms(2000);

    	pixel_and_lines(&ucg);
    	nrf_delay_ms(2000);

    	triangle(&ucg);
    	nrf_delay_ms(2000);

    	fonts(&ucg);
    	nrf_delay_ms(2000);

    	clip(&ucg);
    	nrf_delay_ms(2000);

    	gradient(&ucg);
    	nrf_delay_ms(2000);

    	box(&ucg);
    	nrf_delay_ms(2000);

		ucg_UndoRotate(&ucg);
    	nRF51822_printText(&ucg);
    	nrf_delay_ms(1000);

		NRF_LOG_INFO("tick (%u)\n\r",pos++);

    }
}


/** @} */
