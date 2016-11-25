/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
#ifndef BLE_TEST_BOARD_H
#define BLE_TEST_BOARD_H

// LEDs definitions for PCA10028
#define LEDS_NUMBER    5

#define LED_START      18
#define LED_1          18
#define LED_2          19
#define LED_3          20
#define LED_4          21
#define LED_5          22
#define LED_STOP       22

#define LEDS_LIST { LED_1, LED_2, LED_3, LED_4, LED_5 }

#define BSP_LED_0      LED_1
#define BSP_LED_1      LED_2
#define BSP_LED_2      LED_3
#define BSP_LED_3      LED_4
#define BSP_LED_4      LED_5

#define BSP_LED_0_MASK (1<<BSP_LED_0)
#define BSP_LED_1_MASK (1<<BSP_LED_1)
#define BSP_LED_2_MASK (1<<BSP_LED_2)
#define BSP_LED_3_MASK (1<<BSP_LED_3)
#define BSP_LED_4_MASK (1<<BSP_LED_4)

#define LEDS_MASK      (BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK | BSP_LED_3_MASK| BSP_LED_4_MASK)
/* all LEDs are lit when GPIO is low */
#define LEDS_INV_MASK  ~LEDS_MASK

#define BUTTONS_NUMBER 2

#define BUTTON_START   16
#define BUTTON_1       16
#define BUTTON_2       17
#define BUTTON_STOP    17
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_LIST { BUTTON_1, BUTTON_2 }

#define BSP_BUTTON_0   BUTTON_1
#define BSP_BUTTON_1   BUTTON_2

#define BSP_BUTTON_0_MASK (1<<BSP_BUTTON_0)
#define BSP_BUTTON_1_MASK (1<<BSP_BUTTON_1)

#define BUTTONS_MASK   0x001E0000

#define RX_PIN_NUMBER  1   // -> CPTX
#define TX_PIN_NUMBER  2   // -> CPRX
#define CTS_PIN_NUMBER 7   // -> CPRTS
#define RTS_PIN_NUMBER 8   // -> CPCTS
#define HWFC           false
#define UART0_CONFIG_BAUDRATE     UART_BAUDRATE_BAUDRATE_Baud115200

#define SPIS_MISO_PIN  28    // SPI MISO signal. 
#define SPIS_CSN_PIN   12    // SPI CSN signal. 
#define SPIS_MOSI_PIN  25    // SPI MOSI signal. 
#define SPIS_SCK_PIN   29    // SPI SCK signal. 

#define SPIM0_SCK_PIN       3     /**< SPI clock GPIO pin number. */
#define SPIM0_MOSI_PIN      5     /**< SPI Master Out Slave In GPIO pin number. */
#define SPIM0_MISO_PIN      4     /**< SPI Master In Slave Out GPIO pin number. */
#define SPIM0_SS_PIN        9     /**< SPI Slave Select GPIO pin number. */

#define SPIM1_SCK_PIN       15     /**< SPI clock GPIO pin number. */
#define SPIM1_MOSI_PIN      12     /**< SPI Master Out Slave In GPIO pin number. */
#define SPIM1_MISO_PIN      14     /**< SPI Master In Slave Out GPIO pin number. */
#define SPIM1_SS_PIN        13     /**< SPI Slave Select GPIO pin number. */

// serialization APPLICATION board
#define SER_CONN_CHIP_RESET_PIN     30    // Pin used to reset connectivity chip

#define SER_APP_RX_PIN              RX_PIN_NUMBER    // UART RX pin number.
#define SER_APP_TX_PIN              TX_PIN_NUMBER    // UART TX pin number.
#define SER_APP_CTS_PIN             CTS_PIN_NUMBER    // UART Clear To Send pin number.
#define SER_APP_RTS_PIN             RTS_PIN_NUMBER    // UART Request To Send pin number.

#define SER_APP_SPIM0_SCK_PIN       29    // SPI clock GPIO pin number.
#define SER_APP_SPIM0_MOSI_PIN      25    // SPI Master Out Slave In GPIO pin number
#define SER_APP_SPIM0_MISO_PIN      28    // SPI Master In Slave Out GPIO pin number
#define SER_APP_SPIM0_SS_PIN        12    // SPI Slave Select GPIO pin number
#define SER_APP_SPIM0_RDY_PIN       14    // SPI READY GPIO pin number
#define SER_APP_SPIM0_REQ_PIN       13    // SPI REQUEST GPIO pin number

// serialization CONNECTIVITY board
#define SER_CON_RX_PIN              RX_PIN_NUMBER   // UART RX pin number.
#define SER_CON_TX_PIN              TX_PIN_NUMBER   // UART TX pin number.
#define SER_CON_CTS_PIN             CTS_PIN_NUMBER    // UART Clear To Send pin number. Not used if HWFC is set to false.
#define SER_CON_RTS_PIN             RTS_PIN_NUMBER    // UART Request To Send pin number. Not used if HWFC is set to false.


#define SER_CON_SPIS_SCK_PIN        29    // SPI SCK signal.
#define SER_CON_SPIS_MOSI_PIN       25    // SPI MOSI signal.
#define SER_CON_SPIS_MISO_PIN       28    // SPI MISO signal.
#define SER_CON_SPIS_CSN_PIN        12    // SPI CSN signal.
#define SER_CON_SPIS_RDY_PIN        14    // SPI READY GPIO pin number.
#define SER_CON_SPIS_REQ_PIN        13    // SPI REQUEST GPIO pin number.

// Arduino board mappings
#define ARDUINO_SCL_PIN             3     // SCL signal pin
#define ARDUINO_SDA_PIN             4     // SDA signal pin
#define ARDUINO_AREF_PIN            0     // Aref pin
#define ARDUINO_13_PIN              29    // Digital pin 13
#define ARDUINO_12_PIN              28    // Digital pin 12
#define ARDUINO_11_PIN              25    // Digital pin 11
#define ARDUINO_10_PIN              24    // Digital pin 10
#define ARDUINO_9_PIN               23    // Digital pin 9
#define ARDUINO_8_PIN               20    // Digital pin 8

#define ARDUINO_7_PIN               19    // Digital pin 7
#define ARDUINO_6_PIN               18    // Digital pin 6
#define ARDUINO_5_PIN               17    // Digital pin 5
#define ARDUINO_4_PIN               16    // Digital pin 4
#define ARDUINO_3_PIN               15    // Digital pin 3
#define ARDUINO_2_PIN               14    // Digital pin 2
#define ARDUINO_1_PIN               13    // Digital pin 1
#define ARDUINO_0_PIN               12    // Digital pin 0

#define ARDUINO_A0_PIN              1     // Analog channel 0
#define ARDUINO_A1_PIN              2     // Analog channel 1
#define ARDUINO_A2_PIN              3     // Analog channel 2
#define ARDUINO_A3_PIN              4     // Analog channel 3
#define ARDUINO_A4_PIN              5     // Analog channel 4
#define ARDUINO_A5_PIN              6     // Analog channel 5

// Low frequency clock source to be used by the SoftDevice
/*
#define NRF_CLOCK_LFCLKSRC      NRF_CLOCK_LFCLKSRC_XTAL_20_PPM
*/
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_SYNTH,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}

#endif // BLE_TEST_BOARD_H
