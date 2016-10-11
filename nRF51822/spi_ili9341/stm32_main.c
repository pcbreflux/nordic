#include <stdio.h>
#include <stm32f10x.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_spi.h>
#include <stm32f10x_usart.h>
#include "uartdma.h"
#include "spidma.h"
#include "xprintf.h"
#include "tm_stm32f4_fonts.h"
#include "ili9341.h"

#define TEST_BUFFER_SIZE 20

static __IO uint32_t TimingDelay;

void Delay(uint32_t nTime);

uint8_t TxBuffer[513];
uint8_t TxOutBuffer[513];
uint8_t wbuf[TEST_BUFFER_SIZE], rbuf[TEST_BUFFER_SIZE];
uint16_t wbuf16[TEST_BUFFER_SIZE], rbuf16[TEST_BUFFER_SIZE];
#define countof(a)   (sizeof(a) / sizeof(*(a)))


int main(void) {

	if (SysTick_Config(SystemCoreClock/1000)) 
		while(1);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	uart_init(USART1, 57600, USART_Mode_Rx | USART_Mode_Tx);
	// spi_init(SPI2);

	uint16_t iCount1 = 0;
	uint16_t iCount2 = 0;
        uint16_t iCount3 = 0;
	int i,j, iLine;
	static int ledval = 1;

	ILI9341_backLight(1);
	xsprintf((char *)TxBuffer,"STM32F103xx pre ILI9341_init [%d]               \n",iCount1);
	uart_write(USART1,TxBuffer,countof(TxBuffer)-1);
	ILI9341_init();
	xsprintf((char *)TxBuffer,"STM32F103xx post ILI9341_init [%d]               \n",iCount1);
	uart_write(USART1,TxBuffer,countof(TxBuffer)-1);

	ILI9341_fillscreen(0); // yes this sucks
	//xsprintf((char *)TxOutBuffer,"THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS");
	//iCount3 = countof(TxOutBuffer)-1;
	while (1) {

		if (iCount1%10==0) {
			xsprintf((char *)TxBuffer,"STM32F103xx [%d]               \n                   ",iCount1);
			uart_write(USART1,TxBuffer,countof(TxBuffer)-1);
		}

		xsprintf((char *)TxBuffer,"                    ");
	        for (iLine=0;iLine<9;iLine++) {  // lazy clear screen - yes it's lame
		  ILI9341_Puts(0,  iLine*26,(char *)TxBuffer, &TM_Font_16x26, 0x0000, 0x0000);
		}

		for (i=0;i<3;i++) {
		        for (iLine=0;iLine<23;iLine++) {
			  xsprintf((char *)TxBuffer,"STM32F103xx + ILI9341 Line %d",iLine);
			  ILI9341_Puts(0,  iLine*10,(char *)TxBuffer, &TM_Font_7x10, iCount1+iLine, 0xFFFF);
			}
	
		        for (iLine=0;iLine<23;iLine++) {
			  xsprintf((char *)TxBuffer,"STM32F103xx + ILI9341 Line %d",iLine);
			  ILI9341_Puts(0,  iLine*10, (char *)TxBuffer, &TM_Font_7x10, 0xFFFF, iCount1+iLine);
			}
		}
		xsprintf((char *)TxBuffer,"                    ");
	        for (iLine=0;iLine<9;iLine++) { 
		  ILI9341_Puts(0,  iLine*26,(char *)TxBuffer, &TM_Font_16x26, 0x0000, 0x0000);
		}
		for (i=0;i<3;i++) {
		        for (iLine=0;iLine<13;iLine++) {
			  xsprintf((char *)TxBuffer,"STM32F103xx + ILI9341 Line %d",iLine);
			  ILI9341_Puts(0,  iLine*18,(char *)TxBuffer, &TM_Font_11x18, iCount1+iLine, 0xFFFF);
			}
	
		        for (iLine=0;iLine<13;iLine++) {
			  xsprintf((char *)TxBuffer,"STM32F103xx + ILI9341 Line %d",iLine);
			  ILI9341_Puts(0,  iLine*18, (char *)TxBuffer, &TM_Font_11x18, 0xFFFF, iCount1+iLine);
			}
		}
	        for (iLine=0;iLine<9;iLine++) {
		  ILI9341_Puts(0,  iLine*26,(char *)TxBuffer, &TM_Font_16x26, 0x0000, 0x0000);
		}
		for (i=0;i<3;i++) {
		        for (iLine=0;iLine<9;iLine++) {
			  xsprintf((char *)TxBuffer,"STM32F103xx Line %d",iLine);
			  ILI9341_Puts(0,  iLine*26,(char *)TxBuffer, &TM_Font_16x26, iCount1+iLine, 0xFFFF);
			}
	
		        for (iLine=0;iLine<9;iLine++) {
			  xsprintf((char *)TxBuffer,"STM32F103xx Line %d",iLine);
			  ILI9341_Puts(0,  iLine*26, (char *)TxBuffer, &TM_Font_16x26, 0xFFFF, iCount1+iLine);
			}
		}
		/*

		ILI9341_dup_print("H E L L O  m y S T M 3 2 !", 1,0, iCount1, 0x0000);		
		// ILI9341_dup_print(TxOutBuffer+iCount2, 14,0, iCount1, 0x0000);

		Delay(5000);
		*/
		iCount1++;
		/*
                iCount2++;
		 if (iCount1==0xFFFF) {
		 	iCount1=0;
		 }
		 Delay(10);
		if (iCount2==iCount3) {
		 	iCount2=0;
		}
		*/

	}
}



void Delay(uint32_t nTime) {
	TimingDelay = nTime;
	while (TimingDelay != 0);
}

void SysTick_Handler(void) {
	if (TimingDelay != 0x00) {
		TimingDelay--;
	}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line) {
	while (1);
}
#endif


