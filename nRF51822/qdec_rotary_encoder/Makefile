TEMPLATEROOT = ..

# compilation flags for gdb

CFLAGS  += -O0 -g
CFLAGS += -DBOARD_CUSTOM
ASFLAGS += -g 
ASMFLAGS += -DBOARD_CUSTOM

LDSCRIPT = $(TEMPLATEROOT)/blank_nrf51822_QFAA.ld

# object files

OBJS =  system_nrf51.o
OBJS += app_error.o app_fifo.o app_util_platform.o nrf_assert.o app_uart_fifo.o nrf_drv_common.o nrf_delay.o nrf_drv_uart.o
OBJS += nrf_drv_qdec.o
OBJS += uart.o main.o 

# include common make file

include $(TEMPLATEROOT)/Makefile.common
