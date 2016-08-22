TEMPLATEROOT = ..

# compilation flags for gdb

CFLAGS  += -O0 -g
CFLAGS += -DBOARD_CUSTOM
CFLAGS += -DNRF_LOG_USES_UART=1
ASFLAGS += -g 
ASMFLAGS += -DBOARD_CUSTOM
ASMFLAGS += -DNRF_LOG_USES_UART=1

LDSCRIPT = $(TEMPLATEROOT)/blank_nrf51822_QFAA.ld

# object files

OBJS =  system_nrf51.o app_error.o app_fifo.o app_util_platform.o nrf_assert.o app_uart_fifo.o nrf_drv_common.o nrf_delay.o nrf_drv_uart.o
OBJS += nrf_log.o uart.o main.o

# include common make file

include $(TEMPLATEROOT)/Makefile.common
