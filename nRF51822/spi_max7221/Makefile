TEMPLATEROOT = ..

# compilation flags for gdb

CFLAGS  += -O0 -g
CFLAGS += -DBOARD_CUSTOM
CFLAGS += -DNRF51822
CFLAGS += -DNRF_LOG_USES_UART=1
ASFLAGS += -g 
ASMFLAGS += -DBOARD_CUSTOM
ASMFLAGS += -DNRF51822
ASMFLAGS += -DNRF_LOG_USES_UART=1

LDSCRIPT = $(TEMPLATEROOT)/blank_nrf51822_QFAA.ld

# object files

OBJS =  system_nrf51.o  nrf_log_backend_serial.o nrf_log_frontend.o
OBJS += app_error_weak.o app_error.o app_fifo.o app_util_platform.o nrf_assert.o app_uart_fifo.o nrf_drv_common.o nrf_drv_uart.o
OBJS += nrf_drv_spi.o
OBJS += max7221.o main.o 

# include common make file

include $(TEMPLATEROOT)/Makefile.common
