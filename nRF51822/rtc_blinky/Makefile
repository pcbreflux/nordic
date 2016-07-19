TEMPLATEROOT = ..

# compilation flags for gdb

CFLAGS  += -O0 -g
ASFLAGS += -g 

LDSCRIPT = $(TEMPLATEROOT)/blank_nrf51822_QFAA.ld

# object files

OBJS =  system_nrf51.o app_error.o app_util_platform.o nrf_drv_clock.o nrf_drv_common.o nrf_drv_rtc.o
OBJS += rtc_blinky.o 

# include common make file

include $(TEMPLATEROOT)/Makefile.common
