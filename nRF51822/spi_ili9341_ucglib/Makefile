TEMPLATEROOT = ..

# compilation flags for gdb

CFLAGS  += -O0 -g
CFLAGS += -DBOARD_CUSTOM
#--- ucglib --------
CFLAGS += -DUSE_PIN_LIST
CFLAGS += -DUCG_MSG_DRAW_L90BF
# ------------------
CFLAGS += -DNRF51822
CFLAGS += -DNRF_LOG_USES_UART=1
ASFLAGS += -g 
ASMFLAGS += -DBOARD_CUSTOM
ASMFLAGS += -DNRF51822
ASMFLAGS += -DNRF_LOG_USES_UART=1

LDSCRIPT = $(TEMPLATEROOT)/blank_nrf51822_QFAA.ld

INC_PATHS += -I$(TEMPLATEROOT)/ucglib/csrc

vpath %.c $(TEMPLATEROOT)/ucglib/csrc

# object files

OBJS =  system_nrf51.o  nrf_log_backend_serial.o nrf_log_frontend.o
OBJS += app_error_weak.o app_error.o app_fifo.o app_util_platform.o nrf_assert.o 
OBJS += app_uart_fifo.o nrf_drv_common.o nrf_drv_uart.o
OBJS += nrf_drv_rng.o nrf_drv_spi.o 
OBJS += ucg_dev_ic_ili9341.o ucg_dev_tft_240x320_ili9341.o 
OBJS += ucg_com_msg_api.o ucg_init.o ucg_dev_msg_api.o ucg_pixel.o ucg_line.o ucg_rotate.o ucg_clip.o ucg_ccs.o
OBJS += ucg_box.o ucg_dev_default_cb.o ucg_font.o ucg_pixel_font_data.o ucg_circle.o ucg_scale.o ucg_polygon.o
OBJS += ucg_nrf51822.o main.o 

# include common make file

include $(TEMPLATEROOT)/Makefile.common
