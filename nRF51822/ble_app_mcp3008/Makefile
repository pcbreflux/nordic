TEMPLATEROOT = ..

# compilation flags for gdb

CFLAGS  += -O0 -g
CFLAGS += -DBOARD_CUSTOM
CFLAGS += -DNRF51822
CFLAGS += -DNRF_LOG_USES_UART=1
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DS130
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DSWI_DISABLE0
CFLAGS += -DNRF_SD_BLE_API_VERSION=2
CFLAGS += -D__HEAP_SIZE=0
ASFLAGS += -g 
ASMFLAGS += -DBOARD_CUSTOM
ASMFLAGS += -DNRF51822
ASMFLAGS += -DNRF_LOG_USES_UART=1
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DS130
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DSWI_DISABLE0
ASMFLAGS += -DNRF_SD_BLE_API_VERSION=2
ASMFLAGS += -D__HEAP_SIZE=0
#LDLIBS += -lc -lgcc -lrdimon

LDSCRIPT = $(TEMPLATEROOT)/s130_v2_nrf51822_QFAA.ld
INC_PATHS += -I$(SDK_PATH)/components/softdevice/s130/headers

vpath %.c $(SDK_PATH)/components/softdevice/s130/headers

# object files

OBJS += system_nrf51.o nrf_log_backend_serial.o nrf_log_frontend.o
OBJS += app_error_weak.o app_error.o app_fifo.o app_util_platform.o nrf_assert.o app_uart_fifo.o 
OBJS += app_timer.o
OBJS += nrf_drv_common.o nrf_drv_uart.o nrf_drv_spi.o nrf_drv_clock.o
OBJS += bsp.o main.o fstorage.o fds.o sdk_mapped_flags.o
OBJS += ble_advdata.o ble_advertising.o ble_conn_params.o
OBJS += mcp3008.o ble_mcp3008.o ble_srv_common.o ble_conn_state.o
OBJS += gatt_cache_manager.o gatts_cache_manager.o id_manager.o peer_data.o
OBJS += peer_data_storage.o peer_database.o peer_id.o peer_manager.o
OBJS += pm_buffer.o pm_mutex.o security_dispatcher.o security_manager.o
OBJS += softdevice_handler.o
# include common make file

include $(TEMPLATEROOT)/Makefile.common
