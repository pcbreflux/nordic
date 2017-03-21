/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
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

/** @example examples/ble_peripheral/ble_xxxx/main.c changed by pcbreflux
 *
 * @brief Sample Application main file.
 *
 * This file contains the source code for a sample application using the LED Button service.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "boards.h"
#include "sdk_config.h"
#define NRF_LOG_MODULE_NAME "PCBREFLUX BLE"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "app_uart.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "sensorsim.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "fds.h"
#include "fstorage.h"
#include "peer_manager.h"
#include "ble_conn_state.h"
#include "ble_gap.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_rtc.h"
#include "app_error.h"
#include "mcp3008.h"
#include "ble_mcp3008.h"

//#define ADVERTISING_LED_PIN             BSP_LED_0_MASK                              /**< Is on when device is advertising. */
//#define CONNECTED_LED_PIN               BSP_LED_1_MASK                              /**< Is on when device has connected. */
//
//#define LEDBUTTON_LED_PIN               BSP_LED_2_MASK                              /**< LED to be toggled with the help of the LED Button Service. */
//#define LEDBUTTON_BUTTON_PIN            BSP_BUTTON_0                                /**< Button that will trigger the notification event with the LED Button Service */

#define IS_SRVC_CHANGED_CHARACT_PRESENT  0                                          /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define CENTRAL_LINK_COUNT               0                                          /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT            1                                          /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                     "nrf_mcp3008_gcc"                          /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "pcbreflux"                                /**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NUMBER                    "51822"  
#define SERIAL_NUMBER                   "QFAA"
#define HARDWARE_REVISION               "J"

#define PNP_ID_VENDOR_ID_SOURCE          0xAA                                           /**< Vendor ID Source. */
#define PNP_ID_VENDOR_ID                 0x1915                                         /**< Vendor ID. */
#define PNP_ID_PRODUCT_ID                0xEEEE                                         /**< Product ID. */
#define PNP_ID_PRODUCT_VERSION           0x0001                                         /**< Product Version. */

#define APP_ADV_INTERVAL                640                                          /**< The advertising interval (in units of 0.625 ms; this value corresponds to 400 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED       /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            6                                           /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)            /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory time-out (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define APP_GPIOTE_MAX_USERS            1                                           /**< Maximum number of users of the GPIOTE handler. */
#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)    /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define SEC_PARAM_BOND                   1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                   0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                   0                                          /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS               0                                          /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                    0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                                         /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2       /**< Reply when unsupported features are requested. */


static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static ble_mcp3008_t                    m_mcp3008;                                  /**< MCP3008 Service instance. */

#define MCP3008_INTERVAL            APP_TIMER_TICKS(500, APP_TIMER_PRESCALER)     /**< more than MCP3008 conversion time -> interval 500ms (ticks). */
APP_TIMER_DEF(m_mcp3008_timer_id);                                                /**< MCP3008 timer. */

// defining SPI0
const nrf_drv_spi_t my_spi_0 = NRF_DRV_SPI_INSTANCE(0); /**< Declaring an instance of nrf_drv_spi for SPI0. */

const uint32_t led_pin1 = 19;

static mcp3008_config_t mcp3008_config = MCP3008_DEFAULT_CONFIG;
static uint32_t pos=0;

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze 
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name) {
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for handling File Data Storage events.
 *
 * @param[in] p_evt  Peer Manager event.
 * @param[in] cmd
 */
static void fds_evt_handler(fds_evt_t const * const p_evt) {
    if (p_evt->id == FDS_EVT_GC) {
        NRF_LOG_INFO("GC completed\n");
    }
}

/**@brief Function for handling write events to the MCP3008 characteristic.
 *
 * @param[in] p_mcp3008  Instance of MCP3008 Service to which the write applies.
 * @param[in] led_state Written/desired state of the LED.
 */
static void blemcp3008_write_handler(ble_mcp3008_t * p_mcp3008, ble_mcp3008_evt_t *evt) {
    uint32_t err_code;

    if (evt->evt_type == BLE_MCP3008_EVT_NOTIFICATION_ENABLED) {
    	NRF_LOG_INFO("BLE_MCP3008_EVT_NOTIFICATION_ENABLED\r\n");
        err_code = app_timer_start(m_mcp3008_timer_id, MCP3008_INTERVAL, NULL);
        APP_ERROR_CHECK(err_code);
        p_mcp3008->running=1;
    } else { // BLE_MCP3008_EVT_NOTIFICATION_DISABLED | BLE_MCP3008_EVT_DISCONNECTED
    	NRF_LOG_INFO("BLE_MCP3008_EVT_NOTIFICATION_DISABLED\r\n");
		mcp3008_unprepare(&mcp3008_config);
        err_code = app_timer_stop(m_mcp3008_timer_id);
        p_mcp3008->running=0;
        APP_ERROR_CHECK(err_code);
    }
}

/** @brief Function initialization and configuration of RTC driver instance.
 */
static void spi_config(mcp3008_config_t *mcp3008_config) {
    ret_code_t err_code;

    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;

    NRF_LOG_DEBUG("spi config %d %d %d\n\r",mcp3008_config->miso_pin,mcp3008_config->sck_pin,mcp3008_config->cs_pin);
    spi_config.sck_pin = mcp3008_config->sck_pin;
    spi_config.mosi_pin = 0xFF; // not used
    spi_config.miso_pin = mcp3008_config->miso_pin;
    spi_config.ss_pin = 0xFF; // mcp3008 need Conversion Time 220ms, so we set it external
    //spi_config.frequency = NRF_DRV_SPI_FREQ_125K; // tCP = 100ns see MAX7219/MAX7221 Datasheet TIMING CHARACTERISTICS
    spi_config.frequency = NRF_DRV_SPI_FREQ_8M; // tCP = 100ns see MAX7219/MAX7221 Datasheet TIMING CHARACTERISTICS
    spi_config.mode = NRF_DRV_SPI_MODE_0; // SCK active high, sample on leading edge of clock.
    spi_config.bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;

    err_code = nrf_drv_spi_init(&my_spi_0,&spi_config,NULL);
    APP_ERROR_CHECK(err_code);

}

/**@brief Function for performing mcp3008 measurement and updating the Battery Level characteristic
 *        in Battery Service.
 */
static void mcp3008_temp_update(uint16_t temp) {
    uint32_t err_code;

    err_code = ble_mcp3008_temp_update(&m_mcp3008, temp);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_PACKETS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)) {
        APP_ERROR_HANDLER(err_code);
    }
}


static void mcp3008_timeout_handler(void * p_context) {
//    uint32_t err_code;

	uint16_t adcout;

	NRF_LOG_DEBUG("mcp3008_timeout_handler %d %u\n\r",pos);
	if (pos%2==0) {
		if (m_mcp3008.running!=0) {
			mcp3008_prepare(&mcp3008_config);
		}
	} else {
		if (m_mcp3008.running!=0) {
			adcout=mcp3008_readanalog(&mcp3008_config,1);
			NRF_LOG_INFO("mcp3008_timeout_handler adc \x1B[1;32m%04X\x1B[0m    \r",adcout);
			mcp3008_temp_update(adcout);
		}
	}
    pos++;
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void) {
	uint32_t err_code;

	NRF_LOG_INFO("timers_init\r\n");
    // Initialize timer module, making it use the scheduler
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers.
	err_code = app_timer_create(&m_mcp3008_timer_id,
								APP_TIMER_MODE_REPEATED,
								mcp3008_timeout_handler);
	APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void) {
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    // Set GAP device name
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    // Set GAP Appearance value
    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_THERMOMETER);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    // Set GAP Peripheral Preferred Connection Parameters
    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void) {
    uint32_t       err_code;
    ble_mcp3008_init_t blemcp3008_init;

    blemcp3008_init.evt_handler = blemcp3008_write_handler;

    err_code = ble_mcp3008_init(&m_mcp3008, &blemcp3008_init);
    APP_ERROR_CHECK(err_code);

}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module that
 *          are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply
 *       setting the disconnect_on_fail config parameter, but instead we use the event
 *       handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt) {
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error) {
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void) {
    uint32_t err_code;
    ble_conn_params_init_t cp_init;

	NRF_LOG_INFO("conn_params_init\r\n");
    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt) {

	switch (ble_adv_evt) {
    case BLE_ADV_EVT_DIRECTED:
        NRF_LOG_INFO("BLE_ADV_EVT_DIRECTED\r\n");
        break;//BLE_ADV_EVT_DIRECTED
    case BLE_ADV_EVT_FAST:
        NRF_LOG_INFO("BLE_ADV_EVT_FAST\r\n");
        break;//BLE_ADV_EVT_FAST
    case BLE_ADV_EVT_SLOW:
        NRF_LOG_INFO("BLE_ADV_EVT_SLOW\r\n");
        break;//BLE_ADV_EVT_SLOW
    case BLE_ADV_EVT_FAST_WHITELIST:
        NRF_LOG_INFO("BLE_ADV_EVT_FAST_WHITELIST\r\n");
        break;//BLE_ADV_EVT_FAST_WHITELIST
    case BLE_ADV_EVT_SLOW_WHITELIST:
        NRF_LOG_INFO("BLE_ADV_EVT_SLOW_WHITELIST\r\n");
        break;//BLE_ADV_EVT_SLOW_WHITELIST
    case BLE_ADV_EVT_IDLE:
        NRF_LOG_INFO("BLE_ADV_EVT_IDLE\r\n");
        break;//BLE_ADV_EVT_IDLE
    default:
        // No implementation needed.
        break;
    }
}

/**@brief Function for handling the Application's BLE stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt) {
    uint32_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id) {
		case BLE_GAP_EVT_CONNECTED:
			NRF_LOG_INFO("Connected\r\n");
			m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
			break; // BLE_GAP_EVT_CONNECTED

		case BLE_GAP_EVT_DISCONNECTED:
			NRF_LOG_INFO("Disconnected\r\n");
			m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break; // BLE_GAP_EVT_DISCONNECTED
        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(m_conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
			{
				ble_gatts_evt_rw_authorize_request_t  req;
				ble_gatts_rw_authorize_reply_params_t auth_reply;

				req = p_ble_evt->evt.gatts_evt.params.authorize_request;

				if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID) {
					if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
						(req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
						(req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL)) {
						if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE) {
							auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
						} else {
							auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
						}
						auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
						err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
																   &auth_reply);
						APP_ERROR_CHECK(err_code);
					}
				}
			}
            break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

#if (NRF_SD_BLE_API_VERSION == 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                       NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt) {
    ble_conn_state_on_ble_evt(p_ble_evt);
    pm_on_ble_evt(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_mcp3008_on_ble_evt(&m_mcp3008, p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt) {
    // Dispatch the system event to the fstorage module, where it will be
    // dispatched to the Flash Data Storage (FDS) module.
    fs_sys_event_handler(sys_evt);

    // Dispatch to the Advertising module last, since it will check if there are any
    // pending flash operations in fstorage. Let fstorage process system events first,
    // so that it can report correctly to the Advertising module.
    ble_advertising_on_sys_evt(sys_evt);
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void) {
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

	NRF_LOG_INFO("SOFTDEVICE_HANDLER_INIT\r\n");
    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

	NRF_LOG_INFO("softdevice_enable_get_default_config\r\n");
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for system events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt) {
    ret_code_t err_code;

    switch (p_evt->evt_id) {
    case PM_EVT_BONDED_PEER_CONNECTED:
    {
        NRF_LOG_DEBUG("Connected to previously bonded device\r\n");
        err_code = pm_peer_rank_highest(p_evt->peer_id);
        if (err_code != NRF_ERROR_BUSY) {
                APP_ERROR_CHECK(err_code);
        }
    }
    break;//PM_EVT_BONDED_PEER_CONNECTED
        case PM_EVT_CONN_SEC_START:
            break; // PM_EVT_CONN_SEC_START

        case PM_EVT_CONN_SEC_SUCCEEDED:
            NRF_LOG_DEBUG("Link secured. Role: %d. conn_handle: %d, Procedure: %d\r\n",
                                 ble_conn_state_role(p_evt->conn_handle),
                                 p_evt->conn_handle,
                                 p_evt->params.conn_sec_succeeded.procedure);
            err_code = pm_peer_rank_highest(p_evt->peer_id);
            if (err_code != NRF_ERROR_BUSY) {
                APP_ERROR_CHECK(err_code);
            }
            break;  // PM_EVT_CONN_SEC_SUCCEEDED

        case PM_EVT_CONN_SEC_FAILED:

            /** In some cases, when securing fails, it can be restarted directly. Sometimes it can
             *  be restarted, but only after changing some Security Parameters. Sometimes, it cannot
             *  be restarted until the link is disconnected and reconnected. Sometimes it is
             *  impossible, to secure the link, or the peer device does not support it. How to
             *  handle this error is highly application dependent. */
            switch (p_evt->params.conn_sec_failed.error)
            {
                case PM_CONN_SEC_ERROR_PIN_OR_KEY_MISSING:
                    // Rebond if one party has lost its keys.
                    err_code = pm_conn_secure(p_evt->conn_handle, true);
                    if (err_code != NRF_ERROR_INVALID_STATE)
                    {
                        APP_ERROR_CHECK(err_code);
                    }
                    break;

                default:
                    break;
            }
            break; // PM_EVT_CONN_SEC_FAILED

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break; // PM_EVT_CONN_SEC_CONFIG_REQ

        case PM_EVT_STORAGE_FULL:
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code != NRF_ERROR_BUSY)
            {
                APP_ERROR_CHECK(err_code);
            }
            break; // PM_EVT_STORAGE_FULL

        case PM_EVT_ERROR_UNEXPECTED:
            // A likely fatal error occurred. Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
            break;

        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
            break; // PM_EVT_PEER_DATA_UPDATE_SUCCEEDED

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
            // Assert.
            APP_ERROR_CHECK_BOOL(false);
            break; // PM_EVT_ERROR_UNEXPECTED

        case PM_EVT_PEER_DELETE_SUCCEEDED:
            break; // PM_EVT_PEER_DELETE_SUCCEEDED

        case PM_EVT_PEER_DELETE_FAILED:
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
            break; // PM_EVT_PEER_DELETE_FAILED

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            break; // PM_EVT_PEERS_DELETE_SUCCEEDED

        case PM_EVT_PEERS_DELETE_FAILED:
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
            break; // PM_EVT_PEERS_DELETE_FAILED

        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
            break; // PM_EVT_LOCAL_DB_CACHE_APPLIED

        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
            // The local database has likely changed, send service changed indications.
            pm_local_database_has_changed();
            break; // PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED

        case PM_EVT_SERVICE_CHANGED_IND_SENT:
            break; // PM_EVT_SERVICE_CHANGED_IND_SENT

        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
            break; // PM_EVT_SERVICE_CHANGED_IND_SENT

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for the Peer Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Peer Manager.
 */
static void peer_manager_init(bool erase_bonds) {
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

	NRF_LOG_INFO("peer_manager_init\r\n");
    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    if (erase_bonds) {
        err_code = pm_peers_delete();
        APP_ERROR_CHECK(err_code);
    }

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = fds_register(fds_evt_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void) {
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;

	NRF_LOG_INFO("advertising_init\r\n");
    ble_uuid_t adv_uuids[] = {{BLE_MCP3008_UUID_SERVICE,  m_mcp3008.uuid_type}}; /**< Universally unique service identifiers. */

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = true;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = adv_uuids;

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting advertising.
 */
static void advertising_start(void) {
    uint32_t             err_code;
    ble_gap_adv_params_t adv_params;

	NRF_LOG_INFO("advertising_start\r\n");
    // Start advertising
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = APP_ADV_INTERVAL;
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Power Manager.
 */
static void power_manage(void) {
    uint32_t err_code = sd_app_evt_wait();

    APP_ERROR_CHECK(err_code);
}


/**@brief Function for application main entry.
 */
int main(void) {

	// Initialize.
    mcp3008_config.spi_instance = &my_spi_0;

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));

    NRF_LOG_INFO("start ble MCP3008\n\r");
    spi_config(&mcp3008_config);
    mcp3008_init(&mcp3008_config);
    timers_init();

    // NRF_RADIO->TXPOWER    = (RADIO_TXPOWER_TXPOWER_Pos4dBm << RADIO_TXPOWER_TXPOWER_Pos);

    ble_stack_init();
    peer_manager_init(false);  // erase bonds if needed
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();

    // Start execution.
    advertising_start();

    // NRF_RADIO->TXPOWER    = (RADIO_TXPOWER_TXPOWER_0dBm << RADIO_TXPOWER_TXPOWER_Pos);

    // Enter main loop.
    for (;;) {
        power_manage();
    }
}


/**
 * @}
 */
