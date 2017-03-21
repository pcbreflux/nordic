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

/** @file ble_mcp3008.h
 *
 * @brief Temperature Service module.
 *
 * @details This module implements the MCP3008 Service with the MCP3008 characteristic.
 *          During initialization it adds the MCP3008 Service and MCP3008 characteristic
 *          to the BLE stack database.
 *
 *          If specified, the module will support notification of the MCP3008 characteristic
 *          through the ble_mcp3008_read() function.
 *          If an event handler is supplied by the application, the MCP3008 Service will
 *          generate MCP3008 Service events to the application.
 *
 * @note The application must propagate BLE stack events to the MCP3008 Service module by calling
 *       ble_mcp3008_on_ble_evt() from the @ref softdevice_handler callback.
 *
 */

#ifndef BLE_MCP3008_H__
#define BLE_MCP3008_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "app_util.h"

// using random numbers via https://www.uuidgenerator.net/
#define BLE_MCP3008_UUID_BASE        {0x18, 0x1B, 0xDB, 0x53, 0x32, 0x6C, 0x44, 0x23, \
									  0xBB, 0x89, 0x65, 0x87, 0xAA, 0xEE, 0xEE, 0x07}
#define BLE_MCP3008_UUID_SERVICE     0x3141
#define BLE_MCP3008_UUID_COM_CHAR    0x3142
#define BLE_MCP3008_UUID_VALUE_CHAR  0x3143

#define BLE_MCP3008_COM_MAX_LEN      6 /* max number of bytes per BLE package */
#define BLE_MCP3008_VALUE_MAX_LEN    sizeof(uint16_t) /* max number of bytes */

/**@brief MCP3008 Service event type. */
typedef enum {
    BLE_MCP3008_EVT_NOTIFICATION_ENABLED,      /**< MCP3008 value notification enabled event. */
    BLE_MCP3008_EVT_NOTIFICATION_DISABLED,     /**< MCP3008 value notification disabled event. */
    BLE_MCP3008_EVT_DISCONNECTED               /**< MCP3008 value notification disconnected event. */
} ble_mcp3008_evt_type_t;

/**@brief MCP3008 Service event. */
typedef struct {
    ble_mcp3008_evt_type_t evt_type;                                  /**< Type of event. */
} ble_mcp3008_evt_t;

// Forward declaration of the ble_mcp3008_t type.
typedef struct ble_mcp3008_s ble_mcp3008_t;

/**@brief MCP3008 Service event handler type. */
typedef void (*ble_mcp3008_evt_handler_t) (ble_mcp3008_t * p_mcp3008, ble_mcp3008_evt_t *evt);

/**@brief MCP3008 Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct {
    ble_mcp3008_evt_handler_t         evt_handler;     /**< Event handler to be called for handling events in the MCP3008 Service. */
} ble_mcp3008_init_t;

typedef enum {
    BLE_MCP3008_COM_START,                             /**< command truncate channel info. */
    BLE_MCP3008_COM_STOP,                              /**< command truncate channel info. */
    BLE_MCP3008_COM_NOTIFY                            /**< command truncate channel info. */
} ble_mcp3008_command_type_t;

typedef enum {
    BLE_MCP3008_START_STOP,                              /**< command truncate channel info. */
    BLE_MCP3008_START_TIMER,                             /**< command truncate channel info. */
    BLE_MCP3008_START_PWM,                               /**< command truncate channel info. */
    BLE_MCP3008_START_SOFTBLINK,                         /**< command truncate channel info. */
    BLE_MCP3008_START_SERVO
} ble_mcp3008_start_type_t;

/**@brief MCP3008 Service structure. This contains various status information for the service. */
struct ble_mcp3008_s {
    uint16_t                      service_handle;                 /**< Handle of MCP3008 Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t      com_char_handles;               /**< Handles related to the adding characteristic. */
    ble_gatts_char_handles_t      value_char_handles;             /**< Handles related to the notifying values characteristic. */
    uint16_t                      report_ref_handle;              /**< Handle of the Report Reference descriptor. */
    uint8_t                       uuid_type;                      /**< UUID type for the MCP3008 Service. */
    uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack). BLE_CONN_HANDLE_INVALID if not in a connection. */
    ble_mcp3008_evt_handler_t     evt_handler;                    /**< Event handler to be called for handling events in the MCP3008 Service. */
    uint16_t                      last_temp;                      /**< Temperature in 0.25 degrees celsius. */
    uint8_t                       running;                         /**< Temperature in 0.25 degrees celsius. */
};

/**@brief Function for initializing the MCP3008 Service.
 *
 * @param[out]  p_mcp3008   MCP3008 Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_mcp3008_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_mcp3008_init(ble_mcp3008_t * p_mcp3008, const ble_mcp3008_init_t * p_mcp3008_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the MCP3008 Service.
 *
 * @param[in]   p_mcp3008  MCP3008 Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_mcp3008_on_ble_evt(ble_mcp3008_t * p_mcp3008, ble_evt_t * p_ble_evt);

/**@brief Function for initializing the MCP3008 Service.
 *
 * @param[in]   p_mcp3008  MCP3008 Service structure.
 * @param[in]   temp  Temperature in 0.25 degrees celsius.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_mcp3008_temp_update(ble_mcp3008_t * p_mcp3008, uint16_t temp);

#endif // BLE_MCP3008_H__

/** @} */
