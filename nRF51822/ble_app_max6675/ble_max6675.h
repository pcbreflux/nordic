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

/** @file ble_max6675.h
 *
 * @brief Temperature Service module.
 *
 * @details This module implements the MAX6675 Service with the MAX6675 characteristic.
 *          During initialization it adds the MAX6675 Service and MAX6675 characteristic
 *          to the BLE stack database.
 *
 *          If specified, the module will support notification of the MAX6675 characteristic
 *          through the ble_max6675_read() function.
 *          If an event handler is supplied by the application, the MAX6675 Service will
 *          generate MAX6675 Service events to the application.
 *
 * @note The application must propagate BLE stack events to the MAX6675 Service module by calling
 *       ble_max6675_on_ble_evt() from the @ref softdevice_handler callback.
 *
 */

#ifndef BLE_MAX6675_H__
#define BLE_MAX6675_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "app_util.h"

// using random numbers via https://www.uuidgenerator.net/
#define BLE_MAX6675_UUID_BASE        {0x18, 0x1B, 0xDB, 0x53, 0x32, 0x6C, 0x44, 0x23, \
									  0xBB, 0x89, 0x65, 0x87, 0xAA, 0xEE, 0xEE, 0x07}
#define BLE_MAX6675_UUID_SERVICE     0x3141
#define BLE_MAX6675_UUID_COM_CHAR    0x3142
#define BLE_MAX6675_UUID_VALUE_CHAR  0x3143

#define BLE_MAX6675_COM_MAX_LEN      6 /* max number of bytes per BLE package */
#define BLE_MAX6675_VALUE_MAX_LEN    sizeof(uint16_t) /* max number of bytes */

/**@brief MAX6675 Service event type. */
typedef enum {
    BLE_MAX6675_EVT_NOTIFICATION_ENABLED,      /**< MAX6675 value notification enabled event. */
    BLE_MAX6675_EVT_NOTIFICATION_DISABLED,     /**< MAX6675 value notification disabled event. */
    BLE_MAX6675_EVT_DISCONNECTED               /**< MAX6675 value notification disconnected event. */
} ble_max6675_evt_type_t;

/**@brief MAX6675 Service event. */
typedef struct {
    ble_max6675_evt_type_t evt_type;                                  /**< Type of event. */
} ble_max6675_evt_t;

// Forward declaration of the ble_max6675_t type.
typedef struct ble_max6675_s ble_max6675_t;

/**@brief MAX6675 Service event handler type. */
typedef void (*ble_max6675_evt_handler_t) (ble_max6675_t * p_max6675, ble_max6675_evt_t *evt);

/**@brief MAX6675 Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct {
    ble_max6675_evt_handler_t         evt_handler;     /**< Event handler to be called for handling events in the MAX6675 Service. */
} ble_max6675_init_t;

typedef enum {
    BLE_MAX6675_COM_START,                             /**< command truncate channel info. */
    BLE_MAX6675_COM_STOP,                              /**< command truncate channel info. */
    BLE_MAX6675_COM_NOTIFY                            /**< command truncate channel info. */
} ble_max6675_command_type_t;

typedef enum {
    BLE_MAX6675_START_STOP,                              /**< command truncate channel info. */
    BLE_MAX6675_START_TIMER,                             /**< command truncate channel info. */
    BLE_MAX6675_START_PWM,                               /**< command truncate channel info. */
    BLE_MAX6675_START_SOFTBLINK,                         /**< command truncate channel info. */
    BLE_MAX6675_START_SERVO
} ble_max6675_start_type_t;

/**@brief MAX6675 Service structure. This contains various status information for the service. */
struct ble_max6675_s {
    uint16_t                      service_handle;                 /**< Handle of MAX6675 Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t      com_char_handles;               /**< Handles related to the adding characteristic. */
    ble_gatts_char_handles_t      value_char_handles;             /**< Handles related to the notifying values characteristic. */
    uint16_t                      report_ref_handle;              /**< Handle of the Report Reference descriptor. */
    uint8_t                       uuid_type;                      /**< UUID type for the MAX6675 Service. */
    uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack). BLE_CONN_HANDLE_INVALID if not in a connection. */
    ble_max6675_evt_handler_t     evt_handler;                    /**< Event handler to be called for handling events in the MAX6675 Service. */
    uint16_t                      last_temp;                      /**< Temperature in 0.25 degrees celsius. */
    uint8_t                       running;                         /**< Temperature in 0.25 degrees celsius. */
};

/**@brief Function for initializing the MAX6675 Service.
 *
 * @param[out]  p_max6675   MAX6675 Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_max6675_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_max6675_init(ble_max6675_t * p_max6675, const ble_max6675_init_t * p_max6675_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the MAX6675 Service.
 *
 * @param[in]   p_max6675  MAX6675 Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_max6675_on_ble_evt(ble_max6675_t * p_max6675, ble_evt_t * p_ble_evt);

/**@brief Function for initializing the MAX6675 Service.
 *
 * @param[in]   p_max6675  MAX6675 Service structure.
 * @param[in]   temp  Temperature in 0.25 degrees celsius.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_max6675_temp_update(ble_max6675_t * p_max6675, uint16_t temp);

#endif // BLE_MAX6675_H__

/** @} */
