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

/** @file ble_si7021.h
 *
 * @brief Relative Humidity and Temperature Service module.
 *
 * @details This module implements the si7021 Service with the si7021 characteristic.
 *          During initialization it adds the si7021 Service and si7021 characteristic
 *          to the BLE stack database.
 *
 *          If specified, the module will support notification of the si7021 characteristic
 *          through the ble_si7021_read() function.
 *          If an event handler is supplied by the application, the si7021 Service will
 *          generate si7021 Service events to the application.
 *
 * @note The application must propagate BLE stack events to the si7021 Service module by calling
 *       ble_si7021_on_ble_evt() from the @ref softdevice_handler callback.
 *
 */

#ifndef BLE_si7021_H__
#define BLE_si7021_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "app_util.h"

// using use linux tool uuidgen or random numbers via https://www.uuidgenerator.net/
#define BLE_si7021_UUID_BASE        {0xa8, 0xb4, 0xc9, 0x4a, 0xad, 0x6c, 0x11, 0xe6, \
                                     0x9a, 0xd8, 0x5d, 0x07, 0xa0, 0x9d, 0x94, 0x6b}


#define BLE_si7021_UUID_SERVICE     0x3144
#define BLE_si7021_UUID_COM_CHAR    0x3145
#define BLE_si7021_UUID_VALUE_CHAR  0x3146

#define BLE_si7021_COM_MAX_LEN      2 /* max number of bytes */
#define BLE_si7021_VALUE_MAX_LEN    4 /* max number of bytes */

/**@brief si7021 Service event type. */
typedef enum {
    BLE_si7021_EVT_NOTIFICATION_ENABLED,      /**< si7021 value notification enabled event. */
    BLE_si7021_EVT_NOTIFICATION_DISABLED,     /**< si7021 value notification disabled event. */
	BLE_si7021_EVT_INTERVAL_CHANGE,           /**< si7021 interval change event. */
    BLE_si7021_EVT_DISCONNECTED               /**< si7021 value notification disconnected event. */
} ble_si7021_evt_type_t;

/**@brief si7021 Service event. */
typedef struct {
    ble_si7021_evt_type_t evt_type;                                  /**< Type of event. */
} ble_si7021_evt_t;

// Forward declaration of the ble_si7021_t type.
typedef struct ble_si7021_s ble_si7021_t;

/**@brief si7021 Service event handler type. */
typedef void (*ble_si7021_evt_handler_t) (ble_si7021_t * p_si7021, ble_si7021_evt_t *evt);

/**@brief si7021 Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct {
    ble_si7021_evt_handler_t         evt_handler;     /**< Event handler to be called for handling events in the si7021 Service. */
} ble_si7021_init_t;

typedef enum {
    BLE_si7021_COM_START,                             /**< command truncate channel info. */
    BLE_si7021_COM_STOP,                              /**< command truncate channel info. */
    BLE_si7021_COM_NOTIFY                            /**< command truncate channel info. */
} ble_si7021_command_type_t;

/**@brief si7021 Service structure. This contains various status information for the service. */
struct ble_si7021_s {
    uint16_t                      service_handle;                 /**< Handle of si7021 Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t      com_char_handles;               /**< Handles related to the adding characteristic. */
    ble_gatts_char_handles_t      value_char_handles;             /**< Handles related to the notifying values characteristic. */
    uint16_t                      report_ref_handle;              /**< Handle of the Report Reference descriptor. */
    uint8_t                       uuid_type;                      /**< UUID type for the si7021 Service. */
    uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack). BLE_CONN_HANDLE_INVALID if not in a connection. */
    ble_si7021_evt_handler_t     evt_handler;                     /**< Event handler to be called for handling events in the si7021 Service. */
    uint16_t                      last_humraw;                    /**< Relative Humidity Raw 16-Bit. */
    uint16_t                      last_tempraw;                   /**< Temperature Raw 16-Bit. */
    uint8_t                       running;                        /**< running indicator. */
    uint16_t                      interval;                       /**< Sensor mesurement intervall. */
};

/**@brief Function for initializing the si7021 Service.
 *
 * @param[out]  p_si7021   si7021 Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_si7021_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_si7021_init(ble_si7021_t * p_si7021, const ble_si7021_init_t * p_si7021_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the si7021 Service.
 *
 * @param[in]   p_si7021  si7021 Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_si7021_on_ble_evt(ble_si7021_t * p_si7021, ble_evt_t * p_ble_evt);

/**@brief Function for initializing the si7021 Service.
 *
 * @param[in]   p_si7021  si7021 Service structure.
 * @param[in]   hum  Relative Humidity Raw 16-Bit.
 * @param[in]   temp Temperature Raw 16-Bit.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_si7021_sensor_update(ble_si7021_t * p_si7021, uint16_t hum, uint16_t temp);

#endif // BLE_si7021_H__

/** @} */
