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

#include <string.h>
#include "ble_max6675.h"
#include "nordic_common.h"
#include "ble_types.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "app_error.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

/**@brief Function for handling the Connect event.
 *
 * @param[in] p_max6675  MAX6675 Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_connect(ble_max6675_t * p_max6675, ble_evt_t * p_ble_evt) {
    p_max6675->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in] p_max6675  MAX6675 Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_disconnect(ble_max6675_t * p_max6675, ble_evt_t * p_ble_evt) {
    UNUSED_PARAMETER(p_ble_evt);
    p_max6675->conn_handle = BLE_CONN_HANDLE_INVALID;
    if (p_max6675->evt_handler != NULL) {
    	ble_max6675_evt_t evt;
        evt.evt_type = BLE_MAX6675_EVT_DISCONNECTED;
        p_max6675->evt_handler(p_max6675, &evt);
    }
}

/**@brief Function for handling the Disconnect event.
 *
 * @param[in] p_max6675  MAX6675 Service structure.
 * @param[in] temp  Temperature in 0.25 degrees celsius.
 */
uint32_t ble_max6675_temp_update(ble_max6675_t * p_max6675, uint16_t temp) {
	uint8_t encoded_temp[BLE_MAX6675_VALUE_MAX_LEN];
	//uint8_t len;

    if (p_max6675 == NULL) {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    if (temp != p_max6675->last_temp && temp != 0) {
    	NRF_LOG_DEBUG("ble_max6675_temp_update %u\n\r",temp);

	    //len = uint16_encode(temp,encoded_temp);
	    uint16_encode(temp,encoded_temp);

        // Initialize value struct.
        memset(&gatts_value, 0, sizeof(gatts_value));

        gatts_value.len     = BLE_MAX6675_VALUE_MAX_LEN;
        gatts_value.offset  = 0;
        gatts_value.p_value = &encoded_temp[0];

        // Update database. Set the value of a given attribute
        err_code = sd_ble_gatts_value_set(p_max6675->conn_handle,
        		                          p_max6675->value_char_handles.value_handle,
                                          &gatts_value);
        if (err_code == NRF_SUCCESS) {
            // save new temperature value.
        	p_max6675->last_temp = temp;
        } else {
            return err_code;
        }

        // Send value if connected and notifying.
        if (p_max6675->conn_handle != BLE_CONN_HANDLE_INVALID) {
            ble_gatts_hvx_params_t hvx_params;

            memset(&hvx_params, 0, sizeof(hvx_params));

            hvx_params.handle = p_max6675->value_char_handles.value_handle;
            hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset = gatts_value.offset;
            hvx_params.p_len  = &gatts_value.len;
            hvx_params.p_data = gatts_value.p_value;

            // Notify or Indicate an attribute value
            err_code = sd_ble_gatts_hvx(p_max6675->conn_handle, &hvx_params);
        } else {
            err_code = NRF_ERROR_INVALID_STATE;
        }
    }

    return err_code;
}


/**@brief Function for handling the Write event.
 *
 * @param[in] p_max6675  MAX6675 Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_write(ble_max6675_t * p_max6675, ble_evt_t * p_ble_evt) {
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_evt_write->handle == p_max6675->value_char_handles.cccd_handle) {
        if (p_evt_write->len == 2 && p_max6675->evt_handler != NULL) {
        	ble_max6675_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data)) {
                evt.evt_type = BLE_MAX6675_EVT_NOTIFICATION_ENABLED;
            } else {
                evt.evt_type = BLE_MAX6675_EVT_NOTIFICATION_DISABLED;
            }
            p_max6675->evt_handler(p_max6675, &evt);
        }
	} else {
			// No implementation needed.
	}
}

/**@brief Function for handling the ble events.
 *
 * @param[in] p_max6675  MAX6675 Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
void ble_max6675_on_ble_evt(ble_max6675_t * p_max6675, ble_evt_t * p_ble_evt) {

    switch (p_ble_evt->header.evt_id) {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_max6675, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_max6675, p_ble_evt);
            break;
            
        case BLE_GATTS_EVT_WRITE:
            on_write(p_max6675, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for adding the notify value Characteristic.
 *
 * @param[in] p_max6675      MAX6675 Service structure.
 * @param[in] p_max6675_init MAX6675 Service initialization structure.
 *
 * @retval NRF_SUCCESS on success, else an error value from the SoftDevice
 */
static uint32_t ble_max6675_value_char_add(ble_max6675_t * p_max6675, const ble_max6675_init_t * p_max6675_init) {
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    
    memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.read   = 1;
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    //BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_TEMPERATURE_TYPE_CHAR);
    ble_uuid.type = p_max6675->uuid_type;
    ble_uuid.uuid = BLE_MAX6675_UUID_VALUE_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = BLE_MAX6675_VALUE_MAX_LEN;
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = BLE_MAX6675_VALUE_MAX_LEN;
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_max6675->service_handle,
                                               &char_md,
                                               &attr_char_value,
                                               &p_max6675->value_char_handles);
}

uint32_t ble_max6675_init(ble_max6675_t * p_max6675, const ble_max6675_init_t * p_max6675_init) {
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure.
    p_max6675->conn_handle = BLE_CONN_HANDLE_INVALID;
    p_max6675->evt_handler = p_max6675_init->evt_handler;
    p_max6675->last_temp = 0;
    p_max6675->running = 0;

    // Add a Vendor Specific UUID 128-Bit
    ble_uuid128_t base_uuid = {BLE_MAX6675_UUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_max6675->uuid_type);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    ble_uuid.type = p_max6675->uuid_type;
    ble_uuid.uuid = BLE_MAX6675_UUID_SERVICE;

    // Add a service declaration to the Attribute Table
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_max6675->service_handle);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    //
    err_code = ble_max6675_value_char_add(p_max6675, p_max6675_init);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }
    return NRF_SUCCESS;
}
