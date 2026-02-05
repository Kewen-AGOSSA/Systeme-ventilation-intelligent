#include "ble_cscs_c.h"
#include "ble_srv_common.h"
#include "ble_db_discovery.h"   // ✅ pour ble_db_discovery_evt_t
#include "ble_gatt_db.h"        // ✅ pour ble_gatt_db_char_t
#include "nrf_log.h"
#include <string.h>


#define BLE_UUID_CSC_MEASUREMENT_CHAR 0x2A5B

static void on_hvx(ble_cscs_c_t * p_cscs_c, const ble_evt_t * p_ble_evt)
{
    const ble_gattc_evt_hvx_t * p_notif = &p_ble_evt->evt.gattc_evt.params.hvx;
    const uint8_t * data = p_notif->data;
    uint16_t len = p_notif->len;

    if (len < 1) return;

    ble_cscs_c_evt_t evt;
    memset(&evt, 0, sizeof(evt));
    evt.evt_type = BLE_CSCS_C_EVT_CSC_MEASUREMENT;
    evt.conn_handle = p_ble_evt->evt.gattc_evt.conn_handle;

    uint8_t flags = data[0];
    uint8_t index = 1;

    if (flags & 0x01 && len >= index + 6) {
        evt.params.csc_measurement.is_wheel_rev_data_present = true;
        evt.params.csc_measurement.cumulative_wheel_revs = uint32_decode(&data[index]); index += 4;
        evt.params.csc_measurement.last_wheel_event_time = uint16_decode(&data[index]); index += 2;
    }

    if (flags & 0x02 && len >= index + 4) {
        evt.params.csc_measurement.is_crank_rev_data_present = true;
        evt.params.csc_measurement.cumulative_crank_revs = uint16_decode(&data[index]); index += 2;
        evt.params.csc_measurement.last_crank_event_time = uint16_decode(&data[index]);
    }

    if (p_cscs_c->evt_handler != NULL) {
        p_cscs_c->evt_handler(p_cscs_c, &evt);
    }
}

uint32_t ble_cscs_c_init(ble_cscs_c_t * p_cscs_c, ble_cscs_c_init_t * p_cscs_c_init)
{
    VERIFY_PARAM_NOT_NULL(p_cscs_c);
    VERIFY_PARAM_NOT_NULL(p_cscs_c_init);

    p_cscs_c->evt_handler = p_cscs_c_init->evt_handler;
    p_cscs_c->conn_handle = BLE_CONN_HANDLE_INVALID;
    p_cscs_c->cscs_uuid.uuid = BLE_UUID_CYCLING_SPEED_AND_CADENCE;
    p_cscs_c->cscs_uuid.type = BLE_UUID_TYPE_BLE;

    return NRF_SUCCESS;
}

void ble_cscs_c_on_ble_evt(ble_cscs_c_t * p_cscs_c, const ble_evt_t * p_ble_evt)
{
    if ((p_cscs_c == NULL) || (p_ble_evt == NULL)) return;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTC_EVT_HVX:
            on_hvx(p_cscs_c, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            p_cscs_c->conn_handle = BLE_CONN_HANDLE_INVALID;
            if (p_cscs_c->evt_handler)
            {
                ble_cscs_c_evt_t evt = {
                    .evt_type = BLE_CSCS_C_EVT_DISCONNECTED,
                    .conn_handle = p_ble_evt->evt.gap_evt.conn_handle
                };
                p_cscs_c->evt_handler(p_cscs_c, &evt);
            }
            break;

        default:
            break;
    }
}

void ble_cscs_c_on_db_disc_evt(ble_cscs_c_t * p_cscs_c, const ble_db_discovery_evt_t * p_evt)
{
    VERIFY_PARAM_NOT_NULL_VOID(p_cscs_c);
    VERIFY_PARAM_NOT_NULL_VOID(p_evt);

    if (p_evt->evt_type != BLE_DB_DISCOVERY_COMPLETE) return;
    if (p_evt->params.discovered_db.srv_uuid.uuid != BLE_UUID_CYCLING_SPEED_AND_CADENCE) return;

    ble_gatt_db_char_t csc_meas_char = {0};

    for (uint8_t i = 0; i < p_evt->params.discovered_db.char_count; i++)
    {
        ble_gatt_db_char_t char_data = p_evt->params.discovered_db.charateristics[i];
        if (char_data.characteristic.uuid.uuid == BLE_UUID_CSC_MEASUREMENT_CHAR)
        {
            csc_meas_char = char_data;

            p_cscs_c->csc_meas_handle       = char_data.characteristic.handle_value;
            p_cscs_c->csc_meas_cccd_handle  = char_data.cccd_handle;
            p_cscs_c->conn_handle           = p_evt->conn_handle;

            if (p_cscs_c->evt_handler)
            {
                ble_cscs_c_evt_t evt = {
                    .evt_type = BLE_CSCS_C_EVT_DISCOVERY_COMPLETE,
                    .conn_handle = p_evt->conn_handle,
                    .params.cscs_db = csc_meas_char
                };
                p_cscs_c->evt_handler(p_cscs_c, &evt);
            }

            break;
        }
    }
}

uint32_t ble_cscs_c_handles_assign(ble_cscs_c_t * p_cscs_c, uint16_t conn_handle, const ble_gatt_db_char_t * p_cscs_db)
{
    VERIFY_PARAM_NOT_NULL(p_cscs_c);
    VERIFY_PARAM_NOT_NULL(p_cscs_db);

    p_cscs_c->conn_handle           = conn_handle;
    p_cscs_c->csc_meas_handle      = p_cscs_db->characteristic.handle_value;
    p_cscs_c->csc_meas_cccd_handle = p_cscs_db->cccd_handle;

    return NRF_SUCCESS;
}

uint32_t ble_cscs_c_measurement_notif_enable(ble_cscs_c_t * p_cscs_c)
{
    VERIFY_PARAM_NOT_NULL(p_cscs_c);

    uint8_t cccd_val[2] = {BLE_GATT_HVX_NOTIFICATION, 0};

    ble_gattc_write_params_t write_params = {
        .write_op = BLE_GATT_OP_WRITE_REQ,
        .flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE,
        .handle   = p_cscs_c->csc_meas_cccd_handle,
        .offset   = 0,
        .len      = sizeof(cccd_val),
        .p_value  = cccd_val
    };

    return sd_ble_gattc_write(p_cscs_c->conn_handle, &write_params);
}
