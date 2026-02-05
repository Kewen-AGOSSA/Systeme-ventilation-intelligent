#ifndef BLE_CSCS_C_H__
#define BLE_CSCS_C_H__

#include <stdint.h>
#include "ble.h"
#include "ble_gattc.h"
#include "ble_types.h"
#include "ble_gatt_db.h"      // Pour ble_gatt_db_char_t
#include "ble_srv_common.h"   // Pour BLE_UUID_CYCLING_SPEED_AND_CADENCE


#define BLE_CSCS_C_DEF(_name) \
    static ble_cscs_c_t _name;

typedef enum
{
    BLE_CSCS_C_EVT_DISCOVERY_COMPLETE,
    BLE_CSCS_C_EVT_CSC_MEASUREMENT,
    BLE_CSCS_C_EVT_DISCONNECTED,
} ble_cscs_c_evt_type_t;

typedef struct
{
    bool     is_wheel_rev_data_present;
    bool     is_crank_rev_data_present;
    uint32_t cumulative_wheel_revs;
    uint16_t last_wheel_event_time;
    uint16_t cumulative_crank_revs;
    uint16_t last_crank_event_time;
} ble_cscs_c_meas_t;

typedef struct
{
    ble_cscs_c_evt_type_t evt_type;
    uint16_t              conn_handle;
    union
    {
        ble_cscs_c_meas_t csc_measurement;
        ble_gatt_db_char_t cscs_db;
    } params;
} ble_cscs_c_evt_t;

typedef struct ble_cscs_c_s ble_cscs_c_t;

typedef void (*ble_cscs_c_evt_handler_t)(ble_cscs_c_t * p_cscs_c, ble_cscs_c_evt_t const * p_evt);

typedef struct
{
    ble_cscs_c_evt_handler_t evt_handler;
} ble_cscs_c_init_t;

struct ble_cscs_c_s
{
    ble_cscs_c_evt_handler_t evt_handler;
    uint16_t                 conn_handle;
    uint16_t                 csc_meas_cccd_handle;
    uint16_t                 csc_meas_handle;
    ble_uuid_t               cscs_uuid;
};

// === Prototypes à déclarer ici ===

uint32_t ble_cscs_c_init(ble_cscs_c_t * p_cscs_c, ble_cscs_c_init_t * p_cscs_c_init);
void     ble_cscs_c_on_ble_evt(ble_cscs_c_t * p_cscs_c, ble_evt_t const * p_ble_evt);
uint32_t ble_cscs_c_handles_assign(ble_cscs_c_t * p_cscs_c, uint16_t conn_handle, const ble_gatt_db_char_t * p_cscs_db);
uint32_t ble_cscs_c_measurement_notif_enable(ble_cscs_c_t * p_cscs_c);
void     ble_cscs_c_on_db_disc_evt(ble_cscs_c_t * p_cscs_c, const ble_db_discovery_evt_t * p_evt); // à ajouter si utilisée

#endif // BLE_CSCS_C_H__
