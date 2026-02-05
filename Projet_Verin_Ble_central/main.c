#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

// Includes Nordic SDK
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h" 
#include "nrf_pwr_mgmt.h"
#include "app_timer.h"
#include "boards.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
#include "ble_lbs_c.h"
#include "ble_nus.h" 
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_scan.h"
#include "nrfx_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "app_button.h" 

// Include Capteur
#include "bme68x.h"

// ==========================================
// 1. CONFIGURATION
// ==========================================

#define DEVICE_NAME_PHONE       "AirMonitor"
#define BUTTON_VERIN_PIN        2   
#define BUTTON_DETECTION_DELAY  50
#define APP_BLE_CONN_CFG_TAG    1
#define APP_BLE_OBSERVER_PRIO   3

// I2C
#define TWI_INSTANCE_ID         0
#define BME68X_I2C_ADDR         0x76 
#define PIN_SCL                 27
#define PIN_SDA                 26

// Commandes (Entiers)
#define COMMAND_AVANCE  1
#define COMMAND_RECULE  2
#define COMMAND_STOP    3

// Logique Air
#define SEUIL_POLLUTION     50    
#define SENSOR_INTERVAL_MS  3000  
#define COOLDOWN_CYCLES     10    

// Paramètres de Connexion
#define MIN_CONN_INTERVAL       MSEC_TO_UNITS(100, UNIT_1_25_MS) 
#define MAX_CONN_INTERVAL       MSEC_TO_UNITS(200, UNIT_1_25_MS) 
#define SLAVE_LATENCY           0
#define CONN_SUP_TIMEOUT        MSEC_TO_UNITS(4000, UNIT_10_MS)
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)
#define MAX_CONN_PARAMS_UPDATE_COUNT    3

typedef enum {
    STATE_STOPPED_AVANCE,   
    STATE_ADVANCING,        
    STATE_STOPPED_RECULE,   
    STATE_RECULING          
} verin_control_state_t;

// ==========================================
// 2. VARIABLES GLOBALES
// ==========================================

static uint16_t m_conn_handle_verin = BLE_CONN_HANDLE_INVALID;
static uint16_t m_conn_handle_phone = BLE_CONN_HANDLE_INVALID;

static char const m_target_periph_name[] = "Verin_BLE";
static volatile bool button_event_flag = false;
static verin_control_state_t verin_state = STATE_STOPPED_AVANCE;
static bool m_service_verin_ready = false; 
static bool m_nus_ready = false; 

// Capteur
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
static struct bme68x_dev bme_sensor;
static uint8_t m_dev_addr = BME68X_I2C_ADDR; 
static uint32_t gas_baseline = 0;
static uint32_t m_manual_cooldown = 0;

// Timer
APP_TIMER_DEF(m_sensor_timer_id);
static volatile bool m_check_sensor_flag = false;

// Instances BLE
NRF_BLE_SCAN_DEF(m_scan);
BLE_LBS_C_DEF(m_ble_lbs_c);
BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);
NRF_BLE_GATT_DEF(m_gatt);
BLE_DB_DISCOVERY_DEF(m_db_disc);
BLE_ADVERTISING_DEF(m_advertising);

static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, BLE_UUID_TYPE_BLE}};

// ==========================================
// 3. FONCTIONS UTILITAIRES
// ==========================================

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name) {
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

static void sensor_timer_handler(void * p_context) {
    m_check_sensor_flag = true;
    if (m_manual_cooldown > 0) m_manual_cooldown--;
}

int8_t user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    uint8_t dev_addr = *(uint8_t *)intf_ptr;
    nrf_drv_twi_tx(&m_twi, dev_addr, &reg_addr, 1, true);
    return (nrf_drv_twi_rx(&m_twi, dev_addr, reg_data, len) == NRF_SUCCESS) ? 0 : -1;
}

int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    uint8_t dev_addr = *(uint8_t *)intf_ptr;
    uint8_t buffer[32];
    if (len + 1 > sizeof(buffer)) return -1;
    buffer[0] = reg_addr;
    memcpy(&buffer[1], reg_data, len);
    return (nrf_drv_twi_tx(&m_twi, dev_addr, buffer, len + 1, false) == NRF_SUCCESS) ? 0 : -1;
}

void user_delay_us(uint32_t period, void *intf_ptr) { nrf_delay_us(period); }

// ==========================================
// 4. LOGIQUE METIER & COMMS
// ==========================================

int calculate_iaq(uint32_t gas_res, float hum) {
    if (gas_res > gas_baseline && gas_res > 50000) gas_baseline = gas_res;
    if (gas_baseline == 0) return 50; 

    float gas_score = 0;
    float gas_ratio = (float)(gas_baseline - gas_res) / (float)gas_baseline;
    if (gas_ratio < 0) gas_ratio = 0; 
    gas_score = (1.0f - gas_ratio) * 100.0f;

    float hum_score = 0;
    float diff = 0;
    if (hum >= 38 && hum <= 42) hum_score = 100.0f;
    else {
        diff = (hum < 38) ? (38 - hum) : (hum - 42);
        hum_score = 100.0f - (diff * 2.5f);
        if (hum_score < 0) hum_score = 0;
    }
    return (int)((gas_score * 0.75f) + (hum_score * 0.25f));
}

void send_to_phone(const char* p_string)
{
    if (m_conn_handle_phone != BLE_CONN_HANDLE_INVALID && m_nus_ready)
    {
        uint16_t length = (uint16_t)strlen(p_string);
        ret_code_t err;
        do {
            err = ble_nus_data_send(&m_nus, (uint8_t *)p_string, &length, m_conn_handle_phone);
        } while (err == NRF_ERROR_BUSY);
    }
}

void send_verin_command(uint8_t cmd, char* reason)
{
    ret_code_t err_code;
    char buffer[64];

    if (m_ble_lbs_c.conn_handle != BLE_CONN_HANDLE_INVALID && m_service_verin_ready)
    {
        // Envoi de l'entier (cmd) directement via LBS
        err_code = ble_lbs_led_status_send(&m_ble_lbs_c, cmd);
        
        if (err_code == NRF_SUCCESS) {
            NRF_LOG_INFO("CMD %d Envoyee (%s)", cmd, reason);
            sprintf(buffer, "Action: CMD %d (%s)\r\n", cmd, reason);
            send_to_phone(buffer);
        }
        else {
            NRF_LOG_WARNING("Echec CMD %d: Err 0x%X", cmd, err_code);
        }
    }
}

// ==========================================
// 5. GESTION BLE (RECEPTION ENTIER)
// ==========================================

static void scan_start(void) {
    nrf_ble_scan_start(&m_scan);
}

static void nus_data_handler(ble_nus_evt_t * p_evt) {
    if (p_evt->type == BLE_NUS_EVT_COMM_STARTED) {
        NRF_LOG_INFO("NUS: Notifications Activees.");
        m_nus_ready = true;
        uint16_t len = 20;
        ble_nus_data_send(&m_nus, (uint8_t *)"AirMonitor Connecte\r\n", &len, m_conn_handle_phone);
    }
    else if (p_evt->type == BLE_NUS_EVT_COMM_STOPPED) {
        NRF_LOG_INFO("NUS: Notifications Arretees.");
        m_nus_ready = false;
    }
    else if (p_evt->type == BLE_NUS_EVT_RX_DATA) {
        // CORRECTION TYPE: 'const uint8_t *' pour éviter l'erreur de compilation
        const uint8_t * p_data = p_evt->params.rx_data.p_data;
        uint16_t length = p_evt->params.rx_data.length;

        NRF_LOG_INFO("Recu du tel: %.*s", length, p_data);

        if (length > 0)
        {
            m_manual_cooldown = COOLDOWN_CYCLES;

            // RECUPERATION DE LA VALEUR
            uint8_t received_val = p_data[0];

            // CONVERSION ASCII -> ENTIER
            // Si le téléphone envoie le caractère '1' (0x31), on le transforme en 1 (0x01)
            // Si le téléphone envoie déjà 1 (0x01), cette condition est fausse et on garde 1.
            if (received_val >= '0' && received_val <= '9') {
                received_val = received_val - '0';
            }

            // SWITCH SUR L'ENTIER (Comme "led_write_handler")
            switch(received_val)
            {
                case 1: // Entier 1
                    send_verin_command(COMMAND_AVANCE, "App Mobile");
                    verin_state = STATE_ADVANCING; 
                    break;

                case 2: // Entier 2
                    send_verin_command(COMMAND_RECULE, "App Mobile");
                    verin_state = STATE_RECULING;
                    break;

                case 3: // Entier 3
                    send_verin_command(COMMAND_STOP, "App Mobile");
                    verin_state = STATE_STOPPED_AVANCE; // Ou neutre
                    break;

                default:
                    send_to_phone("CMD Inconnue (1, 2 ou 3)\r\n");
                    break;
            }
        }
    }
}

static void lbs_c_evt_handler(ble_lbs_c_t * p_lbs_c, ble_lbs_c_evt_t * p_lbs_c_evt) {
    if (p_lbs_c_evt->evt_type == BLE_LBS_C_EVT_DISCOVERY_COMPLETE) {
        ble_lbs_c_handles_assign(&m_ble_lbs_c, p_lbs_c_evt->conn_handle, &p_lbs_c_evt->params.peer_db);
        ble_lbs_c_button_notif_enable(p_lbs_c);
        m_service_verin_ready = true;
        NRF_LOG_INFO("VERIN CONNECTE ET PRET.");
        send_to_phone("Info: Verin Connecte\r\n");
    }
}

static void on_adv_evt(ble_adv_evt_t ble_adv_evt) {
    switch (ble_adv_evt) {
        case BLE_ADV_EVT_FAST: NRF_LOG_INFO("Adv: FAST"); break;
        case BLE_ADV_EVT_IDLE: NRF_LOG_INFO("Adv: IDLE"); break;
        default: break;
    }
}

static void on_conn_params_evt(ble_conn_params_evt_t * p_evt) {
    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
        // Pas de déconnexion forcée
    }
}
static void conn_params_error_handler(uint32_t nrf_error) { APP_ERROR_HANDLER(nrf_error); }

static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context) {
    switch (p_ble_evt->header.evt_id) {
        case BLE_GAP_EVT_CONNECTED:
            if (p_ble_evt->evt.gap_evt.params.connected.role == BLE_GAP_ROLE_PERIPH) {
                NRF_LOG_INFO("TELEPHONE CONNECTE !");
                m_conn_handle_phone = p_ble_evt->evt.gap_evt.conn_handle;
            } 
            else {
                NRF_LOG_INFO("Verin connecte. Init services...");
                m_conn_handle_verin = p_ble_evt->evt.gap_evt.conn_handle;
                ble_db_discovery_start(&m_db_disc, m_conn_handle_verin);
                
                if (m_conn_handle_phone == BLE_CONN_HANDLE_INVALID) {
                    ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
                }
            }
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            if (p_ble_evt->evt.gap_evt.conn_handle == m_conn_handle_phone) {
                NRF_LOG_INFO("Telephone deconnecte.");
                m_conn_handle_phone = BLE_CONN_HANDLE_INVALID;
                m_nus_ready = false; 
                ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
            }
            else if (p_ble_evt->evt.gap_evt.conn_handle == m_conn_handle_verin) {
                NRF_LOG_INFO("Verin deconnecte.");
                m_conn_handle_verin = BLE_CONN_HANDLE_INVALID;
                m_service_verin_ready = false;
                scan_start(); 
            }
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            break;
            
        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            ble_gap_phys_t const phys = {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
        } break;
    }
}

static void db_disc_handler(ble_db_discovery_evt_t * p_evt) {
    ble_lbs_on_db_disc_evt(&m_ble_lbs_c, p_evt);
}

static void scan_evt_handler(scan_evt_t const * p_scan_evt) {
    if (p_scan_evt->scan_evt_id == NRF_BLE_SCAN_EVT_CONNECTING_ERROR)
        APP_ERROR_CHECK(p_scan_evt->params.connecting_err.err_code);
}

// ==========================================
// 6. INITS
// ==========================================

static void ble_stack_init(void) {
    ret_code_t err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

static void services_init(void) {
    ble_nus_init_t nus_init = {0};
    nus_init.data_handler = nus_data_handler;
    APP_ERROR_CHECK(ble_nus_init(&m_nus, &nus_init));

    ble_lbs_c_init_t lbs_c_init_obj;
    lbs_c_init_obj.evt_handler = lbs_c_evt_handler;
    APP_ERROR_CHECK(ble_lbs_c_init(&m_ble_lbs_c, &lbs_c_init_obj));
}

static void conn_params_init(void)
{
    ret_code_t err_code;
    ble_conn_params_init_t cp_init;

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

static void advertising_init(void) {
    ret_code_t err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));
    init.advdata.name_type = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    
    m_adv_uuids[0].type = m_nus.uuid_type; 
    init.srdata.uuids_complete.uuid_cnt = 1;
    init.srdata.uuids_complete.p_uuids = m_adv_uuids;

    init.config.ble_adv_fast_enabled = true;
    init.config.ble_adv_fast_interval = 64;
    init.config.ble_adv_fast_timeout = 0; // Infini
    
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);
    
    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

static void twi_init(void) {
    const nrf_drv_twi_config_t config = {
        .scl = PIN_SCL, .sda = PIN_SDA, .frequency = NRF_DRV_TWI_FREQ_100K,
        .interrupt_priority = 6, .clear_bus_init = false 
    };
    APP_ERROR_CHECK(nrf_drv_twi_init(&m_twi, &config, NULL, NULL));
    nrf_drv_twi_enable(&m_twi);
}

static void gatt_init(void) { APP_ERROR_CHECK(nrf_ble_gatt_init(&m_gatt, NULL)); }
static void db_discovery_init(void) { APP_ERROR_CHECK(ble_db_discovery_init(db_disc_handler)); }

static void scan_init(void) {
    nrf_ble_scan_init_t init_scan = {0};
    init_scan.connect_if_match = true;
    init_scan.conn_cfg_tag = APP_BLE_CONN_CFG_TAG;
    APP_ERROR_CHECK(nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler));
    APP_ERROR_CHECK(nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER, false));
    APP_ERROR_CHECK(nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, m_target_periph_name));
}

static void button_event_handler(uint8_t pin_no, uint8_t button_action) {
    if (pin_no == BUTTON_VERIN_PIN && button_action == APP_BUTTON_PUSH) button_event_flag = true;
}

static void buttons_init(void) {
    if (!nrfx_gpiote_is_init()) APP_ERROR_CHECK(nrfx_gpiote_init());
    static app_button_cfg_t buttons[] = {{BUTTON_VERIN_PIN, APP_BUTTON_ACTIVE_LOW, NRF_GPIO_PIN_PULLUP, button_event_handler}};
    APP_ERROR_CHECK(app_button_init(buttons, ARRAY_SIZE(buttons), BUTTON_DETECTION_DELAY));
    APP_ERROR_CHECK(app_button_enable());
}

static void log_init(void) {
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}
static void power_management_init(void) { APP_ERROR_CHECK(nrf_pwr_mgmt_init()); }

static void gap_params_init(void) {
    ble_gap_conn_params_t gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)DEVICE_NAME_PHONE, strlen(DEVICE_NAME_PHONE));

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));
    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;
    sd_ble_gap_ppcp_set(&gap_conn_params);
}

// ==========================================
// 7. MAIN
// ==========================================

int main(void)
{
    log_init();
    app_timer_init();
    buttons_init();
    power_management_init();
    
    ble_stack_init();
    gap_params_init(); 
    gatt_init();
    db_discovery_init();
    
    services_init();
    scan_init();
    conn_params_init(); 
    advertising_init(); 

    twi_init();
    bme_sensor.intf = BME68X_I2C_INTF;
    bme_sensor.read = user_i2c_read;
    bme_sensor.write = user_i2c_write;
    bme_sensor.delay_us = user_delay_us;
    bme_sensor.intf_ptr = &m_dev_addr;
    bme_sensor.amb_temp = 25;

    if (bme68x_init(&bme_sensor) != BME68X_OK) {
        NRF_LOG_ERROR("ERREUR BME68X.");
    } else {
        struct bme68x_conf conf = {BME68X_OS_8X, BME68X_OS_4X, BME68X_OS_2X, BME68X_FILTER_SIZE_3, NRFX_PDM_CONFIG_EDGE};
        bme68x_set_conf(&conf, &bme_sensor);
        struct bme68x_heatr_conf heatr_conf = {BME68X_ENABLE, 320, 150};
        bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme_sensor);
    }

    app_timer_create(&m_sensor_timer_id, APP_TIMER_MODE_REPEATED, sensor_timer_handler);
    app_timer_start(m_sensor_timer_id, APP_TIMER_TICKS(SENSOR_INTERVAL_MS), NULL);

    NRF_LOG_INFO("=== CENTRAL MULTIROLE PRET ===");
    
    scan_start();
    ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);

    for (;;)
    {
        // 1. LECTURE DU CAPTEUR
        if (m_check_sensor_flag)
        {
            m_check_sensor_flag = false;
            bme68x_set_op_mode(BME68X_FORCED_MODE, &bme_sensor);
            nrf_delay_ms(200); 

            struct bme68x_data data;
            uint8_t n_fields;
            if (bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &bme_sensor) == BME68X_OK && n_fields > 0)
            {
                int iaq = calculate_iaq(data.gas_resistance, data.humidity);
                int temp = (int)data.temperature;
                
                NRF_LOG_INFO("IAQ: %d %% (Seuil < %d)", iaq, SEUIL_POLLUTION);
                
                char phone_msg[64];
                sprintf(phone_msg, "T:%dC H:%d%% IAQ:%d\r\n", temp, (int)data.humidity, iaq);
                send_to_phone(phone_msg);

                // AUTOMATISME
                if (iaq < SEUIL_POLLUTION)
                {
                    if (m_manual_cooldown == 0 && verin_state != STATE_ADVANCING)
                    {
                        send_verin_command(COMMAND_AVANCE, "AUTO POLLUTION");
                        verin_state = STATE_ADVANCING;
                        send_to_phone("ALERTE: OUVERTURE AUTO\r\n");
                    }
                }
            }
        }

        // 2. GESTION DU BOUTON PHYSIQUE
        if (button_event_flag)
        {
            button_event_flag = false;
            m_manual_cooldown = COOLDOWN_CYCLES; 
            
            switch (verin_state)
            {
                case STATE_STOPPED_AVANCE:
                    verin_state = STATE_ADVANCING;
                    send_verin_command(COMMAND_AVANCE, "Bouton");
                    break;
                case STATE_ADVANCING:
                    verin_state = STATE_STOPPED_RECULE; 
                    send_verin_command(COMMAND_STOP, "Bouton");
                    break;
                case STATE_STOPPED_RECULE:
                    verin_state = STATE_RECULING;
                    send_verin_command(COMMAND_RECULE, "Bouton");
                    break;
                case STATE_RECULING:
                    verin_state = STATE_STOPPED_AVANCE; 
                    send_verin_command(COMMAND_STOP, "Bouton");
                    break;
                default:
                    verin_state = STATE_STOPPED_AVANCE; 
                    send_verin_command(COMMAND_STOP, "Reset");
                    break;
            }
        }

        NRF_LOG_FLUSH();
        nrf_pwr_mgmt_run(); 
    }
}