#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

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
#include "nrf_ble_gatt.h"
#include "nrf_ble_scan.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_drv_pwm.h"
#include "nrf_gpio.h"

// --------------------------------------------------------------------
// CONFIGURATION
// --------------------------------------------------------------------
#define CENTRAL_SCANNING_LED    BSP_BOARD_LED_0
#define CENTRAL_CONNECTED_LED   BSP_BOARD_LED_2
#define VERIN_ACTIVE_LED        BSP_BOARD_LED_1

#define BUTTON_VERIN_TOGGLE     BSP_BUTTON_0

#define SCAN_INTERVAL           0x00A0
#define SCAN_WINDOW             0x0050
#define SCAN_DURATION           0x0000

#define MIN_CONNECTION_INTERVAL MSEC_TO_UNITS(7.5, UNIT_1_25_MS)
#define MAX_CONNECTION_INTERVAL MSEC_TO_UNITS(30, UNIT_1_25_MS)
#define SLAVE_LATENCY           0
#define SUPERVISION_TIMEOUT     MSEC_TO_UNITS(4000, UNIT_10_MS)

#define APP_BLE_CONN_CFG_TAG    1
#define APP_BLE_OBSERVER_PRIO   3

static char const m_target_periph_name[] = "Verin_BLE";

// --------------------------------------------------------------------
// PWM / VÉRIN CONFIG
// --------------------------------------------------------------------
#define PWM_PIN   24
#define DIR_PIN1  22
#define DIR_PIN2  23

static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);
static bool verin_active = false;
static bool verin_forward = true;

// --------------------------------------------------------------------
// BLE OBJETS
// --------------------------------------------------------------------
NRF_BLE_SCAN_DEF(m_scan);
BLE_LBS_C_DEF(m_ble_lbs_c);
NRF_BLE_GATT_DEF(m_gatt);
BLE_DB_DISCOVERY_DEF(m_db_disc);

// --------------------------------------------------------------------
// ASSERT / LOG / INIT
// --------------------------------------------------------------------
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

static void leds_init(void)
{
    bsp_board_init(BSP_INIT_LEDS);
}

// --------------------------------------------------------------------
// PWM & VERIN COMMANDS
// --------------------------------------------------------------------
static void pwm_init(void)
{
    nrf_drv_pwm_config_t const config = {
        .output_pins = {PWM_PIN, NRF_DRV_PWM_PIN_NOT_USED,
                        NRF_DRV_PWM_PIN_NOT_USED, NRF_DRV_PWM_PIN_NOT_USED},
        .irq_priority = 6,
        .base_clock   = NRF_PWM_CLK_1MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = 1000,
        .load_mode    = NRF_PWM_LOAD_COMMON,
        .step_mode    = NRF_PWM_STEP_AUTO
    };
    APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm0, &config, NULL));
}

static void pwm_set_duty(uint16_t duty)
{
    static nrf_pwm_values_common_t seq_values[1];
    seq_values[0] = duty;

    nrf_pwm_sequence_t seq = {
        .values.p_common = seq_values,
        .length         = 1,
        .repeats        = 0,
        .end_delay      = 0
    };

    nrf_drv_pwm_simple_playback(&m_pwm0, &seq, 1, NRF_DRV_PWM_FLAG_LOOP);
}

static void set_direction(bool forward)
{
    nrf_gpio_pin_write(DIR_PIN1, forward ? 1 : 0);
    nrf_gpio_pin_write(DIR_PIN2, forward ? 0 : 1);
}

static void verin_control(bool activate)
{
    verin_active = activate;
    if (verin_active)
    {
        set_direction(verin_forward);
        pwm_set_duty(500);
        bsp_board_led_on(VERIN_ACTIVE_LED);
        NRF_LOG_INFO("Vérin en mouvement (%s)", verin_forward ? "EXTENSION" : "RETRACTION");
    }
    else
    {
        pwm_set_duty(0);
        bsp_board_led_off(VERIN_ACTIVE_LED);
        NRF_LOG_INFO("Vérin arrêté");
        verin_forward = !verin_forward; // Inverser la prochaine direction
    }
}

// --------------------------------------------------------------------
// BLE SCAN / CONNEXION
// --------------------------------------------------------------------
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    if (p_scan_evt->scan_evt_id == NRF_BLE_SCAN_EVT_CONNECTING_ERROR)
    {
        APP_ERROR_CHECK(p_scan_evt->params.connecting_err.err_code);
    }
}

static void scan_start(void)
{
    ret_code_t err_code = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(err_code);

    bsp_board_led_off(CENTRAL_CONNECTED_LED);
    bsp_board_led_on(CENTRAL_SCANNING_LED);
}

static void lbs_c_evt_handler(ble_lbs_c_t * p_lbs_c, ble_lbs_c_evt_t * p_lbs_c_evt)
{
    switch (p_lbs_c_evt->evt_type)
    {
        case BLE_LBS_C_EVT_DISCOVERY_COMPLETE:
        {
            ret_code_t err_code = ble_lbs_c_handles_assign(&m_ble_lbs_c,
                                                           p_lbs_c_evt->conn_handle,
                                                           &p_lbs_c_evt->params.peer_db);
            APP_ERROR_CHECK(err_code);

            err_code = ble_lbs_c_button_notif_enable(p_lbs_c);
            APP_ERROR_CHECK(err_code);

            bsp_board_led_on(CENTRAL_CONNECTED_LED);
            bsp_board_led_off(CENTRAL_SCANNING_LED);

            NRF_LOG_INFO("Service vérin trouvé. Connexion établie.");
        } break;

        case BLE_LBS_C_EVT_BUTTON_NOTIFICATION:
            NRF_LOG_INFO("Notification du périphérique: bouton = %d",
                         p_lbs_c_evt->params.button.button_state);
            // Active ou désactive le vérin selon la notification
            verin_control(p_lbs_c_evt->params.button.button_state);
            break;

        default:
            break;
    }
}

static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connecté au vérin BLE.");
            err_code = ble_lbs_c_handles_assign(&m_ble_lbs_c,
                                                p_ble_evt->evt.gap_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);

            err_code = ble_db_discovery_start(&m_db_disc,
                                              p_ble_evt->evt.gap_evt.conn_handle);
            APP_ERROR_CHECK(err_code);

            bsp_board_led_on(CENTRAL_CONNECTED_LED);
            bsp_board_led_off(CENTRAL_SCANNING_LED);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Déconnecté. Relance du scan...");
            scan_start();
            bsp_board_led_off(CENTRAL_CONNECTED_LED);
            break;

        default:
            break;
    }
}

// --------------------------------------------------------------------
// INIT BLE STACK
// --------------------------------------------------------------------
static void ble_stack_init(void)
{
    ret_code_t err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO,
                         ble_evt_handler, NULL);
}

static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}

static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_lbs_on_db_disc_evt(&m_ble_lbs_c, p_evt);
}

static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}

static void lbs_c_init(void)
{
    ble_lbs_c_init_t lbs_c_init_obj;
    lbs_c_init_obj.evt_handler = lbs_c_evt_handler;
    ret_code_t err_code = ble_lbs_c_init(&m_ble_lbs_c, &lbs_c_init_obj);
    APP_ERROR_CHECK(err_code);
}

static void scan_init(void)
{
    nrf_ble_scan_init_t init_scan = {0};
    init_scan.connect_if_match = true;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

    ret_code_t err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER, false);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, m_target_periph_name);
    APP_ERROR_CHECK(err_code);
}

// --------------------------------------------------------------------
// BOUTON LOCAL
// --------------------------------------------------------------------
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    if (pin_no == BUTTON_VERIN_TOGGLE && button_action == APP_BUTTON_PUSH)
    {
        verin_control(!verin_active);

        // Optionnel : notifier le périphérique du changement
        ret_code_t err_code = ble_lbs_led_status_send(&m_ble_lbs_c, verin_active);
        if (err_code != NRF_SUCCESS &&
            err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
            err_code != NRF_ERROR_INVALID_STATE)
        {
            APP_ERROR_CHECK(err_code);
        }
    }
}

static void buttons_init(void)
{
    static app_button_cfg_t buttons[] =
    {
        {BUTTON_VERIN_TOGGLE, false, BUTTON_PULL, button_event_handler}
    };

    ret_code_t err_code = app_button_init(buttons, ARRAY_SIZE(buttons), APP_TIMER_TICKS(50));
    APP_ERROR_CHECK(err_code);
    err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);
}

// --------------------------------------------------------------------
// SYSTEM INIT
// --------------------------------------------------------------------
static void log_init(void)
{
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

static void power_management_init(void)
{
    ret_code_t err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

static void idle_state_handle(void)
{
    NRF_LOG_FLUSH();
    nrf_pwr_mgmt_run();
}

// --------------------------------------------------------------------
// MAIN
// --------------------------------------------------------------------
int main(void)
{
    log_init();
    leds_init();
    app_timer_init();
    buttons_init();
    power_management_init();

    nrf_gpio_cfg_output(DIR_PIN1);
    nrf_gpio_cfg_output(DIR_PIN2);
    pwm_init();
    set_direction(verin_forward);
    pwm_set_duty(0);

    ble_stack_init();
    gatt_init();
    db_discovery_init();
    lbs_c_init();
    scan_init();

    NRF_LOG_INFO("Central Verin BLE demarre !");
    scan_start();
    bsp_board_led_on(CENTRAL_SCANNING_LED);

    for (;;)
    {
        idle_state_handle();
    }
}
