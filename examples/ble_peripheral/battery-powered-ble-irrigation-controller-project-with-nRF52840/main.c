#include <stdint.h>
#include <string.h>

//#include "nrf_dfu_ble_svci_bond_sharing.h"
#include "nrf_svci_async_function.h"
#include "nrf_svci_async_handler.h"

#include "nrf.h"
#include "nrf_soc.h"
#include "nrf_drv_saadc.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_bas.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_util.h"
#include "nordic_common.h"
#include "boards.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "ble_advertising.h"
#include "nrf_ble_gatt.h"
#include "ble_dfu.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"
//#include "nrf_bootloader_info.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "SEGGER_RTT.h"

#include "fds.h"
#include "nrf_sdm.h"
#include "ble_dis.h"
#include "app_scheduler.h"
#include "ble_conn_state.h"

#include "ble_gap.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

//#include "ble_dfu.h"
#include "nrf_power.h"

/*
UART driver and UART LOG is disabled to minimise current consumption.  

To re-enable you need to import the nrf_drv_uart.c file back into the nRF_Drivers folder. (or optionally uncheck "exclude from Build" when you right click on the file 
As well as uncomment the header files the init function and the event handler function.

to change the battery increments you need to open the app_util.h file and change them there.  It's default is 3000 mV as 100%.
right click on the battery_level_in_percent() function below and click "go to definition" to get to that file.
  
Note I have added the following to line 100 in app_error_weak.c
in order to reset nRF52 when there is a app error

NRF_LOG_WARNING("System reset");     ////
NVIC_SystemReset();                  ////
*/



#include "bsp_btn_ble.h"

#include "ble_cus.h"
#include "ble_opt.h"

#define DEVICE_NAME                     "Front_Garden"                         /**< Name of device. Will be included in the advertising data. */
#define TX_POWER_LEVEL                  4
#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define BATTERY_LEVEL_MEAS_INTERVAL_5_MIN        APP_TIMER_TICKS(60000) //for some reason 60000 is 5 min for this????   //5 minutes   /**< Battery level measurement interval (ticks). This value corresponds to 60 minutes. */
#define CUSTOM_SERVICE_NOTIFICATION_INTERVAL     APP_TIMER_TICKS(60000) 

#define APP_ADV_FAST_INTERVAL           MSEC_TO_UNITS(400, UNIT_1_25_MS)  // 500 ms                       /**< Fast advertising interval (in units of 0.625 ms. This value corresponds to 25 ms.). */
#define APP_ADV_SLOW_INTERVAL           MSEC_TO_UNITS(800, UNIT_1_25_MS)  // 7 secs  ~= 3.3 uA                   /**< Slow advertising interval (in units of 0.625 ms. This value corresponds to 100 ms.). */

#define APP_ADV_FAST_DURATION           MSEC_TO_UNITS(50000, UNIT_10_MS)                                        /**< The advertising duration of fast advertising in units of 10 milliseconds. */
#define APP_ADV_SLOW_DURATION           0                                       /**< The advertising duration of slow advertising in units of 10 milliseconds. 0 means indefinitely. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.2 seconds). */
//#define MAX_CONN_INTERVAL               BLE_GAP_CP_MAX_CONN_INTVL_MAX         /**< Maximum acceptable connection interval (4 second). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)//~= 3.3 uA/

#define SLAVE_LATENCY                   3                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                BLE_GAP_CP_CONN_SUP_TIMEOUT_MAX         /**< Connection supervisory time-out (32 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(15000)                  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000)                   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    5                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS   600                                     /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION    6                                       /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS  270                                     /**< Typical forward voltage drop of the diode . */
#define ADC_RES_10BIT                   1024                                    /**< Maximum digital value for 10-bit ADC conversion. */

#define LESC_DEBUG_MODE                     0                                       /**< Set to 1 to use LESC debug keys, allows you to use a sniffer to inspect traffic. */

#define BLE_ADV_WHITELIST_ENABLED       true
#define SEC_PARAM_BOND                  1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define MAX_RTC_COUNTER_VAL     0x00FFFFFF  

#define TAP_1 LED_1
#define TAP_2 LED_2
#define TAP_FULLY_CLOSED_RED_PIN  BUTTON_1
#define TAP_FULLY_OPEN_GREEN_PIN  BUTTON_2

static bool switch_event_reached        = false;
static bool tap_fully_closed            = false;
static bool tap_fully_open              = false;
static bool tap_state                   = 0;
static uint8_t tap_time_left            = 0;
static bool custom_svc_char1_ntf_enabled = false;
static bool custom_svc_char2_ntf_enabled = false;

/**@brief Macro to convert the result of ADC conversion in millivolts.
 *
 * @param[in]  ADC_VALUE   ADC result..
 *
 * @retval     Result converted to millivolts.
 */
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE) ((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION)

static void update_tap_state(void);

APP_TIMER_DEF(m_our_char_timer_id);
APP_TIMER_DEF(m_battery_timer_id);                                                /**< Battery measurement timer. */
APP_TIMER_DEF(m_battery_frequency_id);                                                /**< Battery measurement timer. */
//APP_TIMER_DEF(m_notification_timer_id);
NRF_BLE_GATT_DEF(m_gatt);                                                         /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                           /**< Context for the Queued Write module.*/
BLE_BAS_DEF(m_bas);                                                               /**< Battery service instance. */
BLE_ADVERTISING_DEF(m_advertising);                                               /**< Advertising module instance. */
BLE_CUS_DEF(m_cus);                                                               /**< Context for the Queued Write module.*/
BLE_OPT_DEF(m_opt);                                                               /**< Context for the Queued Write module.*/

static uint16_t           m_conn_handle = BLE_CONN_HANDLE_INVALID;                          /**< Handle of the current connection. */
static pm_peer_id_t       m_peer_id;                                               /**< Device reference handle to the current bonded central. */
static bool               m_in_boot_mode = false;                                  /**< Current protocol mode. */
static uint8_t            m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;                     /**< Advertising handle used to identify an advertising set. */
static uint8_t            m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                      /**< Buffer for storing an encoded advertising set. */
static uint8_t            m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX];           /**< Buffer for storing an encoded scan data. */
static uint16_t           battery_level_measure_interval_counter = 0;
static uint16_t           battery_measure_interval = 5;  // 1 hour is the default 

static uint16_t           m_battery_conn_handle = BLE_CONN_HANDLE_INVALID;

static nrf_saadc_value_t adc_buf[2];
static uint32_t watering_duration =  0;                                      
//static uint8_t m_custom_value = 0;
static uint8_t pm_paired = 0;
//Step 5: Declare variable holding our service UUID
static ble_uuid_t m_adv_uuids[] = {{CUSTOM_SERVICE_UUID,BLE_UUID_TYPE_VENDOR_BEGIN}};

static void advertising_start(bool erase_bonds);
static void on_bas_evt(ble_bas_t * p_bas, ble_bas_evt_t * p_evt);

static void disconnect(uint16_t conn_handle, void * p_context)
{
    UNUSED_PARAMETER(p_context);

    ret_code_t err_code = sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    if (err_code != NRF_SUCCESS)
    {
        //NRF_LOG_WARNING("Failed to disconnect connection. Connection handle: %d Error: %d", conn_handle, err_code);
    }
    else
    {
        //NRF_LOG_DEBUG("Disconnected connection handle %d", conn_handle);
    }
}

static void tap_notification_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    ret_code_t err_code;
   
    err_code = ble_cus_custom_value_update(&m_cus, custom_svc_char1_ntf_enabled);
    APP_ERROR_CHECK(err_code);
}

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for setting filtered whitelist.
 *
 * @param[in] skip  Filter passed to @ref pm_peer_id_list.
 */
static void whitelist_set(pm_peer_id_list_skip_t skip)
{
    pm_peer_id_t peer_ids[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
    uint32_t     peer_id_count = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

    ret_code_t err_code = pm_peer_id_list(peer_ids, &peer_id_count, PM_PEER_ID_INVALID, skip);
    APP_ERROR_CHECK(err_code);
    SEGGER_RTT_printf(0,"m_whitelist_peer_cnt %d, MAX_PEERS_WLIST %d\n", peer_id_count + 1, BLE_GAP_WHITELIST_ADDR_MAX_COUNT);
   
    err_code = pm_whitelist_set(peer_ids, peer_id_count);
    APP_ERROR_CHECK(err_code);

    //SEGGER_RTT_printf(0,"m_whitelist_peers %d", );
}

/**@brief Function for setting filtered device identities.
 *
 * @param[in] skip  Filter passed to @ref pm_peer_id_list.
 */
static void identities_set(pm_peer_id_list_skip_t skip)
{
    pm_peer_id_t peer_ids[BLE_GAP_DEVICE_IDENTITIES_MAX_COUNT];
    uint32_t     peer_id_count = BLE_GAP_DEVICE_IDENTITIES_MAX_COUNT;

    ret_code_t err_code = pm_peer_id_list(peer_ids, &peer_id_count, PM_PEER_ID_INVALID, skip);
    APP_ERROR_CHECK(err_code);

    err_code = pm_device_identities_list_set(peer_ids, peer_id_count);
    APP_ERROR_CHECK(err_code);
}

/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;
    SEGGER_RTT_WriteString(0,"Erasing Bonds!.\n");
    //SEGGER_RTT_WriteString(0,"Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Service errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in] nrf_error   Error code containing information about what went wrong.
 */
static void service_error_handler(uint32_t nrf_error)
{
    //SEGGER_RTT_WriteString(0,nrf_error);
    SEGGER_RTT_WriteString(0, " the nrf error is: " + nrf_error);
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for handling advertising errors.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void ble_advertising_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for handling the ADC interrupt.
 *
 * @details  This function will fetch the conversion result from the ADC, convert the value into
 *           percentage and send it to peer.
 */
void saadc_event_handler(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        nrf_saadc_value_t adc_result;
        uint16_t          batt_lvl_in_milli_volts;
        uint8_t           percentage_batt_lvl;
        uint32_t          err_code;

        adc_result = p_event->data.done.p_buffer[0];

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, 1);
        APP_ERROR_CHECK(err_code);

        batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result) +
                                  DIODE_FWD_VOLT_DROP_MILLIVOLTS;
        percentage_batt_lvl = battery_level_in_percent(batt_lvl_in_milli_volts);
        SEGGER_RTT_printf(0,"the battery level in percent is: %d\n", percentage_batt_lvl);
        err_code = ble_bas_battery_level_update(&m_bas, percentage_batt_lvl, BLE_CONN_HANDLE_ALL/*,m_battery_conn_handle*/);
        if ((err_code != NRF_SUCCESS) //&&
            //(err_code != NRF_ERROR_INVALID_STATE) &&
            //(err_code != NRF_ERROR_RESOURCES) &&
            //(err_code != NRF_ERROR_BUSY) &&
            //(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
           )
        {
            APP_ERROR_HANDLER(err_code);
        }
    }
    nrf_drv_saadc_uninit();
    NRF_SAADC->INTENCLR = (SAADC_INTENCLR_END_Clear << SAADC_INTENCLR_END_Pos);
    NVIC_ClearPendingIRQ(SAADC_IRQn);
}

/**@brief Function for configuring ADC to do battery level conversion.
 */
static void adc_configure(void)
{
    ret_code_t err_code = nrf_drv_saadc_init(NULL, saadc_event_handler);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_channel_config_t config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);
    err_code = nrf_drv_saadc_channel_init(0, &config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(&adc_buf[0], 1);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(&adc_buf[1], 1);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *          This function will start the ADC.
 *
 * @param[in] p_context   Pointer used for passing some arbitrary information (context) from the
 *                        app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    //SEGGER_RTT_WriteString(0,"im in battery_level_meas_timeout_handler function \n");
    battery_level_measure_interval_counter++;
    if (battery_level_measure_interval_counter >= battery_measure_interval){
    SEGGER_RTT_WriteString(0,"and the battery level will be measured.\n");
    UNUSED_PARAMETER(p_context);
    ret_code_t err_code;
    adc_configure();
    err_code = nrf_drv_saadc_sample();
    APP_ERROR_CHECK(err_code);
    
    battery_level_measure_interval_counter = 0;
    }
}

static void notifyOfFrequencyChange(void * p_context)
{
    UNUSED_PARAMETER(p_context);

    ret_code_t err_code;
    err_code = ble_opt_notify_client(&m_opt);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for handling the Battery Service events.
 *
 * @details This function will be called for all Battery Service events which are passed to the
 |          application.
 *
 * @param[in] p_bas  Battery Service structure.
 * @param[in] p_evt  Event received from the Battery Service.
 */
static void on_bas_evt(ble_bas_t * p_bas, ble_bas_evt_t * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_type)
    {
        case BLE_BAS_EVT_NOTIFICATION_ENABLED:
            // Start battery timer
            //SEGGER_RTT_WriteString(0,"the battery service has been notify enabled\n");


//This below line of code ensure the conn_handle is correct that is getting to softdevice
            m_battery_conn_handle = p_evt->conn_handle;

            err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL_5_MIN, NULL);
            APP_ERROR_CHECK(err_code);
            SEGGER_RTT_WriteString(0,"the battery service has been notify enabled\n");
            break; // BLE_BAS_EVT_NOTIFICATION_ENABLED

        case BLE_BAS_EVT_NOTIFICATION_DISABLED:
            SEGGER_RTT_WriteString(0,"the battery service has been notify disabled\n");
            err_code = app_timer_stop(m_battery_timer_id);
            APP_ERROR_CHECK(err_code);
            break; // BLE_BAS_EVT_NOTIFICATION_DISABLED

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for handling the Custom Service Service events.
 *
 * @details This function will be called for all Custom Service events which are passed to
 *          the application.
 *
 * @param[in]   p_cus_service  Custom Service structure.
 * @param[in]   p_evt          Event received from the Custom Service.
 *
 */
static void on_cus_evt(ble_cus_t * p_cus_service, ble_cus_evt_t * p_evt)
{
    ret_code_t err_code;
    
    switch(p_evt->evt_type)
    {
        case BLE_CUS_EVT_NOTIFICATION_ENABLED:
            custom_svc_char1_ntf_enabled = true;
            SEGGER_RTT_WriteString(0,"the custom service has been notify enabled\n");
            break;

        case BLE_CUS_EVT_NOTIFICATION_DISABLED:
            custom_svc_char1_ntf_enabled = false;
            SEGGER_RTT_WriteString(0,"the custom service has been notify disabled\n");
            err_code = app_timer_stop(m_our_char_timer_id);
            APP_ERROR_CHECK(err_code);                        
            break;
        case BLE_CUS2_EVT_NOTIFICATION_ENABLED:
            custom_svc_char2_ntf_enabled = true;
            SEGGER_RTT_WriteString(0,"the custom service char 2 has been notify enabled\n");

            ble_tap_state_update(&m_cus, tap_state, custom_svc_char2_ntf_enabled);
            break;
        case BLE_CUS2_EVT_NOTIFICATION_DISABLED:
            custom_svc_char2_ntf_enabled = false;
            SEGGER_RTT_WriteString(0,"the custom service char 2 has been notify disabled\n");                    
            break;
        case BLE_CUS_BEGIN_TAP_TIMER:
              SEGGER_RTT_WriteString(0,"tap timer is on and counting down to zero.\n");
              err_code = app_timer_start(m_our_char_timer_id, CUSTOM_SERVICE_NOTIFICATION_INTERVAL, NULL);
              APP_ERROR_CHECK(err_code);
              
              
              nrf_gpio_pin_set(TAP_2);
              nrf_gpio_pin_clear(TAP_1);

              tap_time_left = p_evt->params.rx_data.p_data[0];

              break;
        case BLE_CUS_END_TAP_TIMER:
              SEGGER_RTT_WriteString(0,"Tap timer will stop now.\n");
              err_code = app_timer_stop(m_our_char_timer_id);
              APP_ERROR_CHECK(err_code);

              nrf_gpio_pin_set(TAP_1);
              nrf_gpio_pin_clear(TAP_2);

              tap_time_left = 0;

              break;
        case BLE_CUS_EVT_DISCONNECTED:
//            SEGGER_RTT_WriteString(0,"if tap timer is on then then this will turn it off on disconnection of esp32\n");
//            err_code = app_timer_stop(m_our_char_timer_id);
//            APP_ERROR_CHECK(err_code); 
              break;

        case BLE_CUS_EVT_CONNECTED:
            break;


        default:
              // No implementation needed.
              break;
    }
}

/**@brief Function for handling the Custom Service Service events.
 *
 * @details This function will be called for all Custom Service events which are passed to
 *          the application.
 *
 * @param[in]   p_cus_service  Custom Service structure.
 * @param[in]   p_evt          Event received from the Custom Service.
 *
 */
static void on_opt_evt(ble_option_t * p_opt_service, ble_opt_evt_t * p_evt)
{
    ret_code_t err_code;
    
    switch(p_evt->evt_type)
    {
        case BLE_OPT_BAT_CHECK_1_MIN:
            err_code = app_timer_stop(m_battery_timer_id);
            APP_ERROR_CHECK(err_code);
            battery_level_measure_interval_counter = 1; // the number of times that 5 mins is run
            err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL_5_MIN, NULL);
            APP_ERROR_CHECK(err_code);
            SEGGER_RTT_WriteString(0,"battery read frequency was changed to 5 min!\n");
            break;
        case BLE_OPT_BAT_CHECK_10_MIN:
            err_code = app_timer_stop(m_battery_timer_id);
            APP_ERROR_CHECK(err_code);
            battery_level_measure_interval_counter = 2; // the number of times that 5 mins is run
            err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL_5_MIN, NULL);
            APP_ERROR_CHECK(err_code);
            SEGGER_RTT_WriteString(0,"battery read frequency was changed to 10 min!\n");
            break;
        case BLE_OPT_BAT_CHECK_30_MIN:
            err_code = app_timer_stop(m_battery_timer_id);
            APP_ERROR_CHECK(err_code);
            battery_level_measure_interval_counter = 6; // the number of times that 5 mins is run
            err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL_5_MIN, NULL);
            APP_ERROR_CHECK(err_code);
            SEGGER_RTT_WriteString(0,"battery read frequency was changed to 30 min!\n");
            break;
        case BLE_OPT_BAT_CHECK_1_HR:
            err_code = app_timer_stop(m_battery_timer_id);
            APP_ERROR_CHECK(err_code);
            battery_level_measure_interval_counter = 12; // the number of times that 5 mins is run
            err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL_5_MIN, NULL);
            APP_ERROR_CHECK(err_code);
            SEGGER_RTT_WriteString(0,"battery read frequency was changed to 1 hour!\n");
            break;
            err_code = app_timer_stop(m_battery_timer_id);
            APP_ERROR_CHECK(err_code);
            battery_level_measure_interval_counter = 72; // the number of times that 5 mins is run
            err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL_5_MIN, NULL);
            APP_ERROR_CHECK(err_code);
            SEGGER_RTT_WriteString(0,"battery read frequency was changed to 6 hours!\n");
            break;
        case BLE_OPT_BAT_CHECK_12_HR:
            err_code = app_timer_stop(m_battery_timer_id);
            APP_ERROR_CHECK(err_code);
            battery_level_measure_interval_counter = 144; // the number of times that 5 mins is run
            err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL_5_MIN, NULL);
            APP_ERROR_CHECK(err_code);
            SEGGER_RTT_WriteString(0,"battery read frequency was changed to 12 hours!\n");
            break;
        case BLE_OPT_BAT_CHECK_24_HR:
            err_code = app_timer_stop(m_battery_timer_id);
            APP_ERROR_CHECK(err_code);
            battery_level_measure_interval_counter = 288; // the number of times that 5 mins is run
            err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL_5_MIN, NULL);
            APP_ERROR_CHECK(err_code);
            SEGGER_RTT_WriteString(0,"battery read frequency was changed to 24 hours!\n");
            break;
        case BLE_OPT_EVT_CONNECTED:
            break;

        case BLE_OPT_EVT_DISCONNECTED:
              break;
        default:
              // No implementation needed.
              break;
    }
}

/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
        whitelist_set(PM_PEER_ID_LIST_SKIP_NO_ID_ADDR);

        ret_code_t ret = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(ret);
    }
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    ret_code_t err_code;

    switch (p_evt->evt_id)
        {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            SEGGER_RTT_WriteString(0,"Connected to a previously bonded device.\n");
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            SEGGER_RTT_printf(0,"Connection secured: role: %d, conn_handle: 0x%x, procedure: %d.\n",
                         ble_conn_state_role(p_evt->conn_handle),
                         p_evt->conn_handle,
                         p_evt->params.conn_sec_succeeded.procedure);
        } break;

        case PM_EVT_CONN_SEC_FAILED:
        {
        SEGGER_RTT_WriteString(0,"Failed to secure the connection.  Long Press Button to erase bonds.\n");
            /* Often, when securing fails, it shouldn't be restarted, for security reasons.
             * Other times, it can be restarted directly.
             * Sometimes it can be restarted, but only after changing some Security Parameters.
             * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
             * Sometimes it is impossible, to secure the link, or the peer device does not support it.
             * How to handle this error is highly application dependent. */
        } break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;

        case PM_EVT_STORAGE_FULL:
        {
        SEGGER_RTT_WriteString(0,"PM_EVT_STORAGE_FULL event storage is full.\n");
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
          SEGGER_RTT_WriteString(0,"PM_EVT_PEERS_DELETE_SUCCEEDED\n");
          //NVIC_SystemReset();
          //advertising_start(false);
        } break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
            // This can happen when the local DB has changed.
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }

}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
//static void sleep_mode_enter(void)
//{
//    ret_code_t err_code;

//    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
//    APP_ERROR_CHECK(err_code);

//    // Prepare wakeup buttons.
//    err_code = bsp_btn_ble_sleep_mode_prepare();
//    APP_ERROR_CHECK(err_code);

//    // Go to system-off mode (this function will not return; wakeup will cause a reset).
//    err_code = sd_power_system_off();
//    APP_ERROR_CHECK(err_code);
//}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_WHITELIST_REQUEST:
        {
            ble_gap_addr_t whitelist_addrs[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            ble_gap_irk_t  whitelist_irks[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            uint32_t       addr_cnt = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
            uint32_t       irk_cnt  = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

            err_code = pm_whitelist_get(whitelist_addrs, &addr_cnt, whitelist_irks,  &irk_cnt);
            APP_ERROR_CHECK(err_code);
            //NRF_LOG_DEBUG("pm_whitelist_get returns %d addr in whitelist and %d irk whitelist", addr_cnt, irk_cnt);
            SEGGER_RTT_WriteString(0,"a whitelist request has been made\n");
            // Set the correct identities list (no excluding peers with no Central Address Resolution).
            identities_set(PM_PEER_ID_LIST_SKIP_NO_IRK);

            // Apply the whitelist.
            err_code = ble_advertising_whitelist_reply(&m_advertising,whitelist_addrs,addr_cnt,whitelist_irks,irk_cnt);
            APP_ERROR_CHECK(err_code);
        }
        break;

        case BLE_ADV_EVT_PEER_ADDR_REQUEST:
        {
            pm_peer_data_bonding_t peer_bonding_data;
            SEGGER_RTT_WriteString(0,"a peer address request has been made\n");
            // Only Give peer address if we have a handle to the bonded peer.
            if (m_peer_id != PM_PEER_ID_INVALID)
            {
                err_code = pm_peer_data_bonding_load(m_peer_id, &peer_bonding_data);
                if (err_code != NRF_ERROR_NOT_FOUND)
                {
                    APP_ERROR_CHECK(err_code);

                    // Manipulate identities to exclude peers with no Central Address Resolution.
                    identities_set(PM_PEER_ID_LIST_SKIP_ALL);

                    ble_gap_addr_t * p_peer_addr = &(peer_bonding_data.peer_ble_id.id_addr_info);
                    err_code = ble_advertising_peer_addr_reply(&m_advertising, p_peer_addr);
                    APP_ERROR_CHECK(err_code);
                }

            }
            break;
        }

        default:
            break;
    }
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            //SEGGER_RTT_WriteString(0,"Connected");
            SEGGER_RTT_WriteString(0, "Connected.\n");

            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);

            if(pm_paired == 0) {
              pm_conn_secure(m_conn_handle,false);
              pm_paired++;
            }

            break;

        case BLE_GAP_EVT_DISCONNECTED:
            //SEGGER_RTT_WriteString(0,"Disconnected");
            SEGGER_RTT_WriteString(0, "Disconnected\n");
            // LED indication will be changed when advertising starts.

            m_conn_handle = BLE_CONN_HANDLE_INVALID;

            nrf_gpio_pin_set(TAP_1);
            nrf_gpio_pin_clear(TAP_2);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            //NRF_LOG_DEBUG("PHY update request.");
            SEGGER_RTT_WriteString(0, "PHY update request\n");
            ble_gap_phys_t const phys =
            {
              .rx_phys = BLE_GAP_PHY_AUTO,
              .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;


        case BLE_GATTC_EVT_TIMEOUT:
            SEGGER_RTT_WriteString(0, "GATT Client Timeout.\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            SEGGER_RTT_WriteString(0, "GATT Server Timeout.\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_GATTS_EVT_WRITE:
            SEGGER_RTT_WriteString(0, "GATT Write Event\n");
            break;
        case BLE_GAP_EVT_ADV_REPORT:
            SEGGER_RTT_printf(0,"RSSI: %d", p_ble_evt->evt.gap_evt.params.adv_report.rssi);


            break;
        default:
            // No implementation needed.
            break;
    }
}


static void update_tap_state(void){

  if(tap_fully_closed){
              
    tap_state = 0;         

  } else if(tap_fully_open){

    tap_state = 1;
            
  }else  {
    tap_state = 2;
  }
}



/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
static void bsp_event_handler(bsp_event_t event)
{
    uint32_t         err_code;
    switch (event)
    {
      case BSP_EVENT_KEY_0: //On Button 1 press
          tap_fully_open = false;
          tap_fully_closed = false;
          //update_tap_state();

          tap_state = 2;

          ble_tap_state_update(&m_cus, 2, custom_svc_char2_ntf_enabled);

          SEGGER_RTT_WriteString(0,"BSP_EVENT_KEY_0\ngreen wire is no longer high\nvalve is partially open\n");
          break;

      case BSP_EVENT_KEY_1: //On Button 2 press
          tap_fully_open = false;
          tap_fully_closed = false;
          //update_tap_state();

          tap_state = 2;

          ble_tap_state_update(&m_cus, 2, custom_svc_char2_ntf_enabled);

          SEGGER_RTT_WriteString(0,"BSP_EVENT_KEY_1\nred wire is high\nvalve is partially open\n");
          break;

      case BSP_EVENT_KEY_0_RELEASE: // valve is fully open
          tap_fully_open = true;
          tap_fully_closed = false;
          update_tap_state();

          ble_tap_state_update(&m_cus, 1, custom_svc_char2_ntf_enabled);

          nrf_gpio_pin_write(TAP_1,0);

          SEGGER_RTT_WriteString(0,"BSP_EVENT_KEY_0_RELEASE\ngreen wire is high\nvalve is fully open\n");
          break;

      case BSP_EVENT_KEY_1_RELEASE: // Valve is fully closed
          tap_fully_open = false;
          tap_fully_closed = true;
          update_tap_state();

          ble_tap_state_update(&m_cus, 0, custom_svc_char2_ntf_enabled);

          nrf_gpio_pin_write(TAP_2,0);

          SEGGER_RTT_WriteString(0,"BSP_EVENT_KEY_1_RELEASE\nred wire is high\nvalve is fully closed\n");
          break;

      case BSP_EVENT_KEY_2: //On Button 3 press   

            break;
      default:
          break;
    }
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    // Initialize timer module, making it use the scheduler
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create repeated event timer
    err_code = app_timer_create(&m_our_char_timer_id, APP_TIMER_MODE_REPEATED, tap_notification_timeout_handler);  //temp timer
    APP_ERROR_CHECK(err_code);

    // Create battery timer.
    err_code = app_timer_create(&m_battery_timer_id, APP_TIMER_MODE_REPEATED, battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);

    // Create battery timer.
    err_code = app_timer_create(&m_battery_frequency_id, APP_TIMER_MODE_SINGLE_SHOT, notifyOfFrequencyChange);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
  ret_code_t              err_code;
  ble_gap_conn_params_t   gap_conn_params;
  ble_gap_conn_sec_mode_t sec_mode;

      BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&sec_mode);

  err_code = sd_ble_gap_device_name_set(&sec_mode,
                                        (const uint8_t *)DEVICE_NAME,
                                        strlen(DEVICE_NAME));
  APP_ERROR_CHECK(err_code);

  memset(&gap_conn_params, 0, sizeof(gap_conn_params));

  gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
  gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
  gap_conn_params.slave_latency     = SLAVE_LATENCY;
  gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

  err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
  APP_ERROR_CHECK(err_code);
}





/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Battery Service.
 */
static void bas_init(void)
{
    ret_code_t     err_code;
    ble_bas_init_t bas_init_obj;

    memset(&bas_init_obj, 0, sizeof(bas_init_obj));

    bas_init_obj.evt_handler          = on_bas_evt;
    bas_init_obj.support_notification = true;
    bas_init_obj.p_report_ref         = NULL;
    bas_init_obj.initial_batt_level   = 100;

    bas_init_obj.bl_rd_sec        = SEC_JUST_WORKS;
    bas_init_obj.bl_cccd_wr_sec   = SEC_JUST_WORKS;
    bas_init_obj.bl_report_rd_sec = SEC_JUST_WORKS;

    err_code = ble_bas_init(&m_bas, &bas_init_obj);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init = {0};
    ble_bas_init_t bas_init_obj;
    ble_cus_init_t      cus_init = {0};
    ble_option_init_t      opt_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize CUS Service init structure to zero.
    cus_init.evt_handler                = on_cus_evt;
    opt_init.evt_handler                = on_opt_evt;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&cus_init.custom_value_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&cus_init.custom_value_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&cus_init.custom_value_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&opt_init.custom_value_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&opt_init.custom_value_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&opt_init.custom_value_char_attr_md.write_perm);

    err_code = ble_cus_init(&m_cus, &cus_init);
    APP_ERROR_CHECK(err_code);

    err_code =  ble_option_init(&m_opt, &opt_init);
    APP_ERROR_CHECK(err_code);

    bas_init();
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);

}

/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
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
}

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

	
    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids = m_adv_uuids;
		
    init.config.ble_adv_whitelist_enabled = BLE_ADV_WHITELIST_ENABLED; //added for white listing
    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_FAST_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_FAST_DURATION;
    init.config.ble_adv_slow_enabled = true;
    init.config.ble_adv_slow_interval = APP_ADV_SLOW_INTERVAL;
    init.config.ble_adv_slow_timeout = APP_ADV_SLOW_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    tap_fully_closed = nrf_gpio_pin_read(TAP_FULLY_CLOSED_RED_PIN);
    tap_fully_open   = nrf_gpio_pin_read(TAP_FULLY_OPEN_GREEN_PIN);

    update_tap_state();

    nrf_gpio_pin_set(TAP_1);
    nrf_gpio_pin_clear(TAP_2);

    SEGGER_RTT_printf(0,"The red pin is %d and the green pin is %d\n", tap_fully_open,tap_fully_closed);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}

static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for application main entry.
 */
int main(void)

 {
    ret_code_t err_code;
    bool erase_bonds;

    //Initialize.
    log_init();
    timers_init();
    buttons_leds_init(&erase_bonds);
    power_management_init();

    ble_stack_init();
    sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);

    ble_gap_conn_sec_mode_t sec_mode;
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&sec_mode);
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                        (const uint8_t *)DEVICE_NAME,
                                        strlen(DEVICE_NAME));


    sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV,m_adv_handle, 8);
    sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV,m_conn_handle, 8);

    gatt_init();
    services_init();       //services init called before advertising init to ensure all services available
    advertising_init(); 
    peer_manager_init();
    advertising_start(erase_bonds);    
    SEGGER_RTT_WriteString(0, "Doing first battery Read\n");

    adc_configure();
    err_code = nrf_drv_saadc_sample();
    APP_ERROR_CHECK(err_code);
    SEGGER_RTT_WriteString(0, "Entering main loop\n");
    // Enter main loop.
    for (;;)
    {
        nrf_pwr_mgmt_run();
    }
}
