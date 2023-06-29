#include "sdk_common.h"
#include "ble_opt.h"
#include <string.h>
#include "ble_srv_common.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "SEGGER_RTT.h"

/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_opt       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_option_t * p_opt, ble_evt_t const * p_ble_evt)
{
    p_opt->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

    ble_opt_evt_t evt;

    evt.evt_type = BLE_OPT_EVT_CONNECTED;

    p_opt->evt_handler(p_opt, &evt);
}

/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_opt       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_option_t * p_opt, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_opt->conn_handle = BLE_CONN_HANDLE_INVALID;
    
    ble_opt_evt_t evt;

    evt.evt_type = BLE_OPT_EVT_DISCONNECTED;

    p_opt->evt_handler(p_opt, &evt);
}

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_opt       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_option_t * p_opt, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    SEGGER_RTT_printf(0, "There frequency value is: %d\n",*p_evt_write->data);

    
    // Custom Value Characteristic Written to.
    if (p_evt_write->handle == p_opt->custom_value_handles.value_handle)
    {
        ble_opt_evt_t evt;  
        
        switch (*p_evt_write->data){
          case 1:
            evt.evt_type = BLE_OPT_BAT_CHECK_1_MIN;
            break;
          case 2:
              evt.evt_type = BLE_OPT_BAT_CHECK_10_MIN;
              break;
          case 3:
              evt.evt_type = BLE_OPT_BAT_CHECK_30_MIN;
              break;
          case 4:
              evt.evt_type = BLE_OPT_BAT_CHECK_1_HR;
              break;
          case 5:
              evt.evt_type = BLE_OPT_BAT_CHECK_6_HR;
              break;
          case 6:
              evt.evt_type = BLE_OPT_BAT_CHECK_12_HR;
              break;
          case 7:
              evt.evt_type = BLE_OPT_BAT_CHECK_24_HR;
              break;
          default:
            // No implementation needed.
            evt.evt_type = BLE_OPT_BAT_CHECK_24_HR;
            break;
         }
        
          p_opt->evt_handler(p_opt, &evt);
    }

//    // Check if the Custom value CCCD is written to and that the value is the appropriate length, i.e 2 bytes.
    if ((p_evt_write->handle == p_opt->custom_value_handles.cccd_handle) && (p_evt_write->len == 2) )
    {
        // CCCD written, call application event handler
        if (p_opt->evt_handler != NULL)
        {
            ble_opt_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_OPT_EVT_NOTIFICATION_ENABLED;                
            }
            else
            {
                evt.evt_type = BLE_OPT_EVT_NOTIFICATION_DISABLED;                
            }

            // Call the application event handler.
            p_opt->evt_handler(p_opt, &evt);
        }
    }
}


void ble_opt_on_ble_evt( ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_option_t * p_opt = (ble_option_t *) p_context;
    
    NRF_LOG_INFO("BLE event received. Event type = %d\r\n", p_ble_evt->header.evt_id); 
    //SEGGER_RTT_printf(0,"BLE event received. Event type = %d\n",p_ble_evt->header.evt_id);
    if (p_opt == NULL || p_ble_evt == NULL)
    {
        return;
    }
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_opt, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_opt, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            //SEGGER_RTT_WriteString(0, "the battery frequency characteristic has been written to \n");
            on_write(p_opt, p_ble_evt);
            break;
/* Handling this event is not necessary
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            NRF_LOG_INFO("EXCHANGE_MTU_REQUEST event received.\r\n");
            break;
*/
        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for adding the Custom Value characteristic.
 *
 * @param[in]   p_opt        Battery Service structure.
 * @param[in]   p_opt_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t option_value_char_add(ble_option_t * p_opt, const ble_option_init_t * p_opt_init)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    // Add Custom Value characteristic
    memset(&cccd_md, 0, sizeof(cccd_md));

    //  Read  operation on cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    cccd_md.write_perm = p_opt_init->custom_value_char_attr_md.cccd_write_perm;
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.char_props.notify = 1; 
    char_md.p_char_user_desc  = "battery_frequency";
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md; 
    char_md.p_sccd_md         = NULL;
		
    ble_uuid.type = p_opt->uuid_type;
    ble_uuid.uuid = OPTION_VALUE_CHAR_UUID;

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_opt_init->custom_value_char_attr_md.read_perm;
    attr_md.write_perm = p_opt_init->custom_value_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof(uint8_t);
    attr_char_value.p_value   = 0x00;

    err_code = sd_ble_gatts_characteristic_add(p_opt->service_handle, &char_md, &attr_char_value, &p_opt->custom_value_handles);
//    if (err_code != NRF_SUCCESS)
//    {
        APP_ERROR_CHECK(err_code); //return err_code;
//    }

//    return NRF_SUCCESS;
}

uint32_t ble_option_init(ble_option_t * p_opt, const ble_option_init_t * p_opt_init)
{
    if (p_opt == NULL || p_opt_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_opt->evt_handler               = p_opt_init->evt_handler;
    p_opt->conn_handle               = BLE_CONN_HANDLE_INVALID;

    // Add Custom Service UUID
    ble_uuid128_t base_uuid = {OPTION_SERVICE_UUID_BASE};
    err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_opt->uuid_type);
    VERIFY_SUCCESS(err_code);
    
    ble_uuid.type = p_opt->uuid_type;
    ble_uuid.uuid = OPTION_SERVICE_UUID;

    // Add the Custom Service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_opt->service_handle);
    APP_ERROR_CHECK(err_code); 
//    if (err_code != NRF_SUCCESS)
//    {
//        return err_code;
//    }

    // Add Custom Value characteristic
    err_code =  option_value_char_add(p_opt, p_opt_init);
    APP_ERROR_CHECK(err_code); 
}

uint32_t ble_opt_notify_client(ble_option_t *p_opt)
{       
    NRF_LOG_INFO("In ble_opt_custom_value_update. \r\n"); 
    SEGGER_RTT_WriteString(0, "sending notification of battery frequency change \n");
    if (p_opt == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code = NRF_SUCCESS;
    uint8_t btry_chk_frqcy; 
    ble_gatts_value_t gatts_value;
    
    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(uint8_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = &btry_chk_frqcy;

    err_code = sd_ble_gatts_value_get(BLE_CONN_HANDLE_INVALID, p_opt->custom_value_handles.value_handle, &gatts_value);
    APP_ERROR_CHECK(err_code);
   
    if ((p_opt->conn_handle != BLE_CONN_HANDLE_INVALID)) 
    {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_opt->custom_value_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(p_opt->conn_handle, &hvx_params);
        NRF_LOG_INFO("sd_ble_gatts_hvx result: %x. \r\n", err_code);
         SEGGER_RTT_printf(0, "sd_ble_gatts_hvx result: %x. \n", err_code);
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
        NRF_LOG_INFO("sd_ble_gatts_hvx result: NRF_ERROR_INVALID_STATE. \r\n"); 
        SEGGER_RTT_WriteString(0, "sd_ble_gatts_hvx result: NRF_ERROR_INVALID_STATE. \n");
    }



    return err_code;
}