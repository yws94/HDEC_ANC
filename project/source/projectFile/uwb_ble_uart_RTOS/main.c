/*Nordic Include*/
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
/*Softdevice Include*/
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_freertos.h"
#include "app_timer.h"
/*RTOS Include*/
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "fds.h"
#include "queue.h"
/*BLE Include*/
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_nus_c.h"
#include "ble_conn_state.h"
#include "ble_db_discovery.h"
#include "nrf_ble_scan.h"
#include "nrf_ble_gatt.h"
#include "bsp_btn_ble.h"
/*NRF Driver Include*/
#include "nrf_drv_clock.h"
#include "nrf_drv_saadc.h"
#include "nrf_stack_guard.h"
/*Uart Include*/
#include "nrf_uart.h"
#include "nrf_uarte.h"
#include "app_uart.h"
#include "app_util_platform.h"
/*LOG Include*/
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
/*UWB Include*/
#include "deca_device_api.h"
#include <boards.h>
#include <portu.h>
#include <deca_spi.h>
#include <deca_regs.h>
#include <examples_defines.h>
#include <shared_defines.h>
#include <shared_functions.h>
#include <config_options.h>

#define CENTRAL_SCANNING_LED            BSP_BOARD_LED_0                     /**< Scanning LED will be on when the device is scanning. */
#define CENTRAL_CONNECTED_1_LED         BSP_BOARD_LED_2                     /**< Connected LED will be on when the device is connected. */
#define CENTRAL_CONNECTED_2_LED         BSP_BOARD_LED_3                     /**< Connected LED will be on when the device is connected. */

#define APP_BLE_OBSERVER_PRIO               3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG                1                                       /**< A tag identifying the SoftDevice BLE configuration. */
#define UWB_ROUND_INTERVAL         APP_TIMER_TICKS(15)//APP_TIMER_TICKS(30)   //1 : 60ms, 0.06               /**< Battery level measurement interval (ticks). This value corresponds to 120 seconds. */

//#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(500, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.5 seconds).  */
//#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(1000, UNIT_1_25_MS)       /**< Maximum acceptable connection interval (1 second). */
//#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
//#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

//#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
//#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
//#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

//#define SEC_PARAM_BOND                      1                                       /**< Perform bonding. */
//#define SEC_PARAM_MITM                      0                                       /**< Man In The Middle protection not required. */
//#define SEC_PARAM_LESC                      0                                       /**< LE Secure Connections not enabled. */
//#define SEC_PARAM_KEYPRESS                  0                                       /**< Keypress notifications not enabled. */
//#define SEC_PARAM_IO_CAPABILITIES           BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
//#define SEC_PARAM_OOB                       0                                       /**< Out Of Band data not available. */
//#define SEC_PARAM_MIN_KEY_SIZE              7                                       /**< Minimum encryption key size. */
//#define SEC_PARAM_MAX_KEY_SIZE              16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                           0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define UART_TX_BUF_SIZE                    256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                    256                                         /**< UART RX buffer size. */

//#define ADC_REF_VOLTAGE_IN_MILLIVOLTS      600                                     /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
//#define ADC_PRE_SCALING_COMPENSATION       6                                       /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
//#define DIODE_FWD_VOLT_DROP_MILLIVOLTS     270                                     /**< Typical forward voltage drop of the diode . */
//#define ADC_RES_10BIT                      1024                                    /**< Maximum digital value for 10-bit ADC conversion. */
#define OSTIMER_WAIT_FOR_QUEUE             10                                      /**< Number of ticks to wait for the timer queue to be ready */

#define RNG_DELAY_MS 30
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

#define ALL_MSG_COMMON_LEN 10
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define RX_BUF_LEN 24
#define POLL_RX_TO_RESP_TX_DLY_UUS 900
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500
#define FINAL_RX_TIMEOUT_UUS 220  
#define PRE_TIMEOUT 5
#define ANC_ID 0xAA
#define Range_Check 30   // Safe_Dist 2m 
#define TAG_NUM 8

//extern example_ptr example_pointer;
//extern int unit_test_main(void);
//extern void build_examples(void);
extern int ds_resp(void);
extern int rcm_tx(void);

static uint8_t rcm_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'R', 'C', 'M', ANC_ID, 0, 0};
static uint8_t rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8_t tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8_t rx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint64_t poll_rx_ts;
static uint64_t resp_tx_ts;
static uint64_t final_rx_ts;
static double tof;
static double distance;

static uint8_t frame_seq_nb = 0;
static uint8_t rx_buffer[RX_BUF_LEN];
static uint32_t status_reg = 0;
static uint8_t cnt;

static dwt_config_t config = {
    5,               /* Channel number. */
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    DWT_PHRRATE_STD, /* PHY header rate. */
    (129 + 8 - 8),   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,/* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0      /* PDOA mode off */
};



extern dwt_txconfig_t txconfig_options;

NRF_BLE_SCAN_DEF(m_scan);                                       /**< Scanning module instance. */
NRF_BLE_GATT_DEF(m_gatt);                                       /**< GATT module instance. */

BLE_NUS_C_ARRAY_DEF(m_ble_nus_c, NRF_SDH_BLE_CENTRAL_LINK_COUNT);
BLE_DB_DISCOVERY_ARRAY_DEF(m_db_disc, NRF_SDH_BLE_CENTRAL_LINK_COUNT);  /**< Database discovery module instances. */

static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

static char const m_target_periph_name[] = "woorim_test";     /**< Name of the device we try to connect to. This name is searched in the scan report data*/


static uint16_t m_conn_handle         = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static TimerHandle_t uwb_timer;                               /**< Definition of battery timer. */

static uint8_t m_range_round[TAG_NUM];  /**< Range rounde data. >**/                                    
static uint8_t status_flag;                           /**< Flag for Starting the Timer task to make Block. >**/    
static uint8_t Handler1[TAG_NUM];                           /**< Handler1[conn_handle] = Round_ID >**/
static uint8_t Handler2[TAG_NUM];                           /**< Handler2[Round_ID] = conn_handle >**/
static uint8_t TAG_ID[TAG_NUM];                             /**< TAG_ID[Round_ID] = TAG_ID >**/
static uint8_t round;                                 /**< Arranged round >**/
static uint8_t block_cnt;                             /**< Count of the single block > **/
static uint8_t Round_Max_ID;                          /**< Definition of the max of range at single block. For BLE >**/
static double Dist_Max_ID;                            /**< Definition of the max of range. For BLE >**/
static uint8_t Out_of_Range_ID;                       /**< Out-of-Range round id. For BLE >**/
static uint8_t Round_Warning_ID;                      /**< Definition of the Warning of round id. For BLE >**/
static uint8_t SDES_Flag;                             /**< Session Discard for Establish SEND Flag. For BLE & UWB >**/
static uint8_t SDESA_Flag;                            /**< Session Discard for Establish SEND ACK Flag. For BLE & UWB >**/

static QueueHandle_t xQueue;

typedef struct Task_Message
{
    uint8_t round;
    uint8_t ble_evt;
} T_Msg;

#define QUEUE_LENGTH  10
#define QUEUE_ITEM_SIZE sizeof( T_Msg )

static TickType_t xLastWakeTime;

#if NRF_LOG_ENABLED
static TaskHandle_t m_logger_thread;                                /**< Definition of Logger thread. */
#endif
static TaskHandle_t uwb_thread;                                /**< Definition of Logger thread. */
#define FPU_EXCEPTION_MASK 0x0000009F
void FPU_IRQHandler(void)
{
        uint32_t *fpscr = (uint32_t *)(FPU->FPCAR+0x40);
        (void)__get_FPSCR();

        *fpscr = *fpscr & ~(FPU_EXCEPTION_MASK);
}

void test_run_info(unsigned char *data)
{
    printf("%s\n", data);
}


//void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
//{
//        app_error_handler(DEAD_BEEF, line_num, p_file_name);
//}


/**@brief Function for Sending Alert message to peripheral BLE.
 *
 * @param[in]   Round_ID   
 */
static void Alert_MSG_SEND(uint8_t Round_ID)
{
        ret_code_t send_check;
        uint8_t conn_handle;

        conn_handle = Handler2[Round_ID-1];

        printf("\n<info> Alert_MSG_SEND ");
        uint8_t data_array[4] = {0x03, ANC_ID, TAG_ID[Round_ID], 1};
        static uint16_t index = 4;             
        send_check = ble_nus_c_string_send(&m_ble_nus_c[conn_handle], data_array, index);
        APP_ERROR_CHECK(send_check);
        printf("Finish Alert_MSG. \n\n");
}

/**@brief Function for Sending Session_Discard message to peripheral BLE.
 *
 * @param[in]   Round_ID   
 */
static void Session_Discard_SEND(uint8_t Round_ID)
{
        ret_code_t send_check;
        uint8_t conn_handle;

        printf("\n<info> Session_Discard_SEND ");
        conn_handle = Handler2[Round_ID-1];
        uint8_t data_array[4] = {0x02, TAG_ID[Round_ID-1], ANC_ID, 0};
        
        m_range_round[Round_ID] = 0;
        Handler1[conn_handle] = 0;
        Handler2[Round_ID-1] = 0;
        TAG_ID[Round_ID-1] = 0; 

        if (Check_Round()){
            status_flag = 0;
        } 

        static uint16_t index = 4;               
        send_check = ble_nus_c_string_send(&m_ble_nus_c[conn_handle], data_array, index);
        APP_ERROR_CHECK(send_check);

        //send_check = sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        //APP_ERROR_CHECK(send_check);
        printf("Finish Session_Discard. \n\n");
}

/**@brief Function for Sending Session Discard for Establish message to peripheral BLE.
 *
 * @param[in]   Round_ID   
 *              conn_handle
 */
static void Session_Discard_for_Establish_SEND(uint8_t Round_ID, uint16_t conn_handle)
{
        ret_code_t send_check;
        uint16_t used_conn_handle;

        printf("\n<info> Session_Discard_for_Establish_SEND ");
        used_conn_handle = Handler2[Round_ID-1];
        uint8_t data_array[4] = {0x02, TAG_ID[Round_ID-1], ANC_ID, 0};      

        static uint16_t index = 4;                
        send_check = ble_nus_c_string_send(&m_ble_nus_c[used_conn_handle], data_array, index);
        APP_ERROR_CHECK(send_check);

        Round_ID = Handler1[used_conn_handle];

        //send_check = sd_ble_gap_disconnect(used_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        //APP_ERROR_CHECK(send_check);

        Handler2[conn_handle] = conn_handle;

        uint8_t data_array1[3] = {0x01, ANC_ID, Round_ID};
        index = 3;
        send_check = ble_nus_c_string_send(&m_ble_nus_c[conn_handle], data_array1, index);
        APP_ERROR_CHECK(send_check);

        printf("Finish Session_Discard_for_Establish_SEND. \n\n");
}



/**@brief Function to start scanning.
 */
static void scan_start(void)
{
        ret_code_t err_code;
       
        err_code = nrf_ble_scan_start(&m_scan);
        APP_ERROR_CHECK(err_code);

        err_code = bsp_indication_set((BSP_INDICATE_SCANNING));        
        APP_ERROR_CHECK(err_code);

}

/**@brief Function for handling Scaning events.
 *
 * @param[in]   p_scan_evt   Scanning event.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
        ret_code_t err_code;

        switch(p_scan_evt->scan_evt_id)
        {
        case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
                err_code = p_scan_evt->params.connecting_err.err_code;
                APP_ERROR_CHECK(err_code);
                break;

        case NRF_BLE_SCAN_EVT_CONNECTED:
        {
                ble_gap_evt_connected_t const * p_connected =
                        p_scan_evt->params.connected.p_connected;
                // Scan is automatically stopped by the connection.
               
                //printf("<info> Connecting to target 0x%02x%02x%02x%02x%02x%02x\n",
                //             p_connected->peer_addr.addr[0],
                //             p_connected->peer_addr.addr[1],
                //             p_connected->peer_addr.addr[2],
                //             p_connected->peer_addr.addr[3],
                //             p_connected->peer_addr.addr[4],
                //             p_connected->peer_addr.addr[5]
                //             );
              
                //test_run_info(p_connected->peer_addr);
        } break;

        case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:        
                printf("Scan timed out.");
                scan_start();
                break;

        default:
                break;
        }
}

static void scan_init(void)
{
        ret_code_t err_code;
        nrf_ble_scan_init_t init_scan;

        memset(&init_scan, 0, sizeof(init_scan));

        init_scan.connect_if_match = true;
        init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

        err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
        APP_ERROR_CHECK(err_code);

        // Setting filters for scanning.
        err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER, false);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, m_target_periph_name);
        APP_ERROR_CHECK(err_code);
}

/**@brief Function for Arranging the Round_ID for peripheral BLE.
 *  
 *    @param[out]   Empty_Round[0]   
 */

int Arrange_Round(void){
    uint8_t cnt = 0;
    uint8_t Empty_Round[TAG_NUM]; 

    for (int i=0;i<TAG_NUM/2;i++)
    {
        if(m_range_round[2*i+1] == 0)
        {
          Empty_Round[cnt] = 2*i+1;
          cnt++;
        } 
    }

    for(int i=1;i<TAG_NUM/2;i++)
    {
        if (m_range_round[2*i] == 0)
        {
            Empty_Round[cnt] = 2*i;
            cnt++;
        }
    }

    return Empty_Round[0];
}

/**@brief Function for Checking the Empty round after BLE connection disable.
 *  
 *    @param[out]   0   - No empty round
 *                  1   - All round is empty
 */
int Check_Round(void)
{
    uint8_t cnt = 0;

    for(int i=1;i<TAG_NUM;i++)
    {
        if(m_range_round[i] == 1)
        {
            return 0;
        }
    }
    return 1;   
}

/**@brief Message handler for Received message from BLE peirpheral.
 *  
 *      Message format  1. ACK session [0x01, TAG_ID, 0xAC, Round_ID]
 *                      2. ACK Update  [0x02, ANC_ID, TAG_ID, 0xAC, Round_ID]                                         
 */
static void RECIVE_MSG_handler(uint8_t * r_data)
{
      T_Msg UWB_TO_BLE;
      const TickType_t xTicksToWait = pdMS_TO_TICKS(1);

      switch(r_data[0])
      {
        case 0x01:        
           if (r_data[2] == 0xAC)
           {
                 TAG_ID[r_data[3]-1] = r_data[1];
                 status_flag = 1;
                 if (SDES_Flag == 1)
                 {
                    SDESA_Flag = 1;
                 }
                 else
                 {
                    SDESA_Flag = 0;
                    UWB_TO_BLE.round = r_data[3];
                    //UWB_TO_BLE.ble_evt = 0;

                    xQueueSendToBack(xQueue, &UWB_TO_BLE, xTicksToWait);

                 }
                 printf("<info> Session Established \n");
           }
           else
           {
                 printf("%d", r_data[2]);
                 printf("<info> ACK Fail\n");
           }
           break;

        case 0x02:
           printf("<info> ACK Update \n");
           if (r_data[1] == ANC_ID && r_data[3] == 0xAC && TAG_ID[r_data[4]] == r_data[2])
           {
              printf("<info> Session Management\n TAG_ID: %d, round_id: %d \n\n", TAG_ID[r_data[4]], r_data[4]);            
           }           
           break;

        default:
           printf("<info> RECIVE_MSG_handler ERORR. \n\n"); 
      }
}

/**@brief Function for handling characters received by the Tag.
 */
static void ble_nus_chars_received_uart_print(uint8_t * p_data, uint16_t data_len)
{
        ret_code_t ret_val;

        printf("<info> Receiving data.");
        NRF_LOG_HEXDUMP_DEBUG(p_data, data_len);
        // Receive ACK MSG from Tag and checking the status
        RECIVE_MSG_handler(p_data);
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function receives a single character from the app_uart module and appends it to
 *          a string. The string is sent over BLE when the last character received is a
 *          'new line' '\n' (hex 0x0A) or if the string reaches the maximum data length.
 */
void uart_event_handle(app_uart_evt_t * p_event)
{
        static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
        static uint16_t index = 0;
        uint32_t ret_val;

        switch (p_event->evt_type)
        {
        /**@snippet [Handling data from UART] */
        case APP_UART_DATA_READY:
                UNUSED_VARIABLE(app_uart_get(&data_array[index]));
                index++;

                if ((data_array[index - 1] == '\n') || (index >= (m_ble_nus_max_data_len)))
                {
                        printf("<info> Ready to send data over BLE NUS\n");
                        NRF_LOG_HEXDUMP_DEBUG(data_array, index);

                        do
                        {
                                for (uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
                                {
                                        ret_val = ble_nus_c_string_send(&m_ble_nus_c[i], data_array, index);
//                                        if ( (ret_val != NRF_ERROR_INVALID_STATE) && (ret_val != NRF_ERROR_RESOURCES) )
//                                        {
//                                                APP_ERROR_CHECK(ret_val);
//                                        }
                                }
                        } while (ret_val == NRF_ERROR_RESOURCES);

                        index = 0;
                }
                break;

        /**@snippet [Handling data from UART] */
        case APP_UART_COMMUNICATION_ERROR:
                printf("<Error> Communication error occurred while handling UART.\n");
                APP_ERROR_HANDLER(p_event->data.error_communication);
                break;

        case APP_UART_FIFO_ERROR:
                 printf("<Error> Error occurred in FIFO module used by UART.\n");
                APP_ERROR_HANDLER(p_event->data.error_code);
                break;

        default:
                break;
        }
}

/**@brief Function for Session_Establish
 *       
  *      Message format  Session_Establish [0x01, ANC_ID, Round_ID]          
 */
static void Session_Establish_SEND(ble_nus_c_evt_t const * p_ble_nus_c_evt_RMS)
{
        ret_code_t send_check;

        round = Arrange_Round();
        Handler1[p_ble_nus_c_evt_RMS->conn_handle] = round;
        Handler2[round-1] = p_ble_nus_c_evt_RMS->conn_handle;
        m_range_round[round] = 1;
        uint8_t data_array[3] = {0x01, ANC_ID, round};
        printf("\n<info> Anc_id: %d, Arranged round: %d. \n", ANC_ID, round); 
        static uint16_t index = 3;         
        send_check = ble_nus_c_string_send(&m_ble_nus_c[p_ble_nus_c_evt_RMS->conn_handle], data_array, index);
        APP_ERROR_CHECK(send_check);
        //printf("<info> Finish RCM config message. \n\n");
}

/**@snippet [Handling events from the ble_nus_c module] */
static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_c_evt)
{
        ret_code_t err_code;
        uint16_t conn_handle;
        uint8_t Round_ID;

        switch (p_ble_nus_c_evt->evt_type)
        {
          case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
                  //printf("<info> NUS Service discovered on conn_handle 0x%x\n", p_ble_nus_c_evt->conn_handle);
                  err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_c_evt->conn_handle, &p_ble_nus_c_evt->handles);
                  APP_ERROR_CHECK(err_code);

                  //printf("<info> Before enable the tx notification\n");
                  NRF_LOG_HEXDUMP_DEBUG(p_ble_nus_c, sizeof(ble_nus_c_t));
                  err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c);
                  APP_ERROR_CHECK(err_code);

                  //printf("\n<info> Connected to device with Nordic UART Service.\n");
               
                  // To Distinguish the Session Establish & Session Discard
                  if(SDES_Flag != 1)
                  {
                      Session_Establish_SEND(p_ble_nus_c_evt);
                  }
                  else if(SDES_Flag ==1)
                  {
                      Session_Discard_for_Establish_SEND(Round_Max_ID, p_ble_nus_c_evt->conn_handle);                       
                  }
                  break;

          case BLE_NUS_C_EVT_NUS_TX_EVT:
                  ble_nus_chars_received_uart_print(p_ble_nus_c_evt->p_data, p_ble_nus_c_evt->data_len);
                  break;

          case BLE_NUS_C_EVT_DISCONNECTED:
                  //Handler1[p_ble_nus_c_evt->conn_handle] = round;
                  //Handler2[round] = p_ble_nus_c_evt->conn_handle;
                  //TAG_ID[round] = 0;
                  //m_range_round[round] = 0;
                  //Handler1[p_ble_nus_c_evt->conn_handle] = 0;
                  //Handler2[round] = 0;

                  //if(Check_Round == 0)
                  //{
                  //    status_flag = 0;
                  //}
                  printf("<info> Conn_handle %d is disconnected", p_ble_nus_c_evt->conn_handle);
                  printf("\n <info> Disconnected.\n");
                  scan_start();
                  break;
        }
}

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    uint32_t data_length;
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        printf("<info> Data len is set to 0x%X(%d)\n", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    else if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_DATA_LENGTH_UPDATED))
    {
        data_length = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH - 4;
        printf("<info> gatt_event: Data len is set to 0x%X (%d)\n", data_length, data_length);
        m_ble_nus_max_data_len = data_length;
    }
    //printf("<info> ATT MTU exchange completed. central 0x%x peripheral 0x%x\n", p_gatt->att_mtu_desired_central, p_gatt->att_mtu_desired_periph);
}

/**@brief Function for initializing the GATT module. */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_data_length_set(&m_gatt, BLE_CONN_HANDLE_INVALID, NRF_SDH_BLE_GAP_DATA_LENGTH);
    APP_ERROR_CHECK(err_code);
}

/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
        uint32_t err_code;
        app_uart_comm_params_t const comm_params =
        {
                .rx_pin_no    = RX_PIN_NUMBER,
                .tx_pin_no    = TX_PIN_NUMBER,
                .rts_pin_no   = RTS_PIN_NUMBER,
                .cts_pin_no   = CTS_PIN_NUMBER,
                .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
                .use_parity   = false,
#if defined (UART_PRESENT)
                .baud_rate    = NRF_UART_BAUDRATE_115200
#else
                .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
        };

        APP_UART_FIFO_INIT(&comm_params,
                           UART_RX_BUF_SIZE,
                           UART_TX_BUF_SIZE,
                           uart_event_handle,
                           APP_IRQ_PRIORITY_LOWEST,
                           err_code);
        APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
        //uint32_t err_code;
        ret_code_t err_code;

        // For readability.
        ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

        switch (p_ble_evt->header.evt_id)
        {
            case BLE_GAP_EVT_CONNECTED:
                printf("<info> Connection 0x%x established, starting DB discovery.\n", p_gap_evt->conn_handle);

                APP_ERROR_CHECK_BOOL(p_gap_evt->conn_handle < NRF_SDH_BLE_CENTRAL_LINK_COUNT);

                err_code = ble_nus_c_handles_assign(&m_ble_nus_c[p_gap_evt->conn_handle], p_gap_evt->conn_handle, NULL);
                APP_ERROR_CHECK(err_code);

                err_code = ble_db_discovery_start(&m_db_disc[p_gap_evt->conn_handle], p_gap_evt->conn_handle);
                APP_ERROR_CHECK(err_code);

                if (ble_conn_state_central_conn_count() == NRF_SDH_BLE_CENTRAL_LINK_COUNT)
                {                      
                      //uint16_t conn_handle;
                      //uint8_t Round_ID;
                      // Session Management
                      //Round_Max_ID
                      SDES_Flag = 1;
                      //Session_Discard_for_Establish_SEND(Round_Max_ID, p_gap_evt->conn_handle);                                           
                }
                else
                {
                      // Resume scanning.
                      err_code = bsp_indication_set(BSP_INDICATE_SCANNING);
                      APP_ERROR_CHECK(err_code);
                      scan_start();
                }
                break;

            case BLE_GAP_EVT_DISCONNECTED:
            {
                printf("\n <info> Central link 0x%x disconnected (reason: 0x%x)\n", p_gap_evt->conn_handle, p_gap_evt->params.disconnected.reason);
              
                round = Handler1[p_gap_evt->conn_handle];
                Handler2[round-1] = p_gap_evt->conn_handle;
                TAG_ID[round-1] = 0;
                m_range_round[round] = 0;
                Handler1[p_gap_evt->conn_handle] = 0;
                Handler2[round-1] = 0;

                //int i = Check_Round();
                if(Check_Round())
                {
                    status_flag = 0;
                }

                switch (p_gap_evt->conn_handle)
                {
                    case 0:
                        bsp_board_led_off(CENTRAL_CONNECTED_1_LED);
                        break;
                    case 1:
                        bsp_board_led_off(CENTRAL_CONNECTED_2_LED);
                        break;
                }

                if (ble_conn_state_central_conn_count() == 0)
                {
                    // Turn off the LED that indicates the connection.
                    //bsp_board_led_off(CENTRAL_CONNECTED_LED);
                    err_code = bsp_indication_set(BSP_INDICATE_SCANNING);
                    APP_ERROR_CHECK(err_code);
                }
                scan_start();
            } break;

            case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            {
                // Accept parameters requested by peer.
                err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle, &p_gap_evt->params.conn_param_update_request.conn_params);
                APP_ERROR_CHECK(err_code);
            } break;

           case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
            {
                printf("<info> PHY update request.\n");
                ble_gap_phys_t const phys =
                {
                        .rx_phys = BLE_GAP_PHY_AUTO,
                        .tx_phys = BLE_GAP_PHY_AUTO,
                };
                err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
                APP_ERROR_CHECK(err_code);
            } break;

            case BLE_GAP_EVT_TIMEOUT:
            {
                // We have not specified a timeout for scanning, so only connection attemps can timeout.
                if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
                {
                    printf("<info> Connection request timed out.\n");
                }
            } break;

            case BLE_GATTC_EVT_TIMEOUT:
                // Disconnect on GATT Client timeout event.
                printf("<info> GATT Client Timeout.\n");
                err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
                break;

            case BLE_GATTS_EVT_TIMEOUT:
                // Disconnect on GATT Server timeout event.
                printf("<info> GATT Server Timeout.\n");
                err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
                break;

            //case BLE_GAP_EVT_ADV_REPORT:
            //    printf("BLE_GAP_EVT_ADV_REPORT\n\n");  
            //    break;
                 
            default:
                // No implementation needed.
                break;
        } // Switch p_ble_evt->header.evt_id
}


/**@brief Function for initializing the Nordic UART Service (NUS) client. */
static void nus_c_init(void)
{
        ret_code_t err_code;
        ble_nus_c_init_t init;

        init.evt_handler = ble_nus_c_evt_handler;

        for (uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
        {
            err_code = ble_nus_c_init(&m_ble_nus_c[i], &init);
            APP_ERROR_CHECK(err_code);
        }
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

        ble_cfg_t ble_cfg;
        // Configure the GATTS attribute table.
        memset(&ble_cfg, 0x00, sizeof(ble_cfg));
        ble_cfg.gap_cfg.role_count_cfg.periph_role_count  = NRF_SDH_BLE_PERIPHERAL_LINK_COUNT;
        ble_cfg.gap_cfg.role_count_cfg.central_role_count = NRF_SDH_BLE_CENTRAL_LINK_COUNT;
        ble_cfg.gap_cfg.role_count_cfg.qos_channel_survey_role_available = false; /* Enable channel survey role */

        err_code = sd_ble_cfg_set(BLE_GAP_CFG_ROLE_COUNT, &ble_cfg, &ram_start);
        if (err_code != NRF_SUCCESS)
        {
             printf("<Error> sd_ble_cfg_set() returned %s when attempting to set BLE_GAP_CFG_ROLE_COUNT.\n", nrf_strerror_get(err_code));
        }

        // Enable BLE stack.
        err_code = nrf_sdh_ble_enable(&ram_start);
        APP_ERROR_CHECK(err_code);

        err_code = sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
        APP_ERROR_CHECK(err_code);

        err_code = sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
        APP_ERROR_CHECK(err_code);


        // Register a handler for BLE events.
        NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
        printf("<info> call to ble_nus_c_on_db_disc_evt for instance %d and link 0x%x!\n", p_evt->conn_handle, p_evt->conn_handle);
        ble_nus_c_on_db_disc_evt(&m_ble_nus_c[p_evt->conn_handle], p_evt);

}

/**@brief Database discovery initialization.
 */
static void db_discovery_init(void)
{
        ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
        APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
static void bsp_event_handler(bsp_event_t event)
{
        ret_code_t err_code;

        switch (event)
        {
            case BSP_EVENT_DISCONNECT:
                err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
                break;

            case BSP_EVENT_WHITELIST_OFF:
//                if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
//                {
//                        err_code = ble_advertising_restart_without_whitelist(&m_advertising);
//                        if (err_code != NRF_ERROR_INVALID_STATE)
//                        {
//                                APP_ERROR_CHECK(err_code);
//                        }
//                }
                break;

            default:
                break;
        }
}



/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
        ret_code_t err_code = NRF_LOG_INIT(NULL);
        APP_ERROR_CHECK(err_code);

        NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
        ret_code_t err_code;
        bsp_event_t startup_event;

        err_code = bsp_init(BSP_INIT_LEDS, bsp_event_handler);
        APP_ERROR_CHECK(err_code);

        err_code = bsp_btn_ble_init(NULL, &startup_event);
        APP_ERROR_CHECK(err_code);

        *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


#if NRF_LOG_ENABLED
static void logger_thread(void * arg)
{
        UNUSED_PARAMETER(arg);

        while (1)
        {
                NRF_LOG_FLUSH();

                vTaskSuspend(NULL); // Suspend myself
        }
}
#endif 

/**@brief A function which is hooked to idle task.
 * @note Idle hook must be enabled in FreeRTOS configuration (configUSE_IDLE_HOOK).
 */
void vApplicationIdleHook( void )
{
#if NRF_LOG_ENABLED
        vTaskResume(m_logger_thread);
#endif
}


/**@brief Function for initializing the clock.
 */
static void clock_init(void)
{
        ret_code_t err_code = nrf_drv_clock_init();
        APP_ERROR_CHECK(err_code);
}

static void timer_event(void)
{
    if (status_flag == 1)
    {
            //xTaskNotify(uwb_thread, (1<<0), eSetBits);
            if (cnt == 0){            
                xTaskNotify(uwb_thread, (0<<0), eSetBits);
                cnt++;
            }
            else if (cnt == 1){
                xTaskNotify(uwb_thread, (1<<0), eSetBits);
                cnt++;
            }
            else if (cnt == 2){
                xTaskNotify(uwb_thread, (2<<0), eSetBits);
                cnt++;
            }
            else if (cnt == 3){
                xTaskNotify(uwb_thread, (3<<0), eSetBits);
                cnt++;
            }
            else if (cnt == 4){
                xTaskNotify(uwb_thread, (4<<0), eSetBits);
                cnt++;
            }
            else if (cnt == 5){
                xTaskNotify(uwb_thread, (5<<0), eSetBits);
                cnt++;
            }
            else if (cnt == 6){
                xTaskNotify(uwb_thread, (6<<0), eSetBits);
                cnt++;
            }
            else if (cnt == 7){
                xTaskNotify(uwb_thread, (7<<0), eSetBits);
                cnt++;
                
                //block_cnt++;
                //if (block_cnt % 41 == 0)
                //{

                //    block_cnt = 0;
                //}
            }
            else if (cnt > 7 & cnt != 15){
                xTaskNotify(uwb_thread, (8<<0), eSetBits);
                cnt++;
            }
            else if (cnt == 15)
            {
                cnt = 0;
            }
        //printf("TaskNotify: %d \n\n", cnt);
   } // IF status_flag
   else
   {
    ;;
   }
}
/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
        // Initialize timer module.
        ret_code_t err_code = app_timer_init();
        APP_ERROR_CHECK(err_code);

        // Create timers.
        uwb_timer = xTimerCreate("UWB",
                                 UWB_ROUND_INTERVAL,
                                 pdTRUE,
                                 NULL,
                                 timer_event);
        /* Error checking */
        if ( (NULL == uwb_timer))
        {
                APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
        }
}



/**@brief   Function for starting application timers.
 * @details Timers are run after the scheduler has started.
 */
 
static void uwb_timers_start(void)
{
        // Start application timers.
        if (pdPASS != xTimerStart(uwb_timer, OSTIMER_WAIT_FOR_QUEUE))
        {
                APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
        }
}

/**@brief Function for application main entry.
 */
int main(void)
{
        bool erase_bonds;
        uart_init();
        log_init();
        clock_init();
        nrf_drv_clock_lfclk_request(NULL);
        APP_ERROR_CHECK(nrf_stack_guard_init());
      
        printf("UWB Anchor Start.\n");
        
        if (pdPASS != xTaskCreate(logger_thread, "LOGGER", 128, NULL, 1, &m_logger_thread))
        {
                APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
        }
        
        // Configure and initialize the BLE stack.
        ble_stack_init();

        // Initialize modules.
        buttons_leds_init(&erase_bonds);
        scan_init();
        gatt_init();
        db_discovery_init();
        nus_c_init();
        ble_conn_state_init();
        
        // UWB initialize
        nrf52840_dk_spi_init();
        dw_irq_init();
        nrf_drv_gpiote_in_event_disable(DW3000_IRQn_Pin);
        nrf_delay_ms(2);
 
        // UWB timer Start
        timers_init();
        uwb_timers_start();

        nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0,24));
        
        xQueue = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);

        if (pdPASS != xTaskCreate(ds_resp, "UWB", 1024, NULL, 0, &uwb_thread))
        {
                APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
        }
        
        // Create a FreeRTOS task for the BLE stack.
        nrf_sdh_freertos_init(scan_start, NULL);

        printf("UWB & BLE RTOS Start.\n");
        vTaskStartScheduler();
        for (;;)
        {
                APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
        }
}


int ds_resp(void)
{
    const TickType_t  xTicksToWait = pdMS_TO_TICKS(1);

    uint32_t ulNotifiedValue;                 /**<Notification from Timer. >**/  
    double Dist_Max;                          /**< Storing the longest Dist in single block. >**/  
    uint8_t Round_Max;                        /**< Storing the longest Round_ID in single block. >**/  
    uint8_t OutofRange[TAG_NUM];              /**< Checking the Out-of-Range Round_ID. >**/ 
    //uint8_t Safe_Check[7] = {0,0,0,0,0,0,0};  /**< Checking the Safe Range Round_ID. >**/ 
    uint8_t Range_Round[TAG_NUM];
    T_Msg FROM_BLE;

    port_set_dw_ic_spi_fastrate();

    reset_DWIC(); /* Target specific drive of RSTn line into DW IC low for a period. */

    Sleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC

    while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */
    { };

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        test_run_info((unsigned char *)"INIT FAILED     ");
        while (1)
        { };
    }

    /* Configure DW IC. See NOTE 15 below. */
    if(dwt_configure(&config)) /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
    {
        test_run_info((unsigned char *)"CONFIG FAILED     ");
        while (1)
        { };
    }

    /* Configure the TX spectrum parameters (power, PG delay and PG count) */
    dwt_configuretxrf(&txconfig_options);

    /* Apply default antenna delay value. See NOTE 1 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug, and also TX/RX LEDs
     * Note, in real low power applications the LEDs should not be used. */
    //dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

    /* Loop forever responding to ranging requests. */
    while (1)
    {
        if( xTaskNotifyWait( 0xFFFFFFFF, 0, &ulNotifiedValue, portMAX_DELAY) == pdTRUE)
        {          
            if( ulNotifiedValue == 0 )
            {
                  rcm_tx();                        
                  if ( xQueueReceive(xQueue, &FROM_BLE, xTicksToWait) != pdPASS)
                  {
                      ;                      
                  }
                  else
                  {
                      Range_Round[FROM_BLE.round] = 1;
                  }
                  Round_Max = 0;
                  Dist_Max = 0;            
            }
            else if( ulNotifiedValue != 0)
            {
                  if ( Range_Round[ulNotifiedValue] == 1 && ulNotifiedValue < TAG_NUM)
                  {             
                               nrf_gpio_pin_toggle(NRF_GPIO_PIN_MAP(0,24));  
                              //printf("This is DS_RESP \n\n");    
                              dwt_setpreambledetecttimeout(0);
                              /* Clear reception timeout to start next ranging process. */
                              dwt_setrxtimeout(10000);

                              /* Activate reception immediately. */
                              dwt_rxenable(DWT_START_RX_IMMEDIATE);

                              /* Poll for reception of a frame or error/timeout. See NOTE 8 below. */
                              while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
                              { };
        
                              if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
                              {
                                  uint32_t frame_len;

                                  /* Clear good RX frame event in the DW IC status register. */
                                  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

                                  /* A frame has been received, read it into the local buffer. */
                                  frame_len = dwt_read32bitreg(RX_FINFO_ID) & FRAME_LEN_MAX_EX;
                                  if (frame_len <= RX_BUF_LEN)
                                  {
                                      dwt_readrxdata(rx_buffer, frame_len, 0);
                                  }

                                  /* Check that the frame is a poll sent by "DS TWR initiator" example.
                                   * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
                                  rx_buffer[ALL_MSG_SN_IDX] = 0;
                                  if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)
                                  {
                                      uint32_t resp_tx_time;
                                      int ret;

                                      /* Retrieve poll reception timestamp. */
                                      poll_rx_ts = get_rx_timestamp_u64();

                                      /* Set send time for response. See NOTE 9 below. */
                                      resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                                      dwt_setdelayedtrxtime(resp_tx_time);

                                      /* Set expected delay and timeout for final message reception. See NOTE 4 and 5 below. */
                                      dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
                                      dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);
                                      /* Set preamble timeout for expected frames. See NOTE 6 below. */
                                      dwt_setpreambledetecttimeout(PRE_TIMEOUT);

                                      /* Write and send the response message. See NOTE 10 below.*/
                                      tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                                      dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); /* Zero offset in TX buffer. */
                                      dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
                                      ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

                                      /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 11 below. */
                                      if (ret == DWT_ERROR)
                                      {
                                          continue;
                                      }

                                      /* Poll for reception of expected "final" frame or error/timeout. See NOTE 8 below. */
                                      while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
                                      { };

                                      /* Increment frame sequence number after transmission of the response message (modulo 256). */
                                      frame_seq_nb++;

                                      if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
                                      {
                                          /* Clear good RX frame event and TX frame sent in the DW IC status register. */
                                          dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_TXFRS_BIT_MASK);

                                          /* A frame has been received, read it into the local buffer. */
                                          frame_len = dwt_read32bitreg(RX_FINFO_ID) & FRAME_LEN_MAX_EX;
                                          if (frame_len <= RX_BUF_LEN)
                                          {
                                              dwt_readrxdata(rx_buffer, frame_len, 0);
                                          }

                                          /* Check that the frame is a final message sent by "DS TWR initiator" example.
                                           * As the sequence number field of the frame is not used in this example, it can be zeroed to ease the validation of the frame. */
                                          rx_buffer[ALL_MSG_SN_IDX] = 0;
                                          if (memcmp(rx_buffer, rx_final_msg, ALL_MSG_COMMON_LEN) == 0)
                                          {
                                              uint32_t poll_tx_ts, resp_rx_ts, final_tx_ts;
                                              uint32_t poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
                                              double Ra, Rb, Da, Db;
                                              int64_t tof_dtu;

                                              /* Retrieve response transmission and final reception timestamps. */
                                              resp_tx_ts = get_tx_timestamp_u64();
                                              final_rx_ts = get_rx_timestamp_u64();

                                              /* Get timestamps embedded in the final message. */
                                              final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
                                              final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
                                              final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);
                                              /* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. See NOTE 12 below. */
                                              poll_rx_ts_32 = (uint32_t)poll_rx_ts;
                                              resp_tx_ts_32 = (uint32_t)resp_tx_ts;
                                              final_rx_ts_32 = (uint32_t)final_rx_ts;
                                              Ra = (double)(resp_rx_ts - poll_tx_ts);
                                              Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
                                              Da = (double)(final_tx_ts - resp_rx_ts);
                                              Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
                                              tof_dtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

                                              tof = tof_dtu * DWT_TIME_UNITS;
                                              distance = tof * SPEED_OF_LIGHT;

                                              ///* Display computed distance on LCD. */
                                              //sprintf(dist_str, "@@@ DIST: %3.2f m @@@\n", distance);
                                              printf("@@@ DIST %d: %3.2f m @@@ \n", ulNotifiedValue, distance);
                                              //test_run_info((unsigned char *)dist_str);

                                              /* as DS-TWR initiator is waiting for RNG_DELAY_MS before next poll transmission
                                               * we can add a delay here before RX is re-enabled again
                                               */
                                              //Sleep(RNG_DELAY_MS - 10);  //start couple of ms earlier

                                              // Check the Out of Range Round.
                                              if (distance > Range_Check)
                                              {
                                                  // Safe_Check Algorithm
                                                  OutofRange[ulNotifiedValue]++; 
                                                  if (OutofRange[ulNotifiedValue] == 3)
                                                  {                                                
                                                      Out_of_Range_ID = ulNotifiedValue;
                                                      printf("@@ Out of Range TAG %d @@ \n", Out_of_Range_ID);
                                                      Session_Discard_SEND(Out_of_Range_ID);
                                                      OutofRange[ulNotifiedValue] = 0;
                                                  }
                                              }
                                              else if (distance <= Range_Check)
                                              {
                                                  OutofRange[ulNotifiedValue] = 0;
                                              }
                                        
                                              // Finding the largest Distance in Single Block.
                                              //if (Round_Max_ID == ulNotifiedValue && SDES_Flag == 1 && SDESA_Flag == 1)
                                              //{
                                              //    if (distance > Dist_Max_ID)
                                              //    {
                                              //        // remove data about ble (LAST_ID)
                                              //        Handler1[7] = 0;
                                              //        Handler2[7-1] = 0;
                                              //        TAG_ID[7-1] = 0;   
                                              //        Session_Discard_SEND(7);                                           
                                              //    }
                                              //    else
                                              //    {
                                              //        Session_Discard_SEND(Round_Max_ID);       
                                              //    }
                                              //}

                                              //if (distance > Dist_Max)
                                              //{
                                              //    // Check Max Dist Algorithm
                                              //    Dist_Max = distance;
                                              //    Round_Max = ulNotifiedValue;

                                              //    Round_Max_ID = Round_Max;
                                              //    Dist_Max_ID = Dist_Max;                                                                                     
                                              //}
                                          }
                                      }
                                      else
                                      {
                                          /* Clear RX error/timeout events in the DW IC status register. */
                                          dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
                                      }
                                  }
                              }
                              else
                              {
                                  /* Clear RX error/timeout events in the DW IC status register. */
                                  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
                              }
                       }//if value check
                       else if (ulNotifiedValue > TAG_NUM)
                       {
                          ;;
                       }
              	} // elseif !=0 
        }//if signalwait
        else
        {
            printf("Wait \n\n");
        }
    } // while(1) 
}

int rcm_tx(void)
{
      //printf("This is simple_tx \n\n");

      dwt_writetxdata(sizeof(rcm_msg), rcm_msg, 0); /* Zero offset in TX buffer. */
      dwt_writetxfctrl(sizeof(rcm_msg), 0, 0); /* Zero offset in TX buffer, no ranging. */

      /* Start transmission. */
      //nrf_gpio_pin_toggle(NRF_GPIO_PIN_MAP(0,24));  
      dwt_starttx(DWT_START_TX_IMMEDIATE); //DWT_START_TX_DELAYED
      while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
      {};

      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);     
}