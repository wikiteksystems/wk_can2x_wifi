#include "bt_service.h"


/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ESP_LOG.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#include "time.h"
#include "sys/time.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"


#include <stdbool.h>

#include <instructions.h>
#include <sock_service.h>
    
#define SPP_TAG "Bluetooth"
#define SPP_SERVER_NAME "SPP_SERVER"
#define EXAMPLE_DEVICE_NAME "AP-000000001"
#define SPP_SHOW_DATA 0
#define SPP_SHOW_SPEED 1
#define SPP_SHOW_MODE SPP_SHOW_DATA    /*Choose show mode: show data or speed*/

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;

// static struct timeval time_new, time_old;


static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;



// 

#define RX_QUEUE_SIZE 5100
#define TX_QUEUE_SIZE 1000
#define SPP_TX_QUEUE_TIMEOUT 1000
#define SPP_TX_DONE_TIMEOUT 1000
#define SPP_CONGESTED_TIMEOUT 1000
static QueueHandle_t _spp_rx_queue ;
static QueueHandle_t _spp_tx_queue ;
static TaskHandle_t _spp_task_handle = NULL;
static SemaphoreHandle_t _spp_tx_done = NULL;
static EventGroupHandle_t _spp_event_group = NULL;
static EventGroupHandle_t _bt_event_group = NULL;

static bool secondConnectionAttempt;
uint8_t BTrxData[5000] ;
uint16_t TotalBTBytes;
uint8_t Noofbyteswithoutpayload=5;

static uint32_t _spp_client = 0;



const uint16_t SPP_TX_MAX = 1200;
static uint8_t _spp_tx_buffer[1200];
static uint16_t _spp_tx_buffer_len = 0;

// uint8_t *BTrxData = NULL;
typedef struct {
        size_t len;
        uint8_t data[];
} spp_packet_t;

// _spp_event_group
#define SPP_RUNNING     0x01
#define SPP_CONNECTED   0x02
#define SPP_CONGESTED   0x04
// true until OPEN successful, changes to false on CLOSE
#define SPP_DISCONNECTED 0x08
// true until connect(), changes to true on CLOSE
#define SPP_CLOSED      0x10

// _bt_event_group
#define BT_DISCOVERY_RUNNING    0x01
#define BT_DISCOVERY_COMPLETED  0x02

#define BT_SDP_RUNNING          0x04
#define BT_SDP_COMPLETED        0x08


// 

// static char *bda2str(uint8_t * bda, char *str, size_t size)
// {
//     if (bda == NULL || str == NULL || size < 18) {
//         return NULL;
//     }

//     uint8_t *p = bda;
//     sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
//             p[0], p[1], p[2], p[3], p[4], p[5]);
//     return str;
// }


static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    // char bda_str[18] = {0};

    switch (event) {
    case ESP_SPP_INIT_EVT:
        if (param->init.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
            esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
        } else {
            ESP_LOGE(SPP_TAG, "ESP_SPP_INIT_EVT status:%d", param->init.status);
        }
        break;

    case ESP_SPP_DISCOVERY_COMP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
        break;

    case ESP_SPP_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
        if (!_spp_client){
                _spp_client = param->open.handle;
        } else {
            secondConnectionAttempt = true;
            esp_spp_disconnect(param->open.handle);
        }
        xEventGroupClearBits(_spp_event_group, SPP_DISCONNECTED);
        xEventGroupSetBits(_spp_event_group, SPP_CONNECTED);
        xEventGroupSetBits(_spp_event_group, SPP_CONGESTED);
        break;
        break;
    case ESP_SPP_CLOSE_EVT:
        // ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT status:%d handle:%d close_by_remote:%d", param->close.status,
        //          param->close.handle, param->close.async);
        if ((param->close.async == false && param->close.status == ESP_SPP_SUCCESS) || param->close.async) {
            ESP_LOGI(SPP_TAG,"ESP_SPP_CLOSE_EVT status:%u handle:%"PRIu32" close_by_remote:%d attempt %u", param->close.status,
                 param->close.handle, param->close.async, secondConnectionAttempt);
            if(secondConnectionAttempt) {
                secondConnectionAttempt = false;
            } else {
                _spp_client = 0;
                xEventGroupSetBits(_spp_event_group, SPP_DISCONNECTED);
                xEventGroupSetBits(_spp_event_group, SPP_CONGESTED);
                xEventGroupSetBits(_spp_event_group, SPP_CLOSED);
                xEventGroupClearBits(_spp_event_group, SPP_CONNECTED);
            }        
        } else {
            ESP_LOGE(SPP_TAG,"ESP_SPP_CLOSE_EVT failed!, status:%d", param->close.status);
        }
        break;
    case ESP_SPP_START_EVT:
        if (param->start.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT handle:%"PRIu32" sec_id:%d scn:%d", param->start.handle, param->start.sec_id,
                     param->start.scn);
            esp_bt_dev_set_device_name(EXAMPLE_DEVICE_NAME);
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        } else {
            ESP_LOGE(SPP_TAG, "ESP_SPP_START_EVT status:%d", param->start.status);
        }
        break;
    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
           if (param->cl_init.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(SPP_TAG,"ESP_SPP_CL_INIT_EVT handle:%"PRIu32" sec_id:%d", param->cl_init.handle, param->cl_init.sec_id);
        } else {
            ESP_LOGI(SPP_TAG,"ESP_SPP_CL_INIT_EVT status:%d", param->cl_init.status);
        }
        break;
    case ESP_SPP_DATA_IND_EVT: //dataevent

        // //ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT len:%d handle:%d",
        //          param->data_ind.len, param->data_ind.handle);
        // if (param->data_ind.len < 128) {
        //     //ESP_LOG_buffer_hex("", param->data_ind.data, param->data_ind.len);
        // }
        if (_spp_rx_queue != NULL){
            for (int i = 0; i < param->data_ind.len; i++){
                if(xQueueSend(_spp_rx_queue, param->data_ind.data + i, (TickType_t)0) != pdTRUE){
                    ESP_LOGI(SPP_TAG,"RX Full! Discarding %u bytes", param->data_ind.len - i);
                    break;
                }
            }
        }
        break;
    
    case ESP_SPP_CONG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT");
        if(param->cong.cong){
            xEventGroupClearBits(_spp_event_group, SPP_CONGESTED);
        } else {
            xEventGroupSetBits(_spp_event_group, SPP_CONGESTED);
        }
        ESP_LOGV(SPP_TAG,"ESP_SPP_CONG_EVT: %s", param->cong.cong?"CONGESTED":"FREE");
        break;

    case ESP_SPP_WRITE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT");
        if (param->write.status == ESP_SPP_SUCCESS) {
            if(param->write.cong){
                xEventGroupClearBits(_spp_event_group, SPP_CONGESTED);
            }
            // log_v("ESP_SPP_WRITE_EVT: %u %s", param->write.len, param->write.cong?"CONGESTED":"");
        } else {
            ESP_LOGE(SPP_TAG,"ESP_SPP_WRITE_EVT failed!, status:%d", param->write.status);
        }
        xSemaphoreGive(_spp_tx_done);//we can try to send another packet
        break;

    case ESP_SPP_SRV_OPEN_EVT:
        // //ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT status:%d handle:%d, rem_bda:[%s]", param->srv_open.status,
        //          param->srv_open.handle, bda2str(param->srv_open.rem_bda, bda_str, sizeof(bda_str)));
        // gettimeofday(&time_old, NULL);
        if (param->srv_open.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(SPP_TAG,"ESP_SPP_SRV_OPEN_EVT: %"PRIu32"", _spp_client);
            if (!_spp_client){
                _spp_client = param->srv_open.handle;
                _spp_tx_buffer_len = 0;
            } else {
                secondConnectionAttempt = true;
                esp_spp_disconnect(param->srv_open.handle);
            }
            xEventGroupClearBits(_spp_event_group, SPP_DISCONNECTED);
            xEventGroupSetBits(_spp_event_group, SPP_CONNECTED);
        } else {
            ESP_LOGE(SPP_TAG,"ESP_SPP_SRV_OPEN_EVT Failed!, status:%d", param->srv_open.status);
        }
        break;

    case ESP_SPP_SRV_STOP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_STOP_EVT");
        break;
    case ESP_SPP_UNINIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_UNINIT_EVT");
        break;
    default:
        break;
    }
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    // char bda_str[18] = {0};

    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:{
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            //ESP_LOGI(SPP_TAG, "authentication success: %s bda:[%s]", param->auth_cmpl.device_name,
                    //  bda2str(param->auth_cmpl.bda, bda_str, sizeof(bda_str)));
        } else {
            ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT:{
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
            ESP_LOGI(SPP_TAG, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    case ESP_BT_GAP_CFM_REQ_EVT:
        //ESP_LOGI(SPP_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d", param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        //ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%d", param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        //ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        break;
#endif

    case ESP_BT_GAP_MODE_CHG_EVT:
        // ESP_LOGI(SPP_TAG, "ESP_BT_GAP_MODE_CHG_EVT mode:%d bda:[%s]", param->mode_chg.mode,
        //          bda2str(param->mode_chg.bda, bda_str, sizeof(bda_str)));
        break;

    default: {
        ESP_LOGI(SPP_TAG, "event: %d", event);
        break;
    }
    }
    return;
}


int BTAvailable(void)
{
    if (_spp_rx_queue == NULL){
        return 0;
    }
    return uxQueueMessagesWaiting(_spp_rx_queue);
}


uint8_t BTRead()
{
    uint8_t c = 0;
    if (_spp_rx_queue && xQueueReceive(_spp_rx_queue, &c, pdMS_TO_TICKS(10000))){
        // return c;
    }
    return c;
}


uint8_t BTReadBytes(uint8_t *buffer, size_t size)
{
    // size_t avail = BTAvailable();
    // if (size < avail) {
    //     avail = size;
    // }
    size_t count = 0;
    uint8_t c;
    while(count < size) {
        c=BTRead();
        *(buffer++) = c;
        count++;
    }
    return count;
}

static esp_err_t _spp_queue_packet(uint8_t *data, size_t len){
    if(!data || !len){
        // log_w("No data provided");
        ESP_LOGW(SPP_TAG, "No data provided");
        return ESP_OK;
    }
    spp_packet_t * packet = (spp_packet_t*)malloc(sizeof(spp_packet_t) + len);
    if(!packet){
        // log_e("SPP TX Packet Malloc Failed!");
        ESP_LOGE(SPP_TAG,"SPP TX Packet Malloc Failed!" );
        return ESP_FAIL;
    }
    packet->len = len;
    memcpy(packet->data, data, len);
    if (!_spp_tx_queue || xQueueSend(_spp_tx_queue, &packet, SPP_TX_QUEUE_TIMEOUT) != pdPASS) {
        // log_e("SPP TX Queue Send Failed!");
        ESP_LOGE(SPP_TAG,"SPP TX Queue Send Failed!" );
        free(packet);
        return ESP_FAIL;
    }
    return ESP_OK;
}

size_t BTWrite(const uint8_t *buffer, size_t size)
{
    if (!_spp_client){
        return 0;
    }
    return (_spp_queue_packet((uint8_t *)buffer, size) == ESP_OK) ? size : 0;
}



static void _spp_rx_task(void * arg){

  for(;;)
    {

      if (BTAvailable()) {
        Activechannel=BT_CH;
        int btindex=0;
        BTrxData[btindex++]=BTRead();
        BTrxData[btindex++]=BTRead();
        TotalBTBytes=((uint16_t) (0x0F & BTrxData[0]) << 8)| (uint16_t) BTrxData[1];
        // ESP_LOGI(SPP_TAG, " Total byte to be in %d",TotalBTBytes);
        // for(int i=btindex;i<TotalBTBytes+2;i++)
        // {
            while(btindex<(TotalBTBytes+Noofbyteswithoutpayload)){
            BTrxData[btindex++]=BTRead();
            }
        // }
        processAcquiredData(&BTrxData[0],btindex);
    //    ESP_LOG_BUFFER_HEX(SPP_TAG,&BTrxData[0],btindex);
    //    BTWrite(BTrxData,btindex);

        }
    vTaskDelay(1);
    }

}



static bool _spp_send_buffer(){
    if((xEventGroupWaitBits(_spp_event_group, SPP_CONGESTED, pdFALSE, pdTRUE, SPP_CONGESTED_TIMEOUT) & SPP_CONGESTED) != 0){
        if(!_spp_client){
            ESP_LOGI(SPP_TAG,"SPP Client Gone!");
            return false;
        }
        ESP_LOGI(SPP_TAG,"SPP Write %u", _spp_tx_buffer_len);
        ESP_LOG_BUFFER_HEX(SPP_TAG,_spp_tx_buffer,_spp_tx_buffer_len);
        
        esp_err_t err = esp_spp_write(_spp_client, _spp_tx_buffer_len, _spp_tx_buffer);
        if(err != ESP_OK){
            ESP_LOGE(SPP_TAG,"SPP Write Failed! [0x%X]", err);
            return false;
        }
        _spp_tx_buffer_len = 0;
        if(xSemaphoreTake(_spp_tx_done, SPP_TX_DONE_TIMEOUT) != pdTRUE){
            ESP_LOGE(SPP_TAG,"SPP Ack Failed!");
            return false;
        }
        return true;
    }
    ESP_LOGE(SPP_TAG,"SPP Write Congested!");
    return false;
}

static void _spp_tx_task(void * arg){
    spp_packet_t *packet = NULL;
    size_t len = 0, to_send = 0;
    uint8_t * data = NULL;
    for (;;) {
        if(_spp_tx_queue && xQueueReceive(_spp_tx_queue, &packet, portMAX_DELAY) == pdTRUE && packet){
            if(packet->len <= (SPP_TX_MAX - _spp_tx_buffer_len)){
                memcpy(_spp_tx_buffer+_spp_tx_buffer_len, packet->data, packet->len);
                _spp_tx_buffer_len+=packet->len;
                free(packet);
                packet = NULL;
                if(SPP_TX_MAX == _spp_tx_buffer_len || uxQueueMessagesWaiting(_spp_tx_queue) == 0){
                    _spp_send_buffer();
                }
            } else {
                len = packet->len;
                data = packet->data;
                to_send = SPP_TX_MAX - _spp_tx_buffer_len;
                memcpy(_spp_tx_buffer+_spp_tx_buffer_len, data, to_send);
                _spp_tx_buffer_len = SPP_TX_MAX;
                data += to_send;
                len -= to_send;
                if(!_spp_send_buffer()){
                    len = 0;
                }
                while(len >= SPP_TX_MAX){
                    memcpy(_spp_tx_buffer, data, SPP_TX_MAX);
                    _spp_tx_buffer_len = SPP_TX_MAX;
                    data += SPP_TX_MAX;
                    len -= SPP_TX_MAX;
                    if(!_spp_send_buffer()){
                        len = 0;
                        break;
                    }
                }
                if(len){
                    memcpy(_spp_tx_buffer, data, len);
                    _spp_tx_buffer_len += len;
                    if(uxQueueMessagesWaiting(_spp_tx_queue) == 0){
                        _spp_send_buffer();
                    }
                }
                free(packet);
                packet = NULL;
            }
        } else {
            ESP_LOGE(SPP_TAG,"Something went horribly wrong");
        }
    }
    vTaskDelete(NULL);
    _spp_task_handle = NULL;
}

static bool _init_bt(){

    if(!_bt_event_group){
            _bt_event_group = xEventGroupCreate();
            if(!_bt_event_group){
                ESP_LOGE(SPP_TAG,"BT Event Group Create Failed!");
                return false;
            }
            xEventGroupClearBits(_bt_event_group, 0xFFFFFF);
    }

    if(!_spp_event_group){
        _spp_event_group = xEventGroupCreate();
        if(!_spp_event_group){
            ESP_LOGE(SPP_TAG,"SPP Event Group Create Failed!");
            return false;
        }
        xEventGroupClearBits(_spp_event_group, 0xFFFFFF);
        xEventGroupSetBits(_spp_event_group, SPP_CONGESTED);
        xEventGroupSetBits(_spp_event_group, SPP_DISCONNECTED);
        xEventGroupSetBits(_spp_event_group, SPP_CLOSED);
    }

    if (_spp_rx_queue == NULL){
            _spp_rx_queue = xQueueCreate(RX_QUEUE_SIZE, sizeof(uint8_t)); //initialize the queue
            if (_spp_rx_queue == NULL){
                ESP_LOGE(SPP_TAG,"RX Queue Create Failed");
                return false;
            }
        }

    if (_spp_tx_queue == NULL){
        _spp_tx_queue = xQueueCreate(TX_QUEUE_SIZE, sizeof(spp_packet_t*)); //initialize the queue
        if (_spp_tx_queue == NULL){
            ESP_LOGE(SPP_TAG,"TX Queue Create Failed");
            return false;
        }
    }

    if(_spp_tx_done == NULL){
    _spp_tx_done = xSemaphoreCreateBinary();
    if (_spp_tx_done == NULL){
        ESP_LOGE(SPP_TAG,"TX Semaphore Create Failed");
        return false;
    }
    xSemaphoreTake(_spp_tx_done, 0);

    }
    return true;
}

// static bool _stop_bt()
// {
//     if (btStarted()){
//         if(_spp_client)
//             esp_spp_disconnect(_spp_client);
//         esp_spp_deinit();
//         esp_bluedroid_disable();
//         esp_bluedroid_deinit();
//         btStop();
//     }
//     _spp_client = 0;
//     if(_spp_task_handle){
//         vTaskDelete(_spp_task_handle);
//         _spp_task_handle = NULL;
//     }
//     if(_spp_event_group){
//         vEventGroupDelete(_spp_event_group);
//         _spp_event_group = NULL;
//     }
//     if(_spp_rx_queue){
//         vQueueDelete(_spp_rx_queue);
//         //ToDo: clear RX queue when in packet mode
//         _spp_rx_queue = NULL;
//     }
//     if(_spp_tx_queue){
//         spp_packet_t *packet = NULL;
//         while(xQueueReceive(_spp_tx_queue, &packet, 0) == pdTRUE){
//             free(packet);
//         }
//         vQueueDelete(_spp_tx_queue);
//         _spp_tx_queue = NULL;
//     }
//     if (_spp_tx_done) {
//         vSemaphoreDelete(_spp_tx_done);
//         _spp_tx_done = NULL;
//     }
//     if (_bt_event_group) {
//         vEventGroupDelete(_bt_event_group);
//         _bt_event_group = NULL;
//     }
//     return true;
// }

void bt_initialize(void)
{
    // char bda_str[18] = {0};
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    // if ((ret = esp_spp_enhanced_init(esp_spp_mode)) != ESP_OK) {
    //     ESP_LOGE(SPP_TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
    //     return;
    // }

#if (CONFIG_BT_SSP_ENABLED == true)
    /* Set default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

    /*
     * Set default parameters for Legacy Pairing
     * Use variable pin, input pin code when pairing
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);

    // ESP_LOGI(SPP_TAG, "Own address:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));

//////////////////////////////////
// spp_packet_t *packet = NULL;



    _init_bt();
    xTaskCreate(_spp_rx_task, "spp_tx", 3*2048, NULL, 10, NULL);
     if(!_spp_task_handle){
    }

    xTaskCreatePinnedToCore(_spp_tx_task, "spp_tx", 4096, NULL, 10, &_spp_task_handle, 0);
        if(!_spp_task_handle){
            ESP_LOGE(SPP_TAG,"Network Event Task Start Failed!");
            return;
        }

}
