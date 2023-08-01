#include "canbus.h"

#include <string.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "esp_timer.h"

#include "utility.h"
#include "iso_tp.h"
#include "instructions.h"
#include "dev_settings.h"
#include "can_iface.h"


const static char *TAG = "CANBUS";

// #define CANBUS_QUEUE_DATA_SIZE (sizeof(ISOTPFrame))
// #define CANBUS_QUEUE_LENGTH ((4096/7) + 25)

//CAN2ET
// #define CAN_GPIO_TX_PIN GPIO_NUM_33  //5
// #define CAN_GPIO_RX_PIN GPIO_NUM_32  //4

//TataVCI
#define CAN_GPIO_TX_PIN GPIO_NUM_5  //5
#define CAN_GPIO_RX_PIN GPIO_NUM_4  //4

                          
twai_timing_config_t t_config_250_kbps = TWAI_TIMING_CONFIG_250KBITS();
twai_timing_config_t t_config_500_kbps = TWAI_TIMING_CONFIG_500KBITS();
twai_timing_config_t t_config_1mbps = TWAI_TIMING_CONFIG_1MBITS();


int canbus_updateProtocol(ChannelAllConfig *channel, uint8_t protocol);
int canbus_setRxId(ChannelAllConfig *channel, uint32_t rxid, uint32_t rx_mask);
int canbus_setTxId(ChannelAllConfig *channel, uint32_t txid);
int canbus_setPadding(ChannelAllConfig *channel, bool status, uint8_t paddingByte);
int canbus_clearReceiveQueue(void);
int canbus_getLatestIVNframe(ChannelAllConfig *channel, uint32_t rxid, ISOTPFrame *frame);
int canbus_sendISOTPFrameOld(ChannelAllConfig *channel, ISOTPFrame *frame);
CANRxRetType canbus_recieveISOTPFrame(ChannelAllConfig *channel, ISOTPFrame *frame, uint16_t waitTimeMs);


static void canbus_start(uint8_t protocol, CanIdType id_type, uint32_t rxId, uint32_t rxMask) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_GPIO_TX_PIN, CAN_GPIO_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config;
    switch(protocol) {
        case PROTOCOL_250KB_11BIT_CAN:
        case PROTOCOL_250KB_29BIT_CAN:
        case PROTOCOL_ISO15765_250KB_11BIT_CAN:
        case PROTOCOL_ISO15765_250KB_29BIT_CAN:
        case PROTOCOL_OE_IVN_250KB_11BIT_CAN:
        case PROTOCOL_OE_IVN_250KB_29BIT_CAN:
            ESP_LOGI(TAG, "SPEED at 250kbps");
            memcpy(&t_config, &t_config_250_kbps, sizeof(twai_timing_config_t));
            break;
        case PROTOCOL_500KB_11BIT_CAN:
        case PROTOCOL_500KB_29BIT_CAN:
        case PROTOCOL_ISO15765_500KB_11BIT_CAN:
        case PROTOCOL_ISO15765_500KB_29BIT_CAN:
        case PROTOCOL_OE_IVN_500KB_11BIT_CAN:
        case PROTOCOL_OE_IVN_500KB_29BIT_CAN:
            ESP_LOGI(TAG, "SPEED at 500kbps");
            memcpy(&t_config, &t_config_500_kbps, sizeof(twai_timing_config_t));
            break;
        case PROTOCOL_1MB_11BIT_CAN:
        case PROTOCOL_1MB_29BIT_CAN:
        case PROTOCOL_ISO15765_1MB_11BIT_CAN:
        case PROTOCOL_ISO15765_1MB_29BIT_CAN:
        case PROTOCOL_OE_IVN_1MB_11BIT_CAN:
        case PROTOCOL_OE_IVN_1MB_29BIT_CAN:
            ESP_LOGI(TAG, "SPEED at 1000kbps");
            memcpy(&t_config, &t_config_1mbps, sizeof(twai_timing_config_t));
            break;
        default:
            ESP_LOGW(TAG, "Unsupported CAN protocol provided %x", protocol);
            break;
    }

    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (id_type == CAN_ID_TYPE_11BIT) {
        f_config.acceptance_code = rxId << (32-11);
        f_config.acceptance_mask = ~(rxMask << (32-11));
    } else if (id_type == CAN_ID_TYPE_29BIT) {
        f_config.acceptance_code = rxId << (32-29);
        f_config.acceptance_mask = ~(rxMask << (32-29));
    }

    g_config.rx_queue_len = 100;
    g_config.tx_queue_len = 200; //TODO(MAYANK): NO MAGIC NUMBER
    

    //Install TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        ESP_LOGI(TAG, "Driver installed");
    } else {
        ESP_LOGE(TAG, "Failed to install driver");
        return;
    }

    //Start TWAI driver
    if (twai_start() == ESP_OK) {
        ESP_LOGI(TAG, "Driver started");
    } else {
        ESP_LOGE(TAG, "Failed to start driver");
        return;
    }
}

static void canbus_stop(void) {
    //Stop the TWAI driver
    if (twai_stop() == ESP_OK) {
        ESP_LOGI(TAG, "Driver stopped");
    } else {
        ESP_LOGE(TAG, "Failed to stop driver");
        return;
    }

    //Uninstall the TWAI driver
    if (twai_driver_uninstall() == ESP_OK) {
        ESP_LOGI(TAG, "Driver uninstalled");
    } else {
        ESP_LOGE(TAG, "Failed to uninstall driver");
        return;
    }
}

void initialise_canbus(ChannelAllConfig *channel) {
    channel->fnConfig->updateProtocol = canbus_updateProtocol;
    channel->fnConfig->setRxId = canbus_setRxId;
    channel->fnConfig->setTxId = canbus_setTxId;
    channel->fnConfig->setPadding = canbus_setPadding;
    channel->fnConfig->clearReceiveQueue = canbus_clearReceiveQueue;
    channel->fnConfig->sendCANMessage = canbus_sendISOTPFrameOld;
    channel->fnConfig->receiveCANMessage = canbus_recieveISOTPFrame;
    channel->fnConfig->getLatestIVNFrame = canbus_getLatestIVNframe;
    channel->fnConfig->setPeriodicTesterPresentStatus = settings_setPeriodicTesterPresentStatus;

    if (channel->otherConfig->canbusUseMutex == NULL) {
        channel->otherConfig->canbusUseMutex = xSemaphoreCreateMutex(); // Default status is released
        if (channel->otherConfig->canbusUseMutex != NULL) {
            ESP_LOGI(TAG, "CANBUS mutex created successfully");
        } else {
            ESP_LOGW(TAG, "CANBUS mutex failed to create");
            return;
        }
    }
    if (pdTRUE == xSemaphoreTake(channel->otherConfig->canbusUseMutex, pdMS_TO_TICKS(1000))) {
        canbus_start(channel->config->protocol, channel->config->id_type, channel->config->rx_id, channel->config->rx_mask);
        initialise_isotp(channel);
        xSemaphoreGive(channel->otherConfig->canbusUseMutex);
    }
}

void deinitialise_canbus(ChannelAllConfig *channel) {

    if (pdTRUE == xSemaphoreTake(channel->otherConfig->canbusUseMutex, portMAX_DELAY)) {
        uint8_t protocol = channel->config->protocol;
        switch(protocol) {
            case PROTOCOL_250KB_11BIT_CAN:
            case PROTOCOL_250KB_29BIT_CAN:
            case PROTOCOL_500KB_11BIT_CAN:
            case PROTOCOL_500KB_29BIT_CAN:
            case PROTOCOL_1MB_11BIT_CAN:
            case PROTOCOL_1MB_29BIT_CAN:
                break;
            case PROTOCOL_OE_IVN_250KB_11BIT_CAN:
            case PROTOCOL_OE_IVN_250KB_29BIT_CAN:
            case PROTOCOL_OE_IVN_500KB_11BIT_CAN:
            case PROTOCOL_OE_IVN_500KB_29BIT_CAN:
            case PROTOCOL_OE_IVN_1MB_11BIT_CAN:
            case PROTOCOL_OE_IVN_1MB_29BIT_CAN:
                break;
            case PROTOCOL_ISO15765_250KB_11BIT_CAN:
            case PROTOCOL_ISO15765_250KB_29BIT_CAN:
            case PROTOCOL_ISO15765_500KB_11BIT_CAN:
            case PROTOCOL_ISO15765_500KB_29BIT_CAN:
            case PROTOCOL_ISO15765_1MB_11BIT_CAN:
            case PROTOCOL_ISO15765_1MB_29BIT_CAN:
                deinitialise_isotp(channel);
                break;
            default:
                ESP_LOGW(TAG, "Unsupported CAN protocol set %x", protocol);
                break;
        }

        canbus_stop();

        xSemaphoreGive(channel->otherConfig->canbusUseMutex);

        vSemaphoreDelete(channel->otherConfig->canbusUseMutex);
        channel->otherConfig->canbusUseMutex = NULL;
    }
}

int canbus_updateProtocol(ChannelAllConfig *channel, uint8_t protocol) {
    channel->config->protocol = protocol;
    
    switch(channel->config->protocol) {
        case PROTOCOL_250KB_11BIT_CAN:
        case PROTOCOL_ISO15765_250KB_11BIT_CAN:
        case PROTOCOL_OE_IVN_250KB_11BIT_CAN:
        case PROTOCOL_500KB_11BIT_CAN:
        case PROTOCOL_ISO15765_500KB_11BIT_CAN:
        case PROTOCOL_OE_IVN_500KB_11BIT_CAN:
        case PROTOCOL_1MB_11BIT_CAN:
        case PROTOCOL_ISO15765_1MB_11BIT_CAN:
        case PROTOCOL_OE_IVN_1MB_11BIT_CAN:
            channel->config->id_type = CAN_ID_TYPE_11BIT;
            break;
        case PROTOCOL_250KB_29BIT_CAN:
        case PROTOCOL_ISO15765_250KB_29BIT_CAN:
        case PROTOCOL_OE_IVN_250KB_29BIT_CAN:
        case PROTOCOL_500KB_29BIT_CAN:
        case PROTOCOL_ISO15765_500KB_29BIT_CAN:
        case PROTOCOL_OE_IVN_500KB_29BIT_CAN:
        case PROTOCOL_1MB_29BIT_CAN:
        case PROTOCOL_ISO15765_1MB_29BIT_CAN:
        case PROTOCOL_OE_IVN_1MB_29BIT_CAN:
            channel->config->id_type = CAN_ID_TYPE_29BIT;
            break;
        default:
            ESP_LOGW(TAG, "Unsupported CAN ID_TYPE provided %x", channel->config->protocol);
            break;
    }
    if (pdTRUE == xSemaphoreTake(channel->otherConfig->canbusUseMutex, pdMS_TO_TICKS(1000))) {
        int64_t strtTime = esp_timer_get_time();
        canbus_stop();
        canbus_start(channel->config->protocol, channel->config->id_type, channel->config->rx_id, channel->config->rx_mask);
        ESP_LOGI(TAG, "RESTART TIME TAKEN : %" PRIi64, esp_timer_get_time() - strtTime);
        xSemaphoreGive(channel->otherConfig->canbusUseMutex);
    }
    return 0;
}

int canbus_setRxId(ChannelAllConfig *channel, uint32_t rxid,uint32_t rx_mask) {
    channel->config->rx_id = rxid;
    channel->config->rx_mask = rx_mask;
    
    if (pdTRUE == xSemaphoreTake(channel->otherConfig->canbusUseMutex, pdMS_TO_TICKS(1000))) {
        int64_t strtTime = esp_timer_get_time();
        canbus_stop();
        canbus_start(channel->config->protocol, channel->config->id_type, channel->config->rx_id, channel->config->rx_mask);
        ESP_LOGI(TAG, "RESTART TIME TAKEN : %" PRIi64, esp_timer_get_time() - strtTime);
        xSemaphoreGive(channel->otherConfig->canbusUseMutex);
    }

    return 0;
}

int canbus_setTxId(ChannelAllConfig *channel, uint32_t txid) {
    channel->config->tx_id = txid;
    return 0;
}

int canbus_setPadding(ChannelAllConfig *channel, bool status, uint8_t paddingByte) {
    channel->config->paddingEnabled = status;
    if (status) {
        channel->config->paddingByte = paddingByte;
    }
    return 0;
}

int canbus_getLatestIVNframe(ChannelAllConfig *channel, uint32_t rxid, ISOTPFrame *frame) {
    channel->config->rx_id = rxid;
    int ret = -1;
    if (pdTRUE == xSemaphoreTake(channel->otherConfig->canbusUseMutex, pdMS_TO_TICKS(10000))) {
        int64_t strtTime = esp_timer_get_time();
        canbus_stop();
        // canbus_clearReceiveQueue();
        canbus_start(channel->config->protocol, channel->config->id_type, rxid, 0xFFFFFFFF);
        ESP_LOGI(TAG, "RESTART TIME TAKEN : %" PRIi64, esp_timer_get_time() - strtTime);
        ret = canbus_recieveISOTPFrame(channel, frame, 5000);
        if (ret != 0) {
            ESP_LOGW(TAG, "Invalid IVN Frame response or no response");
        }
        strtTime = esp_timer_get_time();
        canbus_stop();
        // canbus_clearReceiveQueue();
        canbus_start(channel->config->protocol, channel->config->id_type, rxid, 0xFFFFFFFF);
        ESP_LOGI(TAG, "RESTART TIME TAKEN : %" PRIi64, esp_timer_get_time() - strtTime);
        xSemaphoreGive(channel->otherConfig->canbusUseMutex);
        if (ret == 0) {
            return 0;
        } else {
            return -2;
        }
    } else {
        return -1;
    }
}

int canbus_clearReceiveQueue(void) {
    esp_err_t ret = twai_clear_receive_queue();
    if (ret == ESP_OK) {
        return 0;
    } else {
        ESP_LOGW(TAG, "FAILED TO CLEAR RX queue");
        return -1;
    }
}

int canbus_sendCANdata(uint8_t *data, uint8_t dataLen, uint32_t id_type, uint32_t tx_id, bool paddingEnabled, uint8_t paddingByte) {
    
    // ESP_LOGI(TAG, "ISOTP frame to be sent to CANBUS: %d", frame->dataLen);
    // DEBUG_PRINT_HEX(data, dataLen);

    twai_message_t message;
    message.identifier = tx_id;
    message.extd = id_type;
    message.rtr = 0;
    message.self = 0;
    message.ss = 0;
    // printf("extd: %d rtr: %d self: %d ss: %d\n",
    //     message.extd, message.rtr, message.self, message.ss);
    memcpy(message.data, data, dataLen);

    if (paddingEnabled && dataLen < 8) {
        memset(&message.data[dataLen], paddingByte, 8 - dataLen);
    	message.data_length_code = 8;
    } else {
    	message.data_length_code = dataLen;
    }

    //Queue message for transmission
    esp_err_t txRet = twai_transmit(&message, pdMS_TO_TICKS(1000));
    if (txRet == ESP_OK) {
        // ESP_LOGI(TAG, "Message queued for transmission");
        return 0;
    } else {
        ESP_LOGW(TAG, "Failed to queue message for transmission");
        return -1;
    }
}

int canbus_sendISOTPFrameOld(ChannelAllConfig *channel, ISOTPFrame *frame) {
    if (pdFALSE == xSemaphoreTake(channel->otherConfig->canbusUseMutex, pdMS_TO_TICKS(10000))) {
        ESP_LOGW(TAG, "Could not acquire mutex for Send isotp");
        return -5;
    }
    int ret = canbus_sendCANdata(frame->data, frame->dataLen, channel->config->id_type, channel->config->tx_id,
        channel->config->paddingEnabled, channel->config->paddingByte);
    xSemaphoreGive(channel->otherConfig->canbusUseMutex);
    return ret;
}

CANRxRetType canbus_recieveISOTPFrame(ChannelAllConfig *channel, ISOTPFrame *frame, uint16_t waitTimeMs) {
    //Wait for message to be received
    twai_message_t message;
    
    if (pdFALSE == xSemaphoreTake(channel->otherConfig->canbusUseMutex, pdMS_TO_TICKS(10000))) {
        ESP_LOGW(TAG, "Could not acquire mutex for RX");
        return CAN_RX_RET_MUTEX_ACQUIRE_FAIL;
    }
    esp_err_t twaiRet = twai_receive(&message, pdMS_TO_TICKS(waitTimeMs));
    xSemaphoreGive(channel->otherConfig->canbusUseMutex);
    
    if (twaiRet == ESP_OK) {
        ESP_LOGD(TAG, "Message received\n");
    } else if (twaiRet == ESP_ERR_TIMEOUT) {
        // ESP_LOGI(TAG, "Timeout in RX CAN bus msg, It's Normal\n");
        return CAN_RX_RET_TIMEOUT;
    } else if (twaiRet == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "CAN bus is not running\n");
        return CAN_RX_RET_INTERNAL_ERR;
    } else {
        ESP_LOGW(TAG, "Failed to receive message %d\n", twaiRet);
        return CAN_RX_RET_INTERNAL_ERR;
    }

    //Process received message
    // if (message.extd) {
    //     ESP_LOGI(TAG, "Message is in Extended Format\n");
    // } else {
    //     ESP_LOGW(TAG, "Message is in Standard Format\n");
    // }
    ESP_LOGD(TAG, "ID is %" PRIu32 "\n" , message.identifier);
    if (!(message.rtr)) {
        frame->dataLen = message.data_length_code;
        memcpy(frame->data, message.data, message.data_length_code);
        // DEBUG_PRINT_HEX_DATA(frame->data, frame->dataLen);
        return CAN_RX_RET_OK;
    } else {
        ESP_LOGW(TAG, "RTR packet frame received");
        return CAN_RX_RET_RTR_FRAME;
    }
}
