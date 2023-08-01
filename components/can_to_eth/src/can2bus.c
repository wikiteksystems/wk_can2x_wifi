#include "can2bus.h"
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

#include "mcp2515.h"

#include "utility.h"
#include "iso_tp.h"
#include "instructions.h"
#include "dev_settings.h"
#include "can_iface.h"


#define ESP_INTR_FLAG_DEFAULT 0

const static char *TAG = "CAN2BUS";




int can2bus_updateProtocol(ChannelAllConfig *channel, uint8_t protocol);
int can2bus_setRxId(ChannelAllConfig *channel, uint32_t rxid, uint32_t rx_mask);
int can2bus_setTxId(ChannelAllConfig *channel, uint32_t txid);
int can2bus_setPadding(ChannelAllConfig *channel, bool status, uint8_t paddingByte);
int can2bus_clearReceiveQueue(void);
int can2bus_getLatestIVNframe(ChannelAllConfig *channel, uint32_t rxid, ISOTPFrame *frame);
int can2bus_sendISOTPFrameOld(ChannelAllConfig *channel, ISOTPFrame *frame);
CANRxRetType can2bus_recieveISOTPFrame(ChannelAllConfig *channel, ISOTPFrame *frame, uint16_t waitTimeMs);


static QueueHandle_t txQueue = NULL;
static QueueHandle_t rxQueue = NULL;
static TaskHandle_t txTaskHandle = NULL;
static TaskHandle_t rxTaskHandle = NULL;
static QueueHandle_t interruptQueueHandle = NULL;

//vocom
static const int mcpInterruptPin = GPIO_NUM_36;
//Can2xET
//static const int mcpInterruptPin = GPIO_NUM_39;

//vocom and tatVCI
//#define MCP_CRYSTAL_FREQ MCP_10MHZ
//can2xET
#define MCP_CRYSTAL_FREQ MCP_8MHZ

// static void IRAM_ATTR gpio_isr_handler(void* arg)
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    // uint32_t gpio_num = (uint32_t) arg;
    // BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // uint8_t val = MCP____readRegister(MCP_CANINTF);
    // if ( val & STAT_RXIF_MASK) {
    //     // MCP____modifyRegister(MCP_CANINTF, val, 0x00); // Set raised interrupts to 0
    //     xQueueSendFromISR(interruptQueueHandle, (void *) &val, &xHigherPriorityTaskWoken);
    // }
    // else {
    //     // ESP_LOGW(TAG, "FALSE ISR FOR MCP");
    //     xQueueSendFromISR(interruptQueueHandle, (void *) &val, &xHigherPriorityTaskWoken);
    // }
    // uint8_t val = STAT_RXIF_MASK;
    // xQueueSendFromISR(interruptQueueHandle, (void *) &val, &xHigherPriorityTaskWoken);
    MCP____readRXDataIfInterruptFromISR(rxQueue);
}

void rxTask(void *p) {
    uint8_t val = 0x00;
    while(1) {
        BaseType_t ret = xQueueReceive(interruptQueueHandle, &val, pdMS_TO_TICKS(100));
        if ( ret == pdTRUE ) 
        {
            // struct can_frame canMsg2;
            // MCP__ERROR readRet = MCP__ERROR_FAIL;
            // int bufferNum = -1;
            // if ((val & MCP__STAT_RX0IF)) {
            //     //   ESP_LOGI(TAG, "Interrupt val: %x %x", val, MCP____readRegister(MCP_CANINTF));
            //       ESP_LOGI(TAG, "Interrupt val: %x", val);
            //     readRet = MCP____readMessageFromBuffer(RXB0, &canMsg2);
            //     // bufferNum = 0;
            // } else if ((val & MCP__STAT_RX1IF)) {
            //     readRet = MCP____readMessageFromBuffer(RXB1, &canMsg2);
            //     // bufferNum = 1;
            // } else {
            //     ESP_LOGW(TAG, "False interrupt with value: %02x", val);
            // }
            // readRet = MCP____readRXDataIfInterruptFromISR(rxQueue);
            // if (readRet != 0) {
            //     ESP_LOGW(TAG, "ERROR in reading reg: %d", readRet);
            // }
            // ESP_LOGI(TAG, "READING CAN MSG: %d", readRet);
            // if (readRet == MCP__ERROR_OK) {
                // ESP_LOGI(TAG, "READVAL: %" PRIu32 " %d", canMsg2.can_id, canMsg2.can_dlc);
                // if ( bufferNum == 0) {
                //     ESP_LOGI(TAG, "\r\nRXB0CTRL: %x\r\n", MCP____readRegister(MCP_RXB0CTRL));
                // } else {
                //     ESP_LOGI(TAG, "\r\nRXB1CTRL: %x\r\n", MCP____readRegister(MCP_RXB1CTRL));
                // }
                // if (readRet == MCP__ERROR_OK) {
                    // BaseType_t retSend = xQueueSend(rxQueue, &canMsg2, pdMS_TO_TICKS(1000));
                    // if ( retSend == pdFALSE ) {
                    //     ESP_LOGW(TAG, "Failed to send RX data into queue");
                    // }
                // }
            // }
            // int mcpRet = MCP____readRXDataIfDataPresent(rxQueue);
            // if (mcpRet != 0) {
            //     ESP_LOGW(TAG, "Data not found in interrupt: %d", mcpRet);
            // }
        } else {
            // ESP_LOGI(TAG, "Cehcking if data is there");
            int mcpRet = MCP____readRXDataIfDataPresent(rxQueue);
            if (mcpRet == 0) {
                ESP_LOGW(TAG, "Data in unexpected way");
            } else if (mcpRet != -1) {
                ESP_LOGW(TAG, "MCP RX RET : %d", mcpRet);
            }
            // else {
            //     // No data and no interrut, check error here
                
            //     int strtSpiRet = MCP____startSPI();
            //     if (strtSpiRet == 0) {
            //         uint8_t eflgVal = MCP____readRegister(MCP_EFLG);
            //         if ( eflgVal != 0x00 ) {
            //             ESP_LOGW(TAG, "ERROR FLG: %x", eflgVal);
            //         }
            //         MCP____endSPI();
            //     }
            // }
        }
    }
    vTaskDelete(NULL);
}


void txTask(void *p) {
    struct can_frame canFrame;
    while (1) {
        // ESP_LOGI(TAG, "Waiting for new tx msg");
        BaseType_t ret = xQueuePeek(txQueue, &canFrame, pdMS_TO_TICKS(2000));
        if ( ret == pdTRUE ) {
            // ESP_LOGI(TAG, "CAN SEND: %x %x %x %x %x %x %x %x", canFrame.data[0], canFrame.data[1], canFrame.data[2], 
            //     canFrame.data[3], canFrame.data[4], canFrame.data[5], canFrame.data[6], canFrame.data[7]);
            // ESP_LOG_BUFFER_HEX(TAG, canFrame.data, canFrame.can_dlc);
            // MCP__ERROR err = MCP____sendMessage(&canFrame);
            // int64_t stpTime = esp_timer_get_time() + (2*1000*1000);
            int startSpiRet = MCP____startSPI();
            if (startSpiRet != 0) {
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            uint8_t ctrlval = MCP____readRegister(MCP_TXB0CTRL);
            if ( (ctrlval & TXB_TXREQ) == 0 ) {
                MCP__ERROR err = MCP____sendMessageUsingBuffer(TXB0, &canFrame);
                if ( err != MCP__ERROR_OK) {
                    ESP_LOGW(TAG, "ERROR in MCP send : %d", err);
                    ctrlval = MCP____readRegister(MCP_TXB0CTRL);
                    ESP_LOGW(TAG, "TXB0CTRL: %x", ctrlval);
                }
                MCP____endSPI();
            } else if (ctrlval & (TXB_ABTF | TXB_MLOA | TXB_TXERR)){
                vTaskDelay(pdMS_TO_TICKS(1));
                MCP____endSPI();
                continue;
            } else if ( (ctrlval & TXB_TXREQ) != 0 ) {
                // vTaskDelay(pdMS_TO_TICKS(1));
                MCP____endSPI();
                continue;
            } else {
                MCP____endSPI();
                ESP_LOGW(TAG, "SOMETHING UNEXPETED: %x", ctrlval);
            }
            
            // ESP_LOG_BUFFER_HEX(TAG, canFrame.data, canFrame.can_dlc);
            ret = xQueueReceive(txQueue, &canFrame, pdMS_TO_TICKS(1000));
            if (ret == pdFALSE) {
                ESP_LOGE(TAG, "NEVER GONNA HAPPEN");
            }
        }
        // else {
        //     ESP_LOGI(TAG, "Waiting for data to transmit");
        // }
    }
    vTaskDelete(NULL);
}

static void can2bus_start(uint8_t protocol, CanIdType id_type, uint32_t rxId,uint32_t rx_mask) {
    switch(protocol) {
        case PROTOCOL_250KB_11BIT_CAN:
        case PROTOCOL_250KB_29BIT_CAN:
        case PROTOCOL_ISO15765_250KB_11BIT_CAN:
        case PROTOCOL_ISO15765_250KB_29BIT_CAN:
        case PROTOCOL_OE_IVN_250KB_29BIT_CAN:
        case PROTOCOL_OE_IVN_250KB_11BIT_CAN:
            ESP_LOGI(TAG, "CAN speed req %d", 250);
            MCP____setBitrate(MCP__CAN_250KBPS, MCP_CRYSTAL_FREQ);
            break;
        case PROTOCOL_500KB_11BIT_CAN:
        case PROTOCOL_500KB_29BIT_CAN:
        case PROTOCOL_ISO15765_500KB_11BIT_CAN:
        case PROTOCOL_ISO15765_500KB_29BIT_CAN:
        case PROTOCOL_OE_IVN_500KB_11BIT_CAN:
        case PROTOCOL_OE_IVN_500KB_29BIT_CAN:
            ESP_LOGI(TAG, "CAN speed req %d", 500);
            MCP____setBitrate(MCP__CAN_500KBPS, MCP_CRYSTAL_FREQ);
            break;
        case PROTOCOL_1MB_11BIT_CAN:
        case PROTOCOL_1MB_29BIT_CAN:
        case PROTOCOL_ISO15765_1MB_11BIT_CAN:
        case PROTOCOL_ISO15765_1MB_29BIT_CAN:
        case PROTOCOL_OE_IVN_1MB_11BIT_CAN:
        case PROTOCOL_OE_IVN_1MB_29BIT_CAN:
            ESP_LOGI(TAG, "CAN speed req %d", 1000);
            MCP____setBitrate(MCP__CAN_1000KBPS, MCP_CRYSTAL_FREQ);
            break;
        default:
            ESP_LOGW(TAG, "Unsupported CAN protocol provided %x", protocol);
            break;
    }

    if (id_type == CAN_ID_TYPE_11BIT) {
        uint32_t acceptance_code = rxId;
        uint32_t acceptance_mask = CAN_SFF_MASK;
        MCP____setFilter(RXF0, 0, acceptance_code);
        MCP____setFilterMask(MCP__MASK0, 0, acceptance_mask);
    } else if (id_type == CAN_ID_TYPE_29BIT) {
        uint32_t acceptance_code = rxId;
        uint32_t acceptance_mask = CAN_EFF_MASK;
        MCP____setFilter(RXF0, 1, acceptance_code);
        MCP____setFilterMask(MCP__MASK0, 1, acceptance_mask);
    }
    
    MCP____setNormalMode();
}

void initialise_can2bus(ChannelAllConfig *channel) {
    channel->fnConfig->updateProtocol = can2bus_updateProtocol;
    channel->fnConfig->setRxId = can2bus_setRxId;
    channel->fnConfig->setTxId = can2bus_setTxId;
    channel->fnConfig->setPadding = can2bus_setPadding;
    channel->fnConfig->clearReceiveQueue = can2bus_clearReceiveQueue;
    channel->fnConfig->sendCANMessage = can2bus_sendISOTPFrameOld;
    channel->fnConfig->receiveCANMessage = can2bus_recieveISOTPFrame;
    channel->fnConfig->getLatestIVNFrame = can2bus_getLatestIVNframe;
    channel->fnConfig->setPeriodicTesterPresentStatus = settings_setPeriodicTesterPresentStatus;


    gpio_config_t gpioCfg = {
        .pin_bit_mask = (1ULL << mcpInterruptPin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    esp_err_t gpioRet = gpio_config(&gpioCfg);
    ESP_ERROR_CHECK(gpioRet);

    interruptQueueHandle = xQueueCreate(64, 1);

    if ( txQueue == NULL ) {
        txQueue = xQueueCreate(1000, sizeof(struct can_frame));
    }
    if ( rxQueue == NULL ) {
        rxQueue = xQueueCreate(100, sizeof(struct can_frame));
    }

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(mcpInterruptPin, gpio_isr_handler, (void*) mcpInterruptPin);

    xTaskCreate(rxTask, "rxIntTask", 2048 + 512, NULL, tskIDLE_PRIORITY + 1, &rxTaskHandle);
    xTaskCreate(txTask, "txC2Task", 2048, NULL, tskIDLE_PRIORITY + 1, &txTaskHandle);


    MCP____INIT();
    MCP____reset();

    MCP____setFilterMask(MCP__MASK1, 1, 0xFFFFFFFF);
    MCP____setFilter(RXF1, 1, 0xFFFFFFFF);
    MCP____setFilter(RXF2, 1, 0xFFFFFFFF);
    MCP____setFilter(RXF3, 1, 0xFFFFFFFF);
    MCP____setFilter(RXF4, 1, 0xFFFFFFFF);
    MCP____setFilter(RXF5, 1, 0xFFFFFFFF);

    if (channel->otherConfig->canbusUseMutex == NULL) {
        channel->otherConfig->canbusUseMutex = xSemaphoreCreateMutex(); // Default status is released
        if (channel->otherConfig->canbusUseMutex != NULL) {
            ESP_LOGI(TAG, "CAN2BUS mutex created successfully");
        } else {
            ESP_LOGW(TAG, "CAN2BUS mutex failed to create");
            return;
        }
    }
    if (pdTRUE == xSemaphoreTake(channel->otherConfig->canbusUseMutex, pdMS_TO_TICKS(1000))) {
        can2bus_start(channel->config->protocol, channel->config->id_type, channel->config->rx_id, channel->config->rx_mask);
        initialise_isotp(channel);
        xSemaphoreGive(channel->otherConfig->canbusUseMutex);
    }
}

static void can2bus_stop(void) {
    MCP____setConfigMode();
    //Stop the TWAI driver

    //Uninstall the TWAI driver
}

void deinitialise_can2bus(ChannelAllConfig *channel) {

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

        can2bus_stop();
        MCP____DEINIT();

        ESP_ERROR_CHECK(gpio_isr_handler_remove(mcpInterruptPin));
        gpio_uninstall_isr_service();

        vTaskDelete(rxTaskHandle);
        rxTaskHandle = NULL;

        vTaskDelete(txTaskHandle);
        txTaskHandle = NULL;

        vQueueDelete(interruptQueueHandle);

        xSemaphoreGive(channel->otherConfig->canbusUseMutex);

        vSemaphoreDelete(channel->otherConfig->canbusUseMutex);
        channel->otherConfig->canbusUseMutex = NULL;
    }
}

int can2bus_updateProtocol(ChannelAllConfig *channel, uint8_t protocol) {
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
        ESP_LOGI(TAG, "Restarting can");
        int64_t strtTime = esp_timer_get_time();
        can2bus_stop();
        ESP_LOGI(TAG, "Stopped can");
        can2bus_start(channel->config->protocol, channel->config->id_type, channel->config->rx_id, channel->config->rx_mask);
        ESP_LOGI(TAG, "RESTART TIME TAKEN : %" PRIi64, esp_timer_get_time() - strtTime);
        xSemaphoreGive(channel->otherConfig->canbusUseMutex);
    } else {
        ESP_LOGW(TAG, "Failed to obtain mutex for update protocol");
    }
    return 0;
}

int can2bus_setRxId(ChannelAllConfig *channel, uint32_t rxid,uint32_t rx_mask) {
    ESP_LOGI(TAG, "SETTING RX_ID : %" PRIu32, rxid);

    channel->config->rx_id = rxid;
    
    if (pdTRUE == xSemaphoreTake(channel->otherConfig->canbusUseMutex, pdMS_TO_TICKS(1000))) {
        int64_t strtTime = esp_timer_get_time();
        can2bus_stop();
        can2bus_start(channel->config->protocol, channel->config->id_type, channel->config->rx_id, channel->config->rx_mask);
        ESP_LOGI(TAG, "RESTART TIME TAKEN : %" PRIi64, esp_timer_get_time() - strtTime);
        xSemaphoreGive(channel->otherConfig->canbusUseMutex);
    }

    return 0;
}

int can2bus_setTxId(ChannelAllConfig *channel, uint32_t txid) {
    ESP_LOGI(TAG, "SETTING TX_ID : %" PRIu32, txid);
    channel->config->tx_id = txid;
    return 0;
}

int can2bus_setPadding(ChannelAllConfig *channel, bool status, uint8_t paddingByte) {
    channel->config->paddingEnabled = status;
    if (status) {
        channel->config->paddingByte = paddingByte;
    }
    return 0;
}

int can2bus_getLatestIVNframe(ChannelAllConfig *channel, uint32_t rxid, ISOTPFrame *frame) {
    channel->config->rx_id = rxid;
    int ret = -1;
    if (pdTRUE == xSemaphoreTake(channel->otherConfig->canbusUseMutex, pdMS_TO_TICKS(10000))) {
        int64_t strtTime = esp_timer_get_time();
        can2bus_stop();
        // can2bus_clearReceiveQueue();
        can2bus_start(channel->config->protocol, channel->config->id_type, rxid, 0xFFFFFFFF);
        ESP_LOGI(TAG, "RESTART TIME TAKEN : %" PRIi64, esp_timer_get_time() - strtTime);
        ret = can2bus_recieveISOTPFrame(channel, frame, 5000);
        if (ret != 0) {
            ESP_LOGW(TAG, "Invalid IVN Frame response or no response");
        }
        strtTime = esp_timer_get_time();
        can2bus_stop();
        // can2bus_clearReceiveQueue();
        can2bus_start(channel->config->protocol, channel->config->id_type, channel->config->rx_id, channel->config->rx_mask);
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

int can2bus_clearReceiveQueue(void) {
    esp_err_t ret = xQueueReset(rxQueue);
    if (ret == pdPASS) {
        return 0;
    } else {
        ESP_LOGW(TAG, "FAILED TO CLEAR RX queue");
        return -1;
    }
}

int can2bus_sendCANdata(uint8_t *data, uint8_t dataLen, CanIdType id_type, uint32_t tx_id, bool paddingEnabled, uint8_t paddingByte) {
    
    // ESP_LOGI(TAG, "ISOTP frame to be sent to CAN2BUS: %d", frame->dataLen);
    // DEBUG_PRINT_HEX(data, dataLen);

    struct can_frame message;
    
    // canid_t can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
    // uint8_t can_dlc; /* frame payload length in byte (0 .. CAN_MAX_DLEN) */
    // uint8_t data[CAN_MAX_DLEN] __attribute__((aligned(8)));
    canid_ut can_id = {
        .id = tx_id,
        .ERR = 0,
        .RTR = 0,
        .EFF = id_type == CAN_ID_TYPE_29BIT
    };

    message.can_id = can_id.val;
    // message.extd = id_type;
    memcpy(message.data, data, dataLen);

    if (paddingEnabled && dataLen < CAN_MAX_DLEN) {
        memset(&message.data[dataLen], paddingByte, CAN_MAX_DLEN - dataLen);
    	message.can_dlc = CAN_MAX_DLEN;
    } else {
    	message.can_dlc = dataLen;
    }

    //Queue message for transmission
    BaseType_t txRet = xQueueSend(txQueue, &message, pdMS_TO_TICKS(1000));
    if (txRet == pdTRUE) {
        // ESP_LOGI(TAG, "Message queued for transmission");
        return 0;
    } else {
        ESP_LOGW(TAG, "Failed to queue message for transmission");
        return -1;
    }
}

int can2bus_sendISOTPFrameOld(ChannelAllConfig *channel, ISOTPFrame *frame) {
    if (pdFALSE == xSemaphoreTake(channel->otherConfig->canbusUseMutex, pdMS_TO_TICKS(10000))) {
        ESP_LOGW(TAG, "Could not acquire mutex for Sending");
        return -5;
    }
    int ret = can2bus_sendCANdata(frame->data, frame->dataLen, channel->config->id_type, channel->config->tx_id,
        channel->config->paddingEnabled, channel->config->paddingByte);
    xSemaphoreGive(channel->otherConfig->canbusUseMutex);
    return ret;
}

CANRxRetType can2bus_recieveISOTPFrame(ChannelAllConfig *channel, ISOTPFrame *frame, uint16_t waitTimeMs) {
    //Wait for message to be received
    struct can_frame message;

    if (pdFALSE == xSemaphoreTake(channel->otherConfig->canbusUseMutex, pdMS_TO_TICKS(10000))) {
        ESP_LOGW(TAG, "Could not acquire mutex for RX");
        return CAN_RX_RET_MUTEX_ACQUIRE_FAIL;
    }
    BaseType_t rxRet = xQueueReceive(rxQueue, &message, pdMS_TO_TICKS(waitTimeMs));
    xSemaphoreGive(channel->otherConfig->canbusUseMutex);

    if (rxRet == pdTRUE) {
        ESP_LOGD(TAG, "Message received\n");
        // ESP_LOG_BUFFER_HEX(TAG, message.data, message.can_dlc);
    } else {
        // ESP_LOGW(TAG, "Failed to receive message\n");
        return CAN_RX_RET_TIMEOUT;
    }
    ESP_LOGD(TAG, "ID is %" PRIu32 "\n" , message.can_id);
    canid_ut can_id = { .val = message.can_id };
    //Process received message
    if (can_id.EFF) {
        ESP_LOGD(TAG, "Message is in Extended Format\n");
    } else {
        ESP_LOGD(TAG, "Message is in Standard Format\n");
    }
    if (!(can_id.RTR)) {
        frame->dataLen = message.can_dlc;
        memcpy(frame->data, message.data, frame->dataLen);
        // DEBUG_PRINT_HEX_DATA(frame->data, frame->dataLen);
        return CAN_RX_RET_OK;
    } else {
        ESP_LOGW(TAG, "RTR packet frame received");
        return CAN_RX_RET_RTR_FRAME;
    }
}
