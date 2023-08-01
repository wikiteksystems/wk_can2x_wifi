#include "iso_tp.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/semphr.h"

// #define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include "esp_log.h"
#include "driver/twai.h"
#include "esp_timer.h"

#include "utility.h"
#include "canbus.h"
#include "sock_service.h"
#include "tcp_packet_handler.h"
#include "crc.h"
#include "instructions.h"
#include "dev_settings.h"

#include "channel.h"
#include "can_iface.h"

#include "usb_service.h"
#include "bt_service.h"

bool cancomm;

#define ISOTP_TASK_EVENT_RX_TASK_CLOSED         (1 << 0)
#define ISOTP_TASK_EVENT_TX_TASK_CLOSED         (1 << 1)
#define ISOTP_TASK_EVENT_TASK_CLOSE_REQUESTED   (1 << 2)
#define ISOTP_TASK_EVENT_MAIN_TASK_CLOSED       (1 << 3)


const static char *TAG = "iso_tp";

#define DEVICE_TESTER_PRESENT_WAIT_TIME_uS (500 * 1000)

uint8_t dtpDataReference[] = {0x3E, 0x00};
uint8_t dtpDataExpectedResponseReference[] = {0x7e, 0x00};

static int isotp_send_data(ChannelAllConfig *channel, uint8_t *data, uint16_t dataLen);
static int destructRxHolder(ISOTPRxHolder *rxHolder);


static void isotpTxTask(void *p) {
    ChannelAllConfig *channel = (ChannelAllConfig *) p;

    while(1) {
        EventBits_t bits = xEventGroupGetBits(channel->otherConfig->xISOTP_taskEventHandle);
        if ( bits & ISOTP_TASK_EVENT_TASK_CLOSE_REQUESTED ) {
            break;
        }

        /* ISO TP Socket Data Handle Sequence */
        ISOTPQueueData isotpQueueData;
        BaseType_t ret = xQueuePeek(channel->otherConfig->xISOTP_socketToCANQueueHandle, 
            &isotpQueueData, pdMS_TO_TICKS(100));
        if (ret == pdFALSE) {
            if (channel->config->periodicTesterPresentEnabled) {
                EventBits_t eBits = xEventGroupGetBits(channel->otherConfig->xISOTP_dtpEventHandle);
                if (eBits & DTP_EVENT_NORMAL_RESPONSE_REQUESTED) {
                    ESP_LOGD(TAG, "DTP TASK: NoRMAl response expected");
                    vTaskDelay(pdMS_TO_TICKS(50));
                    continue;
                } else if (eBits & DTP_EVENT_DTP_RESPONSE_REQUESTED) {
                    ESP_LOGD(TAG, "DTP TASK: DTP response expected");
                    vTaskDelay(pdMS_TO_TICKS(50));
                    continue;
                } else {
                    int64_t curTime = esp_timer_get_time();
                    if (curTime - channel->otherConfig->lastDeviceResponseTime >
                            DEVICE_TESTER_PRESENT_WAIT_TIME_uS ) {
                        eBits = xEventGroupClearBits(
                                    channel->otherConfig->xISOTP_dtpEventHandle,
                                    DTP_EVENT_DTP_RESPONSE_RECEIVED);
                        eBits = xEventGroupSetBits(
                                channel->otherConfig->xISOTP_dtpEventHandle,
                                DTP_EVENT_DTP_RESPONSE_REQUESTED);
                        int isoTpRet = isotp_send_data(channel, dtpDataReference, sizeof(dtpDataReference));
                        if (isoTpRet == 0) {
                            ESP_LOGD(TAG, "QUEUED Device Tester Present");
                            channel->otherConfig->isDeviceTesterPresentResponseExpected = true;
                            if (channel->config->maxMicrosBetweenReqResp != 0x0000) {
                                if (channel->otherConfig->xWaitReqRespTimerHandle != NULL) {
                                    gptimer_start(channel->otherConfig->xWaitReqRespTimerHandle);
                                } else {
                                    ESP_LOGW(TAG, "Timer handle should been available but it isn't");
                                }
                            }
                        } else {
                            ESP_LOGW(TAG, "Failed to queue device tester present data");
                        }
                    }
                }
            }
        } else {
            ESP_LOGD(TAG, "Received ISOTP data in queue: %d", isotpQueueData.len);
            
            EventBits_t eBits;
            if (channel->config->periodicTesterPresentEnabled) {
                eBits = xEventGroupGetBits(channel->otherConfig->xISOTP_dtpEventHandle);
                if ( eBits & DTP_EVENT_DTP_RESPONSE_REQUESTED) {
                    ESP_LOGD(TAG, "TX TASK: PTP response expected");
                    vTaskDelay(pdMS_TO_TICKS(50));
                    continue;
                } else if (eBits & DTP_EVENT_NORMAL_RESPONSE_REQUESTED) {
                    ESP_LOGD(TAG, "TX TASK: NORMAL response expected");
                    vTaskDelay(pdMS_TO_TICKS(50));
                    continue;
                }
            }

            if (channel->config->periodicTesterPresentEnabled) {
                eBits = xEventGroupSetBits(channel->otherConfig->xISOTP_dtpEventHandle,
                    DTP_EVENT_NORMAL_RESPONSE_REQUESTED);
            }
            int isoTpRet = isotp_send_data(channel, isotpQueueData.data, isotpQueueData.len);
            if (isoTpRet == 0) {
                ESP_LOGD(TAG, "Successful ISOTP transmission");
                if (channel->config->maxMicrosBetweenReqResp != 0x0000) {
                    if (channel->otherConfig->xWaitReqRespTimerHandle != NULL) {
                        gptimer_start(channel->otherConfig->xWaitReqRespTimerHandle);
                    } else {
                        ESP_LOGW(TAG, "Timer handle should been available but it isn't");
                    }
                }
            } else {
                if (channel->config->periodicTesterPresentEnabled) {
                    eBits = xEventGroupClearBits(channel->otherConfig->xISOTP_dtpEventHandle,
                        DTP_EVENT_NORMAL_RESPONSE_REQUESTED);
                }
                ESP_LOGW(TAG, "Failed to send ISOTP frame");
            }
            
            vPortFree(isotpQueueData.data);
            isotpQueueData.data = NULL;
            ret = xQueueReceive(channel->otherConfig->xISOTP_socketToCANQueueHandle, 
                &isotpQueueData, pdMS_TO_TICKS(1000));
            if (ret == pdFALSE) {
                ESP_LOGE(TAG, "NEVER EXPECTED THIS TO HAPPEN");
            }
        } 
        /* ISO TP Socket Data Handle Sequence */

    }

    xEventGroupSetBits(channel->otherConfig->xISOTP_taskEventHandle, ISOTP_TASK_EVENT_TX_TASK_CLOSED);
    vTaskDelete(NULL);
}

void handleISOTPRxData(ChannelAllConfig *channel, ISOTPRxHolder *rxHolder) {
    // Send data to socket queue
    // ESP_LOGI(TAG, "CREATING PKT FOR SOCKET");
    // DEBUG_PRINT_HEX_DATA(rxHolder->data, rxHolder->dataLen);
    EventBits_t eBits;
    if (channel->config->periodicTesterPresentEnabled) {
        bool isWaitResponse = false;
        eBits = xEventGroupGetBits(channel->otherConfig->xISOTP_dtpEventHandle);
        if (eBits & DTP_EVENT_DTP_RESPONSE_REQUESTED) {
            if (
                rxHolder->dataLen == sizeof(dtpDataExpectedResponseReference)
                && memcmp(rxHolder->data, dtpDataExpectedResponseReference, 
                    sizeof(dtpDataExpectedResponseReference)) == 0
            ) {
                // Matched device tester response, Ignore Data
                ESP_LOGD(TAG, "DTP PROPER RECEIVED");
                            
                if (channel->config->maxMicrosBetweenReqResp != 0x0000) {
                    if (channel->otherConfig->xWaitReqRespTimerHandle != NULL) {
                        ESP_ERROR_CHECK(gptimer_set_raw_count(channel->otherConfig->xWaitReqRespTimerHandle, (5 * 1000 * 1000)));
                        gptimer_stop(channel->otherConfig->xWaitReqRespTimerHandle);
                    } else {
                        ESP_LOGW(TAG, "Timer handle should been available but it isn't - 1");
                    }
                }
                
                channel->otherConfig->lastDeviceResponseTime = esp_timer_get_time();
                eBits = xEventGroupClearBits(
                            channel->otherConfig->xISOTP_dtpEventHandle,
                            DTP_EVENT_DTP_RESPONSE_REQUESTED);
                eBits = xEventGroupSetBits(
                            channel->otherConfig->xISOTP_dtpEventHandle,
                            DTP_EVENT_DTP_RESPONSE_RECEIVED);

                destructRxHolder(rxHolder);
                return;
            }
        } else if (eBits & DTP_EVENT_NORMAL_RESPONSE_REQUESTED) {
            if ( rxHolder->dataLen == 3 ) {
                // ESP_LOGI(TAG, "DTXSDDD: %x %x %x", rxHolder->data[0], rxHolder->data[1], rxHolder->data[2]);
                if (rxHolder->data[0] == 0x7F && rxHolder->data[2] == 0x78) {
                    ESP_LOGW(TAG, "WAIT REQUESTED");
                    
                    if (channel->config->maxMicrosBetweenReqResp != 0x0000) {
                        if (channel->otherConfig->xWaitReqRespTimerHandle != NULL) {
                            ESP_ERROR_CHECK(gptimer_set_raw_count(channel->otherConfig->xWaitReqRespTimerHandle, 
                                (((5 * 1000 * 1000) + (channel->config->maxMicrosBetweenReqResp * 1000)) - (3 * 1000 * 1000)) ));
                                // 5 sec + timeinms - 3sec equals 3 sec to timer timeout
                        } else {
                            ESP_LOGW(TAG, "Timer handle should been available but it isn't - 1");
                        }
                    }
                    eBits = xEventGroupGetBits(channel->otherConfig->xISOTP_dtpEventHandle);
                    if (eBits & DTP_EVENT_NORMAL_RESPONSE_REQUESTED) {
                        if (eBits & DTP_EVENT_NORMAL_RESPONSE_RECEIVED) {
                            ESP_LOGW(TAG, "RX TASK: wait resp but response rec is set, so clearing it"); //TODO(MAYANK): Temporary solution
                            eBits = xEventGroupClearBits(
                                        channel->otherConfig->xISOTP_dtpEventHandle,
                                        DTP_EVENT_NORMAL_RESPONSE_RECEIVED);
                        }
                    }
                    // channel->otherConfig->lastDeviceResponseTime = esp_timer_get_time();
                    isWaitResponse = true;
                }
            }
        }
        
        if (
            rxHolder->dataLen == sizeof(dtpDataExpectedResponseReference)
            && memcmp(rxHolder->data, dtpDataExpectedResponseReference, 
                sizeof(dtpDataExpectedResponseReference)) == 0
        ) {
            // Matched device tester response, Ignore Data
            if (channel->config->maxMicrosBetweenReqResp != 0x0000) {
                if (channel->otherConfig->xWaitReqRespTimerHandle != NULL) {
                    ESP_ERROR_CHECK(gptimer_set_raw_count(channel->otherConfig->xWaitReqRespTimerHandle, (5 * 1000 * 1000)));
                    gptimer_stop(channel->otherConfig->xWaitReqRespTimerHandle);
                } else {
                    ESP_LOGW(TAG, "Timer handle should been available but it isn't - 1");
                }
            }
            
            channel->otherConfig->lastDeviceResponseTime = esp_timer_get_time();
            eBits = xEventGroupClearBits(
                        channel->otherConfig->xISOTP_dtpEventHandle,
                        DTP_EVENT_DTP_RESPONSE_REQUESTED);
            eBits = xEventGroupSetBits(
                        channel->otherConfig->xISOTP_dtpEventHandle,
                        DTP_EVENT_DTP_RESPONSE_RECEIVED);

            destructRxHolder(rxHolder);
            return;
        }

        if (!isWaitResponse) {
            channel->otherConfig->lastDeviceResponseTime = esp_timer_get_time();
            ESP_LOGD(TAG, "RX TASK: NORMAL RESP RECEIVED");
            eBits = xEventGroupClearBits(
                        channel->otherConfig->xISOTP_dtpEventHandle,
                        DTP_EVENT_NORMAL_RESPONSE_REQUESTED);
            eBits = xEventGroupSetBits(
                        channel->otherConfig->xISOTP_dtpEventHandle,
                        DTP_EVENT_NORMAL_RESPONSE_RECEIVED);
        }
    }
    // else {
    //     if (
    //         rxHolder->dataLen == sizeof(dtpDataExpectedResponseReference)
    //         && memcmp(rxHolder->data, dtpDataExpectedResponseReference, 
    //             sizeof(dtpDataExpectedResponseReference)) == 0
    //     ) {
    //         ESP_LOGW(TAG, "UNEXPECTED DTP PACKET IN DTP DISABLE");
    //         channel->otherConfig->lastDeviceResponseTime = esp_timer_get_time();
    //         eBits = xEventGroupClearBits(
    //                     channel->otherConfig->xISOTP_dtpEventHandle,
    //                     DTP_EVENT_DTP_RESPONSE_REQUESTED);
    //         eBits = xEventGroupSetBits(
    //                     channel->otherConfig->xISOTP_dtpEventHandle,
    //                     DTP_EVENT_DTP_RESPONSE_RECEIVED);
    //         destructRxHolder(rxHolder);
    //         return;
    //     }
    // }
    if (channel->config->maxMicrosBetweenReqResp != 0x0000) {
        if (channel->otherConfig->xWaitReqRespTimerHandle != NULL) {
            ESP_ERROR_CHECK(gptimer_set_raw_count(channel->otherConfig->xWaitReqRespTimerHandle, (5 * 1000 * 1000)));
            gptimer_stop(channel->otherConfig->xWaitReqRespTimerHandle);
        } else {
            ESP_LOGW(TAG, "Timer handle should been available but it isn't - 1");
        }
    }

    UWord cpStart = { .word = rxHolder->dataLen };
    cpStart.bytes.hbyte |= NIBBLE_VALUE_TO_HIGHER_NIBBLE(CUSTOM_PACKET_START_NIBBLE_DATA);
    uint16_t index = 0;
    rxHolder->customPktData[index++] = cpStart.bytes.hbyte;
    rxHolder->customPktData[index++] = cpStart.bytes.lbyte;
    
    rxHolder->customPktData[index++] = channel->otherConfig->index; /* Channel sel byte*/
    uint16_t crcDataStartLoc = index;

    index += rxHolder->dataLen;

    UWord crc;
    crc.word = CRC16_CCITT(&rxHolder->customPktData[crcDataStartLoc], index - crcDataStartLoc);

    rxHolder->customPktData[index++] = crc.bytes.hbyte;
    rxHolder->customPktData[index++] = crc.bytes.lbyte;

    // DEBUG_PRINT_HEX_DATA(rxHolder->customPktData, index);

    ESP_LOGI(TAG, "Sending data to socket");
    //sock_queue_dataPointerWithLen(rxHolder->customPktData, index);
    cancomm=true;
    if(Activechannel==USB_CH){
        senddatatouart(rxHolder->customPktData, index);
    }
    else if(Activechannel==TCPSCKT){
        sock_queue_dataPointerWithLen(rxHolder->customPktData, index);
    }
    else if(Activechannel==TCPSCKT){
        sock_queue_dataPointerWithLen(rxHolder->customPktData, index);
    }
    else if(Activechannel==BT_CH){
            BTWrite(rxHolder->customPktData, index);
    } 


    rxHolder->customPktData = NULL;
    destructRxHolder(rxHolder);
}

static void isotpRxTask(void *p) {
    ChannelAllConfig *channel = (ChannelAllConfig *) p;

    uint16_t waitTime = 1;
    int canBusRxFailCnt = 0;
    ESP_LOGI(TAG, "ISOTP RX TASK STARTED");
    while(1) {
        EventBits_t bits = xEventGroupGetBits(channel->otherConfig->xISOTP_taskEventHandle);
        if ( bits & ISOTP_TASK_EVENT_TASK_CLOSE_REQUESTED ) {
            break;
        }
        
        /* ISO TP Receive CAN Data Sequence */
        if (channel->otherConfig->rxHolder.customPktData == NULL) {
            waitTime = 1;
        } else {
            waitTime = 1000;
        }
        ISOTPFrame rxFrame;
        CANRxRetType retRxFrame = channel->fnConfig->receiveCANMessage(channel, &rxFrame, waitTime);
        if (retRxFrame != CAN_RX_RET_OK) {
            if (retRxFrame == CAN_RX_RET_TIMEOUT) {

            } else if (retRxFrame == CAN_RX_RET_RTR_FRAME) {
                ESP_LOGW(TAG, "Unwanted RTR frame received");
            } else if (retRxFrame == CAN_RX_RET_MUTEX_ACQUIRE_FAIL
                    || retRxFrame == CAN_RX_RET_INTERNAL_ERR) {
                if (canBusRxFailCnt++ > 10) {
                    ESP_LOGW(TAG, "Unexpected RX data failure");
                    // rxRet = ISOTP_RX_RET_RX_NO_DATA;
                    // break;
                }
            }
            // if (rxRet == ISOTP_RX_RET_RX_NO_DATA) {
            //     break;
            // }
            // ESP_LOGW(TAG, "Contintue in can rx");
            vTaskDelay(pdMS_TO_TICKS(10)); // Let other task do something
            continue;
        }
        ISOTPRxRetCode rxRet = isotp_parse_recieved_data(channel, &rxFrame, &channel->otherConfig->rxHolder);
        if (rxRet == ISOTP_RX_RET_OK) {

        } else if (rxRet == ISOTP_RX_RET_RX_FINISHED) {
            handleISOTPRxData(channel, &channel->otherConfig->rxHolder);
        } else if (rxRet == ISOTP_RX_RET_RX_NO_DATA) {
            // NO LOG HERE OTHERWISE will be printed every 10 ms
        } else { //if (rxRet == ISOTP_RX_RET_INVALID_ISOTP_FRAME_CODE) {
            ESP_LOGW(TAG, "Failed to receive isotp packet, reason: %d", rxRet);
            if (channel->otherConfig->rxHolder.customPktData != NULL) {
                destructRxHolder(&channel->otherConfig->rxHolder);
            }
        }

        /* ISO TP Receive CAN Data Sequence */
        // vTaskDelay(pdMS_TO_TICKS(1)); // Let other task do something
        vTaskDelay(pdMS_TO_TICKS(10)); // Let other task do something
    }
    xEventGroupSetBits(channel->otherConfig->xISOTP_taskEventHandle, ISOTP_TASK_EVENT_RX_TASK_CLOSED);
    vTaskDelete(NULL);
}

static void isotpTask(void *p) {
    ChannelAllConfig *channel = (ChannelAllConfig *) p;

    ESP_LOGI(TAG, "ISOTP TASK STARTED");
    while(1) {
        EventBits_t bits = xEventGroupGetBits(channel->otherConfig->xISOTP_taskEventHandle);
        if ( bits & ISOTP_TASK_EVENT_TASK_CLOSE_REQUESTED ) {
            break;
        }
        /* Device Tester Sequence */

        /* Device Tester Sequence */
        if (channel->otherConfig->xWaitReqRespTimeoutQueueHandle != NULL) {
            /* Max REQ RESP Wait Time */
            uint64_t ticksCount = 0;
            BaseType_t ret = xQueueReceive(channel->otherConfig->xWaitReqRespTimeoutQueueHandle, &ticksCount, pdMS_TO_TICKS(100));
            if (ret == pdPASS) {
                // Missed ECU response in time
                uint8_t refNoDataResp[] = {0x40, 0x00, 0x00, 0xFF, 0xFF};
                uint8_t *data = (uint8_t *) pvPortMalloc(sizeof(refNoDataResp));
                if (data != NULL) {
                    memcpy(data, refNoDataResp, sizeof(refNoDataResp));
                    data[2] = channel->otherConfig->index;
                    if(Activechannel==USB_CH){
                        senddatatouart(data, sizeof(refNoDataResp));
                    }
                    else if(Activechannel==TCPSCKT){
                        sock_queue_dataPointerWithLen(data, sizeof(refNoDataResp));
                    }else if(Activechannel==BT_CH){
                        BTWrite(data, sizeof(refNoDataResp));
                    } 
                     
                } else {
                    ESP_LOGW(TAG, "Could not allocate buffer for NoDataResponse");
                }
                ESP_LOGW(TAG, "No response Timer timed out at value: %" PRIu64, ticksCount);
            } 
            /* Max REQ RESP Wait Time */
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    xEventGroupSetBits(channel->otherConfig->xISOTP_taskEventHandle, ISOTP_TASK_EVENT_MAIN_TASK_CLOSED);
    vTaskDelete(NULL);
}

void initialise_isotp(ChannelAllConfig *channel) {
    // esp_log_level_set(TAG, ESP_LOG_DEBUG);

    if (channel->otherConfig->xISOTP_socketToCANQueueHandle == NULL) {
        channel->otherConfig->xISOTP_socketToCANQueueHandle = xQueueCreate(15, sizeof(ISOTPQueueData));
    }
    
    if (channel->otherConfig->xISOTP_taskEventHandle == NULL) {
        channel->otherConfig->xISOTP_taskEventHandle = xEventGroupCreate();
    }

    if (channel->otherConfig->xISOTP_flowPacketQueueHandle == NULL) {
        channel->otherConfig->xISOTP_flowPacketQueueHandle = xQueueCreate(10, sizeof(ISOTPFrame));
    }
    channel->config->periodicTesterPresentEnabled = false;

    char taskName[15];
    sprintf(taskName, "isotpTx_%d", channel->otherConfig->index);
    xTaskCreate(isotpTxTask, taskName, 4096, channel, tskIDLE_PRIORITY + 2, NULL);
    
    sprintf(taskName, "isotpRx_%d", channel->otherConfig->index);
    xTaskCreate(isotpRxTask, taskName, 4096, channel, tskIDLE_PRIORITY + 2, NULL);

    if (channel->otherConfig->xISOTP_TaskHandle == NULL) {
        sprintf(taskName, "isotp_%d", channel->otherConfig->index);
        xTaskCreate(isotpTask, taskName, 4096, channel, tskIDLE_PRIORITY + 2, &channel->otherConfig->xISOTP_TaskHandle);
    }
    settings_setMaxReqRespWaitTime(channel, 1 * 1000); // 1000 ms
}

void deinitialise_isotp(ChannelAllConfig *channel) {

    xEventGroupSetBits(channel->otherConfig->xISOTP_taskEventHandle, ISOTP_TASK_EVENT_TASK_CLOSE_REQUESTED);

    EventBits_t retBits = xEventGroupWaitBits(channel->otherConfig->xISOTP_taskEventHandle, 
            ISOTP_TASK_EVENT_RX_TASK_CLOSED | ISOTP_TASK_EVENT_TX_TASK_CLOSED | ISOTP_TASK_EVENT_MAIN_TASK_CLOSED,
            pdTRUE, pdTRUE, portMAX_DELAY);
    if (retBits & (ISOTP_TASK_EVENT_RX_TASK_CLOSED | ISOTP_TASK_EVENT_TX_TASK_CLOSED 
            | ISOTP_TASK_EVENT_MAIN_TASK_CLOSED)) {
        ESP_LOGI(TAG, "ISOTP all task closed");
    }

    vQueueDelete(channel->otherConfig->xISOTP_socketToCANQueueHandle);
    channel->otherConfig->xISOTP_socketToCANQueueHandle = NULL;

    vQueueDelete(channel->otherConfig->xISOTP_flowPacketQueueHandle);
    channel->otherConfig->xISOTP_flowPacketQueueHandle = NULL;

    vEventGroupDelete(channel->otherConfig->xISOTP_taskEventHandle);
    channel->otherConfig->xISOTP_taskEventHandle = NULL;

    if (channel->otherConfig->xISOTP_dtpEventHandle != NULL) {
        vEventGroupDelete(channel->otherConfig->xISOTP_dtpEventHandle);
        channel->otherConfig->xISOTP_dtpEventHandle = NULL;
        ESP_LOGI(TAG, "DTP event handler stoped in isotp deinit");
    }
    channel->config->periodicTesterPresentEnabled = false;

    channel->otherConfig->xISOTP_TaskHandle = NULL;
}


int isotp_sendDataToQueue(ChannelAllConfig *channel, uint8_t *data, uint16_t dataLen) {
    
    ISOTPQueueData isotpQueueData = {
        .data = data, .len = dataLen
    };

    BaseType_t retQueue = xQueueSend(channel->otherConfig->xISOTP_socketToCANQueueHandle, &isotpQueueData, pdMS_TO_TICKS(10000));
    
    if (retQueue == pdTRUE) {
        ESP_LOGD(TAG, "Successfully queued ISOTP frame");
        return 0;
    } else {
        ESP_LOGW(TAG, "Failed to queue ISOTP frame");
        
        if (data != NULL) {
            vPortFree(data);
            data = NULL;
        }
        return -1;
    }
}


/* TX FUNCTIONS */

int isotp_convertDataToISOTPSingle(const uint8_t *data, const uint16_t dataLen, ISOTPFrame *frame) {
    frame->single.code = ISOTP_FRAME_CODE_SINGLE;
    frame->single.len = dataLen;
    memcpy(frame->single.data, data, dataLen);
    frame->dataLen = ISOTP_PACKET_LEN_SINGLE(dataLen);
    return 0;
}

int isotp_convertDataToISOTPFirst(const uint8_t *data, const uint16_t dataLen, uint16_t *index, ISOTPFrame *frame) {
    ESP_LOGD(TAG, "FIRST PACKET CONVERSION: %d", dataLen);
    frame->first.code = ISOTP_FRAME_CODE_FIRST;
    UWord len = { .word = dataLen };
    frame->first.lenh = len.bytes.hbyte;
    frame->first.lenl = len.bytes.lbyte;
    memcpy(frame->first.data, &data[*index], ISOTP_PACKET_FIRST_MAX_LEN);
    frame->dataLen = ISOTP_PACKET_LEN_FIRST(ISOTP_PACKET_FIRST_MAX_LEN);
    *index += ISOTP_PACKET_FIRST_MAX_LEN;
    return 0;
}

int isotp_conversionDataToISOTPConsecutive(const uint8_t *data, const uint16_t dataLen,
        ConsecutiveIndex *cIndex, uint16_t *index, ISOTPFrame *frame) {
    frame->consecutive.code = ISOTP_FRAME_CODE_CONSECUTIVE;
    frame->consecutive.index = cIndex->lbyte;
    uint16_t remainingPacketLen = dataLen - *index;
    uint8_t lenToUse = ISOTP_PACKET_CONSECUTIVE_MAX_LEN < remainingPacketLen ?
                            ISOTP_PACKET_CONSECUTIVE_MAX_LEN : remainingPacketLen;
    memcpy(frame->consecutive.data, &data[*index], lenToUse);
    frame->dataLen = ISOTP_PACKET_LEN_CONSECUTIVE(lenToUse);
    *index += lenToUse;
    cIndex->value++;
    return 0;
}

int isotp_convertISOTPFlowToData(const ISOTPFrame *frame, MultiframeFlow *flow) {
    MultiframeFlow localFlow;
    if (frame->flow.code != ISOTP_FRAME_CODE_FLOW) {
        ESP_LOGW(TAG, "Flow code does not match in FLow packet\n");
        return -1;
    }
    if (frame->flow.status > 2) {
        ESP_LOGW(TAG, "Flow status is invalid in Flow packet\n");
        return -2;
    }
    if (frame->flow.status == ISOTP_FLOW_STATUS_WAIT
        || frame->flow.status == ISOTP_FLOW_STATUS_OVERFLOW_OR_ABORT) {
        flow->flowStatus = frame->flow.status;
        return 0;
    }
    if ( (frame->flow.separationTime > 0x7F && frame->flow.separationTime < 0xF1)
        || frame->flow.separationTime > 0xF9) {
        ESP_LOGW(TAG, "Flow separation time is invalid in Flow packet\n");
        return -4;
    }
    if (frame->flow.blockSize == 0) {
        localFlow.blockSizeRemaining = MAX_POSSIBLE_ISO_PACKET_BLOCK_SIZE;
    } else {
        localFlow.blockSizeRemaining = frame->flow.blockSize;
    }
    localFlow.flowStatus = frame->flow.status;
    localFlow.separationTimeActualValue = frame->flow.separationTime;
    if (localFlow.separationTimeActualValue <= 0x7F) {
        localFlow.separationTimeMicroseconds = localFlow.separationTimeActualValue * 1000;
    } else {
        localFlow.separationTimeMicroseconds = (localFlow.separationTimeActualValue - 0xF1) * 100;
    }
    
    flow->blockSizeRemaining = localFlow.blockSizeRemaining;
    flow->flowStatus = localFlow.flowStatus;
    flow->separationTimeActualValue = localFlow.separationTimeActualValue;
    flow->separationTimeMicroseconds = localFlow.separationTimeMicroseconds;

    ESP_LOGD(TAG, "FLOW SEPARATION TIME: %d us, BS: %d",
          flow->separationTimeMicroseconds, flow->blockSizeRemaining);

    return 0;
}

int isotp_send_single_frame(ChannelAllConfig *channel, const uint8_t *data, uint16_t dataLen) {
    ISOTPFrame frame;
    isotp_convertDataToISOTPSingle(data, dataLen, &frame);

    channel->fnConfig->sendCANMessage(channel, &frame);

    return 0;
}

int isotp_send_multiple_frame(ChannelAllConfig *channel, const uint8_t *data, uint16_t dataLen) {
    ISOTPFrame frame;

    uint16_t index = 0;
    ConsecutiveIndex cIndex = { .value = 0 };
    ISOTPMultiframeState multiframeState = ISOTP_MULTI_FRAME_FIRST;
    MultiframeFlow flow = {
        .blockSizeRemaining = 0,
        .flowStatus = ISOTP_FLOW_STATUS_CONTINUE_TO_SEND,
        .lastPacketSentTime = 0,
        .separationTimeActualValue = 0,
        .separationTimeMicroseconds = 0
    };
    bool errorOccured = false;
    // bool prevDTPStatus = channel->config->periodicTesterPresentEnabled;
    // channel->config->periodicTesterPresentEnabled = false;
    while(index < dataLen) {
        int ret;
        switch (multiframeState) {
            case ISOTP_MULTI_FRAME_FIRST:
                ret = isotp_convertDataToISOTPFirst(data, dataLen, &index, &frame);
                if (ret != 0) {
                    errorOccured = true;
                    break;
                }
                
                channel->fnConfig->sendCANMessage(channel, &frame);

                multiframeState = ISOTP_MULTI_FRAME_FLOW;
                cIndex.value = 1;
                break;
            case ISOTP_MULTI_FRAME_FLOW:
                int countAtmpt = 0;
                BaseType_t qfRet;
                do {
                    qfRet = xQueueReceive(channel->otherConfig->xISOTP_flowPacketQueueHandle, &frame, pdMS_TO_TICKS(1000));
                    if (qfRet == pdFALSE) {
                        errorOccured = true;
                        ESP_LOGW(TAG, "Invalid flow control frame received %d %d", index, dataLen);
                    }
                } while(qfRet == pdFALSE && ++countAtmpt < 3);
                if (countAtmpt == 3) {
                    break;
                }
                ret = isotp_convertISOTPFlowToData(&frame, &flow);
                if (ret != 0) {
                    errorOccured = true;
                    ESP_LOGW(TAG, "Invalid flow control conversion to data");
                    DEBUG_PRINT_HEX_DATA(frame.data, frame.dataLen);
                    break;
                }
                if (flow.flowStatus == ISOTP_FLOW_STATUS_OVERFLOW_OR_ABORT) {
                    ESP_LOGW(TAG, "ISOTP abort or overflow requested");
                    errorOccured = true;
                    break;
                }
                if (flow.flowStatus == ISOTP_FLOW_STATUS_WAIT) {
                    // Continue again and wait for new flow message
                    ESP_LOGI(TAG, "Wait flow status = Waiting for next flow message\n");
                    break;
                }

                gptimer_alarm_config_t alarm_config;
                if (channel->config->forceSTMinTx != 0x00) {
                    // Force ST_Min is set so follow it only
                    generate_consecutiveAlarmConfig(channel->config->forceSTMinTx, &alarm_config);
                    gptimer_set_alarm_action(channel->otherConfig->xConsecutiveTimerHandle, &alarm_config);
                    gptimer_set_raw_count(channel->otherConfig->xConsecutiveTimerHandle, 0);
                    gptimer_start(channel->otherConfig->xConsecutiveTimerHandle);
                } else if (flow.separationTimeMicroseconds != 0) {
                    // Force ST_Min is not set, following ECU ST_Min
                    alarm_config.alarm_count = flow.separationTimeMicroseconds;
                    alarm_config.flags.auto_reload_on_alarm = 0;
                    gptimer_set_alarm_action(channel->otherConfig->xConsecutiveTimerHandle, &alarm_config);
                    gptimer_set_raw_count(channel->otherConfig->xConsecutiveTimerHandle, 0);
                    gptimer_start(channel->otherConfig->xConsecutiveTimerHandle);
                }

                multiframeState = ISOTP_MULTI_FRAME_CONSECUTIVE;
                break;
            case ISOTP_MULTI_FRAME_CONSECUTIVE:
                ret = isotp_conversionDataToISOTPConsecutive(data, dataLen, &cIndex, &index, &frame);
                if (ret != 0) {
                    ESP_LOGW(TAG, "Consecutive frame failed to convert");
                    errorOccured = true;
                    break;
                }

                if (channel->config->forceSTMinTx != 0x00 || flow.separationTimeMicroseconds != 0) {
                    uint64_t timerCount;
                    BaseType_t retTimerQueue = xQueueReceive(channel->otherConfig->xConsecutiveTimeoutQueueHandle,
                                                            &timerCount, pdMS_TO_TICKS(127000));
                    if ( retTimerQueue != pdPASS ) {
                        ESP_LOGW(TAG, "Consecutive timer didn't timeout, something is not right");
                        errorOccured = true;
                        break;
                    }
                    channel->fnConfig->sendCANMessage(channel, &frame);
                    gptimer_set_raw_count(channel->otherConfig->xConsecutiveTimerHandle, 0);
                    gptimer_start(channel->otherConfig->xConsecutiveTimerHandle);
                } else {
                    channel->fnConfig->sendCANMessage(channel, &frame);
                }                
                // ESP_LOGI(TAG, "CONSEC: %d", index);

                flow.blockSizeRemaining--;
                if (flow.blockSizeRemaining <= 0) {
                    multiframeState = ISOTP_MULTI_FRAME_FLOW;
                }

                break;
            default:
                ESP_LOGE(TAG, "Unexpected Multiframe state");
                break;
        }
        if ( errorOccured ) {
            ESP_LOGI(TAG, "ERROR Occured during multiframe transmission");
            if (flow.flowStatus == ISOTP_FLOW_STATUS_OVERFLOW_OR_ABORT) {
                break;
            }
            break;
        }
    }
    // channel->config->periodicTesterPresentEnabled = prevDTPStatus;
    // channel->otherConfig->isDeviceTesterPresentResponseExpected = false;

    if (errorOccured) {
        return -1;
    }
    ESP_LOGD(TAG, "Multiframe complete %d %d", index, dataLen);
    return 0;
}

static int isotp_send_data(ChannelAllConfig *channel, uint8_t *data, uint16_t dataLen) {
    if (dataLen == 0) {
        // No data to transmit; Consider successfull transmission
        return 0;
    }
    if (dataLen > 0x0FFF) {
        // Invalid data length
        return -1;
    } 
    if (dataLen < 8) {
        // ISO-TP Single Frame can be used
        return isotp_send_single_frame(channel, data, dataLen);
    } else {
        // Start multi frame ISO-TP
        return isotp_send_multiple_frame(channel, data, dataLen);
    }
}

/* RX FUNCtIONS */

#define RXHOLD_CUSTOM_PKT_STARTNIBBLE_AND_LEN_BYTES_COUNT 2


static int initRxHolder(ISOTPRxHolder *rxHolder, uint16_t len) {
    rxHolder->dataLen = 0;
    rxHolder->consecutiveIndex = 0;
    rxHolder->totalLen = len;
    rxHolder->remainingPacketBeforeNextFlow = 0;
    rxHolder->customPktData = pvPortMalloc(rxHolder->totalLen
            + 7 /* 7 bytes extra to hold padded last consecutive packet */
            + RXHOLD_CUSTOM_PKT_STARTNIBBLE_AND_LEN_BYTES_COUNT /* CustomPacket Nibble + DataLen */
            + 1 /* Channel sel */
            + 2 /* CRC 16 bits*/
        ); 
    if (rxHolder->customPktData == NULL) {
        ESP_LOGW(TAG, "Failed to allocate space for RX buffer");
        return -1;
    }
    rxHolder->data = &rxHolder->customPktData[
            RXHOLD_CUSTOM_PKT_STARTNIBBLE_AND_LEN_BYTES_COUNT
            + 1 /* Channel sel */
        ];
    return 0;
}

static int destructRxHolder(ISOTPRxHolder *rxHolder) {
    if ( rxHolder->customPktData != NULL ) {
        vPortFree(rxHolder->customPktData);
        rxHolder->customPktData = NULL;
    }
    rxHolder->data = NULL;
    rxHolder->customPktData = NULL;
    rxHolder->dataLen = 0;
    rxHolder->consecutiveIndex = 0;
    rxHolder->totalLen = 0;
    rxHolder->remainingPacketBeforeNextFlow = 0;
    return 0;
}

ISOTPRxRetCode isotp_parseISOTPSingleToData(const ISOTPFrame *frame, ISOTPRxHolder *rxHolder) {
    if (frame->single.code != ISOTP_FRAME_CODE_SINGLE) {
        ESP_LOGW(TAG, "Invalid single frame code\n");
        return ISOTP_RX_RET_INVALID_SINGLE_FRAME_PACKET;
    };
    
    initRxHolder(rxHolder, frame->single.len);

    memcpy(&rxHolder->data[rxHolder->dataLen], frame->single.data, frame->single.len);
    rxHolder->dataLen += frame->single.len;
    rxHolder->totalLen += frame->single.len;
    return ISOTP_RX_RET_OK;
}

ISOTPRxRetCode isotp_parseISOTPFirstToData(const ISOTPFrame *frame, ISOTPRxHolder *rxHolder) {
    if (frame->first.code != ISOTP_FRAME_CODE_FIRST) {
        ESP_LOGW(TAG, "Invalid first frame code\n");
        return ISOTP_RX_RET_INVALID_FIRST_FRAME_CODE;
    };
    
    UWord totalPacketLen = { .bytes.hbyte = frame->first.lenh, .bytes.lbyte = frame->first.lenl };
    initRxHolder(rxHolder, totalPacketLen.word);

    uint16_t lenToAdd = rxHolder->totalLen < ISOTP_PACKET_FIRST_MAX_LEN ? rxHolder->totalLen : ISOTP_PACKET_FIRST_MAX_LEN;
    memcpy(&rxHolder->data[rxHolder->dataLen], frame->first.data, lenToAdd);
    rxHolder->dataLen += lenToAdd;
    rxHolder->consecutiveIndex++;
    return ISOTP_RX_RET_OK;
}

ISOTPRxRetCode isotp_parseISOTPConsecutiveToData(const ISOTPFrame *frame, ISOTPRxHolder *rxHolder) {
    if (frame->consecutive.code != ISOTP_FRAME_CODE_CONSECUTIVE) {
        ESP_LOGW(TAG, "Invalid consecutive frame code\n");
        return ISOTP_RX_RET_INVALID_CONSECUTIVE_FRAME_CODE;
    };
    if (frame->consecutive.index != rxHolder->conIndexlnibble) {
        ESP_LOGI(TAG, "Consecutive frame wrong index\n");
        return ISOTP_RX_RET_WRONG_CONSECUTIVE_INDEX;
    }

    uint16_t remainingLen = rxHolder->totalLen - rxHolder->dataLen;

    uint16_t lenToAdd = remainingLen < ISOTP_PACKET_CONSECUTIVE_MAX_LEN ?
          remainingLen : ISOTP_PACKET_CONSECUTIVE_MAX_LEN;
    memcpy(&rxHolder->data[rxHolder->dataLen], frame->consecutive.data, lenToAdd);
    rxHolder->dataLen += lenToAdd;
    rxHolder->consecutiveIndex++;
    rxHolder->remainingPacketBeforeNextFlow--;
    return ISOTP_RX_RET_OK;
}

static ISOTPRxRetCode isotp_send_flow_packet(ChannelAllConfig *channel, ISOTPRxHolder *rxHolder) {
    ISOTPFrame frame;
    frame.flow.code = ISOTP_FRAME_CODE_FLOW;
    frame.flow.status = ISOTP_FLOW_STATUS_CONTINUE_TO_SEND;
    frame.flow.blockSize = MAX_RX_FLOW_SIZE;
    frame.flow.separationTime = RX_FLOW_TIME_SEPARATION;

    rxHolder->remainingPacketBeforeNextFlow = frame.flow.blockSize;
    frame.dataLen = ISOTP_PACKET_LEN_FLOW();

    channel->fnConfig->sendCANMessage(channel, &frame);

    return ISOTP_RX_RET_OK;
}

ISOTPRxRetCode isotp_parse_recieved_data(ChannelAllConfig *channel, const ISOTPFrame *frame, ISOTPRxHolder *rxHolder) {
    ISOTPRxRetCode ret = ISOTP_RX_RET_OK;
    
    switch(frame->single.code) {
    case ISOTP_FRAME_CODE_SINGLE: {
        ret = isotp_parseISOTPSingleToData(frame, rxHolder);
        if (ret == ISOTP_RX_RET_OK) {
            // send to SOCKET
            ret = ISOTP_RX_RET_RX_FINISHED;
            return ret;
        }
    }
    break;
    case ISOTP_FRAME_CODE_FIRST: {
        ret = isotp_parseISOTPFirstToData(frame, rxHolder);
        if (ret == ISOTP_RX_RET_OK) {
            // send flow packet
            ret = isotp_send_flow_packet(channel, rxHolder);
        }
    }
    break;
    case ISOTP_FRAME_CODE_CONSECUTIVE: {
        ret = isotp_parseISOTPConsecutiveToData(frame, rxHolder);
        if (ret == ISOTP_RX_RET_OK) {
            ESP_LOGD(TAG, "in consec: datalen: %d, totalLen: %d, rem: %d",
                rxHolder->dataLen, rxHolder->totalLen, rxHolder->remainingPacketBeforeNextFlow);
            if (rxHolder->dataLen >= rxHolder->totalLen) {
                // send to SOCKET
                ret = ISOTP_RX_RET_RX_FINISHED;
                return ret;
            }
            if (rxHolder->remainingPacketBeforeNextFlow <= 0) {
                //send flow packet
                ret = isotp_send_flow_packet(channel, rxHolder);
            }
        }
    }
    break;
    case ISOTP_FRAME_CODE_FLOW: {
        BaseType_t flowRet = xQueueSend(channel->otherConfig->xISOTP_flowPacketQueueHandle, frame, pdMS_TO_TICKS(1000));
        if ( flowRet == pdFALSE ) {
            ESP_LOGW(TAG, "Failed to queue flow packet");
        }
        ret = ISOTP_RX_RET_RX_NO_DATA;
    }
    break;
    default:
        ret = ISOTP_RX_RET_INVALID_ISOTP_FRAME_CODE;
        ESP_LOGI(TAG, "Invalid Recieved ISOTP frame from CAN bus\n");
        break;
    }

    return ret;
}
