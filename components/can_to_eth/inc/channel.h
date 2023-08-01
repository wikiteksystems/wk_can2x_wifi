#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "driver/gptimer.h"

#include "instruction_desc.h"
#include "isotp_types.h"
#include "can_iface.h"

#define DEFAULT_CAN_PROTOCOL                PROTOCOL_ISO15765_500KB_11BIT_CAN
#define DEFAULT_CAN_ID_TYPE                 CAN_ID_TYPE_11BIT
#define DEFAULT_CAN_RX_ID                   0x7E8
#define DEFAULT_CAN_TX_ID                   0x7E0
#define DEFAULT_CAN_RX_MASK                 0xFFFFFFFF

#define DEFAULT_CAN_PADDING_ENABLED_STATUS  false
#define DEFAULT_CAN_PADDING_BYTE            0x00
#define DEFAULT_CAN_MAX_MICROS_BW_REQ_RESP  0x0000
#define DEFAULT_CAN_PTP_ENABLED_STATUS      false
#define DEFAULT_CAN_FORCE_STMIN_TX          0x00


#define DTP_EVENT_DTP_RESPONSE_RECEIVED         (1 << 0)
#define DTP_EVENT_DTP_RESPONSE_REQUESTED        (1 << 1)
#define DTP_EVENT_NORMAL_RESPONSE_REQUESTED     (1 << 2)
#define DTP_EVENT_NORMAL_RESPONSE_RECEIVED      (1 << 3)
// #define DTP_EVENT_DTP_RESPONSE_RECEIVED     (1 << 0)
// #define DTP_EVENT_DTP_RESPONSE_RECEIVED     (1 << 0)
// #define DTP_EVENT_DTP_RESPONSE_RECEIVED     (1 << 0)


typedef enum {
    CAN_ID_TYPE_11BIT = 0, CAN_ID_TYPE_29BIT = 1
} CanIdType;

struct _channelAllDesc;
typedef struct _channelAllDesc ChannelAllConfig;

typedef struct _channelDesc {
    uint8_t protocol;
    CanIdType id_type;
    uint32_t rx_id;
    uint32_t rx_mask;
    uint32_t tx_id;
    bool paddingEnabled;
    uint8_t paddingByte;
    uint16_t maxMicrosBetweenReqResp;
    bool periodicTesterPresentEnabled;
    uint8_t forceSTMinTx; // 0xFF == OFF
} ChannelConfig;

typedef struct _channelFnDesc {
    int (*updateProtocol)(ChannelAllConfig *, uint8_t);
    int (*setRxId)(ChannelAllConfig *, uint32_t, uint32_t);
    int (*setTxId)(ChannelAllConfig *, uint32_t);
    int (*setPadding)(ChannelAllConfig *, bool, uint8_t);
    int (*clearReceiveQueue)(void);
    int (*sendCANMessage)(ChannelAllConfig *, ISOTPFrame *);
    CANRxRetType (*receiveCANMessage)(ChannelAllConfig *, ISOTPFrame *, uint16_t);
    int (*getLatestIVNFrame)(ChannelAllConfig *, uint32_t, ISOTPFrame *);
    int (*setPeriodicTesterPresentStatus)(ChannelAllConfig *, bool);
} ChannelFnConfig;

typedef struct _channelOtherDesc {
    SemaphoreHandle_t canbusUseMutex;
    TaskHandle_t xISOTP_TaskHandle;
    QueueHandle_t xISOTP_socketToCANQueueHandle;
    QueueHandle_t xISOTP_receivedDataQueueHandle;
    uint8_t index;
    EventGroupHandle_t xISOTP_taskEventHandle;

    QueueHandle_t xISOTP_flowPacketQueueHandle;
    EventGroupHandle_t xISOTP_dtpEventHandle;
    
    int64_t lastDeviceResponseTime;
    bool isDeviceTesterPresentResponseExpected;
    
    bool isNormalResonseExpected;

    ISOTPRxHolder rxHolder;

    QueueHandle_t xWaitReqRespTimeoutQueueHandle;
    gptimer_handle_t xWaitReqRespTimerHandle;

    QueueHandle_t xConsecutiveTimeoutQueueHandle;
    gptimer_handle_t xConsecutiveTimerHandle;
} ChannelOtherConfig;

struct _channelAllDesc {
    ChannelConfig *config;
    ChannelFnConfig *fnConfig;
    ChannelOtherConfig *otherConfig;
};

extern ChannelConfig channels[2];
extern ChannelFnConfig channelFns[2];
extern ChannelOtherConfig channelOthers[2];
extern ChannelAllConfig channelAll[2];



void settings_setMaxReqRespWaitTime(ChannelAllConfig *channel, uint16_t time);
int settings_setPeriodicTesterPresentStatus(ChannelAllConfig *channel, bool status);
void settings_setConsectiveTime(ChannelAllConfig *channel, uint8_t consecTime);
void generate_consecutiveAlarmConfig(uint8_t consecTime, gptimer_alarm_config_t *alarmConfig);
