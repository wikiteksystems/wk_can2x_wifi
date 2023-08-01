#ifndef __MAIN_INC_ISO_TP__
#define __MAIN_INC_ISO_TP__

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "channel.h"

#define ISOTP_FRAME_CODE_SINGLE         0x00
#define ISOTP_FRAME_CODE_FIRST          0x01
#define ISOTP_FRAME_CODE_CONSECUTIVE    0x02
#define ISOTP_FRAME_CODE_FLOW           0x03

#define ISOTP_PACKET_LEN_SINGLE(dataLen) (1 + dataLen)
#define ISOTP_PACKET_LEN_FIRST(dataLen) (2 + dataLen)
#define ISOTP_PACKET_LEN_CONSECUTIVE(dataLen) (1 + dataLen)
#define ISOTP_PACKET_LEN_FLOW() (3)

#define ISOTP_PACKET_FIRST_MAX_LEN 6
#define ISOTP_PACKET_CONSECUTIVE_MAX_LEN 7
#define MAX_POSSIBLE_ISO_PACKET_BLOCK_SIZE ( \
            1 /*6 byte first packet*/ \
            + (int)((4096 - 6/*6 byte first packet*/) / 7 /* 7 bytes in consec frame*/) \
            + 1 /*extra for remaining bytes*/)


#define MAX_RX_FLOW_SIZE 00 // 8 bit value
#define RX_FLOW_TIME_SEPARATION (0x01)



typedef enum {
    ISOTP_MULTI_FRAME_FIRST = 0,
    ISOTP_MULTI_FRAME_CONSECUTIVE,
    ISOTP_MULTI_FRAME_FLOW
} ISOTPMultiframeState;

typedef union {
    struct {
        uint8_t lbyte;
        uint8_t hbyte;
    };
    uint8_t value;
} ConsecutiveIndex;

typedef enum {
    ISOTP_FLOW_STATUS_CONTINUE_TO_SEND = 0,
    ISOTP_FLOW_STATUS_WAIT = 1,
    ISOTP_FLOW_STATUS_OVERFLOW_OR_ABORT = 2
} ISOTPFlowStatus;

typedef struct {
    ISOTPFlowStatus flowStatus;
    uint16_t blockSizeRemaining;
    uint16_t separationTimeActualValue;
    uint16_t separationTimeMicroseconds;
    int64_t lastPacketSentTime;
} MultiframeFlow;


typedef struct {
    uint8_t *data;
    uint16_t len;
} ISOTPQueueData;

typedef enum {
    ISOTP_RX_RET_INVALID_SINGLE_FRAME_PACKET = -1,
    ISOTP_RX_RET_INVALID_FIRST_FRAME_CODE = -2,
    ISOTP_RX_RET_INVALID_CONSECUTIVE_FRAME_CODE = -3,
    ISOTP_RX_RET_WRONG_CONSECUTIVE_INDEX = -4,
    ISOTP_RX_RET_INVALID_ISOTP_FRAME_CODE = -5,
    ISOTP_RX_RET_OK = 0,
    ISOTP_RX_RET_RX_FINISHED = 1,
    ISOTP_RX_RET_RX_CONTINUE = 2,
    ISOTP_RX_RET_RX_NO_DATA = 3
} ISOTPRxRetCode;

extern QueueHandle_t isotp_queue;

void initialise_isotp(ChannelAllConfig *channel);
void deinitialise_isotp(ChannelAllConfig *channel);

int isotp_sendDataToQueue(ChannelAllConfig *channel, uint8_t *data, uint16_t dataLen);
// int isotp_send_data(uint8_t *data, uint16_t dataLen);
int isotp_parse_recieved_data(ChannelAllConfig *channel, const ISOTPFrame *frame, ISOTPRxHolder *rxHolder);

#endif