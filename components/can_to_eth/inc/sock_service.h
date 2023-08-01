#ifndef __MAIN_INC_SOCK_SERVICE_H__
#define __MAIN_INC_SOCK_SERVICE_H__

#include <stdint.h>
#include <stdbool.h>

#define SOCKET_RX_CACHE_SIZE (4096 + 10)
#define SOCKET_RX_BUFFER_SIZE (4096 + 10 + 512 /* Extra buffer */)

typedef struct
{
    uint8_t tcpRxBuffer[SOCKET_RX_BUFFER_SIZE];
    uint16_t head;
    uint16_t tail;
    bool isOverLapped;
} SockRxData;

extern uint8_t mac[6];

// void initialise_sock(void);
// void sock_queue_dataPointerWithLen(uint8_t *data, uint16_t len); //send data to the socket
//void processAcquiredData(SockRxData *s, uint8_t *rxPktBuffer, uint16_t rxPktSize);
void processAcquiredData(uint8_t *rxPktBuffer, uint16_t rxPktSize);
void initialise_sock(void);
uint16_t SockRxData_remainingContinuousLen(const SockRxData *s);
void SockRxData_addLen(SockRxData *s, uint16_t len);
void SockRxData_releaseLen(SockRxData *s, uint16_t len);
void SockRxData_fillBuffer(SockRxData *s, uint8_t *buffer, uint16_t len);
uint16_t SockRxData_usedBuffer(const SockRxData *s);
void sock_queue_dataPointerWithLen(uint8_t *data, uint16_t len);
void sendDataTouartqueue(uint8_t *data, uint16_t len);
void selectResponseRx(uint8_t *data, uint16_t len);
uint8_t *SockRxData_getHeadPtr(SockRxData *s);

#endif // __MAIN_INC_SOCK_SERVICE_H__