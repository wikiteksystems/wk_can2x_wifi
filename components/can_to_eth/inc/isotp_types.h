#pragma once

#include <stdint.h>

typedef struct { 
    union {
        struct {
            uint8_t len: 4;
            uint8_t code : 4;
            uint8_t data[7];
        } single;
        struct {
            uint8_t lenh: 4;
            uint8_t code: 4;
            uint8_t lenl: 8;
            uint8_t data[6];
        } first;
        struct {
            uint8_t index: 4;
            uint8_t code: 4;
            uint8_t data[7];
        } consecutive;
        struct {
            uint8_t status: 4;
            uint8_t code: 4;
            uint8_t blockSize: 8;
            uint8_t separationTime: 8;
            uint8_t reserved[5];
        } flow;
        uint8_t data[8];
    };
    uint8_t dataLen;
} ISOTPFrame;

typedef struct {
    uint8_t *customPktData;
    uint8_t *data;
    uint16_t dataLen;
    uint16_t totalLen;
    union {
        struct {
            uint8_t conIndexlnibble: 4;
            uint8_t conIndexhnibble: 4;
        };
        uint8_t consecutiveIndex;
    };
    uint16_t remainingPacketBeforeNextFlow;
} ISOTPRxHolder;