#ifndef __CRC__
#define __CRC__

#include <stdint.h>

uint16_t CRC16_CCITT(const uint8_t* buffer, uint16_t size);

#endif