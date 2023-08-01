#ifndef __TCP_PACKET_HANDLER__
#define __TCP_PACKET_HANDLER__

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define CUSTOM_PACKET_START_NIBBLE_DATA (4)
#define CUSTOM_PACKET_START_NIBBLE_SECURITY (5)
#define CUSTOM_PACKET_START_NIBBLE_SETTING (2)
#define CUSTOM_PACKET_START_NIBBLE_IVN_CMD (6)

#define CUSTOM_PACKET_OK 0
#define CUSTOM_PACKET_INST_SECURITY_CHECK 1
#define CUSTOM_PACKET_INST_SETTINGS 2
#define ERR_CUSTOM_PACKET_INPUT_LEN_IS_LESS_THAN_MIN_POSSIBLE_PACKET_SIZE -1
#define ERR_CUSTOM_PACKET_START_NIBBLE_IS_INCORRECT -2
#define ERR_CUSTOM_PACKET_TOTAL_LEN_IS_NOT_SUFFICIENT -3
#define ERR_CUSTOM_PACKET_CRC_MISMATCH -4
#define ERR_CUSTOM_PACKET_INTERNAL_ERROR -5


int processCustomPacketIntoCANData(const uint8_t *input, uint16_t inputLen, uint8_t **canData,
      uint16_t *canDataLen, uint16_t *processedBytes, uint8_t *startNibble, uint8_t *channelNum);
bool processCANDataIntoCustomPacket(uint8_t *canData, uint16_t canDataLen,
    uint8_t *customPacketData, uint16_t *customPacketLen);

#endif
