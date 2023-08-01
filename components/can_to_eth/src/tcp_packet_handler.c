#include "tcp_packet_handler.h"

#include <string.h>

#include "esp_log.h"

#include "crc.h"
#include "utility.h"
#include "instructions.h"

static const char *TAG = "tcppackethandler";


int processCustomPacketIntoCANData(const uint8_t *input, uint16_t inputLen, uint8_t **canData,
    uint16_t *canDataLen, uint16_t *processedBytes, uint8_t *startNibble, uint8_t *channel) {
    *processedBytes = 0;
    if (inputLen < 1) {
        ESP_LOGW(TAG, "Input len is less than 1");
        return ERR_CUSTOM_PACKET_INPUT_LEN_IS_LESS_THAN_MIN_POSSIBLE_PACKET_SIZE;
    }
    uint8_t firstByte = input[0];
    uint8_t firstNibble = (firstByte & 0xF0) >> 4;
    *processedBytes += 1;
    if (firstNibble != CUSTOM_PACKET_START_NIBBLE_DATA
        && firstNibble != CUSTOM_PACKET_START_NIBBLE_SECURITY
        && firstNibble != CUSTOM_PACKET_START_NIBBLE_SETTING
        && firstNibble != CUSTOM_PACKET_START_NIBBLE_IVN_CMD
    ) {
        ESP_LOGW(TAG, "First nibble is not correct %x", input[0]);
        return ERR_CUSTOM_PACKET_START_NIBBLE_IS_INCORRECT;
    }
    uint16_t internalDataLen = ((input[0] & 0x0F) << 8 ) | input[1];
    *processedBytes += 1;
    
    uint8_t selChannel = input[*processedBytes];
    *processedBytes += 1;
    *channel = selChannel;

    if ( (inputLen - *processedBytes) < (
            internalDataLen
            + 2 /*checksumLen*/)) {
        *processedBytes = inputLen;
        ESP_LOGD(TAG, "DATA: Data length is less than required length %x", internalDataLen);
        return ERR_CUSTOM_PACKET_TOTAL_LEN_IS_NOT_SUFFICIENT;
    }

    uint16_t dataStartLoc = *processedBytes;
    uint16_t crc = CRC16_CCITT(&input[*processedBytes], internalDataLen);
    *processedBytes += internalDataLen;
    //ESP_LOGI(TAG, "Calculated CRC: 0x%04x", crc); //sunil
    uint16_t inputCRCStartByte = *processedBytes;
    UWord inputCRC;
    inputCRC.bytes.hbyte = input[inputCRCStartByte];
    inputCRC.bytes.lbyte = input[inputCRCStartByte + 1];
    
    *processedBytes += 2;

    if (crc != inputCRC.word) {
        ESP_LOGW(TAG, "CRC mismatch");
        return ERR_CUSTOM_PACKET_CRC_MISMATCH;
    }

    *canData = (uint8_t *)pvPortMalloc(internalDataLen);
    if (*canData == NULL) {
        void *temp = pvPortMalloc(1);
        ESP_LOGW(TAG, "Failed to allocate buffer of : %d %p", internalDataLen, temp);
        vPortFree(temp);
        temp = NULL;
        return ERR_CUSTOM_PACKET_INTERNAL_ERROR;
    } else {
        // ESP_LOGI(TAG, "Location of canData: %p", canData);
        *startNibble = firstNibble;
        memcpy(*canData, &input[dataStartLoc], internalDataLen);
        *canDataLen = internalDataLen;
    }
    return CUSTOM_PACKET_OK;
}

bool processCANDataIntoCustomPacket(uint8_t *canData, uint16_t canDataLen, uint8_t *customPacketData,
    uint16_t *customPacketLen) {
    if (canDataLen > 0x0FFF) {
        return false;
    }
    UWord uCanDataLen = { .word = canDataLen };
    uint16_t index = 0;
    customPacketData[index++] = (CUSTOM_PACKET_START_NIBBLE_DATA << 4) | uCanDataLen.bytes.hbyte;
    customPacketData[index++] = uCanDataLen.bytes.lbyte;
    memcpy(&customPacketData[index], canData, canDataLen);
    index += canDataLen;
    UWord crc;
    crc.word = CRC16_CCITT(canData, canDataLen);
    customPacketData[index++] = crc.bytes.hbyte;
    customPacketData[index++] = crc.bytes.lbyte;
    *customPacketLen = index;
    return true;
}
