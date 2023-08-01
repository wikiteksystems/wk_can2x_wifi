#include "instructions.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

#include "utility.h"
#include "crc.h"
#include "sock_service.h"
#include "canbus.h"

#include "iso_tp.h"
#include "usb_service.h"
#include "wifi_service.h"

#include "f_ota.h"
#include "bt_service.h"


const static char *TAG = "Instructions";

// #define FIXED_SECURITY_CODE_LEN 10
// static const uint8_t fixedSecurityBytes[FIXED_SECURITY_CODE_LEN] = 
//            {0x47, 0x56, 0x8a, 0xfe, 0x56, 0x21, 0x4e, 0x23, 0x80, 0x00};

// static const uint8_t fixedSecurityBytes[FIXED_SECURITY_CODE_LEN] = 
//               {0x47, 0x31, 0x30, 0x30,0x75, 0x77, 0x6c, 0x6b, 0x70, 0x61};

// #define FIXED_SECURITY_CODE_LEN 11
// static const uint8_t fixedSecurityBytes[FIXED_SECURITY_CODE_LEN] = 
//            {0x31, 0x30, 0x30, 0x31, 0x30, 0x70, 0x63, 0x65, 0x61, 0x67, 0x65};


#define FIXED_SECURITY_CODE_LEN 10
//50 0a 00 47 31 30 30 74 65 65 72 70 61 d1 3e
static const uint8_t fixedSecurityBytes[FIXED_SECURITY_CODE_LEN] = 
            {0x47, 0x31, 0x30, 0x30, 0x74, 0x65, 0x65, 0x72, 0x70, 0x61};

#define RESP_START_NIBBLE 0x02
#define RESP_PKT_LEN_NACK 0x0002 
#define RESP_PKT_LEN_ACK 0x0001
#define RESP_PKT_LEN_FW_VERSION 0x0003
#define RESP_PKT_LEN_MAC_ADDR 0x0006
#define RESP_PKT_LEN_CONSECUTIVE_TIME 0x0001

#define RESP_IVN_FRAME_NIBBLE 0x06
#define RESP_IVN_FRAME_NIBBLE_AS_DATA 0x04

// #define RESP_PKT_CHANNEL_SEL_BYTE_DUMMY 0x00
#define RESP_PKT_NACK_FIXED_HBYTE 0x01
#define RESP_PKT_ACK_BYTE 0x00





#define CP_RESP_ACK_LEN 6
#define CP_RESP_NACK_LEN 7
#define CP_RESP_FW_VERSION_LEN 8
#define CP_RESP_MAC_ADDR_LEN 11
#define CP_RESP_CONEC_TIME_LEN 6

void sendACK(uint8_t channelNum) {
    uint8_t *dataPtr = pvPortMalloc(CP_RESP_ACK_LEN);
    uint8_t index = 0;
    if (dataPtr != NULL) {
        UWord uword = {.word = RESP_PKT_LEN_ACK};
        uword.bytes.hbyte |= NIBBLE_VALUE_TO_HIGHER_NIBBLE(RESP_START_NIBBLE);

        dataPtr[index++] = uword.bytes.hbyte;
        dataPtr[index++] = uword.bytes.lbyte;
#ifdef CHANNEL_SELECTION_IN_CUSTOM_PACKET
        dataPtr[index++] = channelNum;
#endif
        uint8_t crcDataStartLoc = index;

        dataPtr[index++] = RESP_PKT_ACK_BYTE;
        
        UWord crc;
        crc.word = CRC16_CCITT(&dataPtr[crcDataStartLoc], index - crcDataStartLoc);

        dataPtr[index++] = crc.bytes.hbyte;
        dataPtr[index++] = crc.bytes.lbyte;
        
        if(Activechannel==USB_CH){
        senddatatouart(dataPtr, index);
        }
        else if(Activechannel==TCPSCKT){
            sock_queue_dataPointerWithLen(dataPtr, index);
        }else if(Activechannel==BT_CH){
            BTWrite(dataPtr, index);
        } 

    }
}

void sendNACK(uint8_t channelNum, uint8_t nackType) {
    uint8_t *dataPtr = pvPortMalloc(CP_RESP_NACK_LEN);
    uint8_t index = 0;
    if (dataPtr != NULL) {
        UWord uword = {.word = RESP_PKT_LEN_NACK};
        uword.bytes.hbyte |= NIBBLE_VALUE_TO_HIGHER_NIBBLE(RESP_START_NIBBLE);

        dataPtr[index++] = uword.bytes.hbyte;
        dataPtr[index++] = uword.bytes.lbyte;
#ifdef CHANNEL_SELECTION_IN_CUSTOM_PACKET
        dataPtr[index++] = channelNum;
#endif
        uint8_t crcDataStartLoc = index;

        dataPtr[index++] = RESP_PKT_NACK_FIXED_HBYTE;
        dataPtr[index++] = nackType;
        
        UWord crc;
        crc.word = CRC16_CCITT(&dataPtr[crcDataStartLoc], index - crcDataStartLoc);

        dataPtr[index++] = crc.bytes.hbyte;
        dataPtr[index++] = crc.bytes.lbyte;
        
        //ESP_LOGI(TAG, "Created NACK packet");
        DEBUG_PRINT_HEX_DATA(dataPtr, index);
        
        //sock_queue_dataPointerWithLen(dataPtr, index);
        if(Activechannel==USB_CH){
        senddatatouart(dataPtr, index);
        }
        else if(Activechannel==TCPSCKT){
            sock_queue_dataPointerWithLen(dataPtr, index);
        }
        else if(Activechannel==BT_CH){
            BTWrite(dataPtr, index);
        } 
    }
}

static int validateSecurityStatus(uint8_t channelNum) {
    ESP_LOGD(TAG, "Security check status: %d", devSettings.status.isSecurityCheckPassed);
    if (devSettings.status.isSecurityCheckPassed) {
        return 0;
    }
    sendNACK(channelNum, RESP_NACK_CODE_SECURITY_ACCESS_DENIED);
    return -1;
}

void sendFirmwareVersion(uint8_t channelNum, FWversion *version) {
    uint8_t *dataPtr = pvPortMalloc(CP_RESP_FW_VERSION_LEN);
    uint8_t index = 0;
    if (dataPtr != NULL) {
        UWord uword = {.word = RESP_PKT_LEN_FW_VERSION};
        uword.bytes.hbyte |= NIBBLE_VALUE_TO_HIGHER_NIBBLE(RESP_START_NIBBLE);

        dataPtr[index++] = uword.bytes.hbyte;
        dataPtr[index++] = uword.bytes.lbyte;
#ifdef CHANNEL_SELECTION_IN_CUSTOM_PACKET
        dataPtr[index++] = channelNum;
#endif
        uint8_t crcDataStartLoc = index;
        
        // dataPtr[index++] = SETTING_TYPE_GET_FIRMWARE_VERSION;

        dataPtr[index++] = version->build;
        dataPtr[index++] = version->major;
        dataPtr[index++] = version->minor;
        
        UWord crc;
        crc.word = CRC16_CCITT(&dataPtr[crcDataStartLoc], index - crcDataStartLoc);

        dataPtr[index++] = crc.bytes.hbyte;
        dataPtr[index++] = crc.bytes.lbyte;
        
        ESP_LOGI(TAG, "Created FW Version packet");
        DEBUG_PRINT_HEX_DATA(dataPtr, index);
        
        //sock_queue_dataPointerWithLen(dataPtr, index);
        if(Activechannel==USB_CH){
        senddatatouart(dataPtr, index);
        }
        else if(Activechannel==TCPSCKT){
            sock_queue_dataPointerWithLen(dataPtr, index);
        }
        else if(Activechannel==BT_CH){
            BTWrite(dataPtr, index);
        } 
    }
}

void sendConsecutiveTime(uint8_t channelNum, uint8_t consecutiveTime) {
    uint8_t *dataPtr = pvPortMalloc(CP_RESP_CONEC_TIME_LEN);
    uint8_t index = 0;
    if (dataPtr != NULL) {
        UWord uword = {.word = RESP_PKT_LEN_CONSECUTIVE_TIME};
        uword.bytes.hbyte |= NIBBLE_VALUE_TO_HIGHER_NIBBLE(RESP_START_NIBBLE);

        dataPtr[index++] = uword.bytes.hbyte;
        dataPtr[index++] = uword.bytes.lbyte;
#ifdef CHANNEL_SELECTION_IN_CUSTOM_PACKET
        dataPtr[index++] = channelNum;
#endif
        uint8_t crcDataStartLoc = index;
        
        // dataPtr[index++] = SETTING_TYPE_GET_FIRMWARE_VERSION;

        dataPtr[index++] = consecutiveTime;
        
        UWord crc;
        crc.word = CRC16_CCITT(&dataPtr[crcDataStartLoc], index - crcDataStartLoc);

        dataPtr[index++] = crc.bytes.hbyte;
        dataPtr[index++] = crc.bytes.lbyte;
        
        ESP_LOGI(TAG, "Created Consecutive Time packet");
        DEBUG_PRINT_HEX_DATA(dataPtr, index);
        
        //sock_queue_dataPointerWithLen(dataPtr, index);
        if(Activechannel==USB_CH){
            senddatatouart(dataPtr, index);
        }
        else if(Activechannel==TCPSCKT){
            sock_queue_dataPointerWithLen(dataPtr, index);
        }
        else if(Activechannel==BT_CH){
            BTWrite(dataPtr, index);
        } 
    }
}

static void sendMacAddr(uint8_t channelNum, uint8_t macAddr[6]) {
    uint8_t *dataPtr = pvPortMalloc(CP_RESP_MAC_ADDR_LEN);
    uint8_t index = 0;
    if (dataPtr != NULL) {
        UWord uword = {.word = RESP_PKT_LEN_MAC_ADDR};
        uword.bytes.hbyte |= NIBBLE_VALUE_TO_HIGHER_NIBBLE(RESP_START_NIBBLE);

        dataPtr[index++] = uword.bytes.hbyte;
        dataPtr[index++] = uword.bytes.lbyte;
#ifdef CHANNEL_SELECTION_IN_CUSTOM_PACKET
        dataPtr[index++] = channelNum;
#endif
        uint8_t crcDataStartLoc = index;
        
        // dataPtr[index++] = SETTING_TYPE_GET_FIRMWARE_VERSION;

        for ( uint8_t i = 0; i < RESP_PKT_LEN_MAC_ADDR; i++) {
            dataPtr[index++] = macAddr[i];
        }
        
        UWord crc;
        crc.word = CRC16_CCITT(&dataPtr[crcDataStartLoc], index - crcDataStartLoc);

        dataPtr[index++] = crc.bytes.hbyte;
        dataPtr[index++] = crc.bytes.lbyte;
        
        ESP_LOGI(TAG, "Created MAC ADDR packet");
        DEBUG_PRINT_HEX_DATA(dataPtr, index);
        
        //sock_queue_dataPointerWithLen(dataPtr, index);
        if(Activechannel==USB_CH){
        senddatatouart(dataPtr, index);
        }
        else if(Activechannel==TCPSCKT){
            sock_queue_dataPointerWithLen(dataPtr, index);
        }
        else if(Activechannel==BT_CH){
            BTWrite(dataPtr, index);
        } 
    }
}

static void sendIVNFrameResponse(uint8_t channelNum, ISOTPFrame *frame, uint8_t startNibble) {

    uint16_t internalDataLen = frame->dataLen + 1 /* IVN CMD CODE */;
    uint16_t totalPktLen = 2 /* CLLL */
#ifdef CHANNEL_SELECTION_IN_CUSTOM_PACKET
        + 1 /* channel sel byte */
#endif
        + internalDataLen + 2 /* crc bytes */;

    uint8_t *dataPtr = pvPortMalloc(totalPktLen);
    uint8_t index = 0;
    if (dataPtr != NULL) {
        UWord uword = {.word = internalDataLen};
        uword.bytes.hbyte |= NIBBLE_VALUE_TO_HIGHER_NIBBLE(startNibble);

        dataPtr[index++] = uword.bytes.hbyte;
        dataPtr[index++] = uword.bytes.lbyte;

#ifdef CHANNEL_SELECTION_IN_CUSTOM_PACKET
        dataPtr[index++] = channelNum;
#endif
        uint8_t crcDataStartLoc = index;

        // dataPtr[index++] = IVN_CMD_TYPE_GET_LATEST_IVN_FRAME;
        
        memcpy(&dataPtr[index], frame->data, frame->dataLen);
        index += frame->dataLen;
        
        UWord crc;
        crc.word = CRC16_CCITT(&dataPtr[crcDataStartLoc], index - crcDataStartLoc);

        dataPtr[index++] = crc.bytes.hbyte;
        dataPtr[index++] = crc.bytes.lbyte;
        
        ESP_LOGI(TAG, "Created Latest IVN frame packet");
        DEBUG_PRINT_HEX_DATA(dataPtr, index);
        
        //sock_queue_dataPointerWithLen(dataPtr, index);
        if(Activechannel==USB_CH){
        senddatatouart(dataPtr, index);
        }
        else if(Activechannel==TCPSCKT){
            sock_queue_dataPointerWithLen(dataPtr, index);
        } 
        else if(Activechannel==BT_CH){
            BTWrite(dataPtr, index);
        }
    }
}

void inst_checkSecurityCode(uint8_t channelNum, uint8_t *data, uint16_t len) {
    ESP_LOGI(TAG, "Checking security code");
    DEBUG_PRINT_HEX_DATA(data, len);

    if (len < FIXED_SECURITY_CODE_LEN) {
        sendNACK(channelNum, RESP_NACK_CODE_INVALID_KEY);
        return;
    }
    if (memcmp(data, fixedSecurityBytes, FIXED_SECURITY_CODE_LEN) != 0) {
        sendNACK(channelNum, RESP_NACK_CODE_INVALID_KEY);
        return;
    }
    devSettings.status.isSecurityCheckPassed = true;

    sendACK(channelNum);
}

void inst_settings(ChannelAllConfig *channel, uint8_t *data, uint16_t len) {
    ESP_LOGI(TAG, "Inside Settings");
    DEBUG_PRINT_HEX_DATA(data, len);

    uint16_t index = 0;
    uint8_t settingType = data[index++];

    if (settingType != SETTING_TYPE_GET_MAC_ADDR) {
        if (validateSecurityStatus(channel->otherConfig->index) != 0) {
            ESP_LOGW(TAG, "Security check failed");
            return;
        }
    }

    if (settingType == SETTING_TYPE_DEVICE_RESET)
    {
        ESP_LOGI(TAG, "Setting Device Reset");
        sendACK(channel->otherConfig->index);
        esp_restart();   
    }
    else if (settingType == SETTING_TYPE_SET_PROTOCOL) {
        ESP_LOGI(TAG, "Setting protocol version");
        uint8_t protocolType = data[index++];
        if ( protocolType < PROTOCOL_OUT_OF_RANGE_VALUE) {
            channel->fnConfig->updateProtocol(channel, protocolType);
            sendACK(channel->otherConfig->index);
        } else {
            ESP_LOGW(TAG, "Protocol not supported : %x", protocolType);
            sendNACK(channel->otherConfig->index, RESP_NACK_CODE_CMD_NOT_SUPPORTED);
        }
    } else if (settingType == SETTING_TYPE_SET_TX_ID) {
        
       if ((len - index) == 2) {
            UWord txid;
            txid.bytes.hbyte = data[index++]; 
            txid.bytes.lbyte = data[index++];
            channel->fnConfig->setTxId(channel, txid.word);
            sendACK(channel->otherConfig->index);
        } 
        
        else if ((len - index) == 4) {
            UDWord txid;
            txid.bytes.hwhbyte = data[index++];
            txid.bytes.hwlbyte = data[index++];
            txid.bytes.lwhbyte = data[index++];
            txid.bytes.lwlbyte = data[index++];
            channel->fnConfig->setTxId(channel, txid.dword);

            sendACK(channel->otherConfig->index);
        } else {
            ESP_LOGW(TAG, "Inalid tx id set length");
            sendNACK(channel->otherConfig->index, RESP_NACK_CODE_INVALID_FORMAT_OR_LENGTH);
        }
    } else if (settingType == SETTING_TYPE_SET_RX_ID) {
        if ((len - index) == 2) {
            UWord rxid;
            rxid.bytes.hbyte = data[index++]; 
            rxid.bytes.lbyte = data[index++];
            channel->fnConfig->setRxId(channel, rxid.word,0xFFFFFFFF);

            sendACK(channel->otherConfig->index);
        } else if ((len - index) == 4) {
            UDWord rxid;
            rxid.bytes.hwhbyte = data[index++];
            rxid.bytes.hwlbyte = data[index++];
            rxid.bytes.lwhbyte = data[index++];
            rxid.bytes.lwlbyte = data[index++];
            channel->fnConfig->setRxId(channel, rxid.dword,0xFFFFFFFF);

            sendACK(channel->otherConfig->index);
        }else if((len - index) == 8){
            UDWord rxmask;
            rxmask.bytes.hwhbyte = data[index++];
            rxmask.bytes.hwlbyte = data[index++];
            rxmask.bytes.lwhbyte = data[index++];
            rxmask.bytes.lwlbyte = data[index++];
                        
            UDWord rxid;
            rxid.bytes.hwhbyte = data[index++];
            rxid.bytes.hwlbyte = data[index++];
            rxid.bytes.lwhbyte = data[index++];
            rxid.bytes.lwlbyte = data[index++];
            
            channel->fnConfig->setRxId(channel, rxid.dword,rxmask.dword);
            sendACK(channel->otherConfig->index);

        } 
        else {
            ESP_LOGW(TAG, "Inalid rx id set length");
            sendNACK(channel->otherConfig->index, RESP_NACK_CODE_INVALID_FORMAT_OR_LENGTH);
        }
    } else if (settingType == SETTING_TYPE_START_PADDING) {
        if ((len - index) == 1) {
            uint8_t paddingByte = data[index++];
            channel->fnConfig->setPadding(channel, true, paddingByte);

            sendACK(channel->otherConfig->index);
        } else {
            ESP_LOGW(TAG, "Inalid Start Padding length");
            sendNACK(channel->otherConfig->index, RESP_NACK_CODE_INVALID_FORMAT_OR_LENGTH);
        }
    } else if (settingType == SETTING_TYPE_STOP_PADDING) {
        if ((len - index) == 0) {
            channel->fnConfig->setPadding(channel, false, 0x00);

            sendACK(channel->otherConfig->index);
        } else {
            ESP_LOGW(TAG, "Inalid stop padding length");
            sendNACK(channel->otherConfig->index, RESP_NACK_CODE_INVALID_FORMAT_OR_LENGTH);
        }
    } else if (settingType == SETTING_TYPE_GET_FIRMWARE_VERSION) {
        if ((len - index) == 0) {
            FWversion *fwVersion = settings_getFirmwareVersion();

            sendFirmwareVersion(channel->otherConfig->index, fwVersion);
        } else {
            ESP_LOGW(TAG, "Inalid getFW version length");
            sendNACK(channel->otherConfig->index, RESP_NACK_CODE_INVALID_FORMAT_OR_LENGTH);
        }
    } else if (settingType == SETTING_TYPE_SET_STA_SSID){
           //SetSSID(data, len);
            SetSSID(data,&data[2],len);
            sendACK(channel->otherConfig->index);


    } else if (settingType == SETTING_TYPE_SET_STA_PASSWORD) {
       
            //SetSSID(data, len);
            SetPSWD(data,&data[2],len);
            sendACK(channel->otherConfig->index);

    } else if (settingType == SETTING_TYPE_APP_REQ_CMD_START_OTA) {
        if (data[len - 1] != 0x00) {
            ESP_LOGW(TAG, "Invalid OTA URL termination character");
            sendNACK(channel->otherConfig->index, RESP_NACK_CODE_INVALID_FORMAT_OR_LENGTH);
        } else {
            if (beginOTA((const char *) &data[index], len - index) == 0) {
                sendACK(channel->otherConfig->index);
            } else {
                sendNACK(channel->otherConfig->index, RESP_NACK_CODE_INVALID_FORMAT_OR_LENGTH);
            }
        }

    } else if (settingType == SETTING_TYPE_SET_MAX_REQ_RESP_WAIT_TIME) {
        if ((len - index) == 2) {
            UWord time;
            time.bytes.hbyte = data[index++];
            time.bytes.lbyte = data[index++];
            settings_setMaxReqRespWaitTime(channel, time.word);

            sendACK(channel->otherConfig->index);
        } else {
            ESP_LOGW(TAG, "Inalid max_req_resp_wait_time length");
            sendNACK(channel->otherConfig->index, RESP_NACK_CODE_INVALID_FORMAT_OR_LENGTH);
        }

    } else if (settingType == SETTING_TYPE_START_PERIODIC_TESTER_PRESENT) {
        if ((len - index) == 0) {
            settings_setPeriodicTesterPresentStatus(channel, true);

            sendACK(channel->otherConfig->index);
        } else {
            ESP_LOGW(TAG, "Inalid start periodic tester event length");
            sendNACK(channel->otherConfig->index, RESP_NACK_CODE_INVALID_FORMAT_OR_LENGTH);
        }

    } else if (settingType == SETTING_TYPE_STOP_PERIODIC_TESTER_PRESENT) {
        if ((len - index) == 0) {
            settings_setPeriodicTesterPresentStatus(channel, false);

            sendACK(channel->otherConfig->index);
        } else {
            ESP_LOGW(TAG, "Inalid stop periodic tester event length");
            sendNACK(channel->otherConfig->index, RESP_NACK_CODE_INVALID_FORMAT_OR_LENGTH);
        }

    } else if (settingType == SETTING_TYPE_GET_LATEST_IVN_FRAME) {
        UDWord rxid;
        bool isValidRxId = false;
        if ((len - index) == 2) {
            rxid.bytes.lwhbyte = data[index++]; 
            rxid.bytes.lwlbyte = data[index++];
            isValidRxId = true;
        } else if ((len - index) == 4) {
            rxid.bytes.hwhbyte = data[index++];
            rxid.bytes.hwlbyte = data[index++];
            rxid.bytes.lwhbyte = data[index++];
            rxid.bytes.lwlbyte = data[index++];
            isValidRxId = true;
        } else {
            ESP_LOGW(TAG, "Inalid rx id ivn cmd length");
            sendNACK(channel->otherConfig->index, RESP_NACK_CODE_INVALID_FORMAT_OR_LENGTH);
        }
        if (isValidRxId) {
            ESP_LOGI(TAG, "IVN RXID = %" PRIu32, rxid.dword);
            ISOTPFrame frame;
            int resp = channel->fnConfig->getLatestIVNFrame(channel, rxid.dword, &frame);
            if ( resp != 0 ) {
                sendNACK(channel->otherConfig->index, RESP_NACK_CODE_CMD_NOT_SUPPORTED);
            } else {
                sendIVNFrameResponse(channel->otherConfig->index, &frame, RESP_IVN_FRAME_NIBBLE_AS_DATA);
            }
        }

    } else if (settingType == SETTING_TYPE_GET_MAC_ADDR) {
        if ((len - index) == 0) {
            sendMacAddr(channel->otherConfig->index, mac);
        } else {
            ESP_LOGW(TAG, "Inalid getFW version length");
            sendNACK(channel->otherConfig->index, RESP_NACK_CODE_INVALID_FORMAT_OR_LENGTH);
        }
    } else if (settingType == SETTING_TYPE_SET_CONSECUTIVE_TIME) {
        if ((len - index) == 1) {
            uint8_t consecutiveTime = data[index++];
            settings_setConsectiveTime(channel, consecutiveTime);

            sendACK(channel->otherConfig->index);
        } else {
            ESP_LOGW(TAG, "Inalid set consecutive_time length");
            sendNACK(channel->otherConfig->index, RESP_NACK_CODE_INVALID_FORMAT_OR_LENGTH);
        }
    } else if (settingType == SETTING_TYPE_GET_CONSECUTIVE_TIME) {
        if ((len - index) == 0) {
            uint8_t consecutiveTime = channel->config->forceSTMinTx;
            sendConsecutiveTime(channel->otherConfig->index, consecutiveTime);
        } else {
            ESP_LOGW(TAG, "Inalid get consecutive_time length");
            sendNACK(channel->otherConfig->index, RESP_NACK_CODE_INVALID_FORMAT_OR_LENGTH);
        }
    } else {
        ESP_LOGW(TAG, "Setting command not supported : %x", settingType);
        sendNACK(channel->otherConfig->index, RESP_NACK_CODE_CMD_NOT_SUPPORTED);
    }
}

void inst_sendDataToCANProcess(ChannelAllConfig *channel, uint8_t *data, uint16_t len) {
    if (validateSecurityStatus(channel->otherConfig->index) != 0) {
        if (data != NULL) {
            vPortFree(data);
            data = NULL;
        }
        return;
    }

    int isoTpRet = isotp_sendDataToQueue(channel, data, len);
    if (isoTpRet == 0) {
        ESP_LOGD(TAG, "Successful ISOTP Queued");
        sendACK(channel->otherConfig->index);
    } else {
        ESP_LOGW(TAG, "Failed to send ISOTP frame to queue");
        sendNACK(channel->otherConfig->index, RESP_NACK_CODE_INVALID_FORMAT_OR_LENGTH);
    }
}

void inst_handleIVNCMD(ChannelAllConfig *channel, uint8_t *data, uint16_t len) {
    ESP_LOGI(TAG, "Inside Handle IVN CMD");
    DEBUG_PRINT_HEX_DATA(data, len);

    if (validateSecurityStatus(channel->otherConfig->index) != 0) {
        ESP_LOGW(TAG, "Security check failed");
        return;
    }

    uint16_t index = 0;
    uint8_t settingType = data[index++];

    if (settingType == IVN_CMD_TYPE_GET_LATEST_IVN_FRAME) {
        UDWord rxid;
        bool isValidRxId = false;
        if ((len - index) == 2) {
            rxid.bytes.lwhbyte = data[index++]; 
            rxid.bytes.lwlbyte = data[index++];
            isValidRxId = true;
        } else if ((len - index) == 4) {
            rxid.bytes.hwhbyte = data[index++];
            rxid.bytes.hwlbyte = data[index++];
            rxid.bytes.lwhbyte = data[index++];
            rxid.bytes.lwlbyte = data[index++];
            isValidRxId = true;
        } else {
            ESP_LOGW(TAG, "Inalid rx id ivn cmd length");
            sendNACK(channel->otherConfig->index, RESP_NACK_CODE_INVALID_FORMAT_OR_LENGTH);
        }
        if (isValidRxId) {
            ESP_LOGI(TAG, "IVN RXID = %" PRIu32, rxid.dword);
            ISOTPFrame frame;
            int resp = channel->fnConfig->getLatestIVNFrame(channel, rxid.dword, &frame);
            if ( resp != 0 ) {
                sendNACK(channel->otherConfig->index, RESP_NACK_CODE_CMD_NOT_SUPPORTED);
            } else {
                sendIVNFrameResponse(channel->otherConfig->index, &frame, RESP_IVN_FRAME_NIBBLE);
            }
        }
    } else {
        ESP_LOGW(TAG, "IVN command not supported : %x", settingType);
        sendNACK(channel->otherConfig->index, RESP_NACK_CODE_CMD_NOT_SUPPORTED);
    }
}

void noresponsefromecu(uint8_t channelNum){
uint8_t *dataPtr = pvPortMalloc(CP_RESP_ACK_LEN);
    uint8_t index = 0;
    if (dataPtr != NULL) {
        //UWord uword = {.word = RESP_PKT_LEN_ACK};
        //uword.bytes.hbyte |= NIBBLE_VALUE_TO_HIGHER_NIBBLE(RESP_START_NIBBLE);

        dataPtr[index++] = 0x40;
        dataPtr[index++] = 0x00;
#ifdef CHANNEL_SELECTION_IN_CUSTOM_PACKET
        dataPtr[index++] = channelNum;
#endif
       // uint8_t crcDataStartLoc = index;

        //dataPtr[index++] = RESP_PKT_ACK_BYTE;
        
        dataPtr[index++] = 0xff;
        dataPtr[index++] = 0xff;
        
        if(Activechannel==USB_CH){
        senddatatouart(dataPtr, index);
        }
        else if(Activechannel==TCPSCKT){
            sock_queue_dataPointerWithLen(dataPtr, index);
        }
        else if(Activechannel==BT_CH){
            BTWrite(dataPtr, index);
        } 

    }

}    