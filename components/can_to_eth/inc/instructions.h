#ifndef __MAIN_INC_INSTRUCTIONS_H__
#define __MAIN_INC_INSTRUCTIONS_H__

#include <stdint.h>
#include "instruction_desc.h"
#include "channel.h"
#include "dev_settings.h"

#define CHANNEL_SELECTION_IN_CUSTOM_PACKET


#define NIBBLE_VALUE_TO_HIGHER_NIBBLE(n) (n << 4)

#define NIBBLE_VALUE_TO_HIGHER_NIBBLE(n) (n << 4)
#define INVALID_CH 0x00
#define TCPSCKT 0x01
#define USB_CH 0x02
#define BT_CH 0x03


extern uint8_t Activechannel;

void inst_checkSecurityCode(uint8_t cannelNum, uint8_t *data, uint16_t len);

void inst_settings(ChannelAllConfig *channel, uint8_t *data, uint16_t len);

void inst_sendDataToCANProcess(ChannelAllConfig *channel, uint8_t *data, uint16_t len);

void inst_handleIVNCMD(ChannelAllConfig *channel, uint8_t *data, uint16_t len);

void sendFirmwareVersion(uint8_t channelNum, FWversion *version);
void sendACK(uint8_t channelNum);
void sendNACK(uint8_t channelNum, uint8_t nackType);
void noresponsefromecu(uint8_t channelNum);


#endif // __MAIN_INC_INSTRUCTIONS_H__