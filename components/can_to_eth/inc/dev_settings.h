#ifndef __MAIN_INC_DEV_SETTINGS_H__
#define __MAIN_INC_DEV_SETTINGS_H__


#include <stdint.h>
#include <stdbool.h>

#include "channel.h"

#define FIRMWARE_BUILD_VERSION 4
#define FIRMWARE_MAJOR_VERSION 0
#define FIRMWARE_MINOR_VERSION 5

typedef struct {
    struct {
        bool isSecurityCheckPassed;
    } status;
} DeviceSettings;

extern DeviceSettings devSettings;

typedef struct {
    uint8_t build;
    uint8_t major;
    uint8_t minor;
} FWversion;

FWversion* settings_getFirmwareVersion(void);


#endif // __MAIN_INC_DEV_SETTINGS_H__