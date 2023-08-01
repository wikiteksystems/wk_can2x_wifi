#include "dev_settings.h"

#include "esp_timer.h"
#include "channel.h"

#define DEFAULT_DEV_SETTING_MAX_REQ_RESP_WAIT_TIME 0x0000
#define DEFAULT_DEV_SETTING_SECURITY_CHECK_STATUS false
#define DEFAULT_DEV_SETTING_DEVICE_TESTER_PRESENT_STATUS false
#define DEFAULT_DEV_SETTING_DEVICE_TESTER_PRESENT_RESPONSE_EXPECT_STATUS false
#define INIT_DEV_SETTING_LAST_REQUEST_SENT_TO_ECU_TIME 0

DeviceSettings devSettings = {
                .status.isSecurityCheckPassed = DEFAULT_DEV_SETTING_SECURITY_CHECK_STATUS
            };

FWversion fwVersion = {
                .build = FIRMWARE_BUILD_VERSION,
                .major = FIRMWARE_MAJOR_VERSION,
                .minor = FIRMWARE_MINOR_VERSION
            };

FWversion* settings_getFirmwareVersion(void) {
    return &fwVersion;
}


