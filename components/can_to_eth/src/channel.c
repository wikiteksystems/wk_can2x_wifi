#include "channel.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gptimer.h"

static const char * TAG = "channel";

#define DEFAULT_CHANNEL_CONFIG \
    {\
        .protocol = DEFAULT_CAN_PROTOCOL,\
        .id_type = DEFAULT_CAN_ID_TYPE,\
        .rx_id = DEFAULT_CAN_RX_ID,\
        .rx_mask = DEFAULT_CAN_RX_MASK,\
        .tx_id = DEFAULT_CAN_TX_ID,\
        .paddingEnabled = DEFAULT_CAN_PADDING_ENABLED_STATUS,\
        .paddingByte = DEFAULT_CAN_PADDING_BYTE,\
        .maxMicrosBetweenReqResp = DEFAULT_CAN_MAX_MICROS_BW_REQ_RESP,\
        .periodicTesterPresentEnabled = DEFAULT_CAN_PTP_ENABLED_STATUS,\
        .forceSTMinTx = DEFAULT_CAN_FORCE_STMIN_TX\
    }

#define DEFAULT_OTHER_CHANNEL_CONFIG(chIndex) \
    {\
        .canbusUseMutex = NULL,\
        .xISOTP_TaskHandle = NULL,\
        .xISOTP_socketToCANQueueHandle = NULL,\
        .xISOTP_receivedDataQueueHandle = NULL,\
        .index = chIndex,\
        .xISOTP_taskEventHandle = NULL,\
        \
        .xISOTP_flowPacketQueueHandle = NULL,\
        .xISOTP_dtpEventHandle = NULL,\
        \
        .lastDeviceResponseTime = 0,\
        .isDeviceTesterPresentResponseExpected = false,\
        \
        .isNormalResonseExpected = false,\
        \
        .rxHolder = {.data = NULL, .dataLen = 0, .totalLen = 0 },\
        \
        .xWaitReqRespTimeoutQueueHandle = NULL,\
        .xWaitReqRespTimerHandle = NULL,\
        \
        .xConsecutiveTimeoutQueueHandle = NULL,\
        .xConsecutiveTimerHandle = NULL\
    }


ChannelConfig channels[2] = {
    DEFAULT_CHANNEL_CONFIG, DEFAULT_CHANNEL_CONFIG
};

ChannelFnConfig channelFns[2] = {{}, {}};
ChannelOtherConfig channelOthers[2] = {
    DEFAULT_OTHER_CHANNEL_CONFIG(0), DEFAULT_OTHER_CHANNEL_CONFIG(1)
};

ChannelAllConfig channelAll[2] = {
    {
        .config = &channels[0], .fnConfig = &channelFns[0], .otherConfig = &channelOthers[0]
    },
    {
        .config = &channels[1], .fnConfig = &channelFns[1], .otherConfig = &channelOthers[1]
    }
};

static bool IRAM_ATTR timer_waitReqResp_isr_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *arg)
{
    BaseType_t high_task_awoken = pdFALSE;

    ChannelAllConfig *channel = (ChannelAllConfig *) arg;
    ESP_ERROR_CHECK(gptimer_stop(timer));
    ESP_ERROR_CHECK(gptimer_set_raw_count(timer, (5 * 1000 * 1000)));
    /* Now just send the event data back to the main program task */
    xQueueSendFromISR(channel->otherConfig->xWaitReqRespTimeoutQueueHandle, &edata->count_value, &high_task_awoken);

    return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR
}

static bool IRAM_ATTR timer_consecutiveTime_isr_callback(gptimer_handle_t timer,
                                                        const gptimer_alarm_event_data_t *edata,
                                                        void *arg)
{
    BaseType_t high_task_awoken = pdFALSE;

    ChannelAllConfig *channel = (ChannelAllConfig *) arg;
    ESP_ERROR_CHECK(gptimer_stop(timer));
    ESP_ERROR_CHECK(gptimer_set_raw_count(timer, 0));
    /* Now just send the event data back to the main program task */
    xQueueSendFromISR(channel->otherConfig->xConsecutiveTimeoutQueueHandle, &edata->count_value, &high_task_awoken);

    return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR
}

void settings_setMaxReqRespWaitTime(ChannelAllConfig *channel, uint16_t time) {
    channel->config->maxMicrosBetweenReqResp = time;

    if (time == 0x0000) {
        if (channel->otherConfig->xWaitReqRespTimeoutQueueHandle != NULL) {
            vQueueDelete(channel->otherConfig->xWaitReqRespTimeoutQueueHandle);
            channel->otherConfig->xWaitReqRespTimeoutQueueHandle = NULL;
        }
        if (channel->otherConfig->xWaitReqRespTimerHandle != NULL) {
            gptimer_stop(channel->otherConfig->xWaitReqRespTimerHandle);
            gptimer_disable(channel->otherConfig->xWaitReqRespTimerHandle);
            gptimer_del_timer(channel->otherConfig->xWaitReqRespTimerHandle);
            channel->otherConfig->xWaitReqRespTimerHandle = NULL;
        }
        return;
    }

    if (channel->otherConfig->xWaitReqRespTimeoutQueueHandle == NULL) {
        channel->otherConfig->xWaitReqRespTimeoutQueueHandle = xQueueCreate(10, sizeof(uint64_t));
    }

    if (channel->otherConfig->xWaitReqRespTimerHandle == NULL) {
        gptimer_config_t timer_config = {
            .clk_src = GPTIMER_CLK_SRC_DEFAULT,
            .direction = GPTIMER_COUNT_UP,
            .resolution_hz = 1 * 1000 * 1000, // * 1000, // 1MHz, 1 tick=1us
        };
        ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &channel->otherConfig->xWaitReqRespTimerHandle));

        gptimer_event_callbacks_t cbs = {
            .on_alarm = timer_waitReqResp_isr_callback,
        };
        ESP_ERROR_CHECK(gptimer_register_event_callbacks(channel->otherConfig->xWaitReqRespTimerHandle, &cbs, channel));

        ESP_LOGI(TAG, "Enable timer");
        ESP_ERROR_CHECK(gptimer_enable(channel->otherConfig->xWaitReqRespTimerHandle));
    } else {
        gptimer_stop(channel->otherConfig->xWaitReqRespTimerHandle);
    }

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = (5 * 1000 * 1000) + (channel->config->maxMicrosBetweenReqResp * 1000),
        .flags.auto_reload_on_alarm = 0
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(channel->otherConfig->xWaitReqRespTimerHandle, &alarm_config));
    ESP_ERROR_CHECK(gptimer_set_raw_count(channel->otherConfig->xWaitReqRespTimerHandle, (5 * 1000 * 1000)));
    
    ESP_LOGI(TAG, "timer setup complete");
    // ESP_ERROR_CHECK(gptimer_start(channel->otherConfig->xWaitReqRespTimerHandle));
}

/**
 * @brief Set the tings setPeriodicTesterPresentStatus object
 * 
 * @param channel 
 * @param status true = START, false = STOP
 */
int settings_setPeriodicTesterPresentStatus(ChannelAllConfig *channel, bool status) {
    if (status && !channel->config->periodicTesterPresentEnabled) {
        channel->config->periodicTesterPresentEnabled = status;

        if (channel->otherConfig->xISOTP_dtpEventHandle == NULL) {
            channel->otherConfig->xISOTP_dtpEventHandle = xEventGroupCreate();
           // ESP_LOGI(TAG, "DTP event handler created");
        } else {
           // ESP_LOGW(TAG, "DTP event handle should have been NULL");
        }
        channel->otherConfig->lastDeviceResponseTime = esp_timer_get_time();
        //ESP_LOGI(TAG, "DPTP enabled");
    } else if (!status && channel->config->periodicTesterPresentEnabled) {
        channel->config->periodicTesterPresentEnabled = status;

        if (channel->otherConfig->xISOTP_dtpEventHandle != NULL) {
            vEventGroupDelete(channel->otherConfig->xISOTP_dtpEventHandle);
            channel->otherConfig->xISOTP_dtpEventHandle = NULL;
         //   ESP_LOGI(TAG, "DTP event handler stoped");
        } else {
          //  ESP_LOGW(TAG, "DTP event handle should not have been NULL");
        }
        channel->otherConfig->isDeviceTesterPresentResponseExpected = false;
       // ESP_LOGI(TAG, "DPTP disabled");
    } else {
      //  ESP_LOGI(TAG, "DTP already in same state");
    }
    return 0;
}

void generate_consecutiveAlarmConfig(uint8_t consecTime, gptimer_alarm_config_t *alarmConfig) {
    memset(alarmConfig, 0, sizeof(gptimer_alarm_config_t));
    alarmConfig->flags.auto_reload_on_alarm = 0;
    if (consecTime <= 127) {
        alarmConfig->alarm_count = consecTime * 1000;
    } else {
        if ((consecTime & 0xF0) == 0xF0) {
            uint8_t lowerNibble = consecTime & 0x0F;
            if (lowerNibble >= 1 && lowerNibble <= 9) {
                alarmConfig->alarm_count = lowerNibble * 100;
            }
        }
    }
}

void settings_setConsectiveTime(ChannelAllConfig *channel, uint8_t consecTime) {
    gptimer_alarm_config_t alarm_config;
    generate_consecutiveAlarmConfig(consecTime, &alarm_config);
    if (alarm_config.alarm_count == 0) {
        channel->config->forceSTMinTx = 0;
    } else {
        channel->config->forceSTMinTx = consecTime;
    }
    if (channel->config->forceSTMinTx == 0xFF) {
        if (channel->otherConfig->xConsecutiveTimeoutQueueHandle != NULL) {
            vQueueDelete(channel->otherConfig->xConsecutiveTimeoutQueueHandle);
            channel->otherConfig->xConsecutiveTimeoutQueueHandle = NULL;
        }
        if (channel->otherConfig->xConsecutiveTimerHandle != NULL) {
            gptimer_stop(channel->otherConfig->xConsecutiveTimerHandle);
            gptimer_disable(channel->otherConfig->xConsecutiveTimerHandle);
            gptimer_del_timer(channel->otherConfig->xConsecutiveTimerHandle);
            channel->otherConfig->xConsecutiveTimerHandle = NULL;
        }
        return;
    }

    if (channel->otherConfig->xConsecutiveTimeoutQueueHandle == NULL) {
        channel->otherConfig->xConsecutiveTimeoutQueueHandle = xQueueCreate(10, sizeof(uint64_t));
    }
    
    if (channel->otherConfig->xConsecutiveTimerHandle == NULL) {
        gptimer_config_t timer_config = {
            .clk_src = GPTIMER_CLK_SRC_DEFAULT,
            .direction = GPTIMER_COUNT_UP,
            .resolution_hz = 1 * 1000 * 1000, // * 1000, // 1MHz, 1 tick=1us
        };
        ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &channel->otherConfig->xConsecutiveTimerHandle));

        gptimer_event_callbacks_t cbs = {
            .on_alarm = timer_consecutiveTime_isr_callback,
        };
        ESP_ERROR_CHECK(gptimer_register_event_callbacks(channel->otherConfig->xConsecutiveTimerHandle, &cbs, channel));

        ESP_LOGI(TAG, "Enable timer");
        ESP_ERROR_CHECK(gptimer_enable(channel->otherConfig->xConsecutiveTimerHandle));
    } else {
        gptimer_stop(channel->otherConfig->xConsecutiveTimerHandle);
    }

    ESP_ERROR_CHECK(gptimer_set_alarm_action(channel->otherConfig->xConsecutiveTimerHandle, &alarm_config));
    ESP_ERROR_CHECK(gptimer_set_raw_count(channel->otherConfig->xConsecutiveTimerHandle, 0));
    
    ESP_LOGI(TAG, "timer setup complete");
}