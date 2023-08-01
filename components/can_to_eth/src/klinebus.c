#include "klinebus.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

#include "sock_service.h"
#include "utility.h"
#include "crc.h"
#include "usb_service.h"
#include "dev_settings.h"
#include "freertos/semphr.h"
#include "instructions.h"
#include "wifi_service.h"

#include <driver/dac.h>

static const char *TAG = "KL";
//--------------
#define KL_BUF_SIZE (600)

#define KL_RD_BUF_SIZE (KL_BUF_SIZE)
static QueueHandle_t kline0_queue;
static QueueHandle_t kline1_queue;
//-----------
#define ACK_LEN 6
#define NACK_LEN 7

#define K_NACK_CODE_CMD_NOT_SUPPORTED 0x10

#define FORMATBYTESIZE 1
#define ADDBYTESIZE 2
#define CKSBYTESIZE 1

//-------------KL0
static uint8_t formatbyte0 , targetbyte0 , sourcebyte0 ;

static uint8_t formatbyte1 , targetbyte1 , sourcebyte1;

// tester present
#define mainAUTO_RELOAD_TIMER_PERIOD pdMS_TO_TICKS(1000)

uint16_t Kline0BusinitTime = 4500;
uint16_t Kline1BusinitTime = 5000;
uint8_t kl0IFtime = 0, kl1IFtime = 0;

TimerHandle_t xKl0TestTimer, xKl1TestTimer, xkl0busintTimer, xkl1busintTimer;
BaseType_t xKl0TestStarted, xKl1TestStarted, xKl0busintstarted, xKl1busintstarted;

bool kl0busint = false, kl1busint = false;

//-------------for task
SemaphoreHandle_t xkl0mutex = NULL, xkl1mutex = NULL;
TaskHandle_t xkl0rectaskhandle;
TaskHandle_t xkl1rectaskhandle;

uint8_t iso_checksum(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++)
        crc = crc + data[i];
    return crc;
}

static void dtpkl0(TimerHandle_t xTimer)
{
    // TickType_t xTimeNow;
    // xTimeNow = xTaskGetTickCount();
    ESP_LOGI(TAG, "DTP");
    uint8_t *dataPtr = pvPortMalloc(6);
    int index = 0;
    dataPtr[index++] = formatbyte0 | 0b1;
    dataPtr[index++] = targetbyte0;
    dataPtr[index++] = sourcebyte0;
    dataPtr[index++] = 0x3e;
    dataPtr[index++] = iso_checksum(dataPtr, 4);
    uart_write_bytes(KL0, (const char *)dataPtr, index);
    vPortFree(dataPtr);
    dataPtr = NULL;
}

static void dtpkl1(TimerHandle_t xTimer)
{
    // TickType_t xTimeNow;
    // xTimeNow = xTaskGetTickCount();
    ESP_LOGI(TAG, "DTP");
    uint8_t *dataPtr = pvPortMalloc(6);
    int index = 0;
    dataPtr[index++] = formatbyte0 | 0b1;
    dataPtr[index++] = targetbyte0;
    dataPtr[index++] = sourcebyte0;
    dataPtr[index++] = 0x3e;
    dataPtr[index++] = iso_checksum(dataPtr, 4);
    uart_write_bytes(KL0, (const char *)dataPtr, index);
    vPortFree(dataPtr);
    dataPtr = NULL;
}

static void buskl0businint(TimerHandle_t xTimer)
{

    // ESP_LOGI(TAG, "BUSINT");
    // if( xTimerIsTimerActive( xkl0busintTimer ) != pdFALSE ) // or more simply and equivalently "if( xTimerIsTimerActive( xTimer ) )"
    //  {
    //     kl0busint = false;
    //      // xTimer is active, do something.
    //  }
    // xTimerReset(xkl0busintTimer, 10);
    kl0busint = false;
}

static void buskl1businint(TimerHandle_t xTimer)
{

    // ESP_LOGI(TAG, "BUSINT1");
    kl1busint = false;
    // xTimerReset(xkl0busintTimer, 10);
}


void kline0fastinit()
{

    if (pdTRUE == xSemaphoreTake(xkl0mutex, pdMS_TO_TICKS(2000)))
    {
      if (uart_is_driver_installed(KL0))
        {
            uart_driver_delete(KL0);
        }
        
        ESP_LOGI(TAG, "Uart Driver delted");

        gpio_set_direction(KL0RXD_PIN, GPIO_MODE_INPUT);
        gpio_pullup_en(KL0RXD_PIN);
        // gpio_set_level(KL0RXD_PIN, PinHigh);

        gpio_set_direction(KL0TXD_PIN, GPIO_MODE_OUTPUT);
        gpio_set_level(KL0TXD_PIN, PinHigh);

        vTaskDelay(KL_INIT_IDLE_BUS_BEFORE / portTICK_PERIOD_MS);

        gpio_set_level(KL0TXD_PIN, PinLow);
       // vTaskDelay(24 / portTICK_PERIOD_MS);
        
        vTaskDelay(pdMS_TO_TICKS(24));
        
        gpio_set_level(KL0TXD_PIN, PinHigh);
        //vTaskDelay(24 / portTICK_PERIOD_MS);
       
        vTaskDelay(pdMS_TO_TICKS(24));


        uart_config_t kline0_config = {
            .baud_rate = KL0_BAUD,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };

        uart_driver_install(KL0, KL_BUF_SIZE, KL_BUF_SIZE, 20, &kline0_queue, 0);
        uart_param_config(KL0, &kline0_config);
        esp_log_level_set(TAG, ESP_LOG_INFO);
        uart_set_pin(KL0, KL0TXD_PIN, KL0RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
       
        klstcframe(0x02);
        kl0busint = true;
        xSemaphoreGive(xkl0mutex);
    }

    
}

void kline1fastinit()
{
    if (pdTRUE == xSemaphoreTake(xkl1mutex, pdMS_TO_TICKS(2000)))
    {
        if (uart_is_driver_installed(KL1))
        {
            uart_driver_delete(KL1);
        }

        gpio_set_direction(KL1RXD_PIN, GPIO_MODE_INPUT);
        gpio_pullup_en(KL1RXD_PIN);

        //gpio_set_level(KL1RXD_PIN, PinHigh);

        gpio_set_direction(KL1TXD_PIN, GPIO_MODE_OUTPUT);
        // gpio_pullup_en(KL1TXD_PIN);

        gpio_set_level(KL1TXD_PIN, PinHigh);

        vTaskDelay(KL_INIT_IDLE_BUS_BEFORE / portTICK_PERIOD_MS);
        gpio_set_level(KL1TXD_PIN, PinLow);

        // vTaskDelay(24 / portTICK_PERIOD_MS);

        vTaskDelay(pdMS_TO_TICKS(KLine_bus_time_delay));
        gpio_set_level(KL1TXD_PIN, PinHigh);

        // vTaskDelay(24 / portTICK_PERIOD_MS);
         vTaskDelay(pdMS_TO_TICKS(KLine_bus_time_delay));
        uart_config_t kline1_config = {
            .baud_rate = KL1_BAUD,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };

        uart_driver_install(KL1, KL_BUF_SIZE, KL_BUF_SIZE, 20, &kline1_queue, 0);
        uart_param_config(KL1, &kline1_config);
        esp_log_level_set(TAG, ESP_LOG_INFO);
        uart_set_pin(KL1, KL1TXD_PIN, KL1RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        klstcframe(0x03);
        kl1busint = true;
        xSemaphoreGive(xkl1mutex);
    }
}

static void kl0_receive_task(void *pvParameters)
{
    uart_config_t kline0_config = {
        .baud_rate = KL0_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_driver_install(KL0, KL_BUF_SIZE, KL_BUF_SIZE, 20, &kline0_queue, 0);
    uart_param_config(KL0, &kline0_config);
    esp_log_level_set(TAG, ESP_LOG_INFO);
    uart_set_pin(KL0, KL0TXD_PIN, KL0RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uint8_t *data = pvPortMalloc(KL_RD_BUF_SIZE);

    while (1)
    {

        if (pdTRUE == xSemaphoreTake(xkl0mutex, pdMS_TO_TICKS(3000)))
        {

            int len = uart_read_bytes(KL0, data, (KL_BUF_SIZE - 1), 200 / portTICK_PERIOD_MS);
            // Write data back to the UART
            int index = 0, rindex = 0;
            uint8_t datalen = 0, responselen = 0;
            // uart_write_bytes(UART, (const char *) k0data, len);

            if (len)
            {
                data[len] = '\0';
                datalen = data[index++] & 0b111111;
                // ESP_LOGI(TAG, "Total received length is: %d", len);
                // ESP_LOGI(TAG, "received frame len %d ", datalen);
                targetbyte0 = data[index++];
                sourcebyte0 = data[index++];

                if (datalen == 0x00)
                {
                    datalen = data[index++];
                }

                index = index + datalen;
                index++; // cks

                if (len > index)
                {

                    responselen = data[index++] & 0b111111;
                    ESP_LOGI(TAG, "length of response: %d", responselen);
                    sourcebyte0 = data[index++];
                    targetbyte0 = data[index++];

                    if (responselen == 0x00)
                    {
                        responselen = data[index++];
                    }
                    uint8_t checkbyte = data[index++];

                    if (checkbyte != 0x7e && checkbyte != 0xc1)
                    {
                        ESP_LOGI(TAG, "without : %d", len);
                        index--;
                        uint8_t *rdata = pvPortMalloc(KL_RD_BUF_SIZE);
                        rdata[rindex++] = 0x40;
                        rdata[rindex++] = responselen;
                        rdata[rindex++] = 02;
                        memcpy(&rdata[3], &data[index++], responselen);
                        UWord crc;
                        crc.word = CRC16_CCITT(&rdata[3], responselen);
                        rindex = rindex + responselen;
                        rdata[rindex++] = crc.bytes.hbyte;
                        rdata[rindex++] = crc.bytes.lbyte;
                        if (Activechannel == USB_CH)
                        {
                            senddatatouart(rdata, rindex);
                        }
                        else if (Activechannel == TCPSCKT)
                        {
                            sock_queue_dataPointerWithLen(rdata, rindex);
                        }
                        // sock_queue_dataPointerWithLen(rdata, rindex);
                        // ESP_LOGI(TAG, "data send from k line to socekt");
                    }
                    data[len] = '\0';
                }
                else if (len == index)
                {
                    noresponsefromecu(0x02);
                    ESP_LOGI(TAG, "No response from ECU");
                }
                len = 0;
                if (xTimerIsTimerActive(xKl0TestTimer))
                {
                    xTimerStart(xKl0TestTimer, 0);
                }
            }

            xSemaphoreGive(xkl0mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    
       
    }

    vTaskDelete(NULL);
}

static void kl1_receive_task(void *pvParameters)
{
    uart_config_t kline1_config = {
        .baud_rate = KL1_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
   // gpio_set_level(KL1TXD_PIN, PinHigh);
    //gpio_pullup_en(KL1TXD_PIN);
    //gpio_pullup_en(KL1RXD_PIN);
    uart_driver_install(KL1, KL_BUF_SIZE, KL_BUF_SIZE, 20, &kline1_queue, 0);
    uart_param_config(KL1, &kline1_config);
    esp_log_level_set(TAG, ESP_LOG_INFO);
    uart_set_pin(KL1, KL1TXD_PIN, KL1RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uint8_t *data = pvPortMalloc(KL_RD_BUF_SIZE);
    while (1)
    {

        if (pdTRUE == xSemaphoreTake(xkl1mutex, pdMS_TO_TICKS(3000)))
        {
            int len = uart_read_bytes(KL1, data, (KL_BUF_SIZE - 1), 200 / portTICK_PERIOD_MS);
            int index = 0, rindex = 0;
            uint8_t datalen = 0, responselen = 0;
            if (len)
            {
                data[len] = '\0';
                datalen = data[index++] & 0b111111;
                // ESP_LOGI(TAG, "KL1 received length is: %d", len);
                // ESP_LOGI(TAG, "KL1 frame len %d ", datalen);
                targetbyte0 = data[index++];
                sourcebyte0 = data[index++];

                if (datalen == 0x00) {
                    datalen = data[index++];
                }

                index = index + datalen;
                index++; // cks

                if (len > index) {
                    responselen = data[index++] & 0b111111;
                    // ESP_LOGI(TAG, "length of response: %d", responselen);
                    sourcebyte0 = data[index++];
                    targetbyte0 = data[index++];

                    if (responselen == 0x00)
                    {
                        responselen = data[index++];
                    }
                    uint8_t checkbyte = data[index++];
                   if (checkbyte != 0x7e && checkbyte != 0xc1)
                    {
                        ESP_LOGI(TAG, "7e : %d", len);
                        index--;
                        uint8_t *rdata = pvPortMalloc(KL_RD_BUF_SIZE);
                        rdata[rindex++] = 0x40;
                        rdata[rindex++] = responselen;
                        rdata[rindex++] = 03;
                        memcpy(&rdata[3], &data[index++], responselen);
                        UWord crc;
                        crc.word = CRC16_CCITT(&rdata[3], responselen);
                        rindex = rindex + responselen;
                        rdata[rindex++] = crc.bytes.hbyte;
                        rdata[rindex++] = crc.bytes.lbyte;
                        // sock_queue_dataPointerWithLen(rdata, rindex);
                        if (Activechannel == USB_CH)
                        {
                            senddatatouart(rdata, rindex);
                        }
                        else if (Activechannel == TCPSCKT)
                        {
                            sock_queue_dataPointerWithLen(rdata, rindex);
                        }
                        // ESP_LOGI(TAG, "data send from k line to socekt");
                    }

                    data[len] = '\0';
                }

                else if (len == index)
                {
                    noresponsefromecu(0x03);
                    ESP_LOGI(TAG, "No response from ECU");
                }

                len = 0;
                if (xTimerIsTimerActive(xKl1TestTimer))
                {
                    xTimerStart(xkl1busintTimer, 0);
                }
            }

            xSemaphoreGive(xkl1mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
    vTaskDelete(NULL);
}

void initialise_kl0(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);

    xkl0mutex = xSemaphoreCreateMutex();

    // kline0fastinit();
    xTaskCreate(kl0_receive_task, "kl0_receive_task", 3048, NULL, tskIDLE_PRIORITY + 3, &xkl0rectaskhandle);
    xKl0TestTimer = xTimerCreate("Testerpresent", mainAUTO_RELOAD_TIMER_PERIOD, pdTRUE, 0, dtpkl0);
    xkl0busintTimer = xTimerCreate("Testerpresent", pdMS_TO_TICKS(Kline0BusinitTime), pdTRUE, 0, buskl0businint);
    // xTimerStart(xkl0busintTimer, 0);
}

void initialise_kl1(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);
    xkl1mutex = xSemaphoreCreateMutex();
    //kline1fastinit();
    xTaskCreate(kl1_receive_task, "kl1_receive_task", 2048, NULL, tskIDLE_PRIORITY + 3, &xkl1rectaskhandle);
    xKl1TestTimer = xTimerCreate("Testerpresent", mainAUTO_RELOAD_TIMER_PERIOD, pdTRUE, 0, dtpkl1);
    xkl1busintTimer = xTimerCreate("Testerpresent", pdMS_TO_TICKS(Kline1BusinitTime), pdTRUE, 0, buskl1businint);
    // xTimerStart(xkl1busintTimer, 0);
}

void inst_kl_settings(uint8_t channel, uint8_t *data, uint16_t len)
{

    ESP_LOGI(TAG, "Inside Settings");
    // DEBUG_PRINT_HEX_DATA(data, len);

    uint16_t index = 0;
    uint8_t settingType = data[index++];
    if (settingType == SETTING_TYPE_DEVICE_RESET)
    {
        ESP_LOGI(TAG, "Setting reset in kline");
        sendACK(channel);
        esp_restart();
    }
    
    else if (settingType == SETTING_TYPE_SET_PROTOCOL)
    {
        uint8_t protocolType = data[index++];

        if (protocolType == KLINE_FAST_Init_80)
        {
            if (channel == 0x02)
            {
                //initialise_kl0();
                formatbyte0 = 0x80;
                xTimerStart(xkl0busintTimer, 0);
            }
            else if (channel == 0x03)
            {
                formatbyte1 = 0x80;
                xTimerStart(xkl1busintTimer, 0);
            }
            // KlineInIT
            sendACK(channel);
        }
        else if (protocolType == KLINE_FAST_Init_C0)
        {
            if (channel == 0x02)
            {
                formatbyte0 = 0xC0;
                xTimerStart(xkl0busintTimer, 0);
            }
            else if (channel == 0x03)
            {
                formatbyte1 = 0xC0;
                xTimerStart(xkl1busintTimer, 0);
            }
            sendACK(channel);
        }
        else
        {
            ESP_LOGW(TAG, "Protocol not supported : %x", protocolType);
            sendNACK(channel, K_NACK_CODE_CMD_NOT_SUPPORTED);
        }
    }
   
    else if (settingType == SETTING_TYPE_SET_TX_ID)
    {
        if (channel == 0x02)
        {
            if ((len - index) == 1)
            {
                targetbyte0 = data[index++];
                sendACK(channel);
            }
        }
        else if (channel == 0x03)
        {
            if ((len - index) == 1)
            {
                targetbyte1 = data[index++];
                sendACK(channel);
            }
        }
    }
   
    else if (settingType == SETTING_TYPE_SET_RX_ID)
    {
        if (channel == 0x02)
        {
            if ((len - index) == 1)
            {
                sourcebyte0 = data[index++];
                sendACK(channel);
            }
        }

        else if (channel == 0x03)
        {
            if ((len - index) == 1)
            {
                sourcebyte1 = data[index++];
                sendACK(channel);
            }
        }
    }

    else if (settingType == SETTING_TYPE_GET_FIRMWARE_VERSION)
    {
        if ((len - index) == 0)
        {
            FWversion *fwVersion = settings_getFirmwareVersion();

            sendFirmwareVersion(channel, fwVersion);
        }
        else
        {
            // ESP_LOGW(TAG, "Inalid getFW version length");
            sendNACK(channel, RESP_NACK_CODE_INVALID_FORMAT_OR_LENGTH);
        }
    }
    
    else if (settingType == SETTING_TYPE_SET_STA_SSID)
    {
        // SetSSID(data, len);
        SetSSID(data, &data[2], len);
        sendACK(channel);
        
    }
   
    else if (settingType == SETTING_TYPE_SET_STA_PASSWORD)
    {

        // SetSSID(data, len);
        SetPSWD(data, &data[2], len);
      sendACK(channel);
    }
   
    else if (settingType == SETTING_TYPE_START_PERIODIC_TESTER_PRESENT)
    {
        if ((len - index) == 0)
        {
            if (channel == 0x02)
            {
                xKl0TestStarted = xTimerStart(xKl0TestTimer, 0);
            }
            else if (channel == 0x03)
            {
                xKl1TestStarted = xTimerStart(xKl1TestTimer, 0);
            }
            sendACK(channel);
        }
        else
        {
            // ESP_LOGW(TAG, "Inalid start periodic tester event length");
            sendNACK(channel, RESP_NACK_CODE_INVALID_FORMAT_OR_LENGTH);
        }
    }
   
    else if (settingType == SETTING_TYPE_STOP_PERIODIC_TESTER_PRESENT)
    {
        if ((len - index) == 0)
        {

            if (channel == 0x02)
            {
                xTimerReset(xKl0TestTimer, 0);
                xTimerStop(xKl0TestTimer, 0);
            }
            else if (channel == 0x03)
            {
                xTimerReset(xKl1TestTimer, 0);
                xTimerStop(xKl1TestTimer, 0);
            }

            // ESP_LOGI(TAG, "Stop tester present");
            sendACK(channel);
        }
        else
        {
            // ESP_LOGW(TAG, "Inalid stop periodic tester event length");
            sendNACK(channel, RESP_NACK_CODE_INVALID_FORMAT_OR_LENGTH);
        }
    }

    else if (settingType == SETTING_TYPE_ENABLE_KLBUSINI)
    {
        if ((len - index) == 0)
        {
            if (channel == 0x02)
            {

                // xkl0busintTimer = xTimerCreate("Testerpresent", pdMS_TO_TICKS(KlineBusinitTime), pdTRUE, 0, buskl0businint);
                xTimerStart(xkl0busintTimer, 0);
            }
            else if (channel == 0x03)
            {

                // xkl1busintTimer = xTimerCreate("Testerpresent", pdMS_TO_TICKS(KlineBusinitTime), pdTRUE, 0, buskl1businint);
                xTimerStart(xkl1busintTimer, 0);
            }
            sendACK(channel);
        }
        else
        {
            // ESP_LOGW(TAG, "Inalid start periodic tester event length");
            sendNACK(channel, RESP_NACK_CODE_INVALID_FORMAT_OR_LENGTH);
        }
    }
   
    else if (settingType == SETTING_TYPE_DISABLE_KLBUSINI)
    {
        if ((len - index) == 0)
        {
            if (channel == 0x02)
            {
                xTimerReset(xkl0busintTimer, 0);
                xTimerStop(xkl0busintTimer, 0);
                // xTimerDelete(xkl0busintTimer, 0);
            }
            else if (channel == 0x03)
            {
                xTimerReset(xkl1busintTimer, 0);
                xTimerStop(xkl1busintTimer, 0);
                // xTimerDelete(xkl1busintTimer, 0);
            }
            sendACK(channel);
        }
        else
        {
            // ESP_LOGW(TAG, "Inalid start periodic tester event length");
            sendNACK(channel, RESP_NACK_CODE_INVALID_FORMAT_OR_LENGTH);
        }
    }

    else if (settingType == SETTING_TYPE_SET_KL_BUSINITIME)
    {
        if ((len - index) == 2)
        {
            uint16_t klbusuinintime;
            klbusuinintime = ((uint16_t)(data[1]) << 8) | (uint16_t)data[2];

            if (channel == 0x02)
            {
                if (xTimerIsTimerActive(xkl0busintTimer))
                {
                    xTimerChangePeriod(xkl0busintTimer,
                                       (klbusuinintime / portTICK_PERIOD_MS),
                                       10);
                }
            }
            else if (channel == 0x03)
            {
                if (xTimerIsTimerActive(xkl1busintTimer))
                {
                    xTimerChangePeriod(xkl1busintTimer,
                                       (klbusuinintime / portTICK_PERIOD_MS),
                                       10);
                }
            }
            sendACK(channel);
        }
        else
        {
            // ESP_LOGW(TAG, "Inalid rx id set length");
            sendNACK(channel, RESP_NACK_CODE_INVALID_FORMAT_OR_LENGTH);
        }
    }

    else if (settingType == SETTING_TYPE_SET_KL_IFTIME)
    {
        if ((len - index) == 1)
        {
            kl0IFtime = data[index++];

            if (channel == 0x02)
            {
            }
            else if (channel == 0x03)
            {
            }
            sendACK(channel);
        }
        else
        {
            // ESP_LOGW(TAG, "Inalid rx id set length");
            sendNACK(channel, RESP_NACK_CODE_INVALID_FORMAT_OR_LENGTH);
        }
    }
   
}

void senddatatokl(uint8_t channel, uint8_t *data, uint8_t len)
{

    uint8_t kldata[len + FORMATBYTESIZE + ADDBYTESIZE + CKSBYTESIZE];
    uint16_t index = 0;

    if (channel == 0x02) {
        
        if (xTimerIsTimerActive(xKl0TestTimer)) {
            xTimerReset(xKl0TestTimer, 0);
            xTimerStop(xKl0TestTimer, 0);
        }

        if (len < 64){
            kldata[index++] = formatbyte0 | len;
        }else {
            kldata[index++] = formatbyte0;
        }

        kldata[index++] = targetbyte0;
        kldata[index++] = sourcebyte0;
        
        if (len >= 64){
            kldata[index++] = len;
        }

        memcpy(&kldata[index++], data, len);
        index = index + len - 1;
        kldata[index++] = iso_checksum(kldata, len + FORMATBYTESIZE + ADDBYTESIZE);

        if (kl0busint == true)
        {
            ESP_LOGI(TAG,"timer of")   ;
            //vTaskDelay(100 / portTICK_PERIOD_MS);
            for (int i = 0; i < len + FORMATBYTESIZE + ADDBYTESIZE + CKSBYTESIZE; i++)
            {
                // uart_write_bytes(KL0, (uint8_t *)kldata, len + FORMATBYTESIZE + ADDBYTESIZE + CKSBYTESIZE);
                uart_write_bytes(KL0, &kldata[i], 1);
                vTaskDelay(kl0IFtime / portTICK_PERIOD_MS);
            }
            sendACK(channel);
        }

        else if (kl0busint == false) {
            ESP_LOGI(TAG, "Sending with init");
            kline0fastinit();
            vTaskDelay(500 / portTICK_PERIOD_MS);
            ESP_LOGI(TAG, "Initialize done");
            
            // uart_write_bytes(KL0, (uint8_t *)kldata, len + FORMATBYTESIZE + ADDBYTESIZE + CKSBYTESIZE);
            for (int i = 0; i < len + FORMATBYTESIZE + ADDBYTESIZE + CKSBYTESIZE; i++)
            {
                // uart_write_bytes(KL0, (uint8_t *)kldata, len + FORMATBYTESIZE + ADDBYTESIZE + CKSBYTESIZE);
                uart_write_bytes(KL0, &kldata[i], 1);
                vTaskDelay(kl0IFtime / portTICK_PERIOD_MS);
            }
            sendACK(channel);

        }
        
        if (xTimerIsTimerActive(xkl0busintTimer))
        {
            xTimerReset(xkl0busintTimer, 0);
            // xTimerStart(xkl0busintTimer, 10);
           // xTimerStop(xKl0TestTimer, 0);
        }
       //vPortFree(data);
    }

    else if (channel == 0x03) {
        
        if (xTimerIsTimerActive(xKl1TestTimer)) {
            xTimerReset(xKl1TestTimer, 0);
            xTimerStop(xKl1TestTimer, 0);
        }

        if (len < 64){
            kldata[index++] = formatbyte1 | len;
        }else {
            kldata[index++] = formatbyte1;
        }

        kldata[index++] = targetbyte1;
        kldata[index++] = sourcebyte1;
        
        if (len >= 64){
            kldata[index++] = len;
        }

        memcpy(&kldata[index++], data, len);
        index = index + len - 1;
        kldata[index++] = iso_checksum(kldata, len + FORMATBYTESIZE + ADDBYTESIZE);

        if (kl1busint == true)
        {
            ESP_LOGI(TAG,"timer of")   ;
            //vTaskDelay(100 / portTICK_PERIOD_MS);
            for (int i = 0; i < len + FORMATBYTESIZE + ADDBYTESIZE + CKSBYTESIZE; i++)
            {
                
                uart_write_bytes(KL1, &kldata[i], 1);
                vTaskDelay(kl1IFtime / portTICK_PERIOD_MS);
            }
            sendACK(channel);
        }

        else if (kl1busint == false) {
            ESP_LOGI(TAG, "Sending with init");
            kline1fastinit();
            vTaskDelay(500 / portTICK_PERIOD_MS);
            
            for (int i = 0; i < len + FORMATBYTESIZE + ADDBYTESIZE + CKSBYTESIZE; i++)
            {
                
                uart_write_bytes(KL1, &kldata[i], 1);
                vTaskDelay(kl1IFtime / portTICK_PERIOD_MS);
            }
            sendACK(channel);

        }
        
        if (xTimerIsTimerActive(xkl1busintTimer))
        {
            xTimerReset(xkl1busintTimer, 0);
            
        }
       //vPortFree(data);
    }
}

void klstcframe(uint8_t channel)
{
    int index = 0;
    uint8_t stcdata[5];
    // stcdata[index++] = formatbyte0 | 0b1;
    // stcdata[index++] = targetbyte0;
    // stcdata[index++] = sourcebyte0;
    // stcdata[index++] = 0x81;
    // stcdata[index++] = iso_checksum(stcdata, 4);
    //uart_write_bytes(KL0, (uint8_t *)stcdata, 5);
    if(channel==0x02){
        stcdata[index++] = formatbyte0 | 0b1;
        stcdata[index++] = targetbyte0;
        stcdata[index++] = sourcebyte0;
        stcdata[index++] = 0x81;
        stcdata[index++] = iso_checksum(stcdata, 4);
        for (int i = 0; i < 5; i++){
            uart_write_bytes(KL0, &stcdata[i], 1);
            vTaskDelay(kl0IFtime / portTICK_PERIOD_MS);
        }
    }

    else if(channel==0x03){
        stcdata[index++] = formatbyte1 | 0b1;
        stcdata[index++] = targetbyte1;
        stcdata[index++] = sourcebyte1;
        stcdata[index++] = 0x81;
        stcdata[index++] = iso_checksum(stcdata, 4);
        for (int i = 0; i < 5; i++) {
            uart_write_bytes(KL1, &stcdata[i], 1);
            vTaskDelay(kl0IFtime / portTICK_PERIOD_MS);
        }
    }

}
// xKl0busintstarted = xTimerStart( xkl0busintTimer, 0 );
//  xTimerReset(xkl1busintTimer,0);
//  xTimerStop(xkl1busintTimer,0);