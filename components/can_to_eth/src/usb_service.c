#include "usb_service.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

#include "sock_service.h"
#include "klinebus.h"
#include "sock_service.h"
#include "instructions.h"
#include "esp_wifi.h"
#include "sock_service.h"

//TaskHandle_t socketTask;
uint8_t Activechannel;

static const char *TAG = "UART TEST";

#define PATTERN_CHR_NUM (3) /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/
#define BUF_SIZE (4096 + 10 + 512)

#define RD_BUF_SIZE (BUF_SIZE)
// static QueueHandle_t uart0_queue;

#define LOCAL_MIN(a, b) (a < b ? a : b)
#define SOCKET_RX_CACHE_SIZE (4096 + 10)

static void uart_event_task(void *pvParameters)
{
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    size_t len;
    while (1) {    
        
       
        len = uart_read_bytes(UART,(uint8_t *) &data[0],3,  200 / portTICK_PERIOD_MS);
        //uart_write_bytes(UART, (const char *) data, len);
        if (len) {
        
            Activechannel=USB_CH;
          
            len= ((uint16_t) (0x0F & data[0]) << 8)| (uint16_t) data[1];
       
            len=uart_read_bytes(UART, (uint8_t *)&data[3],len+2,  200 / portTICK_PERIOD_MS);
            
            // ESP_LOGI(TAG,"RECEIVED BYTE %d",len);
           

           // taskEXIT_CRITICAL(&muxCritical);
            processAcquiredData(&data[0], len+3);
            len=0;
        
        }
    }
    vTaskDelete(NULL);
}




void initialise_uart(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        //.rx_flow_ctrl_thresh=127,
        .source_clk = SOC_MOD_CLK_APB,

    };
    
    // Install UART driver, and get the queue.
    uart_driver_install(UART, BUF_SIZE , BUF_SIZE, 0, NULL, 0);
    uart_param_config(UART, &uart_config);

    // Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    uart_set_pin(UART, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_set_rx_full_threshold(UART,10);
    char* test_str = "BT-FWVERSION-4.0.5\n";
    uart_write_bytes(UART, (const char*)test_str, strlen(test_str));
   // xTaskCreate(uart_event_task, "uart_event_task", 5*1024, NULL,tskIDLE_PRIORITY + 7 , NULL);
     xTaskCreatePinnedToCore(uart_event_task, "uart_event_task", 5*1024, NULL,tskIDLE_PRIORITY + 7 , NULL, 1);
    
}



void senddatatouart(uint8_t *data,uint16_t len){
     uart_write_bytes(UART, (uint8_t *)data, len);
    if (data != NULL) {
        vPortFree(data);
        data = NULL;
    }
}