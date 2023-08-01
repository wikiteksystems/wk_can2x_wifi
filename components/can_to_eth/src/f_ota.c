#include "f_ota.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "string.h"
#include <sys/socket.h>
#include "instructions.h"

#include "iooperation.h"

TaskHandle_t cofotatask_h;

#define HASH_LEN 32

static const char *TAG = "f_ota";
extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");

#define OTA_URL_SIZE 512

char ota_url[OTA_URL_SIZE + 1] = "http://nonsecure.mayankgour.com/Eth-to-CAN.bin";

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id) {
    case HTTP_EVENT_ERROR:
        ESP_LOGI(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGI(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGI(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGI(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGI(TAG, "HTTP_EVENT_ON_FINISH");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
        break;
    case HTTP_EVENT_REDIRECT:
        ESP_LOGI(TAG, "HTTP_EVENT_REDIRECT");
        break;
    }
    return ESP_OK;
}

void http_ota_task(void *pvParameter)
{
    ESP_LOGI(TAG, "Starting OTA task");

    esp_http_client_config_t config = {
        .url = ota_url,
        .cert_pem = (char *)server_cert_pem_start,
        .event_handler = _http_event_handler,
        .keep_alive_enable = true
    };

    config.skip_cert_common_name_check = true;

    esp_https_ota_config_t ota_config = {
        .http_config = &config,
    };
    ESP_LOGI(TAG, "Attempting to download update from %s", config.url);
    esp_err_t fret = esp_https_ota(&ota_config);
    
    if (fret == ESP_OK) {
        
        //sendACK(0x00);
        ESP_LOGI(TAG, "OTA Succeed, Rebooting...");
        //predefined channel byte
              
        esp_restart();
    } else {
        ESP_LOGE(TAG, "Firmware upgrade failed");
        //predefined channel byte
        sendNACK(0x00, RESP_NACK_CODE_INVALID_FORMAT_OR_LENGTH);
        //vTaskDelete(cofotatask_h);

    }
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static void print_sha256(const uint8_t *image_hash, const char *label)
{
    char hash_print[HASH_LEN * 2 + 1];
    hash_print[HASH_LEN * 2] = 0;
    for (int i = 0; i < HASH_LEN; ++i) {
        sprintf(&hash_print[i * 2], "%02x", image_hash[i]);
    }
    ESP_LOGI(TAG, "%s %s", label, hash_print);
}

static void get_sha256_of_partitions(void)
{
    uint8_t sha_256[HASH_LEN] = { 0 };
    esp_partition_t partition;

    // get sha256 digest for bootloader
    partition.address   = ESP_BOOTLOADER_OFFSET;
    partition.size      = ESP_PARTITION_TABLE_OFFSET;
    partition.type      = ESP_PARTITION_TYPE_APP;
    esp_partition_get_sha256(&partition, sha_256);
    print_sha256(sha_256, "SHA-256 for bootloader: ");

    // get sha256 digest for running partition
    esp_partition_get_sha256(esp_ota_get_running_partition(), sha_256);
    print_sha256(sha_256, "SHA-256 for current firmware: ");
}


int beginOTA(const char *url, uint16_t len) {

    if (len > OTA_URL_SIZE) {
        ESP_LOGW(TAG, "OTA URL is longer than max size : %d/%d", len, OTA_URL_SIZE);
        return -1;
    }
    memcpy(ota_url, url, len);
    ota_url[len] = 0x00;

    ESP_LOGI(TAG, "OTA task starting");
    get_sha256_of_partitions();
    xTaskCreate(http_ota_task, "ota_task", 8 * 1024, NULL, 5, NULL);
    ESP_LOGI(TAG, "OTA  LED task starting");
    xTaskCreate(OTALEDTASK, "ota_led_task", 1024 * 2, NULL, tskIDLE_PRIORITY, &cofotatask_h);
    return 0;
}