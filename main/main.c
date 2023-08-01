#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#include "canbus.h"
#include "can2bus.h"
#include "mdns_service.h"
#include "iso_tp.h"

#include "mcp2515.h"

#include "can.h"
#include "driver/gpio.h"

#include "channel.h"
#include "usb_service.h"
#include "klinebus.h"
#include "sock_service.h"

#include "eth_service.h"
#include "wifi_service.h"
#include "iooperation.h"
#include "bt_service.h"
//

static const char *TAG = "main";

void app_main(void)
{
  ESP_LOGI(TAG, "STARTING APPLICATION");
  ESP_LOGI(TAG, "From the Laptop Firmware");

  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
  // Initialize TCP/IP network interface (should be called only once in application)
  ESP_ERROR_CHECK(esp_netif_init());
  // Create default event loop that running in background
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  iointtask();
  initialise_uart();
  initialise_mdns();
  // initialise_kl0();
  initialise_canbus(&channelAll[0]);
  //initialise_can2bus(&channelAll[1]);
  
  initialise_sock();
  // bt_initialize();
//  
  initialise_wifi();
  //initialise_eth();
  
  



}
