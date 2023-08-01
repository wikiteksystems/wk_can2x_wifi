#include "iooperation.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_ota_ops.h"
#include "esp_log.h"
#include "wifi_service.h"
#include "utility.h"
#include "dev_settings.h"
// #include "nvs_flash.h"

TaskHandle_t comLedTask_h;

// nvs_handle_t my_handle;

static const char *TAG = "IO";

bool wificonnect=false;


void ioinit()
{

    gpio_set_direction(SW1, GPIO_MODE_INPUT);
    gpio_set_direction(LED1, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED2, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED3, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED4, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED5, GPIO_MODE_OUTPUT);

    gpio_set_level(LED1, LOW);
    gpio_set_level(LED2, HIGH);
    gpio_set_level(LED3, HIGH);
    gpio_set_level(LED4, HIGH);
    gpio_set_level(LED5, HIGH);
}

void SW_OPERATION_TASK(void *pv)
{
  // log_i("SW_OPERATION_TASK: %u", uxTaskGetStackHighWaterMark());

  int btn_cnt = 0;
  while (1)
  {
   
    if(gpio_get_level(SW1) == 0)
    {
      btn_cnt++;
      vTaskDelay(pdMS_TO_TICKS(300));
      gpio_set_level(LED1, HIGH);
      vTaskDelay(pdMS_TO_TICKS(200));
      gpio_set_level(LED1, LOW);
    }
    else
    {
      // if ((btn_cnt > 1) && (btn_cnt < 20))
      // {
      //   const esp_partition_t *npart = esp_ota_get_next_update_partition(esp_ota_get_running_partition());
      //   // Serial.printf("Next Partition label: %s \r\n", npart->label);
      //     if (esp_ota_set_boot_partition(npart) != ESP_OK){

      //       ESP_LOGI(TAG, "Eror");
      //     }
      //   //Serial.println("Error ");
      //   //ESP.restart();
      //   esp_restart();
      // }
   
      btn_cnt = 0;
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  vTaskDelete(NULL);
}

void SecWifiLedTask(void *pv){
  
  for(;;)
  {
  
  if(wificonnect==false){
    gpio_set_level(LED2, HIGH);
  }
  if(devSettings.status.isSecurityCheckPassed==false){
    gpio_set_level(LED3, HIGH);
  }
  vTaskDelay(pdMS_TO_TICKS(500));
  gpio_set_level(LED2, LOW);
  gpio_set_level(LED3, LOW);
  vTaskDelay(pdMS_TO_TICKS(500));
  }
  vTaskDelete(NULL);
}

void ComLedTask(void *pv)
{
  for (;;)
  {
    if (appcomm==true){
      gpio_set_level(LED4, LOW);
    }
    if (cancomm==true){
      gpio_set_level(LED5, LOW);
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    gpio_set_level(LED4, HIGH);
    gpio_set_level(LED5, HIGH);
    appcomm = false;
    cancomm = false;

    vTaskDelay(pdMS_TO_TICKS(100));
  }
vTaskDelete(NULL);
}

void OTALEDTASK(void *pv){
  
  vTaskDelete(comLedTask_h);
  ESP_LOGI(TAG,"OTA LED TASSK IS ON");
  for (;;)
  {
    gpio_set_level(LED1, LOW);
    gpio_set_level(LED2, LOW);
    gpio_set_level(LED3, LOW);
    gpio_set_level(LED4, LOW);
    gpio_set_level(LED5, LOW);
    vTaskDelay(pdMS_TO_TICKS(250));
    gpio_set_level(LED1, HIGH);
    gpio_set_level(LED2, HIGH);
    gpio_set_level(LED3, HIGH);
    gpio_set_level(LED4, HIGH);
    gpio_set_level(LED5, HIGH);
    vTaskDelay(pdMS_TO_TICKS(250));
  }
vTaskDelete(NULL);
}


void iointtask(void)
{
  ioinit();
  if (xTaskCreate(SW_OPERATION_TASK, "SW_OPERATION_TASK", 1024 * 3, NULL, tskIDLE_PRIORITY, NULL) != pdTRUE) {
  }

  if (xTaskCreate(SecWifiLedTask, "SecWifiLedTask", 1024 * 2, NULL, tskIDLE_PRIORITY, NULL) != pdTRUE){
  }

  if (xTaskCreate(ComLedTask, "APP_TASK", 1024 * 2, NULL, tskIDLE_PRIORITY, &comLedTask_h) != pdTRUE){
  }


}