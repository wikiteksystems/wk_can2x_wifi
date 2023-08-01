#include "wifi_service.h"

#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "sock_service.h"
#include "instructions.h"


#include "driver/uart.h"
#include "usb_service.h"

static const char *TAG = "WIFI_Service";


/* The examples use WiFi configuration that you can set via project configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
// #define EXAMPLE_ESP_WIFI_SSID      "Airtel_9503340304"
// #define EXAMPLE_ESP_WIFI_PASS      "air44591"
// #define EXAMPLE_ESP_WIFI_SSID      "Heximpact"
// #define EXAMPLE_ESP_WIFI_SSID      "Mayank_M51"
// #define EXAMPLE_ESP_WIFI_SSID      "Mayank_RN4"
// #define EXAMPLE_ESP_WIFI_PASS      "infi2022"
// #define EXAMPLE_ESP_WIFI_SSID      "MAYANK-PC"
// #define EXAMPLE_ESP_WIFI_PASS      "ENDURANCE"
// #define EXAMPLE_ESP_WIFI_SSID      "Kpix_Fiber_2G"
// #define EXAMPLE_ESP_WIFI_PASS      "kpix95033"
// #define EXAMPLE_ESP_WIFI_SSID      "TP-Link_9108"
// #define EXAMPLE_ESP_WIFI_PASS      "test1234"
// #define EXAMPLE_ESP_WIFI_SSID      "SavanRaj_2.4G"
// #define EXAMPLE_ESP_WIFI_PASS      "test9428"

#define EXAMPLE_ESP_MAXIMUM_RETRY  3
#define EXAMPLE_ESP_AP_WIFI_SSID "Can2x"
#define EXAMPLE_ESP_AP_WIFI_PASS "paswd1234"
#define EXAMPLE_ESP_WIFI_CHANNEL 9
#define EXAMPLE_MAX_STA_CONN 3





bool userSSID=false;
bool defaultSSID=false;

nvs_handle_t my_handle;

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1


static int s_retry_num = 0;

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        // if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            // s_retry_num++;
            // ESP_LOGI(TAG, "retry to connect to the AP");
        // } else {
            // xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            wificonnect=false;
        // }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        esp_wifi_get_mac(WIFI_IF_STA, mac);
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        
        char* Txdata = (char*) pvPortMalloc(100);
        sprintf (Txdata,"\n WIFIIP:" IPSTR, IP2STR(&event->ip_info.ip));
        uart_write_bytes(UART, Txdata, strlen(Txdata));
        vPortFree(Txdata);
        
        ESP_LOGI(TAG, "WiFi HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        wificonnect=true;
    }
    // if (event_id == WIFI_EVENT_AP_STACONNECTED) {
    //     wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
    //     // ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
    //     //          MAC2STR(event->mac), event->aid);
    // } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
    //     wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
    //     // ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
    //     //          MAC2STR(event->mac), event->aid);
    // }
}


void initialise_wifi(void) {

    esp_wifi_set_ps(WIFI_PS_NONE);
    
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    
    s_wifi_event_group = xEventGroupCreate();

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    //Read both the SSID and Password from the 
    nvs_open("config", NVS_READWRITE, &my_handle);
    size_t urequired_size;
    nvs_get_str(my_handle, "stSSID[1]", NULL, &urequired_size);
    char *ussid_name = pvPortMalloc(urequired_size);
    nvs_get_str(my_handle, "stSSID[1]", ussid_name, &urequired_size);
    //ESP_LOGI(TAG,"Read USSIDdata: %s\n", ussid_name);
    if(ussid_name!=NULL){ 
    ESP_LOGI(TAG,"Read USSIDdata: %s\n", ussid_name);
    }
    else{
        ussid_name=".";
         ESP_LOGI(TAG,"NOT able to read SSID");
    }
    //uart_write_bytes()
    char* ssiddata = (char*) pvPortMalloc(100);
    sprintf (ssiddata,"\n USER SSID: %s \n ",ussid_name );
    uart_write_bytes(UART, ssiddata, strlen(ssiddata));
    vPortFree(ssiddata);


    nvs_close(my_handle);

    nvs_open("config", NVS_READWRITE, &my_handle);
    size_t upaswdrequired_size;
    nvs_get_str(my_handle, "stPASS[1]", NULL, &upaswdrequired_size);
    char *UPassword = pvPortMalloc(upaswdrequired_size);
    nvs_get_str(my_handle, "stPASS[1]", UPassword, &upaswdrequired_size);
    //ESP_LOGI(TAG,"Read upassword: %s\n", UPassword);
    if(UPassword!=NULL){
    ESP_LOGI(TAG,"Read upassword: %s\n", UPassword);
    }
    else{
        UPassword=".";
    }

    
    char* pswddata = (char*) pvPortMalloc(100);
    sprintf (pswddata,"USER PSWD: %s \n ",UPassword );
    uart_write_bytes(UART, pswddata, strlen(pswddata));
    vPortFree(pswddata);

    nvs_close(my_handle);


    char *EXAMPLE_ESP_WIFI_SSID = (char *)ussid_name;
    char *EXAMPLE_ESP_WIFI_PASS = (char *)UPassword;
    
   

    {
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = {},
            .password = {},
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
	        .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    
    strncpy((char *)wifi_config.sta.ssid, (char *)EXAMPLE_ESP_WIFI_SSID, 32);
    strncpy((char *)wifi_config.sta.password, (char *)EXAMPLE_ESP_WIFI_PASS, 32);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start());
    

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
        wificonnect=true; 
    }  else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    }

    // ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    // ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    // vEventGroupDelete(s_wifi_event_group);


    if(wificonnect==false)
    {
        esp_netif_t *p_netif = esp_netif_create_default_wifi_ap();
        // esp_netif_create_default_wifi_ap();
        // wifi_init_config_t Apcfg = WIFI_INIT_CONFIG_DEFAULT();
        // ESP_ERROR_CHECK(esp_wifi_init(&Apcfg));

        ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                            ESP_EVENT_ANY_ID,
                                                            &event_handler,
                                                            NULL,
                                                            NULL));

        wifi_config_t APwifi_config = {
            .ap = {
                .ssid = EXAMPLE_ESP_AP_WIFI_SSID,
                .ssid_len = strlen(EXAMPLE_ESP_AP_WIFI_SSID),
                .channel = EXAMPLE_ESP_WIFI_CHANNEL,
                .password = EXAMPLE_ESP_AP_WIFI_PASS,
                .max_connection = EXAMPLE_MAX_STA_CONN,
                .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            },
        };
    
        if (strlen(EXAMPLE_ESP_AP_WIFI_PASS) == 0){
        APwifi_config.ap.authmode = WIFI_AUTH_OPEN;
        }

        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &APwifi_config));
        ESP_ERROR_CHECK(esp_wifi_start());
        
       // initialise_sock();

        ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s ",
                EXAMPLE_ESP_AP_WIFI_SSID, EXAMPLE_ESP_AP_WIFI_PASS);

        esp_netif_ip_info_t if_info;
        ESP_ERROR_CHECK(esp_netif_get_ip_info(p_netif, &if_info));
        ESP_LOGI(TAG, "ESP32 IP:" IPSTR, IP2STR(&if_info.ip));
        
        char* Txdata = (char*) pvPortMalloc(100);
        sprintf (Txdata,"\n APIP:" IPSTR, IP2STR(&if_info.ip));
        uart_write_bytes(UART, Txdata, strlen(Txdata));
        vPortFree(Txdata);
    }

   
   

}


void SetSSID(uint8_t *data, uint8_t *uint8_t_SSID, uint16_t len)
{
    uint16_t index = 0;
    //
    uint8_t SSIDtype = data[++index];

    char *SSID = (char *)uint8_t_SSID;
    
    ESP_LOGI(TAG, "SetSSID is type %d ", SSIDtype);

    ESP_LOGI(TAG, "SetSSID is: %s ", SSID);

    if (SSIDtype == Default_SSID)
    {
        nvs_open("config", NVS_READWRITE, &my_handle);
        nvs_set_str(my_handle, "DSSID", SSID);
        printf("Write SSID: %s\n", SSID);
        nvs_commit(my_handle);
        nvs_close(my_handle);
        
    }

    else if (SSIDtype == User_SSID)
    {

        // nvs_handle_t my_handle;
        nvs_open("config", NVS_READWRITE, &my_handle);
        nvs_set_str(my_handle, "stSSID[1]", SSID);
        printf("Write SSID: %s\n", SSID);
        nvs_commit(my_handle);
        nvs_close(my_handle);
       
    }
}

void SetPSWD(uint8_t *data, uint8_t *uint8_t_upaswd, uint16_t len)
{
    uint16_t index = 0;
    //
    uint8_t Passwordtype = data[++index];

    char *UPassword = (char *)uint8_t_upaswd;
    ESP_LOGI(TAG, "Password is: %s ", UPassword);

    if (Passwordtype == Default_SSID)
    {
        nvs_open("config", NVS_READWRITE, &my_handle);
        nvs_set_str(my_handle, "DPassword", UPassword);
        printf("Write Password: %s\n", UPassword);
        nvs_commit(my_handle);
        nvs_close(my_handle);
       
    }

    else if (Passwordtype == User_SSID)
    {
        // nvs_handle_t my_handle;
        nvs_open("config", NVS_READWRITE, &my_handle);
        nvs_set_str(my_handle, "stPASS[1]", UPassword);
        printf("Write Password: %s\n", UPassword);
        nvs_commit(my_handle);
        nvs_close(my_handle);
       
    }
}