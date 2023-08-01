#include "mdns_service.h"

#include <string.h>
#include "esp_log.h"
#include "mdns.h"


// #define MDNS_HOSTNAME "egaecp01001"
// #define MDNS_INSTANCE_NAME "egaecp01001"

#define MDNS_HOSTNAME "apreet001"
#define MDNS_INSTANCE_NAME "apreet001"


const static char *TAG = "MDNS_service";


void initialise_mdns(void)
{
    char * hostname = MDNS_HOSTNAME;

    //initialize mDNS
    ESP_ERROR_CHECK( mdns_init() );
    //set mDNS hostname (required if you want to advertise services)
    ESP_ERROR_CHECK( mdns_hostname_set(hostname) );
    ESP_LOGI(TAG, "mdns hostname set to: [%s]", hostname);
    //set default mDNS instance name
    ESP_ERROR_CHECK( mdns_instance_name_set(MDNS_INSTANCE_NAME) );

    //structure with TXT records
    mdns_txt_item_t serviceTxtData[] = {
        {"board", "esp32"},
        // {"u", "user"},
        // {"p", "password"}
    };

    //initialize service
    // ESP_ERROR_CHECK( mdns_service_add("obd2", "_http", "_tcp", 80, serviceTxtData,
    //       sizeof(serviceTxtData)/sizeof(serviceTxtData[0])) );

        //   ESP_ERROR_CHECK( mdns_service_add("egaecp01001", "_http", "_tcp", 80, serviceTxtData,
        //   sizeof(serviceTxtData)/sizeof(serviceTxtData[0])) );

      ESP_ERROR_CHECK( mdns_service_add("apreet001", "_http", "_tcp", 80, serviceTxtData,
          sizeof(serviceTxtData)/sizeof(serviceTxtData[0])) );

    //add another TXT item
    // ESP_ERROR_CHECK( mdns_service_txt_item_set("_http", "_tcp", "path", "/foobar") );
    //change TXT item value
    // ESP_ERROR_CHECK( mdns_service_txt_item_set_with_explicit_value_len("_http", "_tcp", "u", "admin", strlen("admin")) );
}