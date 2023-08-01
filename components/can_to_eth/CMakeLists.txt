file(GLOB_RECURSE cfiles "./src/*.c"
GLOB_RECURSE cppfiles "./src/*.cpp"   ) 
message(STATUS "FILE INSIDE can_to_eth SRC: ${cfiles}")

set(INC_DIRS "inc")

idf_component_register(SRCS ${cfiles} ${cppfiles}
                    INCLUDE_DIRS ${INC_DIRS}
                    REQUIRES    "driver" 
                                "esp_netif" 
                                "esp_eth" 
                                "esp_timer" 
                                "mdns" 
                                "esp_wifi" 
                                "mcp2515" 
                                "nvs_flash" 
                                "app_update" 
                                "esp_http_client"
                                "esp_https_ota"
                                "mbedtls"
                                "lwip"
                                "bt"
                                
                    EMBED_TXTFILES ${project_dir}/server_certs/ca_cert.pem)