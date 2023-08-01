file(GLOB_RECURSE cfiles "./*.c") 
message(STATUS "FILE INSIDE MCP2515 SRC: ${cfiles}")

set(INC_DIRS ".")

idf_component_register(SRCS ${cfiles}
                    INCLUDE_DIRS ${INC_DIRS}
                    REQUIRES "driver" "esp_netif" "esp_eth" "esp_timer" "mdns" "esp_wifi")