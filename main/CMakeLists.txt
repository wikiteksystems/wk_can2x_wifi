file(GLOB_RECURSE cfiles "./src/*.c") 
message(STATUS "FILE INSIDE SRC: ${cfiles}")

set(INC_DIRS ".")

idf_component_register(SRCS main.c ${cfiles}
                    INCLUDE_DIRS ${INC_DIRS}
                    REQUIRES "can_to_eth" "nvs_flash" "mcp2515" "freertos")