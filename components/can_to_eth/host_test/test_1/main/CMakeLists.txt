# idf_component_register(SRCS  "test_mqtt_client.cpp"
#                        INCLUDE_DIRS "$ENV{IDF_PATH}/tools/catch"
#                        REQUIRES cmock mqtt esp_timer esp_hw_support http_parser log)
# idf_component_register(SRCS  "mainx.cpp"
#                        INCLUDE_DIRS "$ENV{IDF_PATH}/tools/catch"
#                        REQUIRES cmock)
idf_component_register(SRCS 
                        "mainx.cpp"
                    INCLUDE_DIRS 
                        "../../"
                        $ENV{IDF_PATH}/tools/catch
                    REQUIRES 
                        cmock)