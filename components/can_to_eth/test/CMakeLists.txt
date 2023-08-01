idf_component_register(SRCS "test_crc.c" "test_test.c"
                       INCLUDE_DIRS "."
                       REQUIRES cmock can_to_eth)