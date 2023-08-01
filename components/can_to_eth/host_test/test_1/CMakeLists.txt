cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
set(COMPONENTS main)

idf_build_set_property(COMPILE_DEFINITIONS "-DNO_DEBUG_STORAGE" APPEND)

idf_component_get_property(original_component_dir "can_to_eth" COMPONENT_OVERRIDEN_DIR)

message(STATUS "original component dir: ${original_component_dir}")

idf_component_mock(INCLUDE_DIRS "${original_component_dir}/inc"
    REQUIRES freertos
    MOCK_HEADER_FILES ${original_component_dir}/inc/instructions.h)


list(APPEND EXTRA_COMPONENT_DIRS "$ENV{IDF_PATH}/tools/mocks/freertos/")

project(host_can_to_eth_test)