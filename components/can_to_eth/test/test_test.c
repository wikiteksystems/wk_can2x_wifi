#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "unity.h"
#include "unity_test_runner.h"

#include "tcp_packet_handler.h"
#include "iso_tp.h"

// const static char *TAG = "mytests";

TEST_CASE("testProcessCustomPacketIntoCANData", "[can_to_eth]") 
{
    const uint8_t checkArray[] = {
        0x40, 0x14,
        0x00,
        0x34, 0x68, 0x44, 0x64, 0x56, 0x46, 0x45, 0x46, 0x84, 0x65,
        0x46, 0x46, 0x21, 0x51, 0x35, 0x46, 0x54, 0x35, 0x0F, 0xAc,
        0xA4, 0x36
    };

    uint8_t canData[4096];
    uint16_t canDataLen = 0;
    uint16_t processedBytes = 0;
    uint8_t startNibble = 0;
    int resp = processCustomPacketIntoCANData(checkArray, sizeof(checkArray), canData, &canDataLen, &processedBytes, &startNibble);
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, resp, "processCustomPacketIntoCANData FAIL: resp == false");
    TEST_ASSERT_EQUAL_UINT16_MESSAGE(20, canDataLen, "processCustomPacketIntoCANData FAIL: canDataLen mismatch");
    TEST_ASSERT_EQUAL_UINT8_ARRAY_MESSAGE(&checkArray[3], canData, canDataLen, "processCustomPacketIntoCANData FAIL: canData mismatch");
}

TEST_CASE("testProcessCANDataIntoCustomPacket", "[can_to_eth]") 
{
    const uint8_t checkArray[] = {
        0x40, 0x14,
        // 0x00,
        0x34, 0x68, 0x44, 0x64, 0x56, 0x46, 0x45, 0x46, 0x84, 0x65,
        0x46, 0x46, 0x21, 0x51, 0x35, 0x46, 0x54, 0x35, 0x0F, 0xAc,
        0xA4, 0x36
    };
    uint8_t canData[] = {
        0x34, 0x68, 0x44, 0x64, 0x56, 0x46, 0x45, 0x46, 0x84, 0x65,
        0x46, 0x46, 0x21, 0x51, 0x35, 0x46, 0x54, 0x35, 0x0F, 0xAc
    };
    uint8_t customPacketData[0x2000+4];
    uint16_t customPacketDataLen = 0;
    bool resp = processCANDataIntoCustomPacket(canData, sizeof(canData), customPacketData, &customPacketDataLen);

    TEST_ASSERT_TRUE_MESSAGE(resp, "resp == false");
    TEST_ASSERT_EQUAL_UINT8_ARRAY_MESSAGE(checkArray, customPacketData, sizeof(checkArray),
        "custom data  mismatch");
}

TEST_CASE("testvalidateISO_TP_frame_packet", "[can_to_eth]") 
{
    ISOTPFrame frame;
    uint8_t refData[8] = {0x17, 0x23, 0x34, 0x45, 0x56, 0x67, 0x78, 0x89}; 
    memcpy(frame.data, refData, sizeof(refData));
    printf("OUTPUT SINGLE : %x %x\n", frame.single.code, frame.single.len);

    TEST_ASSERT_EQUAL_UINT8(0x01, frame.single.code);
    TEST_ASSERT_EQUAL_UINT8(0x07, frame.single.len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(&refData[1], frame.single.data, 7);

    // printf("OUTPUT FIRST : %x %x %x\n", frame.first.code, frame.first.lenh, frame.first.lenl);
    // if (frame.first.code == 0x01 && frame.first.lenh == 0x07 && frame.first.lenl == 0x23
    //     && memcmp(frame.first.data, &refData[2], 6) == 0) {
    //     ESP_LOGI(TAG, "TEST first frame format PASS\n");
    // } else {
    //     ESP_LOGW(TAG, "TEST first frame format FAIL\n");
    // }
    // printf("OUTPUT CONSECUTIVE : %x %x\n", frame.consecutive.code, frame.consecutive.index);
    // if (frame.consecutive.code == 0x01 && frame.consecutive.index == 0x07
    //     && memcmp(frame.consecutive.data, &refData[1], 7) == 0) {
    //     ESP_LOGI(TAG, "TEST consecutive frame format PASS\n");
    // } else {
    //     ESP_LOGW(TAG, "TEST consecutive frame format FAIL\n");
    // }
    // printf("OUTPUT FLOW : %x %x %x %x\n", frame.flow.code, frame.flow.status, 
    //     frame.flow.blockSize, frame.flow.separationTime);
    // if (frame.flow.code == 0x01 && frame.flow.status == 0x07
    //     && frame.flow.blockSize == 0x23
    //     && frame.flow.separationTime == 0x34
    //     && memcmp(frame.flow.reserved, &refData[4], 4) == 0) {
    //     ESP_LOGI(TAG, "TEST flow frame format PASS\n");
    // } else {
    //     ESP_LOGW(TAG, "TEST flow frame format FAIL\n");
    // }
}
