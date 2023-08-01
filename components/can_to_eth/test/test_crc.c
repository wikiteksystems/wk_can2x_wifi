#include <stdint.h>
#include <stdbool.h>
#include "unity.h"
#include "unity_test_runner.h"

#include "crc.h"

TEST_CASE("testCRCChecksum", "[can_to_eth]") 
{
    const uint8_t checkArray[] = {
        0x31, 0x61, 0x34, 0x68, 0x44, 0x64, 0x56, 0x46,
        0x45, 0x46, 0x84, 0x65, 0x46, 0x46, 0x21, 0x51,
        0x35, 0x46, 0x54, 0x35};
    const uint16_t crc = 0xD862;

    uint16_t checkCRC = CRC16_CCITT(checkArray, sizeof(checkArray));
    TEST_ASSERT_EQUAL_UINT16(crc, checkCRC);
}
