#ifndef __MAIN_INC_UTILITY__
#define __MAIN_INC_UTILITY__

#include <stdint.h>

#define DEBUG_PRINT_HEX

#ifdef DEBUG_PRINT_HEX
#define DEBUG_PRINT_HEX_DATA(data, len)  {\
        for (int i = 0; i < len; i++) { \
            printf("%02x ", data[i]); \
        } \
        printf("\n"); \
    }
#else
#define DEBUG_PRINT_HEX_DATA(data, len) 
#endif

typedef union {
    struct {
        uint8_t lbyte;
        uint8_t hbyte;
    } bytes;
    uint16_t word;
} UWord;



typedef union {
    struct {
        uint16_t lword;
        uint16_t hword;
    } words;
    struct {
        uint8_t lwlbyte; // Lower word's lower byte
        uint8_t lwhbyte;
        uint8_t hwlbyte;
        uint8_t hwhbyte; // Higer word's higher byte
    } bytes;
    uint32_t dword;
} UDWord;

extern bool appcomm;
extern bool cancomm;

#endif