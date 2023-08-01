#include <stdint.h>
#include "instruction_desc.h"
#include "instructions.h"


// #define KL0TXD_PIN GPIO_NUM_4
// #define KL0RXD_PIN GPIO_NUM_35


#define KL0TXD_PIN GPIO_NUM_17
#define KL0RXD_PIN GPIO_NUM_16

#define KL0 UART_NUM_1
#define KL0_BAUD 10400

#define KL1TXD_PIN GPIO_NUM_33
#define KL1RXD_PIN GPIO_NUM_32

#define KL1 UART_NUM_2
#define KL1_BAUD 10400


#define PinHigh 1
#define PinLow 0

#define KL_INIT_IDLE_BUS_BEFORE 500
#define KLine_bus_time_delay 25

void initialise_kl0();
void initialise_kl1();
void inst_kl_settings(uint8_t channel, uint8_t *data,uint16_t len);
void senddatatokl(uint8_t channel,uint8_t *data,uint8_t len);
void readkline0(uint16_t totalbytereceived);
void klstcframe(uint8_t channel);


