
#include <stdint.h>

#define TXD_PIN GPIO_NUM_1
#define RXD_PIN GPIO_NUM_3


#define UART UART_NUM_0
//#define UART_BAUD 115200

#define UART_BAUD 460800


void initialise_uart(void);
void usbrxtask();

void senddatatouart(uint8_t *data, uint16_t len);


