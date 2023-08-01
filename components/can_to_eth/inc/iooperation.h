// #ifndef __MAIN_INC_IOOPERATION_H__
// #define __MAIN_INC_IOOPERATION_H__


#define SW1     GPIO_NUM_35
#define LED1    GPIO_NUM_18
#define LED2    GPIO_NUM_19
#define LED3    GPIO_NUM_27
#define LED4    GPIO_NUM_26
#define LED5    GPIO_NUM_25


#define HIGH 1
#define LOW 0


void iointtask(void);
void OTALEDTASK(void *pv);
// #endif