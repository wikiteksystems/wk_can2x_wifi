#ifndef __MAIN_INC_CAN_IFACE_H__
#define __MAIN_INC_CAN_IFACE_H__


typedef enum {
    CAN_RX_RET_TIMEOUT = -127,
    CAN_RX_RET_MUTEX_ACQUIRE_FAIL,
    CAN_RX_RET_RTR_FRAME,
    CAN_RX_RET_INTERNAL_ERR,
    CAN_RX_RET_OK = 0
} CANRxRetType;


#endif // __MAIN_INC_CAN_IFACE_H__