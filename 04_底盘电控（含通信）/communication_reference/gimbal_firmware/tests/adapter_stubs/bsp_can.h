#ifndef TRACKING_TEST_CAN_H
#define TRACKING_TEST_CAN_H
#include <stdint.h>
#include "main.h"
typedef struct { int id; } CAN_HandleTypeDef;
extern CAN_HandleTypeDef hcan1, hcan2;
typedef struct CANInstance CANInstance;
struct CANInstance {
    CAN_HandleTypeDef *can_handle;
    uint32_t tx_id, tx_mailbox;
    uint8_t tx_buff[8], rx_buff[8];
    uint32_t rx_id;
    uint8_t rx_len;
    void (*can_module_callback)(CANInstance *);
    void *id;
};
typedef struct {
    CAN_HandleTypeDef *can_handle;
    uint32_t tx_id, rx_id;
    void (*can_module_callback)(CANInstance *);
    void *id;
} CAN_Init_Config_s;
CANInstance *CANRegister(CAN_Init_Config_s *config);
void CANTransmit(CANInstance *instance, uint8_t length);
#endif
