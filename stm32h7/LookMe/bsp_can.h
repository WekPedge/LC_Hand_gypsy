#ifndef BSP_CAN_H
#define BSP_CAN_H


#include "main.h"



// 打包后的 CAN 帧数据
typedef struct {
    uint32_t ExtID;     // 29位扩展ID
    uint8_t  Data[8];   // 8字节数据包
} CAN_PackedPacket;

uint8_t CAN_Expand_Transmit(FDCAN_HandleTypeDef *hfdcan, uint32_t myExtId, uint8_t *tx_data);
CAN_PackedPacket Pack_Servo_Positions(uint32_t myID, uint16_t s1, uint16_t s2, uint16_t s3, uint16_t s4, uint16_t s5, uint16_t s6);
void CAN_Test_Send(void);


#endif




























