#ifndef BSP_CAN_H
#define BSP_CAN_H


#include "main.h"

typedef struct
{
	uint32_t Extend_ID;
	uint8_t Data[8];
}CAN_Pack_Struct;


void CAN_Filter_Init(void);
uint8_t CAN_Simple_Transmit(CAN_HandleTypeDef *hcan, uint32_t myStdId, uint8_t *tx_data);
uint8_t CAN_Expand_Transmit(CAN_HandleTypeDef *hcan, uint32_t myExtId, uint8_t *tx_data);
void CAN_Data_Unpack(CAN_Pack_Struct* canPack);
































#endif



