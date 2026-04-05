#include "cmsis_os2.h"
#include "bsp_usart.h"
#include "usart.h"
#include "MotorDriver.h"
#include <string.h>


#define Motor_Number 4 // 给单个舵机发送数据无需考虑 主要是给多个舵机同步发，需要改这个参数

uint8_t  ID1[Motor_Number]  = {1, 2, 3, 4};
uint8_t  ACC1[Motor_Number] = {20, 20, 20, 20};
int16_t  PST1[Motor_Number] = {2048, 2048, 2048, 2048};
uint16_t TQE1[Motor_Number] = {10, 10, 10, 10};
uint16_t SPD1[Motor_Number] = {50, 50, 50, 50};

uint8_t  ID2[Motor_Number]  = {1, 2, 3, 4};
uint8_t  ACC2[Motor_Number] = {20, 20, 20, 20};
int16_t  PST2[Motor_Number] = {2048, 2048, 2048, 2048};
uint16_t TQE2[Motor_Number] = {10, 10, 10, 10};
uint16_t SPD2[Motor_Number] = {50, 50, 50, 50};

uint8_t  ID3[Motor_Number]  = {1, 2, 3, 4};
uint8_t  ACC3[Motor_Number] = {20, 20, 20, 20};
int16_t  PST3[Motor_Number] = {2048, 2048, 2048, 2048};
uint16_t TQE3[Motor_Number] = {10, 10, 10, 10};
uint16_t SPD3[Motor_Number] = {50, 50, 50, 50};

uint8_t usart1_transmit_buff[40]; // 发送同步控制指令时，4个舵机需要40个字节的内存
uint8_t usart2_transmit_buff[40];
uint8_t usart3_transmit_buff[40];

extern uint8_t usart1_receive_buff[48];
extern uint8_t usart2_receive_buff[48];
extern uint8_t usart3_receive_buff[48];

uint32_t dicknumber = 0;

void TaskUsartTrans(void *argument)
{
  /* USER CODE BEGIN TaskUsartTrans */
	USART_DMA_Enable_ALL();
	memset(usart1_transmit_buff, 0, 40);
	memset(usart2_transmit_buff, 0, 40);
	memset(usart3_transmit_buff, 0, 40);
	memset(usart1_receive_buff, 0, 40);
	memset(usart2_receive_buff, 0, 40);
	memset(usart3_receive_buff, 0, 40);
	
	// 关闭垃圾信息回传
	Motor_CloseRubbishFeedback(&huart1, usart1_transmit_buff);
	Motor_CloseRubbishFeedback(&huart2, usart2_transmit_buff);
	Motor_CloseRubbishFeedback(&huart3, usart3_transmit_buff);
	// 关闭多圈模式
	Motor_CloseMutiRoundsMode_BuffRead(&huart1, usart1_transmit_buff);
	Motor_CloseMutiRoundsMode_BuffRead(&huart2, usart2_transmit_buff);
	Motor_CloseMutiRoundsMode_BuffRead(&huart3, usart3_transmit_buff);
	// 打开扭矩输出
	Motor_Switch_OutMode(&huart1, usart1_transmit_buff, 1);
	Motor_Switch_OutMode(&huart2, usart2_transmit_buff, 1);
	Motor_Switch_OutMode(&huart3, usart3_transmit_buff, 1);
	// 标定
//	Motor_PositionCalibration_Transmit(&huart1, usart1_transmit_buff);
//	Motor_PositionCalibration_Transmit(&huart2, usart2_transmit_buff);
//	Motor_PositionCalibration_Transmit(&huart3, usart3_transmit_buff);
	
  for(;;)
  {
		// 同步控制
		Motor_SyncControl_Transmit(&huart1,	ID1, ACC1, PST1, TQE1, SPD1, usart1_transmit_buff);
		Motor_SyncControl_Transmit(&huart2,	ID2, ACC2, PST2, TQE2, SPD2, usart2_transmit_buff);
		Motor_SyncControl_Transmit(&huart3,	ID3, ACC3, PST3, TQE3, SPD3, usart3_transmit_buff);
		// 请求返回数据
		osDelay(250);
		Motor_DataSyncRequest_Transmit(&huart1, ID1, usart1_transmit_buff);
		Motor_DataSyncRequest_Transmit(&huart2, ID2, usart2_transmit_buff);
		Motor_DataSyncRequest_Transmit(&huart3, ID3, usart3_transmit_buff);
		osDelay(250);
		
		dicknumber++;
  }
}










