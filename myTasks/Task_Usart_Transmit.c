#include "cmsis_os2.h"
#include "bsp_usart.h"
#include "usart.h"
#include "MotorDriver.h"
#include <string.h>

#define System_Mode 3

#define Motor_Number 4 // 给单个舵机发送数据无需考虑 主要是给多个舵机同步发，需要改这个参数

uint8_t  ID1[Motor_Number]  = {1, 2, 3, 4};
uint8_t  ACC1[Motor_Number] = {20, 20, 20, 20};
int16_t  PST1[Motor_Number] = {2048, 2048, 2048, 2048};
uint16_t TQE1[Motor_Number] = {100, 100, 100, 100};
uint16_t SPD1[Motor_Number] = {20, 20, 20, 20};

uint8_t  ID2[Motor_Number]  = {1, 2, 3, 4};
uint8_t  ACC2[Motor_Number] = {20, 20, 20, 20};
int16_t  PST2[Motor_Number] = {2048, 2048, 2048, 2048};
uint16_t TQE2[Motor_Number] = {100, 100, 100, 100};
uint16_t SPD2[Motor_Number] = {20, 20, 20, 20};

uint8_t  ID3[Motor_Number]  = {1, 2, 3, 4};
uint8_t  ACC3[Motor_Number] = {20, 20, 20, 20};
int16_t  PST3[Motor_Number] = {2048, 2048, 2048, 2048};
uint16_t TQE3[Motor_Number] = {100, 100, 100, 100};
uint16_t SPD3[Motor_Number] = {20, 20, 20, 20};

uint8_t usart1_transmit_buff[40]; // 发送同步控制指令时，4个舵机需要40个字节的内存
uint8_t usart2_transmit_buff[40];
uint8_t usart3_transmit_buff[40];

extern uint8_t usart1_receive_buff[48];
extern uint8_t usart2_receive_buff[48];
extern uint8_t usart3_receive_buff[48];

uint32_t dicknumber = 0;

extern osMessageQueueId_t Queue_CanRxMsgHandle; // 来自can接收的控制队列，其中每个成员就是一个数组，里面有12个位置参数

extern Motor_Feedback_Data uart1_Motors[4];
extern Motor_Feedback_Data uart2_Motors[4];
extern Motor_Feedback_Data uart3_Motors[4];

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
	
	#if System_Mode == 1	// 标定
		// 开锁 掉电保存
		Motor_EpromSwitch_OutMode(&huart1, usart1_transmit_buff, 0);
		Motor_EpromSwitch_OutMode(&huart2, usart2_transmit_buff, 0);
		Motor_EpromSwitch_OutMode(&huart3, usart3_transmit_buff, 0);
		osDelay(100);
		// 标点
		Motor_PositionCalibration_Transmit(&huart1, usart1_transmit_buff);
		Motor_PositionCalibration_Transmit(&huart2, usart2_transmit_buff);
		Motor_PositionCalibration_Transmit(&huart3, usart3_transmit_buff);
		osDelay(100);
		// 关锁
		Motor_EpromSwitch_OutMode(&huart1, usart1_transmit_buff, 1);
		Motor_EpromSwitch_OutMode(&huart2, usart2_transmit_buff, 1);
		Motor_EpromSwitch_OutMode(&huart3, usart3_transmit_buff, 1);
		osDelay(100);
		// 打开扭矩输出
		Motor_Switch_OutMode(&huart1, usart1_transmit_buff, 1);
		Motor_Switch_OutMode(&huart2, usart2_transmit_buff, 1);
		Motor_Switch_OutMode(&huart3, usart3_transmit_buff, 1);
		osDelay(100);
	#elif System_Mode == 2	// 打开扭矩输出
		// 打开扭矩输出
		Motor_Switch_OutMode(&huart1, usart1_transmit_buff, 1);
		Motor_Switch_OutMode(&huart2, usart2_transmit_buff, 1);
		Motor_Switch_OutMode(&huart3, usart3_transmit_buff, 1);
		uint16_t target_position[12] = {0};
		osDelay(100);
	#elif System_Mode == 3	// 打开阻尼输出
		Motor_Switch_OutMode(&huart1, usart1_transmit_buff, 2);
		Motor_Switch_OutMode(&huart2, usart2_transmit_buff, 2);
		Motor_Switch_OutMode(&huart3, usart3_transmit_buff, 2);
		osDelay(100);
	#endif
	
  for(;;)
  {
		#if System_Mode == 1
			// 同步控制
			Motor_SyncControl_Transmit(&huart1,	ID1, ACC1, PST1, TQE1, SPD1, usart1_transmit_buff);
			Motor_SyncControl_Transmit(&huart2,	ID2, ACC2, PST2, TQE2, SPD2, usart2_transmit_buff);
			Motor_SyncControl_Transmit(&huart3,	ID3, ACC3, PST3, TQE3, SPD3, usart3_transmit_buff);
		#elif System_Mode == 2
			osStatus_t ControlMsgState = osMessageQueueGet(Queue_CanRxMsgHandle, target_position, 0, 0);
			if (ControlMsgState == osOK)
			{
				PST1[0] = target_position[0];
				PST1[1] = target_position[1];
				PST1[2] = target_position[2];
				PST1[3] = target_position[3];
				
				PST2[0] = target_position[4];
				PST2[1] = target_position[5];
				PST2[2] = target_position[6];
				PST2[3] = target_position[7];
				
				PST3[0] = target_position[8];
				PST3[1] = target_position[9];
				PST3[2] = target_position[10];
				PST3[3] = target_position[11];
			}
			// 同步控制
			Motor_SyncControl_Transmit(&huart1,	ID1, ACC1, PST1, TQE1, SPD1, usart1_transmit_buff);
			Motor_SyncControl_Transmit(&huart2,	ID2, ACC2, PST2, TQE2, SPD2, usart2_transmit_buff);
			Motor_SyncControl_Transmit(&huart3,	ID3, ACC3, PST3, TQE3, SPD3, usart3_transmit_buff);
		#elif System_Mode == 3
//			PST1[0] = uart1_Motors[0].Pos;
//			PST1[1] = uart1_Motors[1].Pos;
//			PST1[2] = uart1_Motors[2].Pos;
//			PST1[3] = uart1_Motors[3].Pos;
//			
//			PST2[0] = uart2_Motors[0].Pos;
//			PST2[1] = uart2_Motors[1].Pos;
//			PST2[2] = uart2_Motors[2].Pos;
//			PST2[3] = uart2_Motors[3].Pos;
//			
//			PST3[0] = uart3_Motors[0].Pos;
//			PST3[1] = uart3_Motors[1].Pos;
//			PST3[2] = uart3_Motors[2].Pos;
//			PST3[3] = uart3_Motors[3].Pos;
//			// 同步控制
//			Motor_SyncControl_Transmit(&huart1,	ID1, ACC1, PST1, TQE1, SPD1, usart1_transmit_buff);
//			Motor_SyncControl_Transmit(&huart2,	ID2, ACC2, PST2, TQE2, SPD2, usart2_transmit_buff);
//			Motor_SyncControl_Transmit(&huart3,	ID3, ACC3, PST3, TQE3, SPD3, usart3_transmit_buff);
		#endif
		
		// 请求返回数据
		osDelay(10);
		Motor_DataSyncRequest_Transmit(&huart1, ID1, usart1_transmit_buff);
		Motor_DataSyncRequest_Transmit(&huart2, ID2, usart2_transmit_buff);
		Motor_DataSyncRequest_Transmit(&huart3, ID3, usart3_transmit_buff);
		osDelay(10);
		
		dicknumber++;
  }
}










