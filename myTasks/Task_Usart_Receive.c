#include "cmsis_os2.h"
#include "usart.h"
#include "bsp_usart.h"
#include "main.h"
#include "Task_Usart_Receive.h"
#include "MotorDriver.h"

#define Motor_Number 4 // 给单个舵机发送数据无需考虑 主要是给多个舵机同步发，需要改这个参数

#define FLAG_USART1_RX_READY  0x0001
#define FLAG_USART2_RX_READY  0x0002
#define FLAG_USART3_RX_READY  0x0004

extern uint8_t usart1_receive_buff[48];
extern uint8_t usart2_receive_buff[48];
extern uint8_t usart3_receive_buff[48];

uint8_t count1 = 0;
uint8_t count2 = 0;
uint8_t count3 = 0;

Motor_Feedback_Data uart1_Motors[4] = {0};
Motor_Feedback_Data uart2_Motors[4] = {0};
Motor_Feedback_Data uart3_Motors[4] = {0};

void TaskUsartRec(void *argument)
{
  /* USER CODE BEGIN TaskUsartRec */
  /* Infinite loop */
  for(;;)
  {
		// 参数1：期望等待的标志位掩码
		// 参数2：osFlagsWaitAny 表示只要指定的掩码中有任何一位被置位就唤醒
		// 参数3：超时时间，osWaitForever 表示死等，直到有通知
    uint32_t flags = osThreadFlagsWait(FLAG_USART1_RX_READY | FLAG_USART2_RX_READY | FLAG_USART3_RX_READY, osFlagsWaitAny, osWaitForever);
		
		if ((flags & FLAG_USART1_RX_READY) == FLAG_USART1_RX_READY)
		{
			/* 串口一接收后的解包处理 */
			count1 ++;
			Servo_Handle_FeedbackData(usart1_receive_buff, Motor_Number, uart1_Motors);
			USART_DMA_Enable(&huart1, usart1_receive_buff, sizeof(usart1_receive_buff));
		}
		if ((flags & FLAG_USART2_RX_READY) == FLAG_USART2_RX_READY)
		{
			/* 串口二接收后的解包处理 */
			count2 ++;
			Servo_Handle_FeedbackData(usart2_receive_buff, Motor_Number, uart2_Motors);
			USART_DMA_Enable(&huart2, usart2_receive_buff, sizeof(usart2_receive_buff));
		}
		if ((flags & FLAG_USART3_RX_READY) == FLAG_USART3_RX_READY)
		{
			/* 串口三接收后的解包处理 */
			count3 ++;
			Servo_Handle_FeedbackData(usart3_receive_buff, Motor_Number, uart3_Motors);
			USART_DMA_Enable(&huart3, usart3_receive_buff, sizeof(usart3_receive_buff));
		}
  }
  /* USER CODE END TaskUsartRec */
}


