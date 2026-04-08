#include "bsp_usart.h"
#include "usart.h"
#include "cmsis_os2.h"


extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

uint8_t usart1_receive_buff[48];
uint8_t usart2_receive_buff[48];
uint8_t usart3_receive_buff[48];

#define FLAG_USART1_RX_READY  0x0001
#define FLAG_USART2_RX_READY  0x0002
#define FLAG_USART3_RX_READY  0x0004

extern osThreadId_t Task_Usart_RecHandle;


/**
  * @brief  串口接收事件回调函数 (包含 IDLE 空闲中断)
  * @param  huart: 串口句柄
  * @param  Size:  本次实际接收到的数据字节数
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART1)
	{
		osThreadFlagsSet(Task_Usart_RecHandle, FLAG_USART1_RX_READY); // 第0位
	}
	if (huart->Instance == USART2)
	{
		osThreadFlagsSet(Task_Usart_RecHandle, FLAG_USART2_RX_READY); // 第1位
	}
	if (huart->Instance == USART3)
	{
		osThreadFlagsSet(Task_Usart_RecHandle, FLAG_USART3_RX_READY); // 第2位
	}
}

/**
  * @brief  串口发送完成中断
  * @param  huart: 串口句柄
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		// 此时数据已经在物理总线上发送完毕，安全切回接收模式，等待舵机应答
		HAL_HalfDuplex_EnableReceiver(&huart1);
		// 切回接收后，立刻重新开启 DMA 接收，准备抓取舵机的返回数据
		USART_DMA_Enable(&huart1, usart1_receive_buff, sizeof(usart1_receive_buff));
	}
	if(huart->Instance == USART2)
	{
		// 此时数据已经在物理总线上发送完毕，安全切回接收模式，等待舵机应答
		HAL_HalfDuplex_EnableReceiver(&huart2);
		// 切回接收后，立刻重新开启 DMA 接收，准备抓取舵机的返回数据
		USART_DMA_Enable(&huart2, usart2_receive_buff, sizeof(usart2_receive_buff));
	}
	if(huart->Instance == USART3)
	{
		// 此时数据已经在物理总线上发送完毕，安全切回接收模式，等待舵机应答
		HAL_HalfDuplex_EnableReceiver(&huart3);
		// 切回接收后，立刻重新开启 DMA 接收，准备抓取舵机的返回数据
		USART_DMA_Enable(&huart3, usart3_receive_buff, sizeof(usart3_receive_buff));
	}
}

/**
  * @brief  串口发送
  * @param  huart: 串口句柄
	* @param  pData: 缓冲区地址
	* @param  size:  内存长度
  */
void USART_Transmit_DMA(UART_HandleTypeDef *huart, const uint8_t *pData, uint8_t size)
{
	// 切换为发送模式
	HAL_HalfDuplex_EnableTransmitter(huart);
	// 启动 DMA 发送
	HAL_UART_Transmit_DMA(huart, pData, size);
}

/**
  * @brief  DMA使能
  * @param  huart: 串口句柄
	* @param  buff:  缓冲区地址
	* @param  size:  内存长度
  */
void USART_DMA_Enable(UART_HandleTypeDef *huart, uint8_t *pData, uint8_t size)
{
	HAL_UARTEx_ReceiveToIdle_DMA(huart, pData, size);
	__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}

/**
	* @brief  DMA使能所有电机
  */
void USART_DMA_Enable_ALL(void)
{
	USART_DMA_Enable(&huart1, usart1_receive_buff, sizeof(usart1_receive_buff));
	USART_DMA_Enable(&huart2, usart2_receive_buff, sizeof(usart2_receive_buff));
	USART_DMA_Enable(&huart3, usart3_receive_buff, sizeof(usart3_receive_buff));
}













