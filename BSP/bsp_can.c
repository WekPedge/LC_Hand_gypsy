#include "bsp_can.h"
#include "can.h"
#include "cmsis_os2.h"
#include <string.h>


// 队列
extern osMessageQueueId_t Queue_CanPackHandle;

/**
  * @brief  CAN 启动 (带安全重试机制)
  * @retval 0:成功, 1:失败
  */
uint8_t CAN_Enable(CAN_FilterTypeDef* canFilterConfig)
{
	uint8_t retry_count = 0;
	const uint8_t MAX_RETRY = 3; // 最多重试 3 次

	while (retry_count < MAX_RETRY)
	{
		// 应用滤波器配置
		if (HAL_CAN_ConfigFilter(&hcan, canFilterConfig) != HAL_OK) 
		{
			retry_count++;
			continue; 
		}
		// 启动 CAN 外设
		if (HAL_CAN_Start(&hcan) != HAL_OK) 
		{
			retry_count++;
			continue; 
		}
		// 激活接收中断
		if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) 
		{
			retry_count++;
			continue; 
		}
		// 如果三步都顺利通过了，直接返回成功
		return 0; 
	}
	// 如果循环结束了还没 return，说明重试 3 次都失败了
	// 可以在这里让硬件亮红灯
	return 1; 
}

/**
  * @brief  CAN 滤波器初始化、启动并开启接收中断
  */
void CAN_Filter_Init(void)
{
	CAN_FilterTypeDef canFilterConfig;

	// 配置滤波器：接收所有ID的报文 (掩码全为0代表不校验任何位)
	canFilterConfig.FilterBank = 0;                      // 使用滤波器组0 (STM32F103通常有0-13)
	canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  // 掩码模式
	canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // 32位宽
	canFilterConfig.FilterIdHigh = 0x0000;               // ID高位
	canFilterConfig.FilterIdLow = 0x0000;                // ID低位
	canFilterConfig.FilterMaskIdHigh = 0x0000;           // 掩码高位 (全0表示接收所有)
	canFilterConfig.FilterMaskIdLow = 0x0000;            // 掩码低位
	canFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; // 接收到的报文放入 FIFO0 中
	canFilterConfig.FilterActivation = ENABLE;           // 激活滤波器
	canFilterConfig.SlaveStartFilterBank = 14;           // 单 CAN 芯片此项无意义，填默认即可
	
	CAN_Enable(&canFilterConfig);
}

/**
  * @brief  CAN 标准发送
  * @param  hcan: CAN句柄 
  * @param  myStdId：can id
  * @param  tx_data: 要发送的数据缓冲区首地址 
  * @retval 1:发送成功, 0:发送失败 (邮箱满或硬件错误)
  */
uint8_t CAN_Simple_Transmit(CAN_HandleTypeDef *hcan, uint32_t myStdId, uint8_t *tx_data)
{
	CAN_TxHeaderTypeDef TxHeader; // 快递单
	uint32_t TxMailbox;           // 用于记录包裹被扔进了哪个发送邮箱

	// 配置发送参数
	TxHeader.StdId = myStdId;              // 标准帧 ID (0x000 ~ 0x7FF)
	TxHeader.ExtId = 0;           				 // 扩展帧 ID (这里不用，随便填)
	TxHeader.IDE = CAN_ID_STD;             // 标准帧 (CAN_ID_STD)
	TxHeader.RTR = CAN_RTR_DATA;           // 告诉硬件这是数据帧，区分数据帧和标准帧
	TxHeader.DLC = 8;                      // 告诉硬件数据长度为 8 个字节
	TxHeader.TransmitGlobalTime = DISABLE; // 关闭时间戳记录 (通常不用)

	// 包裹发
	// 参数：句柄、快递单地址、货物地址、记录邮箱的地址
	if (HAL_CAN_AddTxMessage(hcan, &TxHeader, tx_data, &TxMailbox) != HAL_OK)
	{
		// 如果三个发送邮箱都满了，或者外设报错，就会走到这里
		return 0; // 发送失败
	}
	
	return 1; // 发送成功
}

/**
  * @brief  CAN 拓展发送
  * @param  hcan: CAN句柄 
  * @param  myStdId：can id
  * @param  tx_data: 要发送的数据缓冲区首地址 
  * @retval 1:发送成功, 0:发送失败 (邮箱满或硬件错误)
  */
uint8_t CAN_Expand_Transmit(CAN_HandleTypeDef *hcan, uint32_t myExtId, uint8_t *tx_data)
{
	CAN_TxHeaderTypeDef TxHeader; // 快递单
	uint32_t TxMailbox;           // 用于记录包裹被扔进了哪个发送邮箱

	// 配置发送参数
	TxHeader.StdId = 0;              			 // 标准帧 ID (这里不用，随便填)
	TxHeader.ExtId = myExtId;           	 // 扩展帧 ID (29位)
	TxHeader.IDE = CAN_ID_EXT;             // 标准帧 (CAN_ID_STD)
	TxHeader.RTR = CAN_RTR_DATA;           // 告诉硬件这是数据帧，区分数据帧和遥控、错误、过载帧
	TxHeader.DLC = 8;                      // 告诉硬件数据长度为 8 个字节
	TxHeader.TransmitGlobalTime = DISABLE; // 关闭时间戳记录 (通常不用)

	// 包裹发
	// 参数：句柄、快递单地址、货物地址、记录邮箱的地址
	if (HAL_CAN_AddTxMessage(hcan, &TxHeader, tx_data, &TxMailbox) != HAL_OK)
	{
		// 如果三个发送邮箱都满了，或者外设报错，就会走到这里
		return 0; // 发送失败
	}
	
	return 1; // 发送成功
}

/**
  * @brief  CAN RX FIFO0 接收中断回调函数 (拆快递的地方)
  * @param  hcan: CAN 句柄
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan->Instance == CAN1)
	{
		// 接收到的数据
		CAN_RxHeaderTypeDef RxHeader; // 仲裁段 控制段
		// 入队数据包
		CAN_Pack_Struct CAN_Data_toPack = {0};
		// 从 FIFO0 中把数据拿出来
		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, CAN_Data_toPack.Data) == HAL_OK)
		{
			// 扩展帧
			if (RxHeader.IDE == CAN_ID_EXT)
			{
				CAN_Data_toPack.Extend_ID = RxHeader.ExtId;
				// 拓展帧和数据包入队
				osMessageQueuePut(Queue_CanPackHandle, &CAN_Data_toPack, 0, 0);
			}
			else if (RxHeader.IDE == CAN_ID_STD)
			{
				// 标准帧 没用上
			}  
		}
	}
}

























