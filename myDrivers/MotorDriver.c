#include "MotorDriver.h"
#include "HlsDriver.h"
#include "bsp_usart.h"
#include "usart.h"
#include "cmsis_os2.h"
#include <string.h>


#define Motor_Number 4 // 给单个舵机发送数据无需考虑 主要是给多个舵机同步发，需要改这个参数

/**
  * @brief  	把舵机的位置标定成2048 单圈模式下这就是中点
	* @para 		huart：					目标串口句柄
	* @para 		Transmit_Buff：	发送缓冲区
  * @retval		None
  */
void Motor_PositionCalibration_Transmit(UART_HandleTypeDef *huart, uint8_t* Transmit_Buff)
{
	for (int i = 0x01; i <= Motor_Number; i++)
	{
		HLS_PositionCalibration_BuffWrite(i, 2048, Transmit_Buff);
		USART_Transmit_DMA(huart, Transmit_Buff, 8);
		osDelay(10);
	}
}

/**
  * @brief  	发送请求帧 读取Motor_Number个舵机的数据
	* @para 		huart：					目标串口句柄
	* @para 		Transmit_Buff：	发送缓冲区
  * @retval 	None
  */
void Motor_DataSyncRequest_Transmit(UART_HandleTypeDef *huart, uint8_t* IDs, uint8_t* Transmit_Buff)
{
	HLS_DataSyncRequest_Prepare(IDs, Transmit_Buff);
	USART_Transmit_DMA(huart, Transmit_Buff, 8+Motor_Number); // 8为固定长度加上id数量
}

/**
  * @brief  	同步控制Motor_Number个舵机转动的指令发送
	* @para 		huart：					目标串口句柄
	* @para 		Transmit_Buff：	发送缓冲区
  * @retval 	None
  */ 
void Motor_SyncControl_Transmit(UART_HandleTypeDef *huart, uint8_t* IDs, uint8_t*  ACCs, int16_t*  PSTs, uint16_t*  TQEs, uint16_t*  SPDs,  uint8_t* Transmit_Buff)
{
	HLS_SyncControl_WriteBuff(IDs, ACCs, PSTs, TQEs, SPDs, Transmit_Buff);
	USART_Transmit_DMA(huart, Transmit_Buff, 12+Motor_Number*7);
}

/**
  * @brief  	轮询关闭四个舵机的多圈模式 把最大角限制改成4095
	* @para 		huart：					目标串口句柄
	* @para 		Transmit_Buff：	发送缓冲区
  * @retval 	None
  */
void Motor_CloseMutiRoundsMode_BuffRead(UART_HandleTypeDef *huart, uint8_t* Transmit_Buff)
{
	for (int id = 1; id <= 4; id++)
	{
		HLS_CloseMutiRoundsMode_BuffWrite(id, Transmit_Buff);
		USART_Transmit_DMA(huart, Transmit_Buff, 5+Motor_Number);
		osDelay(10);
	}
}

/**
  * @brief  	指令：控制单个舵机移动
	* @para 		huart：					目标串口句柄
	* @para 		ID：						标定舵机的ID
	* @para 		ACC:						目标加速度
	* @para 		Position：			目标位置
	* @para 		Torque：				目标扭矩
	* @para 		Speed：					目标速度
	* @para 		Transmit_Buff：	发送缓冲区
  * @retval 	None
  */
void Motor_Control_Transmit(UART_HandleTypeDef *huart, uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC, uint16_t Torque, uint8_t* Transmit_Buff)
{
	// 处理位置值（负数用最高位表示）
	if(Position < 0)
	{
		Position = -Position;
		Position |= (1 << 15);
	}	
	// 构建数据数组（小端模式：低字节在前）
	uint8_t data[7];
	data[0] = ACC;                           // 加速度
	data[1] = Position & 0xFF;               // 位置低字节
	data[2] = (Position >> 8) & 0xFF;        // 位置高字节
	data[3] = Torque & 0xFF;                 // 扭矩低字节
	data[4] = (Torque >> 8) & 0xFF;          // 扭矩高字节
	data[5] = Speed & 0xFF;                  // 速度低字节
	data[6] = (Speed >> 8) & 0xFF;           // 速度高字节	
	// 填充缓冲区
	HLS_Control_BuffWrite(ID, 0x29, data, 7, Transmit_Buff);
	// 发送指令
	USART_Transmit_DMA(huart, Transmit_Buff, 14);
}

/**
  * @brief  	指令：调整舵机反馈级别，使用此函数之后，只有读指令和ping指令才会返回数据
	* @para 		huart：					目标串口句柄
	* @para 		Transmit_Buff：	发送缓冲区
  * @retval 	None
  */
void Motor_CloseRubbishFeedback(UART_HandleTypeDef *huart, uint8_t* Transmit_Buff)
{
	for (uint8_t id = 1; id <= Motor_Number; id ++)
	{
		HLS_CloseRubbishFeedback_BuffWrite(id, Transmit_Buff);
		// 发送指令
		USART_Transmit_DMA(huart, Transmit_Buff, 8);
		osDelay(10);
	}
}

/**
	* @brief  	指令：调整舵机输出类型
	* @para 		huart：					目标串口句柄
	* @para 		Transmit_Buff：	发送缓冲区
	* @para 		Mode：					输出模式   写0：关闭扭力输出；写1：打开扭力输出；写2：阻尼输出
  * @retval 	None
  */
void Motor_Switch_OutMode(UART_HandleTypeDef *huart, uint8_t* Transmit_Buff, uint8_t Mode)
{
	for (uint8_t id = 1; id <= Motor_Number; id ++)
	{
		HLS_TorqueSwitch_BuffWrite(id, Transmit_Buff, Mode);
		USART_Transmit_DMA(huart, Transmit_Buff, 8);
		osDelay(10);
	}
}

/**
  * @brief  单舵机读指令反馈解包 (单包极速版)
  * @param  ReceiveDat: DMA接收缓冲区首地址 (首字节必定为 0xFF)
  * @param  MotorNumber: 该串口总线上挂载的舵机数量上限 (用于安全过滤)
  * @param  motorArray: 要装填数据的目标结构体数组
  * @retval 1: 解析成功并更新数据; 0: 解析失败(包头错/校验错/ID错)
  */
uint8_t Servo_Handle_FeedbackData(const uint8_t* ReceiveDat, uint8_t MotorNumber, Motor_Feedback_Data* motorArray)
{
	// 1. 严格校验包头 (如果不匹配，直接丢弃)
	if (ReceiveDat[0] != 0xFF || ReceiveDat[1] != 0xFF)
	{
		return 0; 
	}

	// 2. 提取 ID 并过滤
	uint8_t motorID = ReceiveDat[2];
	if (motorID < 1 || motorID > MotorNumber)
	{
		return 0; // ID 超出范围，丢弃
	}

	// 3. 计算并验证校验和
	uint8_t checkSum = 0;
	// 校验和范围是 [2] 到 [12]
	for (uint8_t j = 2; j <= 12; j++) 
	{
		checkSum += ReceiveDat[j];
	}
	checkSum = ~checkSum; 
	
	// 对比第13字节的校验和
	if (checkSum != ReceiveDat[13])
	{
		return 0; // 校验和错误，丢弃
	}

	// 4. 获取 Error 状态位 (0x00 表示硬件无故障)
	if (ReceiveDat[4] == 0x00)
	{
		// 通过 ID 直接映射到目标结构体
		Motor_Feedback_Data* pMotor = &motorArray[motorID - 1]; 
		
		// 解包位置 (Offset 5, 6)
		uint16_t rawPos = (uint16_t)(ReceiveDat[6] << 8 | ReceiveDat[5]);
		int16_t valPos = rawPos & 0x7FFF;
		pMotor->Pos = (rawPos & 0x8000) ? (-valPos) : (valPos);
		
		// 解包速度 (Offset 7, 8)
		int16_t servoSpeed = (int16_t)(ReceiveDat[8] << 8 | ReceiveDat[7]);
		if (servoSpeed >= 0) 
		{
			pMotor->Speed = servoSpeed;
		} 
		else 
		{
			pMotor->Speed = -(32768 + servoSpeed);
		}
		
		// 解包负载 (Offset 9, 10)
		pMotor->Load = (int16_t)(ReceiveDat[10] << 8 | ReceiveDat[9]);
		
		// 解包温度 (Offset 12)
		pMotor->Temp = ReceiveDat[12] / 10;
		
		return 1; // 成功解析了这 1 包数据
	}

	return 0; // 硬件有 Error 报错，不更新数据
}































