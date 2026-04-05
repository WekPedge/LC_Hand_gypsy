#include "HlsDriver.h"
#include "bsp_usart.h"
#include "usart.h"


#define Motor_Number 4 // 给单个舵机发送数据无需考虑 主要是给多个舵机同步发，需要改这个参数

/**
  * @brief  	舵机位置标定指令准备
	* @para 		ID：						标定舵机的ID
	* @para 		Position：			要标定的位置
	* @para 		Transmit_Buff：	发送缓冲区
  * @retval 	None
  */
void HLS_PositionCalibration_BuffWrite(uint8_t ID, uint16_t Position, uint8_t* Transmit_Buff)
{
	Transmit_Buff[0] = 0xFF; // 包头
	Transmit_Buff[1] = 0xFF; // 包头
	Transmit_Buff[2] = ID;   // 舵机id
	Transmit_Buff[3] = 0x04; // 数据长度，此为固定值
	Transmit_Buff[4] = 0x0B; // 功能码
	Transmit_Buff[5] = Position & 0xFF; // 位置 低八位
	Transmit_Buff[6] = (Position >> 8) & 0xFF;      // 位置 高八位
	uint8_t CheckSum = 0xFF;
	for (int i = 2; i <= 6; i++)
	{
		CheckSum -= Transmit_Buff[i];
	}
	Transmit_Buff[7] = CheckSum; // 校验和
}

/**
  * @brief  	多舵机同步读指令准备
	* @para 		ID：						标定舵机的ID
	* @para 		Transmit_Buff：	发送缓冲区
  * @retval 	None
  */
void HLS_DataSyncRequest_Prepare(uint8_t* ID, uint8_t* Transmit_Buff)
{
	Transmit_Buff[0] = 0xFF; // 包头
	Transmit_Buff[1] = 0xFF; // 包头
	Transmit_Buff[2] = 0xFE; // 广播ID
	Transmit_Buff[3] = Motor_Number + 4; // 数据长度
	Transmit_Buff[4] = 0x82; // 指令
	Transmit_Buff[5] = 0x38; // 数据首地址 
	Transmit_Buff[6] = 0x08; // 请求8个字节 位置速度负载电压温度
	for (uint8_t i = 0; i < Motor_Number; i++)
	{
		Transmit_Buff[7 + i] = ID[i];
	}
	uint8_t sum = 0;
	for (uint8_t i = 2; i < 7+Motor_Number; i++)
	{
		sum += Transmit_Buff[i];
	}
	Transmit_Buff[7+Motor_Number] = ~sum; // 校验和位置
}

/**
  * @brief  	多舵机同步写指令准备
	* @para 		ID：						标定舵机的ID
	* @para 		ACC:						目标加速度
	* @para 		Position：			目标位置
	* @para 		Torque：				目标扭矩
	* @para 		Speed：					目标速度
	* @para 		Transmit_Buff：	发送缓冲区
  * @retval 	None
  */
void HLS_SyncControl_WriteBuff(uint8_t* ID, uint8_t* ACC, int16_t* Position, uint16_t* Torque, uint16_t* Speed, uint8_t* Transmit_Buff)
{
    const uint8_t L = 7; // 每个舵机写7个字节数据 (ACC(1)+Pos(2)+Tor(2)+Spd(2))
    
    Transmit_Buff[0] = 0xFF; // 包头
    Transmit_Buff[1] = 0xFF; // 包头
    Transmit_Buff[2] = 0xFE; // 广播ID
    Transmit_Buff[3] = (L + 1) * Motor_Number + 4; // 长度 0x44
    Transmit_Buff[4] = 0x83; // 指令码
    Transmit_Buff[5] = 0x29; // 起始地址
    Transmit_Buff[6] = L;    // 数据长度 0x07

    for (uint8_t i = 0; i < Motor_Number; i++)
    {
        uint8_t base = 7 + i * (L + 1); // 每个块起始位置
        Transmit_Buff[base + 0] = ID[i];
        Transmit_Buff[base + 1] = ACC[i];
        Transmit_Buff[base + 2] = Position[i] & 0xFF;
        Transmit_Buff[base + 3] = (Position[i] >> 8) & 0xFF;
        Transmit_Buff[base + 4] = Torque[i] & 0xFF;
        Transmit_Buff[base + 5] = (Torque[i] >> 8) & 0xFF;
        Transmit_Buff[base + 6] = Speed[i] & 0xFF;
        Transmit_Buff[base + 7] = (Speed[i] >> 8) & 0xFF;
    }

    // 自动计算校验和结束位置
    uint8_t checkSumIdx = 7 + Motor_Number * (L + 1); 
    uint8_t sum = 0;
    for (uint8_t i = 2; i < checkSumIdx; i++)
    {
        sum += Transmit_Buff[i];
    }
    Transmit_Buff[checkSumIdx] = ~sum; 
}

/**
  * @brief  	关闭多圈模式指令准备
	* @para 		ID：						标定舵机的ID
	* @para 		Transmit_Buff：	发送缓冲区
  * @retval 	None
  */
void HLS_CloseMutiRoundsMode_BuffWrite(uint8_t ID, uint8_t* Transmit_Buff)
{
	Transmit_Buff[0] = 0xFF; // 包头
	Transmit_Buff[1] = 0xFF; // 包头
	Transmit_Buff[2] = ID;   // 舵机id号
	Transmit_Buff[3] = 0x05; // 长度
	Transmit_Buff[4] = 0x03; // 指令
	Transmit_Buff[5] = 0x0B; // 最大角度值地址
	Transmit_Buff[6] = 0xFF; // 最大角度值
	Transmit_Buff[7] = 0x0F; // 最大角度值 
	uint8_t checkSum = 0xFF; // 校验和
	for (int i = 2; i <= 7; i++)
	{
		checkSum -= Transmit_Buff[i];
	}
	Transmit_Buff[8] = checkSum;
}

/**
  * @brief  	单舵机移动控制指令准备
	* @para 		ID：						舵机ID
	* @para 		addr：					写入地址
	* @para 		data：					数据包(加速度、位置、力矩、速度）
	* @para 		data_len：			写入字节数
	* @para 		Transmit_Buff：	发送缓冲区地址
  * @retval 	None
  */
void HLS_Control_BuffWrite(uint8_t ID, uint8_t addr, uint8_t *data, uint8_t data_len, uint8_t *Transmit_Buff)
{
	// 计算数据长度（从ID到数据结束的总字节数，包括功能码和地址）
	uint8_t length = 3 + data_len; // 功能码(1) + 地址(1) + 数据长度(1)
	Transmit_Buff[0] = 0xFF;        // 包头
	Transmit_Buff[1] = 0xFF;        // 包头
	Transmit_Buff[2] = ID;          // 舵机ID
	Transmit_Buff[3] = length;      // 数据长度
	Transmit_Buff[4] = 0x03;        // 功能码：写指令
	Transmit_Buff[5] = addr;        // 起始地址
	// 复制数据
	for (int i = 0; i < data_len; i++)
	{
		Transmit_Buff[6 + i] = data[i];
	}
	// 计算校验和（从ID开始到数据结束的和，然后取反）
	uint8_t CheckSum = 0;
	for (int i = 2; i < (6 + data_len); i++)
	{
		CheckSum += Transmit_Buff[i];
	}
	Transmit_Buff[6 + data_len] = ~CheckSum; // 校验和
}

/**
  * @brief  	调整舵机反馈级别指令准备
	* @para 		ID：						标定舵机的ID
	* @para 		Transmit_Buff：	发送缓冲区
  * @retval 	None
  */
void HLS_CloseRubbishFeedback_BuffWrite(uint8_t ID, uint8_t *Transmit_Buff)
{
	Transmit_Buff[0] = 0xFF;
	Transmit_Buff[1] = 0xFF;
	Transmit_Buff[2] = ID;
	Transmit_Buff[3] = 0x04;
	Transmit_Buff[4] = 0x03;
	Transmit_Buff[5] = 0x08;
	Transmit_Buff[6] = 0x00;
	uint8_t checkSum = 0xFF;
	for (uint8_t i = 2; i <=6; i++)
	{
		checkSum -= Transmit_Buff[i];
	}
	Transmit_Buff[7] = checkSum;
}

/**
  * @brief  	调整舵机扭矩输出模式指令准备
	* @para 		ID：						标定舵机的ID
	* @para 		Transmit_Buff：	发送缓冲区
	* @para 		Mode：					扭矩输出模式
  * @retval 	None
  */
void HLS_TorqueSwitch_BuffWrite(uint8_t ID, uint8_t *Transmit_Buff, uint8_t Mode)
{
	Transmit_Buff[0] = 0xFF;
	Transmit_Buff[1] = 0xFF;
	Transmit_Buff[2] = ID;
	Transmit_Buff[3] = 0x04;
	Transmit_Buff[4] = 0x03;
	Transmit_Buff[5] = 0x28;
	Transmit_Buff[6] = Mode;
	uint8_t checkSum = 0xFF;
	for (uint8_t i = 2; i <=6; i++)
	{
		checkSum -= Transmit_Buff[i];
	}
	Transmit_Buff[7] = checkSum;
}


















