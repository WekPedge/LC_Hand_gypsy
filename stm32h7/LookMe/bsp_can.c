#include "bsp_can.h"
#include "fdcan.h"
#include <stdint.h>


typedef struct {
	uint32_t extID1_6;
	uint32_t extID2_12;
	uint8_t Data1_6[8];
	uint8_t Data7_12[8];
	uint8_t ReceiveFlag;
} LC_HandleTypeDef;


uint16_t Hand_Positions[12] = {0};
LC_HandleTypeDef HandMotor = { 0 };


extern uint16_t Hand_Positions[12]; // 确保能拿到外部定义的数组



void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	FDCAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[8];
	if(hfdcan->Instance == FDCAN1)
	{
		// 1. 判断是否是“FIFO 0 收到新消息”引发的中断
		if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
		{
			// 2. 从硬件邮箱 (FIFO0) 中将这帧报文的头部(ID等)和载荷(8字节)提取出来
			if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
			{
				// 3. 极严谨的安全过滤：确保收到的是 29 位的扩展帧 (Extended ID)
				if (RxHeader.IdType == FDCAN_EXTENDED_ID)
				{
					uint32_t extId = RxHeader.Identifier;
					// 提取扩展 ID 的高 13 位前缀
					uint32_t id_prefix = (extId >> 16) & 0x1FFF;

					if (id_prefix == 0x1FE) 
					{
						// --- 直接解包前 6 个舵机 (放进 0~5) ---
						Hand_Positions[0] = RxData[0] | ((RxData[1] & 0x0F) << 8);
						Hand_Positions[1] = ((RxData[1] >> 4) & 0x0F) | (RxData[2] << 4);
						Hand_Positions[2] = RxData[3] | ((RxData[4] & 0x0F) << 8);
						Hand_Positions[3] = ((RxData[4] >> 4) & 0x0F) | (RxData[5] << 4);
						Hand_Positions[4] = RxData[6] | ((extId & 0x0F) << 8);
						Hand_Positions[5] = (extId >> 4) & 0x0FFF;
					}
					else if (id_prefix == 0x1FF) 
					{
						// --- 直接解包后 6 个舵机 (放进 6~11) ---
						Hand_Positions[6] = RxData[0] | ((RxData[1] & 0x0F) << 8);
						Hand_Positions[7] = ((RxData[1] >> 4) & 0x0F) | (RxData[2] << 4);
						Hand_Positions[8] = RxData[3] | ((RxData[4] & 0x0F) << 8);
						Hand_Positions[9] = ((RxData[4] >> 4) & 0x0F) | (RxData[5] << 4);
						Hand_Positions[10] = RxData[6] | ((extId & 0x0F) << 8);
						Hand_Positions[11] = (extId >> 4) & 0x0FFF;
					}
				}
			}
		}
	}
}


/**
  * @brief  最简单的 CAN 发送测试函数
  */
void CAN_Test_Send(void)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    // 你要发的 1, 2, 3, 4, 5, 6, 7, 8
    uint8_t TxData[8] = {1, 2, 3, 4, 5, 6, 7, 8}; 

    // 1. 配置发送报文头 (经典 CAN 模式)
    TxHeader.Identifier = 0x123;                 // 随便起个标准帧 ID (比如 0x123)
    TxHeader.IdType = FDCAN_STANDARD_ID;         // 标准 ID (11位)
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;     // 数据帧
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;     // 数据长度 8 字节
    
    // FDCAN 特有的配置 (向下兼容经典 CAN 必须这么写)
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;      // 关闭波特率切换
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;       // 使用经典 CAN 格式
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    // 2. 将报文推入硬件发送队列
    // 注意：如果总线没有接其他设备(没收到 ACK)，硬件会自动无限重发，后续的发送会被阻塞
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK)
    {
        // 走到这里说明发送邮箱满了，或者总线出错了
        // 可以加个断点或者让灯闪烁提示
    }
}




/**
 * @brief  将6个舵机位置(12位)压缩至 29位ID 和 8字节数据包中
 * @param  s1~s6: 舵机1到6的位置值 (0 ~ 4095)
 * @retval CAN_PackedPacket 包含组装好的 ExtID 和 Data 数组
 */
CAN_PackedPacket Pack_Servo_Positions(uint32_t myID, uint16_t s1, uint16_t s2, uint16_t s3, uint16_t s4, uint16_t s5, uint16_t s6)
{
    CAN_PackedPacket packet = {0};

    // 1. 安全过滤：截掉所有高位废数据，确保数据绝对在 12位 (0x0FFF) 范围内
    s1 &= 0x0FFF;
    s2 &= 0x0FFF;
    s3 &= 0x0FFF;
    s4 &= 0x0FFF;
    s5 &= 0x0FFF;
    s6 &= 0x0FFF;

    // ---------------------------------------------------------
    // 2. 打包 8 字节数据区 (共需存放: 4个12位 + 1个8位 = 56位 = 7字节)
    // ---------------------------------------------------------
    
    // 【字节 1~3】：存放 舵机1(12位) 和 舵机2(12位)
    packet.Data[0] = s1 & 0xFF;                                // 舵机1 低8位
    packet.Data[1] = ((s1 >> 8) & 0x0F) | ((s2 & 0x0F) << 4);  // 舵机1 高4位 + 舵机2 低4位
    packet.Data[2] = (s2 >> 4) & 0xFF;                         // 舵机2 高8位

    // 【字节 4~6】：存放 舵机3(12位) 和 舵机4(12位)
    packet.Data[3] = s3 & 0xFF;                                // 舵机3 低8位
    packet.Data[4] = ((s3 >> 8) & 0x0F) | ((s4 & 0x0F) << 4);  // 舵机3 高4位 + 舵机4 低4位
    packet.Data[5] = (s4 >> 4) & 0xFF;                         // 舵机4 高8位

    // 【字节 7】：存放 舵机5的低8位
    packet.Data[6] = s5 & 0xFF;

    // 【字节 8】：补零闲置
    packet.Data[7] = 0x00; // 56个位恰好用满 7 个字节，第 8 字节空闲


    // ---------------------------------------------------------
    // 3. 打包 29 位扩展 ID
    // 结构要求：[前缀位] [舵机6(12位)] [舵机5高4位]
    // ---------------------------------------------------------
    
    uint8_t s5_high_4 = (s5 >> 8) & 0x0F; // 提取舵机5的 高4位

    // 扣除舵机 6 (12位) 和舵机 5 (4位) 后，高位前缀实际可以长达 13 位。
    // 这里我们填充 13 个 1 (即 0x1FFF) 作为最高位前缀标识。
    uint32_t id_prefix = myID; 

    // 组装 ID：将各部分左移到对应的“领地”并合并
    packet.ExtID = (id_prefix << 16) | (s6 << 4) | s5_high_4;

    return packet;
}

/**
  * @brief  FDCAN 拓展数据帧发送 (经典 CAN 模式)
  * @param  hfdcan: FDCAN句柄 
  * @param  myExtId: 扩展帧 ID (29位)
  * @param  tx_data: 要发送的数据缓冲区首地址 
  * @retval 1:发送成功, 0:发送失败 (队列满或硬件错误)
  */
uint8_t CAN_Expand_Transmit(FDCAN_HandleTypeDef *hfdcan, uint32_t myExtId, uint8_t *tx_data)
{
	FDCAN_TxHeaderTypeDef TxHeader; // FDCAN 专用的快递单

	// ================= 1. 基础配置 (原 bxCAN 也有的) =================
	TxHeader.Identifier = myExtId;                 // 扩展帧 ID (在 FDCAN 中标准和扩展 ID 共用这一个变量)
	TxHeader.IdType = FDCAN_EXTENDED_ID;           // 告诉硬件这是扩展帧 (对应原来的 IDE = CAN_ID_EXT)
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;       // 数据帧 (对应原来的 RTR = CAN_RTR_DATA)
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;       // 数据长度为 8 字节

	// ================= 2. FDCAN 专有配置 (为了兼容经典 CAN 必须写) =================
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE; // 默认主动错误状态
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;          // 关闭波特率切换 (CAN-FD 专有，经典 CAN 必须关)
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;           // ★核心：强制使用经典 CAN 2.0 格式！
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;// 不保存发送事件到事件队列
	TxHeader.MessageMarker = 0;                      // 消息标记 (不用管，填 0 即可)

	// ================= 3. 包裹入队 =================
	// 参数：句柄、快递单地址、货物地址 (注意：FDCAN 发送不需要我们传 Mailbox 地址了)
	if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, tx_data) != HAL_OK)
	{
		// 如果发送 FIFO 满了，或者外设报错，就会走到这里
		return 0; // 发送失败
	}
	
	return 1; // 发送成功
}




