#include "ArmMessageDriver.h"
#include "bsp_can.h"
#include "can.h"
#include "cmsis_os2.h"


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
    // 采用底层极速的小端移位拼接法
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













































