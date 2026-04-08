#include "bsp_uart.h"
#include "usart.h"


// 定义接收缓冲区 (稍微开大一点)
uint8_t rx_buffer[100]; 
uint16_t Target_Positions[12]; // 你最终要用的数组

uint8_t rx_data;

// 解包状态机枚举
typedef enum {
    STATE_WAIT_HEADER_1,
    STATE_WAIT_HEADER_2,
    STATE_RECEIVE_PAYLOAD,
    STATE_WAIT_CHECKSUM
} UnpackState;

void Parse_UART_Data(uint8_t byte_in) {
    static UnpackState state = STATE_WAIT_HEADER_1;
    static uint8_t payload_buffer[24];
    static uint8_t payload_index = 0;
    static uint8_t calculated_checksum = 0;

    switch (state) {
        case STATE_WAIT_HEADER_1:
            if (byte_in == 0xAA) {
                state = STATE_WAIT_HEADER_2;
            }
            break;

        case STATE_WAIT_HEADER_2:
            if (byte_in == 0x55) {
                state = STATE_RECEIVE_PAYLOAD;
                payload_index = 0;
                calculated_checksum = 0; // 清空校验和准备计算
            } else {
                state = STATE_WAIT_HEADER_1; // 头不对，重新找
            }
            break;

        case STATE_RECEIVE_PAYLOAD:
            payload_buffer[payload_index++] = byte_in;
            calculated_checksum += byte_in; // 一边接收一边累加校验和
            
            if (payload_index >= 24) {
                state = STATE_WAIT_CHECKSUM; // 收满 24 字节，等校验位
            }
            break;

        case STATE_WAIT_CHECKSUM:
            // 校验码比对
            if (byte_in == calculated_checksum) {
                // ✅ 校验成功！将收到的 byte 还原为 uint16_t 并赋值给你的全局变量
                for (int i = 0; i < 12; i++) {
                    Target_Positions[i] = (payload_buffer[i * 2 + 1] << 8) | payload_buffer[i * 2];
                }
                // 在这里可以触发舵机运动的标志位
            } 
            // 无论成功还是失败，都回到初始状态，准备接下一帧
            state = STATE_WAIT_HEADER_1; 
            break;
    }
}



















