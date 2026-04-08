#include "Task_CAN_Transmit.h"
#include "can.h"
#include "bsp_can.h"
#include "cmsis_os2.h"
#include "MotorDriver.h"
#include "ArmMessageDriver.h"


// 马达数据内存可读通知
#define FLAG_MOTOR1_DATA_READY  0x0001
#define FLAG_MOTOR2_DATA_READY  0x0002
#define FLAG_MOTOR3_DATA_READY  0x0004

// 马达数据
extern Motor_Feedback_Data uart1_Motors[4];
extern Motor_Feedback_Data uart2_Motors[4];
extern Motor_Feedback_Data uart3_Motors[4];

uint16_t Motor_Position_Data[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

CAN_PackedPacket Motor1_6;
CAN_PackedPacket Motor7_12;

void TaskCANTrans(void *argument)
{
	CAN_Filter_Init();
  for(;;)
  {
		uint32_t flags = osThreadFlagsWait(FLAG_MOTOR1_DATA_READY | FLAG_MOTOR2_DATA_READY | FLAG_MOTOR3_DATA_READY, osFlagsWaitAny, 1); // 查通知
		
		// 根据通知抓包位置值
		if ((flags & FLAG_MOTOR1_DATA_READY) == FLAG_MOTOR1_DATA_READY)
		{
			Motor_Position_Data[0] = uart1_Motors[0].Pos;
			Motor_Position_Data[1] = uart1_Motors[1].Pos;
			Motor_Position_Data[2] = uart1_Motors[2].Pos;
			Motor_Position_Data[3] = uart1_Motors[3].Pos;
		}
		if ((flags & FLAG_MOTOR2_DATA_READY) == FLAG_MOTOR2_DATA_READY)
		{
			Motor_Position_Data[4] = uart2_Motors[0].Pos;
			Motor_Position_Data[5] = uart2_Motors[1].Pos;
			Motor_Position_Data[6] = uart2_Motors[2].Pos;
			Motor_Position_Data[7] = uart2_Motors[3].Pos;
		}
		if ((flags & FLAG_MOTOR3_DATA_READY) == FLAG_MOTOR3_DATA_READY)
		{
			Motor_Position_Data[8]  = uart3_Motors[0].Pos;
			Motor_Position_Data[9]  = uart3_Motors[1].Pos;
			Motor_Position_Data[10] = uart3_Motors[2].Pos;
			Motor_Position_Data[11] = uart3_Motors[3].Pos;
		}
		
		Motor1_6 = Pack_Servo_Positions(0x1FE, Motor_Position_Data[0], Motor_Position_Data[1], Motor_Position_Data[2], Motor_Position_Data[3], Motor_Position_Data[4], Motor_Position_Data[5]);
		CAN_Expand_Transmit(&hcan, Motor1_6.ExtID, Motor1_6.Data);
		Motor7_12 = Pack_Servo_Positions(0x1FF, Motor_Position_Data[6], Motor_Position_Data[7], Motor_Position_Data[8], Motor_Position_Data[9], Motor_Position_Data[10], Motor_Position_Data[11]);
		CAN_Expand_Transmit(&hcan, Motor7_12.ExtID, Motor7_12.Data);
		
    osDelay(5);
  }
}























