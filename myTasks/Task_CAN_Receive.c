#include "Task_CAN_Receive.h"
#include "can.h"
#include "bsp_can.h"
#include "cmsis_os2.h"


// 队列
extern osMessageQueueId_t Queue_CanPackHandle;

// CAN数据包 包括拓展段和数据段
CAN_Pack_Struct CAN_Data_frmPack = {0};

void TaskCANRec(void *argument)
{
  for(;;)
  {
    osStatus_t QueueSign = osMessageQueueGet(Queue_CanPackHandle, &CAN_Data_frmPack, NULL, osWaitForever);
		if (QueueSign == osOK)
		{
			CAN_Data_Unpack(&CAN_Data_frmPack); // 解包好后会将从上位机收到的12个舵机的数据直接放进队列，在串口发送任务中取出来
		}
  }
}


























