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
    osMessageQueueGet(Queue_CanPackHandle, &CAN_Data_frmPack, NULL, osWaitForever);
		CAN_Data_Unpack(&CAN_Data_frmPack);
  }
}


























