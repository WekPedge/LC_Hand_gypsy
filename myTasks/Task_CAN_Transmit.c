#include "Task_CAN_Transmit.h"
#include "can.h"
#include "bsp_can.h"
#include "cmsis_os2.h"


uint8_t testbuf[8] = {1, 2, 3, 4, 5, 6, 7, 8};
void TaskCANTrans(void *argument)
{
	CAN_Filter_Init();
  for(;;)
  {
		CAN_Expand_Transmit(&hcan, 0x15555555, testbuf);
    osDelay(1);
  }
}























