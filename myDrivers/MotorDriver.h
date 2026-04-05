#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H


#include "main.h"

typedef struct{
	uint16_t  Pos;
	int16_t 	Speed;
	int16_t 	Load;
	uint8_t		Vol;
	uint8_t		Temp;
}Motor_Feedback_Data;


void Motor_PositionCalibration_Transmit(UART_HandleTypeDef *huart, uint8_t* Transmit_Buff);
void Motor_DataSyncRequest_Transmit(UART_HandleTypeDef *huart, uint8_t* IDs, uint8_t* Transmit_Buff);
void Motor_SyncControl_Transmit(UART_HandleTypeDef *huart, uint8_t* IDs, uint8_t*  ACCs, int16_t*  PSTs, uint16_t*  TQEs, uint16_t*  SPDs,  uint8_t* Transmit_Buff);
void Motor_CloseMutiRoundsMode_BuffRead(UART_HandleTypeDef *huart, uint8_t* Transmit_Buff);
void Motor_Control_Transmit(UART_HandleTypeDef *huart, uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC, uint16_t Torque, uint8_t* Transmit_Buff);
void Motor_CloseRubbishFeedback(UART_HandleTypeDef *huart, uint8_t* Transmit_Buff);
void Motor_Switch_OutMode(UART_HandleTypeDef *huart, uint8_t* Transmit_Buff, uint8_t Mode);
uint8_t Servo_Handle_FeedbackData(const uint8_t* ReceiveDat, uint8_t MotorNumber, Motor_Feedback_Data* motorArray);














































#endif


