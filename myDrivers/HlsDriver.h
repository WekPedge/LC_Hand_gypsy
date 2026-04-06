#ifndef HLSDRIVER_H
#define HLSDRIVER_H


#include "main.h"


void HLS_PositionCalibration_BuffWrite(uint8_t ID, uint16_t Position, uint8_t* Transmit_Buff);
void HLS_DataSyncRequest_Prepare(uint8_t* ID, uint8_t* Transmit_Buff);
void HLS_SyncControl_WriteBuff(uint8_t* ID, uint8_t* ACC, int16_t* Position, uint16_t* Torque, uint16_t* Speed, uint8_t* Transmit_Buff);
void HLS_CloseMutiRoundsMode_BuffWrite(uint8_t ID, uint8_t* Transmit_Buff);
void HLS_Control_BuffWrite(uint8_t ID, uint8_t addr, uint8_t *data, uint8_t data_len, uint8_t *Transmit_Buff);
void HLS_CloseRubbishFeedback_BuffWrite(uint8_t ID, uint8_t *Transmit_Buff);
void HLS_TorqueSwitch_BuffWrite(uint8_t ID, uint8_t *Transmit_Buff, uint8_t Mode);
void HLS_EpromSwitch_BuffWrite(uint8_t ID, uint8_t *Transmit_Buff, uint8_t Mode);










































#endif




