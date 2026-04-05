#ifndef BSP_USART_H
#define BSP_USART_H


#include "main.h"






void USART_Transmit_DMA(UART_HandleTypeDef *huart, const uint8_t *pData, uint8_t size);
void USART_DMA_Enable(UART_HandleTypeDef *huart, uint8_t *pData, uint8_t size);
void USART_DMA_Enable_ALL(void);
















































#endif

