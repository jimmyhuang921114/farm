#ifndef UART_SENDER_H
#define UART_SENDER_H

#include "main.h"

void UART_Sender_Init(UART_HandleTypeDef *huart);

HAL_StatusTypeDef UART_Sender_Send(const char *message);

#endif
