#ifndef UART_SENDER_H
#define UART_SENDER_H

#include "main.h"

// 指定要使用的 UART 傳送介面。
void UART_Sender_Init(UART_HandleTypeDef *huart);

// 阻塞式傳送以 NUL 結尾的字串，回傳 HAL 傳送狀態。
HAL_StatusTypeDef UART_Sender_Send(const char *message);

#endif
