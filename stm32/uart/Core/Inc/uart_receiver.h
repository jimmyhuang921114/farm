#ifndef UART_RECEIVER_H
#define UART_RECEIVER_H
#include "protocol.h"
#include <stdbool.h>
#include "main.h"


HAL_StatusTypeDef UART_Receiver_Start(UART_HandleTypeDef *huart);

bool UART_Recevier_TAkeCommand(RobotCommand *command);

/* 放在 while (1) 持續呼叫 */
void UART_Receiver_Process(void);

/* 由 HAL_UART_RxCpltCallback 呼叫 */
void UART_Receiver_RxCallback(UART_HandleTypeDef *huart);

HAL_StatusTypeDef UART_Receiver_Send(const char *message);

#endif
