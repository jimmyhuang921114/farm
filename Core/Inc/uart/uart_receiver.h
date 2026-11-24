#ifndef UART_RECEIVER_H
#define UART_RECEIVER_H
#include "protocol.h"
#include <stdbool.h>
#include "main.h"


// 清空接收狀態後啟用 UART 接收中斷。
HAL_StatusTypeDef UART_Receiver_Start(UART_HandleTypeDef *huart);

// 有待處理指令時複製到 command 並回傳 true。
bool UART_Receiver_TakeCommand(RobotCommand *command);

// 放在 while (1) 持續呼叫
void UART_Receiver_Process(void);

// 由 HAL_UART_RxCpltCallback 呼叫
void UART_Receiver_RxCallback(UART_HandleTypeDef *huart);

// 由 HAL UART 錯誤回呼通知接收器重新同步。
void UART_Receiver_ErrorCallback(UART_HandleTypeDef *huart);

#endif
