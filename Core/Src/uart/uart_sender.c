#include "uart_sender.h"
#include <string.h>

// 保存主程式指定的 UART 傳送介面。
static UART_HandleTypeDef *sender_uart = NULL;


// 指定後續除錯訊息與控制回覆的傳送介面。
void UART_Sender_Init(UART_HandleTypeDef *huart)
{
    sender_uart = huart;
}


// 以阻塞方式傳送完整字串，逾時為 100 ms。
HAL_StatusTypeDef UART_Sender_Send(const char *message)
{
    if (sender_uart == NULL || message == NULL)
        return HAL_ERROR;

    size_t length = strlen(message);

    if (length == 0 || length > UINT16_MAX)
        return HAL_ERROR;

    return HAL_UART_Transmit(
        sender_uart,
        (uint8_t *)message,
        (uint16_t)length,
        100
    );
}
