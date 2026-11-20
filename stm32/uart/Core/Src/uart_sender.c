#include "uart_sender.h"
#include <string.h>

static UART_HandleTypeDef *sender_uart = NULL;


void UART_Sender_Init(UART_HandleTypeDef *huart)
{
    sender_uart = huart;
}


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
