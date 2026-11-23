#include "uart_receiver.h"
#include "protocol.h"

#define RX_BUFFER_SIZE 512U
#define LINE_BUFFER_SIZE 256U

static UART_HandleTypeDef *uart_handle = NULL;

static uint8_t rx_byte;
static RobotCommand pending_command;
static bool command_ready = false;



/* 中斷寫入，主迴圈讀取 */
static volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;
static volatile uint8_t rx_overflow = 0;

/* 僅由主迴圈使用 */
static char line_buffer[LINE_BUFFER_SIZE];
static uint16_t line_length = 0;
static uint8_t discard_line = 0;


HAL_StatusTypeDef UART_Receiver_Start(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
        return HAL_ERROR;

    uart_handle = huart;

    rx_head = 0;
    rx_tail = 0;
    rx_overflow = 0;

    line_length = 0;
    discard_line = 0;
    command_ready = false;

    return HAL_UART_Receive_IT(
        uart_handle,
        &rx_byte,
        1
    );
}


void UART_Receiver_RxCallback(UART_HandleTypeDef *huart)
{
    if (uart_handle == NULL || huart != uart_handle)
        return;

    uint16_t next_head =
        (uint16_t)((rx_head + 1U) % RX_BUFFER_SIZE);

    if (next_head != rx_tail)
    {
        rx_buffer[rx_head] = rx_byte;
        rx_head = next_head;
    }
    else
    {
        rx_overflow = 1;
    }

    /* 接收完一個 byte 後，重新啟動接收 */
    HAL_UART_Receive_IT(
        uart_handle,
        &rx_byte,
        1
    );
}


void UART_Receiver_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != uart_handle || uart_handle == NULL)
        return;

    rx_overflow = 1;
    if (huart->RxState == HAL_UART_STATE_READY)
        HAL_UART_Receive_IT(huart, &rx_byte, 1);
}


static void Handle_Line(const char *line)
{
    RobotCommand command;

    if (!Protocol_Parse(line, &command))
    {
        /* 收到無效封包，交給主程式執行停止 */
        command = (RobotCommand){0};
        command.type = COMMAND_STOP;
    }

    pending_command = command;
    command_ready = true;
}



void UART_Receiver_Process(void)
{
    /*
     * 若環形緩衝區溢位，清除不完整資料，
     * 並等待下一個換行重新同步。
     */
    if (command_ready)
        return;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint8_t overflow = rx_overflow;

    if (overflow)
    {
        rx_tail = rx_head;
        rx_overflow = 0;
    }

    uint16_t head = rx_head;
    __set_PRIMASK(primask);

    if (overflow)
    {
        line_length = 0;
        discard_line = 1;
    }

    while (rx_tail != head && !command_ready)
    {
        uint8_t byte = rx_buffer[rx_tail];

        rx_tail =
            (uint16_t)((rx_tail + 1U) % RX_BUFFER_SIZE);

        /* 換行代表一個完整封包結束 */
        if (byte == '\n')
        {
            if (!discard_line && line_length > 0)
            {
                line_buffer[line_length] = '\0';
                Handle_Line(line_buffer);
            }

            line_length = 0;
            discard_line = 0;
        }
        else if (discard_line)
        {
            /* 丟棄異常封包，直到收到換行 */
            continue;
        }
        else if (byte == '\r')
        {
            /* 同時相容 \n 與 \r\n */
            continue;
        }
        else if (byte == '\0')
        {
            line_length = 0;
            discard_line = 1;
        }
        else if (line_length < LINE_BUFFER_SIZE - 1U)
        {
            line_buffer[line_length++] = (char)byte;
        }
        else
        {
            /* 封包太長，整包丟棄 */
            line_length = 0;
            discard_line = 1;
        }
    }
}


bool UART_Receiver_TakeCommand(RobotCommand *command)
{
    if (command == NULL || !command_ready)
        return false;

    *command = pending_command;
    command_ready = false;

    return true;
}
