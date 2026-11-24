#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>

// 列出主迴圈可分派的車體與雲台指令。
typedef enum
{
    COMMAND_PERSON_POSITION,
    COMMAND_PERSON_LOST,
    COMMAND_STOP,
    COMMAND_GIMBAL_RELATIVE,
    COMMAND_GIMBAL_ZERO,
    COMMAND_GIMBAL_YAW_QUERY
} CommandType;

typedef struct
{
    // 人物座標，以車體為原點，單位為公尺。
    float x;
    float y;
    float z;
} PersonPosition;

typedef struct
{
    CommandType type;
    PersonPosition person;
    // 相對轉角指令使用的角度，單位為度。
    float gimbal_delta_deg;
} RobotCommand;

// 解析一行指令，成功時寫入 command。
bool Protocol_Parse(
    const char *line,
    RobotCommand *command
);

// 編碼車體速度。回傳實際長度，失敗時回傳 0。
size_t Protocol_EncodeCmdVel(
    char *buffer,
    size_t capacity,
    float linear_mps,
    float angular_radps
);

// 編碼左右車輪速度。回傳實際長度，失敗時回傳 0。
size_t Protocol_EncodeWheelTarget(
    char *buffer,
    size_t capacity,
    float left_mps,
    float right_mps
);

#endif
