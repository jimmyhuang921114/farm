#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>

typedef enum
{
    COMMAND_PERSON_POSITION,
    COMMAND_PERSON_LOST,
    COMMAND_STOP
} CommandType;

typedef struct
{
    float x;
    float y;
    float z;
} PersonPosition;

typedef struct
{
    CommandType type;
    PersonPosition person;
} RobotCommand;

bool Protocol_Parse(
    const char *line,
    RobotCommand *command
);

size_t Protocol_EncodeWheelTarget(
    char *buffer,
    size_t capacity,
    float left_mps,
    float right_mps
);

#endif
