#ifndef AS5600_H
#define AS5600_H

#include "stm32f4xx_hal.h"

typedef struct
{
    uint16_t raw;
    float angle_rad;
} AS5600_Sample;

void AS5600_Init(I2C_HandleTypeDef *i2c);
HAL_StatusTypeDef AS5600_Check(void);
HAL_StatusTypeDef AS5600_ReadStatus(uint8_t *status);
HAL_StatusTypeDef AS5600_ReadSample(AS5600_Sample *sample);

#endif
