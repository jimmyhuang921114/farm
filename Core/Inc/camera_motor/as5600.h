#ifndef AS5600_H
#define AS5600_H

#include "stm32f4xx_hal.h"

typedef struct
{
    // 單圈原始讀值，範圍 0 到 4095。
    uint16_t raw;
    // 展開跨圈後的連續角度，單位為 rad。
    float angle_rad;
} AS5600_Sample;

// 指定 I2C 介面並清除跨圈狀態。
void AS5600_Init(I2C_HandleTypeDef *i2c);
// 確認 AS5600 是否回應 I2C 探測。
HAL_StatusTypeDef AS5600_Check(void);
// 把磁鐵狀態暫存器寫入 status。
HAL_StatusTypeDef AS5600_ReadStatus(uint8_t *status);
// 把原始計數及連續角度寫入 sample。
HAL_StatusTypeDef AS5600_ReadSample(AS5600_Sample *sample);

#endif
