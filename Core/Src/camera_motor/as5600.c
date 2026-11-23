#include "main.h"

#if GIMBAL_ENABLED
#include "camera_motor/as5600.h"
#include <stddef.h>

#define AS5600_ADDR            (0x36U << 1)
#define AS5600_STATUS_REG      0x0BU
#define AS5600_RAW_ANGLE_REG   0x0CU

#define AS5600_COUNTS_PER_REV  4096
#define AS5600_HALF_REV        2048
#define AS5600_RAD_PER_COUNT   (6.28318530718f / 4096.0f)

#define AS5600_CHECK_TIMEOUT_MS  100U
#define AS5600_READ_TIMEOUT_MS     5U

static I2C_HandleTypeDef *as5600_i2c;
static uint16_t last_raw;
static int32_t full_rotations;
static uint8_t has_sample;

void AS5600_Init(I2C_HandleTypeDef *i2c)
{
    as5600_i2c = i2c;
    last_raw = 0U;
    full_rotations = 0;
    has_sample = 0U;
}



HAL_StatusTypeDef AS5600_Check(void)
{
    if (as5600_i2c == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_IsDeviceReady(
        as5600_i2c, AS5600_ADDR, 3U, AS5600_CHECK_TIMEOUT_MS);
}

HAL_StatusTypeDef AS5600_ReadStatus(uint8_t *status)
{
    if (as5600_i2c == NULL || status == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(
        as5600_i2c, AS5600_ADDR, AS5600_STATUS_REG,
        I2C_MEMADD_SIZE_8BIT, status, 1U, AS5600_READ_TIMEOUT_MS);
}

HAL_StatusTypeDef AS5600_ReadSample(AS5600_Sample *sample)
{
    if (as5600_i2c == NULL || sample == NULL)
    {
        return HAL_ERROR;
    }

    uint8_t data[2];
    HAL_StatusTypeDef result = HAL_I2C_Mem_Read(
        as5600_i2c, AS5600_ADDR, AS5600_RAW_ANGLE_REG,
        I2C_MEMADD_SIZE_8BIT, data, sizeof(data), AS5600_READ_TIMEOUT_MS);

    if (result != HAL_OK)
    {
        return result;
    }

    uint16_t raw = (uint16_t)(
        (((uint16_t)data[0] << 8) | data[1]) & 0x0FFFU);

    if (has_sample)
    {
        int32_t delta = (int32_t)raw - (int32_t)last_raw;

        if (delta > AS5600_HALF_REV)
        {
            full_rotations--;
        }
        else if (delta < -AS5600_HALF_REV)
        {
            full_rotations++;
        }
    }

    last_raw = raw;
    has_sample = 1U;

    sample->raw = raw;
    sample->angle_rad =
        ((float)full_rotations * AS5600_COUNTS_PER_REV + (float)raw) *
        AS5600_RAD_PER_COUNT;

    return HAL_OK;
}
#endif
