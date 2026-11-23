#ifndef CAMERA_MOTOR_H
#define CAMERA_MOTOR_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

typedef enum
{
    MOTOR_OK = 0,
    MOTOR_ERR_PWM,
    MOTOR_ERR_ENCODER,
    MOTOR_ERR_ALIGNMENT,
    MOTOR_ERR_YAW_LIMIT,
    MOTOR_ERR_UNCALIBRATED,
    MOTOR_ERR_COMMAND,
    MOTOR_ERR_DISABLED
} MotorResult;

typedef struct
{
    float angle_deg;       /* 與舊程式相同的 ControlAngle 座標 */
    float target_deg;
    float velocity_rad_s;
    float yaw_deg;         /* 以機械正前方為 0° */
    float target_yaw_deg;
    bool yaw_valid;
} MotorTelemetry;

MotorResult Motor_Start(TIM_HandleTypeDef *pwm_timer);
MotorResult Motor_Update(void);
void Motor_Disable(void);

MotorResult Motor_SetTargetYawDeg(float yaw_deg);
void Motor_HoldPosition(void);
MotorResult Motor_MoveRelativeDeg(float delta_deg);
MotorResult Motor_SetYawZero(uint16_t *raw_zero);
void Motor_GetTelemetry(MotorTelemetry *telemetry);

#endif
