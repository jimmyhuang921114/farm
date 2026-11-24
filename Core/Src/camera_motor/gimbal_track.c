/*
 * gimbal_track.c
 *
 *  Created on: Nov 23, 2026
 *      Author: jimmy
 */

#include "main.h"

#if GIMBAL_ENABLED
#include "camera_motor/gimbal_track.h"
#include "camera_motor/as5600.h"
#include "camera_motor/camera_motor.h"
#include <math.h>

static const float RAD_TO_DEG = 57.295779513f;
static const uint32_t MOTOR_UPDATE_MS = 2U;
static uint32_t last_update_ms;
static bool running;
static bool following;
static const char *startup_status = "NOT_INITIALIZED";

void GimbalTrack_Init(I2C_HandleTypeDef *i2c, TIM_HandleTypeDef *pwm_timer)
{
    Motor_Disable();
    AS5600_Init(i2c);

    if (AS5600_Check() != HAL_OK)
    {
        startup_status = "AS5600_NOT_FOUND";
        running = false;
    }
    else
    {
        startup_status = "AS5600_OK_FOC_ALIGNING";
        running = Motor_Start(pwm_timer) == MOTOR_OK;
        startup_status = running ? "FOC_READY" : "FOC_ALIGN_OR_MOTOR_START_FAILED";
    }
    following = false;
    last_update_ms = HAL_GetTick();
}

const char *GimbalTrack_GetStatus(void)
{
    return startup_status;
}

void GimbalTrack_SetPerson(TrackPosition person)
{
    if (!running)
        return;

    float x = person.x - GIMBAL_PIVOT_X_M;
    float y = person.y - GIMBAL_PIVOT_Y_M;
    if (person.valid && (x != 0.0f || y != 0.0f))
    {
        float yaw_deg = atan2f(y, x) * RAD_TO_DEG;
        following = Motor_SetTargetYawDeg(yaw_deg) == MOTOR_OK;
        if (!following)
            GimbalTrack_Stop();
    }
    else if (following)
    {
        Motor_HoldPosition();
        following = false;
    }
}

void GimbalTrack_Update(void)
{
    uint32_t now = HAL_GetTick();
    if (!running || now - last_update_ms < MOTOR_UPDATE_MS)
        return;

    last_update_ms = now;
    if (Motor_Update() != MOTOR_OK)
        GimbalTrack_Stop();
}

void GimbalTrack_Stop(void)
{
    Motor_Disable();
    running = false;
    following = false;
}
#endif
