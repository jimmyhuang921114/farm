/*
 * gimbal_track.h
 *
 *  Created on: Nov 23, 2026
 *      Author: jimmy
 */

#ifndef GIMBAL_TRACK_H
#define GIMBAL_TRACK_H

#include "stm32f4xx_hal.h"
#include "track.h"

void GimbalTrack_Init(I2C_HandleTypeDef *i2c, TIM_HandleTypeDef *pwm_timer);
const char *GimbalTrack_GetStatus(void);
void GimbalTrack_SetPerson(TrackPosition person);
void GimbalTrack_Update(void);
void GimbalTrack_Stop(void);

#endif
