// gimbal_track.h
//
//  Created on: Nov 23, 2026
//      Author: jimmy

#ifndef GIMBAL_TRACK_H
#define GIMBAL_TRACK_H

#include "stm32f4xx_hal.h"
#include "track.h"
#include "uart/protocol.h"

// 檢查 AS5600、完成馬達對齊並排定開機試轉。
void GimbalTrack_Init(I2C_HandleTypeDef *i2c, TIM_HandleTypeDef *pwm_timer);
// 回傳開機檢查結果。
const char *GimbalTrack_GetStatus(void);
// 把人物座標轉成偏航目標，人物失效時結束追蹤。
void GimbalTrack_SetPerson(TrackPosition person);
// 處理已解析的 UART 雲台命令。
void GimbalTrack_HandleCommand(const RobotCommand *command);
// 由主迴圈呼叫，執行控制更新、試轉和遙測。
void GimbalTrack_Update(void);
// 關閉雲台驅動並結束追蹤和試轉。
void GimbalTrack_Stop(void);

#endif
