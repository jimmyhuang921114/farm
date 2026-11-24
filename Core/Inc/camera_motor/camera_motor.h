#ifndef CAMERA_MOTOR_H
#define CAMERA_MOTOR_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

typedef enum
{
    // 啟動、控制更新及指令共用的結果碼。
    MOTOR_OK = 0,
    // 任一相的 PWM 啟動失敗。
    MOTOR_ERR_PWM,
    // 讀取 AS5600 角度失敗。
    MOTOR_ERR_ENCODER,
    // 對齊掃描的實測位移不足。
    MOTOR_ERR_ALIGNMENT,
    // 實測偏航超出機械限位。
    MOTOR_ERR_YAW_LIMIT,
    // 機械零位或方向設定無效。
    MOTOR_ERR_UNCALIBRATED,
    // 角度命令格式或數值無效。
    MOTOR_ERR_COMMAND,
    // 驅動器已停用。
    MOTOR_ERR_DISABLED
} MotorResult;

typedef struct
{
    float angle_deg;       // 與舊程式相同的 ControlAngle 座標
    // 連續控制座標下的目標角，單位為度。
    float target_deg;
    // 濾波後的馬達軸速度，單位為 rad/s。
    float velocity_rad_s;
    float yaw_deg;         // 以機械正前方為 0°
    // 相對機械正前方的目標偏航角，單位為度。
    float target_yaw_deg;
    // 驅動啟用且零位有效時，偏航資料才有效。
    bool yaw_valid;
    // 診斷欄位包含 AS5600 原始計數、q 軸命令電壓、對齊位移與方向。
    uint16_t encoder_raw;
    float uq_volts;
    float alignment_motion_deg;
    int sensor_direction;
} MotorTelemetry;

// 啟用三相 PWM 並完成 FOC 對齊。
MotorResult Motor_Start(TIM_HandleTypeDef *pwm_timer);
// 執行一輪閉迴路控制，失敗時回報原因。
MotorResult Motor_Update(void);
// 關閉驅動器並清空 PWM 占空比。
void Motor_Disable(void);

// 設定受機械限位約束的絕對偏航目標。
MotorResult Motor_SetTargetYawDeg(float yaw_deg);
// 將最近量到的角度設為目標。
void Motor_HoldPosition(void);
// 以當前位置為基準設定相對轉角。
MotorResult Motor_MoveRelativeDeg(float delta_deg);
// 設定本次上電的機械零位，可取回原始計數。
MotorResult Motor_SetYawZero(uint16_t *raw_zero);
// 取得最新量測、控制目標和除錯資料。
void Motor_GetTelemetry(MotorTelemetry *telemetry);

#endif
