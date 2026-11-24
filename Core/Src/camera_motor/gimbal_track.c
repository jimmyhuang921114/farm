// gimbal_track.c
//
//  Created on: Nov 23, 2026
//      Author: jimmy

#include "main.h"

#if GIMBAL_ENABLED
#include "camera_motor/gimbal_track.h"
#include "camera_motor/as5600.h"
#include "camera_motor/camera_motor.h"
#include "uart/uart_sender.h"
#include <math.h>
#include <stdio.h>

static const float RAD_TO_DEG = 57.295779513f;
// 主迴圈的馬達更新最短間隔及串口遙測回報間隔，單位為 ms。
static const uint32_t MOTOR_UPDATE_MS = 2U;
static const uint32_t YAW_TELEMETRY_MS = 200U;
// 開機試轉共 9 步，每步 5°，給 2 秒到達目標的 ±0.5° 內。
static const float STARTUP_JOG_DEG = 5.0f;
static const float STARTUP_JOG_TOLERANCE_DEG = 0.5f;
static const uint32_t STARTUP_JOG_INTERVAL_MS = 2000U;
static const unsigned STARTUP_JOG_STEPS = 9U;
// 記錄上次控制更新與 UART 遙測時間。
static uint32_t last_update_ms;
static uint32_t last_telemetry_ms;
// running 允許控制更新，following 表示目前目標來自人物座標。
static bool running;
static bool following;
// 保存試轉步數、方向、起始與目標角度，以及試轉期間的最大位移。
static bool startup_jog_active;
static uint32_t startup_jog_step_started_ms;
static unsigned startup_jog_step;
static float startup_jog_delta_deg;
static float startup_yaw_deg;
static float startup_jog_target_deg;
static float startup_jog_peak_deg;
// 記錄開機檢查的最終狀態，供主程式送出。
static const char *startup_status = "NOT_INITIALIZED";

// 將馬達回傳碼轉成可在串口辨識的狀態。
static const char *motor_result_name(MotorResult result)
{
    switch (result)
    {
    case MOTOR_OK: return "OK";
    case MOTOR_ERR_PWM: return "PWM_START_FAILED";
    case MOTOR_ERR_ENCODER: return "ENCODER_READ_FAILED";
    case MOTOR_ERR_ALIGNMENT: return "ALIGNMENT_NO_MOTION";
    case MOTOR_ERR_YAW_LIMIT: return "YAW_LIMIT";
    case MOTOR_ERR_UNCALIBRATED: return "UNCALIBRATED";
    case MOTOR_ERR_COMMAND: return "INVALID_COMMAND";
    case MOTOR_ERR_DISABLED: return "DISABLED";
    }
    return "UNKNOWN";
}

// 輸出試轉停止原因、當前角度及整段試轉的最大位移。
static void report_jog_result(const char *result, const MotorTelemetry *telemetry)
{
    char reply[176];
    snprintf(reply, sizeof(reply),
             "{\"gimbal_debug\":\"startup_jog_end\",\"result\":\"%s\","
             "\"yaw_deg\":%.2f,\"peak_movement_deg\":%.2f,\"raw\":%u}\r\n",
             result, telemetry->yaw_deg, startup_jog_peak_deg,
             (unsigned)telemetry->encoder_raw);
    UART_Sender_Send(reply);
}

// 取消後續試轉步進，讓已收到的控制命令接手。
static void cancel_startup_jog(void)
{
    if (startup_jog_active)
    {
        startup_jog_active = false;
        MotorTelemetry telemetry;
        Motor_GetTelemetry(&telemetry);
        report_jog_result("CANCELLED", &telemetry);
    }
}

// 探測 AS5600、回報磁鐵狀態、執行 FOC 對齊，成功後設定第一步試轉目標。
void GimbalTrack_Init(I2C_HandleTypeDef *i2c, TIM_HandleTypeDef *pwm_timer)
{
    Motor_Disable();
    AS5600_Init(i2c);
    startup_jog_active = false;

    if (AS5600_Check() != HAL_OK)
    {
        startup_status = "AS5600_NOT_FOUND";
        running = false;
    }
    else
    {
        uint8_t status;
        char reply[176];
        if (AS5600_ReadStatus(&status) == HAL_OK)
        {
            snprintf(reply, sizeof(reply),
                     "{\"as5600_status\":\"0x%02X\",\"magnet_detected\":%u,"
                     "\"magnet_weak\":%u,\"magnet_strong\":%u}\r\n",
                     (unsigned)status, (unsigned)((status >> 5) & 1U),
                     (unsigned)((status >> 4) & 1U), (unsigned)((status >> 3) & 1U));
            UART_Sender_Send(reply);
        }
        else
        {
            UART_Sender_Send("{\"as5600_status\":\"READ_FAILED\"}\r\n");
        }

        UART_Sender_Send("{\"gimbal_status\":\"AS5600_OK_FOC_ALIGNING\"}\r\n");
        MotorResult result = Motor_Start(pwm_timer);
        MotorTelemetry telemetry;
        Motor_GetTelemetry(&telemetry);
        snprintf(reply, sizeof(reply),
                 "{\"gimbal_debug\":\"alignment\",\"result\":\"%s\","
                 "\"raw\":%u,\"sweep_motion_deg\":%.2f,\"sensor_direction\":%d}\r\n",
                 motor_result_name(result), (unsigned)telemetry.encoder_raw,
                 telemetry.alignment_motion_deg, telemetry.sensor_direction);
        UART_Sender_Send(reply);
        running = result == MOTOR_OK;
        startup_status = running ? "FOC_READY" : "FOC_ALIGN_OR_MOTOR_START_FAILED";
        if (result == MOTOR_ERR_YAW_LIMIT)
            startup_status = "YAW_OUTSIDE_LIMIT_AT_BOOT";
    }
    following = false;
    last_update_ms = HAL_GetTick();
    last_telemetry_ms = last_update_ms;

    if (running)
    {
        MotorTelemetry telemetry;
        Motor_GetTelemetry(&telemetry);
        startup_yaw_deg = telemetry.yaw_deg;
        // Move away from the nearer mechanical limit.
        startup_jog_delta_deg = startup_yaw_deg > 0.0f ? -STARTUP_JOG_DEG : STARTUP_JOG_DEG;
        startup_jog_step = 1U;
        startup_jog_target_deg = startup_yaw_deg + startup_jog_delta_deg;
        startup_jog_peak_deg = 0.0f;
        char reply[136];
        snprintf(reply, sizeof(reply),
                 "{\"gimbal_debug\":\"startup_jog_begin\",\"yaw_deg\":%.2f,"
                 "\"target_yaw_deg\":%.2f,\"raw\":%u}\r\n",
                 startup_yaw_deg, startup_jog_target_deg, (unsigned)telemetry.encoder_raw);
        UART_Sender_Send(reply);
        startup_jog_step_started_ms = HAL_GetTick();
        startup_jog_active = Motor_SetTargetYawDeg(startup_jog_target_deg) == MOTOR_OK;
    }
}

// 取得最近一次開機檢查結果。
const char *GimbalTrack_GetStatus(void)
{
    return startup_status;
}

// 將車體座標的目標點轉成雲台方位角，失去人物時保持目前位置。
void GimbalTrack_SetPerson(TrackPosition person)
{
    if (!running)
        return;

    float x = person.x - GIMBAL_PIVOT_X_M;
    float y = person.y - GIMBAL_PIVOT_Y_M;
    if (person.valid && (x != 0.0f || y != 0.0f))
    {
        float yaw_deg = atan2f(y, x) * RAD_TO_DEG;
        cancel_startup_jog();
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

// 處理 UART 相對轉角、ZERO 與 YAW?，並回報結果。
void GimbalTrack_HandleCommand(const RobotCommand *command)
{
    char reply[96];
    if (command->type == COMMAND_GIMBAL_YAW_QUERY)
    {
        MotorTelemetry telemetry;
        Motor_GetTelemetry(&telemetry);
        if (telemetry.yaw_valid)
        {
            snprintf(reply, sizeof(reply),
                     "{\"gimbal_yaw_deg\":%.2f}\r\n", telemetry.yaw_deg);
            UART_Sender_Send(reply);
        }
        else
        {
            UART_Sender_Send("{\"gimbal_command\":\"YAW_UNAVAILABLE\"}\r\n");
        }
        return;
    }

    uint16_t raw_zero = 0;
    MotorResult result = command->type == COMMAND_GIMBAL_ZERO
        ? Motor_SetYawZero(&raw_zero)
        : Motor_MoveRelativeDeg(command->gimbal_delta_deg);

    if (result != MOTOR_OK)
    {
        UART_Sender_Send("{\"gimbal_command\":\"REJECTED\"}\r\n");
        return;
    }

    cancel_startup_jog();
    following = false;
    if (command->type == COMMAND_GIMBAL_ZERO)
    {
        snprintf(reply, sizeof(reply),
                 "{\"gimbal_zero_raw\":%u}\r\n", (unsigned)raw_zero);
        UART_Sender_Send(reply);
    }
    else
    {
        UART_Sender_Send("{\"gimbal_command\":\"ACCEPTED\"}\r\n");
    }
}

// 定期更新馬達控制、推進試轉步驟，並輸出遙測。
void GimbalTrack_Update(void)
{
    uint32_t now = HAL_GetTick();
    if (!running || now - last_update_ms < MOTOR_UPDATE_MS)
        return;

    last_update_ms = now;
    MotorResult result = Motor_Update();
    if (result != MOTOR_OK)
    {
        if (startup_jog_active)
        {
            MotorTelemetry telemetry;
            Motor_GetTelemetry(&telemetry);
            report_jog_result(motor_result_name(result), &telemetry);
            startup_jog_active = false;
        }
        GimbalTrack_Stop();
        UART_Sender_Send(result == MOTOR_ERR_YAW_LIMIT
            ? "{\"gimbal_status\":\"YAW_OUTSIDE_LIMIT\"}\r\n"
            : "{\"gimbal_status\":\"ENCODER_OR_MOTOR_FAULT\"}\r\n");
        return;
    }

    if (startup_jog_active)
    {
        MotorTelemetry telemetry;
        Motor_GetTelemetry(&telemetry);
        startup_jog_peak_deg = fmaxf(startup_jog_peak_deg,
                                     fabsf(telemetry.yaw_deg - startup_yaw_deg));
        // 上一個目標在期限內到位才進入下一步，逾時就保持當下位置。
        if (now - startup_jog_step_started_ms >= STARTUP_JOG_INTERVAL_MS)
        {
            bool reached = fabsf(telemetry.yaw_deg - startup_jog_target_deg) <=
                           STARTUP_JOG_TOLERANCE_DEG;
            if (!reached || startup_jog_step == STARTUP_JOG_STEPS)
            {
                startup_jog_active = false;
                if (!reached)
                    Motor_HoldPosition();
                report_jog_result(reached ? "REACHED" : "STEP_TIMEOUT", &telemetry);
            }
            else
            {
                ++startup_jog_step;
                startup_jog_target_deg = startup_yaw_deg +
                    startup_jog_delta_deg * (float)startup_jog_step;
                Motor_SetTargetYawDeg(startup_jog_target_deg);
                startup_jog_step_started_ms = now;

                char reply[144];
                snprintf(reply, sizeof(reply),
                         "{\"gimbal_debug\":\"startup_jog_step\",\"step\":%u,"
                         "\"yaw_deg\":%.2f,\"target_yaw_deg\":%.2f}\r\n",
                         startup_jog_step, telemetry.yaw_deg, startup_jog_target_deg);
                UART_Sender_Send(reply);
            }
        }
    }

    if (now - last_telemetry_ms >= YAW_TELEMETRY_MS)
    {
        last_telemetry_ms = now;
        MotorTelemetry telemetry;
        Motor_GetTelemetry(&telemetry);
        char reply[192];
        snprintf(reply, sizeof(reply),
                 "{\"gimbal_yaw_deg\":%.2f,\"angle_deg\":%.2f,\"target_deg\":%.2f,"
                 "\"error_deg\":%.2f,\"velocity_rad_s\":%.2f,\"raw\":%u,\"uq_volts\":%.3f}\r\n",
                 telemetry.yaw_deg, telemetry.angle_deg, telemetry.target_deg,
                 telemetry.target_deg - telemetry.angle_deg, telemetry.velocity_rad_s,
                 (unsigned)telemetry.encoder_raw, telemetry.uq_volts);
        UART_Sender_Send(reply);
    }
}

// 關閉馬達並清除追蹤與試轉狀態。
void GimbalTrack_Stop(void)
{
    Motor_Disable();
    cancel_startup_jog();
    running = false;
    following = false;
    startup_jog_active = false;
}
#endif
