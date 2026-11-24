#include "main.h"

#if GIMBAL_ENABLED
#include "camera_motor/camera_motor.h"
#include "camera_motor/as5600.h"
#include <math.h>
#include <stddef.h>

#define PI_F                    3.14159265359f
#define TWO_PI                  (2.0f * PI_F)
// FOC 掃描起始電角度，單位為 rad。
#define ALIGN_ANGLE             (1.5f * PI_F)
#define DEG_TO_RAD              (PI_F / 180.0f)
#define RAD_TO_DEG              (180.0f / PI_F)

// 馬達極對數、直流母線電壓、控制電壓上限與對齊電壓，電壓單位為 V。
#define POLE_PAIRS              7.0f
#define SUPPLY_VOLTAGE          24.9f
#define MOTOR_VOLTAGE_LIMIT     1.5f
#define ALIGN_VOLTAGE           1.5f

// 速度上限為 rad/s，加速度上限為 rad/s²。
#define MAX_VELOCITY            40.0f
#define MAX_ACCELERATION        40.0f

// 外迴路位置 PID 產生速度命令，內迴路速度 PI 產生 q 軸電壓命令。
#define POS_KP                  4.00f
#define POS_KI                  0.01f
#define POS_KD                  0.262f
#define VEL_KP                  0.60f
#define VEL_KI                  0.27f
// 速度低通濾波的時間常數，單位為秒。
#define VEL_LPF_TF              0.16f

// 機械零位為 AS5600 原始計數，正方向與限位採用車體的偏航座標。
#define YAW_ZERO_RAW_COUNT      GIMBAL_YAW_ZERO_RAW
#define YAW_SIGN                ((float)GIMBAL_YAW_SIGN)
#define YAW_MIN_DEG             (-100.0f)
#define YAW_MAX_DEG             100.0f

// 每次正掃與反掃各跨過一圈電角度，分為 500 段。
#define ALIGN_STEPS             500

// PWM 計時器、最新編碼器資料、驅動狀態與最近一次停機故障。
static TIM_HandleTypeDef *timer;
static AS5600_Sample encoder;
static bool enabled;
static MotorResult fault;

// 對齊取得的編碼器方向與電角零位，及控制迴路使用的連續目標角度。
static int sensor_direction;
static float zero_electric_angle;
static float target_angle;
// 串口診斷用的最後 q 軸命令電壓及正向掃描的實測位移。
static float last_uq;
static float alignment_motion_deg;

// 偏航零位校正狀態、原始計數，以及機械零位在連續控制座標中的位置。
static bool yaw_zero_valid;
static bool alignment_in_progress;
static uint16_t yaw_zero_raw;
static float yaw_zero_control_angle;

// 濾波後的速度、PID 積分與誤差紀錄，以及限加速度後的速度命令。
static float shaft_velocity;
static float pos_integral;
static float pos_prev_error;
static float vel_integral;
static float ramped_velocity;

// 用 DWT 時脈週期計算兩次控制更新的間隔。
static uint32_t last_cycles;
static float seconds_per_cycle;

// 把控制量限制在指定上下界內。
static float clampf(float value, float min, float max)
{
    if (value < min)
        return min;
    if (value > max)
        return max;
    return value;
}

// 將電角度整理到 0 到 2π 的範圍，接著可做三相電壓換算。
static float normalize_angle(float angle)
{
    angle = fmodf(angle, TWO_PI);
    return angle < 0.0f ? angle + TWO_PI : angle;
}

// 將展開後的編碼器角度乘以對齊取得的方向，得到 PID 使用的角度。
static float control_angle(void)
{
    return (float)sensor_direction * encoder.angle_rad;
}

// 相對機械零位換算偏航角，跨過編碼器零點時仍採用最短角差。
static float current_yaw_deg(void)
{
    int32_t delta = (int32_t)encoder.raw - (int32_t)yaw_zero_raw;

    if (delta > 2047)
        delta -= 4096;
    if (delta < -2048)
        delta += 4096;

    return YAW_SIGN * (float)delta * (360.0f / 4096.0f);
}

// 判斷量到的偏航角是否超出機械限位。
static bool yaw_out_of_range(void)
{
    if (!yaw_zero_valid)
        return false;

    float yaw = current_yaw_deg();
    return yaw < YAW_MIN_DEG || yaw > YAW_MAX_DEG;
}

// 將車體偏航目標（度）轉成連續控制角（rad）。
static float yaw_to_control_angle(float yaw_deg)
{
    return yaw_zero_control_angle +
           ((float)sensor_direction / YAW_SIGN) * yaw_deg * DEG_TO_RAD;
}

// 依當前編碼器位置重新對應兩套座標的零點。
static void update_yaw_reference(void)
{
    yaw_zero_control_angle = control_angle() -
        ((float)sensor_direction / YAW_SIGN) *
        current_yaw_deg() * DEG_TO_RAD;
}

// 將三相占空比轉成 PWM 計時器的比較值。
static void set_duty(float a, float b, float c)
{
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(timer) + 1U;

    __HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_1,
        (uint32_t)(clampf(a, 0.0f, 1.0f) * period));
    __HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_2,
        (uint32_t)(clampf(b, 0.0f, 1.0f) * period));
    __HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_3,
        (uint32_t)(clampf(c, 0.0f, 1.0f) * period));
}

// 關閉驅動器 EN，並清除 PWM 占空比。
void Motor_Disable(void)
{
    HAL_GPIO_WritePin(Driver_Enable_GPIO_Port, Driver_Enable_Pin,
                     GPIO_PIN_RESET);
    enabled = false;
    last_uq = 0.0f;

    if (timer != NULL)
        set_duty(0.0f, 0.0f, 0.0f);
}

// 停用馬達並記下故障原因，供控制層回報。
static MotorResult stop_with_fault(MotorResult reason)
{
    Motor_Disable();
    fault = reason;
    return reason;
}

// 將受限的 q 軸電壓與電角度換算成三相 PWM。
static void set_phase_voltage(float uq, float electrical_angle)
{
    uq = clampf(uq, -MOTOR_VOLTAGE_LIMIT, MOTOR_VOLTAGE_LIMIT);
    last_uq = uq;
    electrical_angle = normalize_angle(electrical_angle);

    float alpha = -uq * sinf(electrical_angle);
    float beta = uq * cosf(electrical_angle);
    float half_bus = SUPPLY_VOLTAGE * 0.5f;

    float a = half_bus + alpha;
    float b = half_bus - 0.5f * alpha + 0.8660254f * beta;
    float c = half_bus - 0.5f * alpha - 0.8660254f * beta;

    set_duty(a / SUPPLY_VOLTAGE, b / SUPPLY_VOLTAGE, c / SUPPLY_VOLTAGE);
}

// 更新角度。讀取失敗或在平常控制期間越界時，立即停用馬達。
static MotorResult read_encoder(void)
{
    if (AS5600_ReadSample(&encoder) != HAL_OK)
        return stop_with_fault(MOTOR_ERR_ENCODER);

    if (!alignment_in_progress && yaw_out_of_range())
        return stop_with_fault(MOTOR_ERR_YAW_LIMIT);

    return MOTOR_OK;
}

// 重設控制器的積分、濾波與計時狀態。
static void reset_controllers(void)
{
    pos_integral = 0.0f;
    pos_prev_error = target_angle - control_angle();
    vel_integral = 0.0f;
    shaft_velocity = 0.0f;
    ramped_velocity = 0.0f;
    last_cycles = DWT->CYCCNT;
}

// 輸出一個掃描位置，等轉子短暫反應後讀取編碼器。
static MotorResult align_step(int step)
{
    float angle = ALIGN_ANGLE + TWO_PI * (float)step / ALIGN_STEPS;

    set_phase_voltage(ALIGN_VOLTAGE, angle);
    HAL_Delay(2);
    return read_encoder();
}

// 正反掃描確認方向，再鎖定轉子以量出電角零位。
static MotorResult align_sensor(void)
{
    float start = encoder.angle_rad;

    for (int step = 0; step <= ALIGN_STEPS; ++step)
    {
        MotorResult result = align_step(step);
        if (result != MOTOR_OK)
            return result;
    }

    float moved = encoder.angle_rad - start;
    alignment_motion_deg = moved * RAD_TO_DEG;

    for (int step = ALIGN_STEPS; step >= 0; --step)
    {
        MotorResult result = align_step(step);
        if (result != MOTOR_OK)
            return result;
    }

    if (fabsf(moved) < 0.01f)
        return stop_with_fault(MOTOR_ERR_ALIGNMENT);

    sensor_direction = moved > 0.0f ? 1 : -1;

    set_phase_voltage(ALIGN_VOLTAGE, ALIGN_ANGLE);

    HAL_Delay(700);
    MotorResult result = read_encoder();
    if (result != MOTOR_OK)
        return result;

    zero_electric_angle = normalize_angle(POLE_PAIRS * control_angle());
    set_phase_voltage(0.0f, 0.0f);
    HAL_Delay(200);
    return read_encoder();
}

// 先檢查編碼器與開機限位，再啟用 PWM、執行對齊並保持對齊後的位置。
MotorResult Motor_Start(TIM_HandleTypeDef *pwm_timer)
{
    Motor_Disable();
    timer = pwm_timer;
    fault = MOTOR_OK;
    alignment_motion_deg = 0.0f;

    if (timer == NULL)
        return stop_with_fault(MOTOR_ERR_PWM);

    sensor_direction = 1;
    yaw_zero_valid = YAW_ZERO_RAW_COUNT >= 0 && YAW_ZERO_RAW_COUNT <= 4095;
    yaw_zero_raw = yaw_zero_valid ? (uint16_t)YAW_ZERO_RAW_COUNT : 0U;

    if (!yaw_zero_valid || (YAW_SIGN != 1.0f && YAW_SIGN != -1.0f))
        return stop_with_fault(MOTOR_ERR_UNCALIBRATED);

    MotorResult result = read_encoder();
    if (result != MOTOR_OK)
        return result;

    if (HAL_TIM_PWM_Start(timer, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(timer, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(timer, TIM_CHANNEL_3) != HAL_OK)
    {
        return stop_with_fault(MOTOR_ERR_PWM);
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    seconds_per_cycle = 1.0f / (float)HAL_RCC_GetHCLKFreq();

    set_phase_voltage(0.0f, 0.0f);
    HAL_GPIO_WritePin(Driver_Enable_GPIO_Port, Driver_Enable_Pin, GPIO_PIN_SET);
    enabled = true;

    // 掃描期間略過偏航限位判斷，對齊結束後立即再檢查。
    alignment_in_progress = true;
    result = align_sensor();
    alignment_in_progress = false;
    if (result != MOTOR_OK)
        return result;

    target_angle = control_angle();
    update_yaw_reference();
    if (yaw_out_of_range())
        return stop_with_fault(MOTOR_ERR_YAW_LIMIT);

    reset_controllers();
    return MOTOR_OK;
}

// 執行一輪編碼器取樣、位置 PID、速度 PI 與 FOC 輸出。
MotorResult Motor_Update(void)
{
    if (!enabled)
        return fault == MOTOR_OK ? MOTOR_ERR_DISABLED : fault;

    float old_angle = control_angle();
    MotorResult result = read_encoder();
    if (result != MOTOR_OK)
        return result;

    uint32_t now = DWT->CYCCNT;
    float dt = (float)(now - last_cycles) * seconds_per_cycle;
    last_cycles = now;

    if (dt <= 0.0f || dt > 0.05f)
        dt = 0.001f;

    float angle = control_angle();
    float raw_velocity = (angle - old_angle) / dt;
    float alpha = VEL_LPF_TF / (VEL_LPF_TF + dt);
    shaft_velocity = alpha * shaft_velocity + (1.0f - alpha) * raw_velocity;

    // 位置迴路產生所需速度，積分及輸出皆設上限。
    float position_error = target_angle - angle;
    pos_integral = clampf(pos_integral + position_error * dt, -2.0f, 2.0f);

    float derivative = (position_error - pos_prev_error) / dt;
    pos_prev_error = position_error;

    float requested_velocity = clampf(
        POS_KP * position_error + POS_KI * pos_integral + POS_KD * derivative,
        -MAX_VELOCITY, MAX_VELOCITY);

    // 依本次取樣間隔限制速度變化量。
    float max_dv = MAX_ACCELERATION * dt;
    ramped_velocity += clampf(requested_velocity - ramped_velocity,
                              -max_dv, max_dv);

    // 速度迴路依誤差產生 q 軸電壓，輸出由三相電壓函式限幅。
    float velocity_error = ramped_velocity - shaft_velocity;
    vel_integral = clampf(vel_integral + VEL_KI * velocity_error * dt,
                          -MOTOR_VOLTAGE_LIMIT, MOTOR_VOLTAGE_LIMIT);

    float uq = VEL_KP * velocity_error + vel_integral;
    float electrical_angle = POLE_PAIRS * angle - zero_electric_angle;

    set_phase_voltage(uq, electrical_angle);
    return MOTOR_OK;
}

// 設定絕對偏航目標，並將目標限制在機械範圍內。
MotorResult Motor_SetTargetYawDeg(float yaw_deg)
{
    if (!enabled)
        return fault == MOTOR_OK ? MOTOR_ERR_DISABLED : fault;

    if (!yaw_zero_valid)
        return MOTOR_ERR_UNCALIBRATED;

    if (!isfinite(yaw_deg))
        return MOTOR_ERR_COMMAND;

    target_angle = yaw_to_control_angle(clampf(yaw_deg, YAW_MIN_DEG, YAW_MAX_DEG));
    return MOTOR_OK;
}

// 把最新量測角度設為目標，清除先前命令的控制狀態。
void Motor_HoldPosition(void)
{
    if (enabled)
    {
        target_angle = control_angle();
        reset_controllers();
    }
}

// 讀取當前位置後加上相對角度，並重設位置迴路的誤差紀錄。
MotorResult Motor_MoveRelativeDeg(float delta_deg)
{
    if (!enabled)
        return fault == MOTOR_OK ? MOTOR_ERR_DISABLED : fault;

    if (!yaw_zero_valid)
        return MOTOR_ERR_UNCALIBRATED;

    if (!isfinite(delta_deg) || delta_deg < -180.0f || delta_deg > 180.0f)
        return MOTOR_ERR_COMMAND;

    MotorResult result = read_encoder();
    if (result != MOTOR_OK)
        return result;

    float requested_yaw = current_yaw_deg() +
        ((float)sensor_direction / YAW_SIGN) * delta_deg;

    float limited_yaw = clampf(requested_yaw, YAW_MIN_DEG, YAW_MAX_DEG);
    target_angle = yaw_to_control_angle(limited_yaw);

    pos_integral = 0.0f;
    pos_prev_error = target_angle - control_angle();

    return MOTOR_OK;
}

// 以當前角度校正本次上電的機械零位，並回傳可寫入韌體的原始值。
MotorResult Motor_SetYawZero(uint16_t *raw_zero)
{
    if (!enabled)
        return fault == MOTOR_OK ? MOTOR_ERR_DISABLED : fault;

    MotorResult result = read_encoder();
    if (result != MOTOR_OK)
        return result;

    yaw_zero_raw = encoder.raw;
    yaw_zero_valid = true;
    update_yaw_reference();

    target_angle = control_angle();
    reset_controllers();
    set_phase_voltage(0.0f, 0.0f);

    if (raw_zero != NULL)
        *raw_zero = yaw_zero_raw;

    return MOTOR_OK;
}

// 複製最近的控制與編碼器資料，供串口診斷使用。
void Motor_GetTelemetry(MotorTelemetry *telemetry)
{
    telemetry->encoder_raw = encoder.raw;
    telemetry->uq_volts = last_uq;
    telemetry->alignment_motion_deg = alignment_motion_deg;
    telemetry->sensor_direction = sensor_direction;
    telemetry->angle_deg = control_angle() * RAD_TO_DEG;
    telemetry->target_deg = target_angle * RAD_TO_DEG;
    telemetry->velocity_rad_s = shaft_velocity;

    // 停機後保留最後一筆角度，yaw_valid 用來標示目前是否有效。
    telemetry->yaw_valid = enabled && yaw_zero_valid;
    telemetry->yaw_deg = yaw_zero_valid ? current_yaw_deg() : 0.0f;
    telemetry->target_yaw_deg = yaw_zero_valid
        ? (target_angle - yaw_zero_control_angle) *
          (YAW_SIGN / (float)sensor_direction) * RAD_TO_DEG
        : 0.0f;
}
#endif
