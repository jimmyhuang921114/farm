#include "main.h"

#if GIMBAL_ENABLED
#include "camera_motor/camera_motor.h"
#include "camera_motor/as5600.h"
#include <math.h>
#include <stddef.h>

#define PI_F                    3.14159265359f
#define TWO_PI                  (2.0f * PI_F)
#define ALIGN_ANGLE             (1.5f * PI_F)
#define DEG_TO_RAD              (PI_F / 180.0f)
#define RAD_TO_DEG              (180.0f / PI_F)

#define POLE_PAIRS              7.0f
#define SUPPLY_VOLTAGE          24.9f
#define MOTOR_VOLTAGE_LIMIT     1.5f
#define ALIGN_VOLTAGE           1.5f

#define MAX_VELOCITY            40.0f
#define MAX_ACCELERATION        40.0f

#define POS_KP                  4.00f
#define POS_KI                  0.01f
#define POS_KD                  0.262f
#define VEL_KP                  0.60f
#define VEL_KI                  0.27f
#define VEL_LPF_TF              0.16f

#define YAW_ZERO_RAW_COUNT      GIMBAL_YAW_ZERO_RAW
#define YAW_SIGN                ((float)GIMBAL_YAW_SIGN)
#define YAW_MIN_DEG             (-100.0f)
#define YAW_MAX_DEG             100.0f

#define ALIGN_STEPS             500

static TIM_HandleTypeDef *timer;
static AS5600_Sample encoder;
static bool enabled;
static MotorResult fault;

static int sensor_direction;
static float zero_electric_angle;
static float target_angle;

static bool yaw_zero_valid;
static bool alignment_in_progress;
static uint16_t yaw_zero_raw;
static float yaw_zero_control_angle;

static float shaft_velocity;
static float pos_integral;
static float pos_prev_error;
static float vel_integral;
static float ramped_velocity;

static uint32_t last_cycles;
static float seconds_per_cycle;

static float clampf(float value, float min, float max)
{
    if (value < min)
        return min;
    if (value > max)
        return max;
    return value;
}

static float normalize_angle(float angle)
{
    angle = fmodf(angle, TWO_PI);
    return angle < 0.0f ? angle + TWO_PI : angle;
}

static float control_angle(void)
{
    return (float)sensor_direction * encoder.angle_rad;
}

static float current_yaw_deg(void)
{
    int32_t delta = (int32_t)encoder.raw - (int32_t)yaw_zero_raw;

    if (delta > 2047)
        delta -= 4096;
    if (delta < -2048)
        delta += 4096;

    return YAW_SIGN * (float)delta * (360.0f / 4096.0f);
}

static bool yaw_out_of_range(void)
{
    if (!yaw_zero_valid)
        return false;

    float yaw = current_yaw_deg();
    return yaw < YAW_MIN_DEG || yaw > YAW_MAX_DEG;
}

static float yaw_to_control_angle(float yaw_deg)
{
    return yaw_zero_control_angle +
           ((float)sensor_direction / YAW_SIGN) * yaw_deg * DEG_TO_RAD;
}

static void update_yaw_reference(void)
{
    yaw_zero_control_angle = control_angle() -
        ((float)sensor_direction / YAW_SIGN) *
        current_yaw_deg() * DEG_TO_RAD;
}

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

void Motor_Disable(void)
{
    HAL_GPIO_WritePin(Driver_Enable_GPIO_Port, Driver_Enable_Pin,
                     GPIO_PIN_RESET);
    enabled = false;

    if (timer != NULL)
        set_duty(0.0f, 0.0f, 0.0f);
}

static MotorResult stop_with_fault(MotorResult reason)
{
    Motor_Disable();
    fault = reason;
    return reason;
}

static void set_phase_voltage(float uq, float electrical_angle)
{
    uq = clampf(uq, -MOTOR_VOLTAGE_LIMIT, MOTOR_VOLTAGE_LIMIT);
    electrical_angle = normalize_angle(electrical_angle);

    float alpha = -uq * sinf(electrical_angle);
    float beta = uq * cosf(electrical_angle);
    float half_bus = SUPPLY_VOLTAGE * 0.5f;

    float a = half_bus + alpha;
    float b = half_bus - 0.5f * alpha + 0.8660254f * beta;
    float c = half_bus - 0.5f * alpha - 0.8660254f * beta;

    set_duty(a / SUPPLY_VOLTAGE, b / SUPPLY_VOLTAGE, c / SUPPLY_VOLTAGE);
}

static MotorResult read_encoder(void)
{
    if (AS5600_ReadSample(&encoder) != HAL_OK)
        return stop_with_fault(MOTOR_ERR_ENCODER);

    if (!alignment_in_progress && yaw_out_of_range())
        return stop_with_fault(MOTOR_ERR_YAW_LIMIT);

    return MOTOR_OK;
}

static void reset_controllers(void)
{
    pos_integral = 0.0f;
    pos_prev_error = target_angle - control_angle();
    vel_integral = 0.0f;
    shaft_velocity = 0.0f;
    ramped_velocity = 0.0f;
    last_cycles = DWT->CYCCNT;
}

static MotorResult align_step(int step)
{
    float angle = ALIGN_ANGLE + TWO_PI * (float)step / ALIGN_STEPS;

    set_phase_voltage(ALIGN_VOLTAGE, angle);
    HAL_Delay(2);
    return read_encoder();
}

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

    /* 保持原本 700 ms 的等待，期間持續確認角度與限位。 */
    for (unsigned i = 0; i < 350U; ++i)
    {
        HAL_Delay(2);
        MotorResult result = read_encoder();
        if (result != MOTOR_OK)
            return result;
    }

    zero_electric_angle = normalize_angle(POLE_PAIRS * control_angle());

    set_phase_voltage(0.0f, 0.0f);

    for (unsigned i = 0; i < 100U; ++i)
    {
        HAL_Delay(2);
        MotorResult result = read_encoder();
        if (result != MOTOR_OK)
            return result;
    }

    return MOTOR_OK;
}

MotorResult Motor_Start(TIM_HandleTypeDef *pwm_timer)
{
    Motor_Disable();
    timer = pwm_timer;
    fault = MOTOR_OK;

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

    alignment_in_progress = true;
    result = align_sensor();
    alignment_in_progress = false;
    if (result != MOTOR_OK)
        return result;

    target_angle = control_angle();

    if (yaw_zero_valid)
    {
        update_yaw_reference();
        if (yaw_out_of_range())
            return stop_with_fault(MOTOR_ERR_YAW_LIMIT);
    }

    reset_controllers();
    return MOTOR_OK;
}

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

    float position_error = target_angle - angle;
    pos_integral = clampf(pos_integral + position_error * dt, -2.0f, 2.0f);

    float derivative = (position_error - pos_prev_error) / dt;
    pos_prev_error = position_error;

    float requested_velocity = clampf(
        POS_KP * position_error + POS_KI * pos_integral + POS_KD * derivative,
        -MAX_VELOCITY, MAX_VELOCITY);

    float max_dv = MAX_ACCELERATION * dt;
    ramped_velocity += clampf(requested_velocity - ramped_velocity,
                              -max_dv, max_dv);

    float velocity_error = ramped_velocity - shaft_velocity;
    vel_integral = clampf(vel_integral + VEL_KI * velocity_error * dt,
                          -MOTOR_VOLTAGE_LIMIT, MOTOR_VOLTAGE_LIMIT);

    float uq = VEL_KP * velocity_error + vel_integral;
    float electrical_angle = POLE_PAIRS * angle - zero_electric_angle;

    set_phase_voltage(uq, electrical_angle);
    return MOTOR_OK;
}

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

void Motor_HoldPosition(void)
{
    if (enabled)
    {
        target_angle = control_angle();
        reset_controllers();
    }
}

MotorResult Motor_MoveRelativeDeg(float delta_deg)
{
    if (!enabled)
        return fault == MOTOR_OK ? MOTOR_ERR_DISABLED : fault;

    if (!yaw_zero_valid)
        return MOTOR_ERR_UNCALIBRATED;

    if (!isfinite(delta_deg) || delta_deg < -180.0f || delta_deg > 180.0f)
        return MOTOR_ERR_COMMAND;

    float requested_yaw = current_yaw_deg() +
        ((float)sensor_direction / YAW_SIGN) * delta_deg;

    float limited_yaw = clampf(requested_yaw, YAW_MIN_DEG, YAW_MAX_DEG);
    target_angle = yaw_to_control_angle(limited_yaw);

    pos_integral = 0.0f;
    pos_prev_error = target_angle - control_angle();

    return MOTOR_OK;
}

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

void Motor_GetTelemetry(MotorTelemetry *telemetry)
{
    telemetry->angle_deg = control_angle() * RAD_TO_DEG;
    telemetry->target_deg = target_angle * RAD_TO_DEG;
    telemetry->velocity_rad_s = shaft_velocity;

    /* 停機後保留最後一筆角度，yaw_valid 用來標示目前是否有效。 */
    telemetry->yaw_valid = enabled && yaw_zero_valid;
    telemetry->yaw_deg = yaw_zero_valid ? current_yaw_deg() : 0.0f;
    telemetry->target_yaw_deg = yaw_zero_valid
        ? (target_angle - yaw_zero_control_angle) *
          (YAW_SIGN / (float)sensor_direction) * RAD_TO_DEG
        : 0.0f;
}
#endif
