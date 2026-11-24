#include "track.h"
#include <math.h>
#include <string.h>

// Simple 2D person follower for differential drive.
// It is NOT the full Nav2 Regulated Pure Pursuit path-following controller:
// there is no planned path, obstacle checking, or localization here.
//
// Sample defaults are placeholders; confirm Carter's geometry and limits.
// Coordinate frame: robot +X forward, +Y left, +Z up; units are meters.
// IMPORTANT: call Track_Update() at a stable rate and continuously refresh
// Track_SetPosition() when new target measurements arrive.

#define TRACK_PI 3.14159265358979323846f

// 人物追蹤的預設參數，車輪尺寸與輪距須依實車量測。
static const TrackConfig k_default_config = {
    .wheel_radius_m = 0.10f,             // PLACEHOLDER, NOT verified Carter data
    .wheel_base_m = 0.40f,               // PLACEHOLDER, NOT verified Carter data
    .follow_distance_m = 1.20f,
    .stop_tolerance_m = 0.15f,
    .hard_stop_distance_m = 0.50f,
    .max_linear_speed_mps = 0.30f,
    .max_angular_speed_radps = 0.80f,
    .max_wheel_speed_radps = 8.00f,
    .max_linear_accel_mps2 = 0.40f,
    .max_angular_accel_radps2 = 2.00f,
    .distance_kp = 0.80f,
    .heading_kp = 1.50f,
    .heading_deadband_rad = 3.0f * TRACK_PI / 180.0f,
    .turn_in_place_angle_rad = 45.0f * TRACK_PI / 180.0f,
    .target_timeout_s = 0.30f
};

// 保存控制參數、最近人物座標、車體輸出及目標經過時間。
static TrackConfig config;
static TrackPosition target;
static TrackCommand command;
static float target_age_s = 0.0f;
static bool initialized = false;

// 將控制量限制在上下界內。
static float clampf_local(float value, float min_value, float max_value)
{
    if (value < min_value) { return min_value; }
    if (value > max_value) { return max_value; }
    return value;
}

// 依每輪允許的變化量讓速度逐步接近目標。
static float approachf_local(float current, float desired, float max_step)
{
    return current + clampf_local(desired - current, -max_step, max_step);
}

// 首次呼叫時載入預設值，並清除目標與速度。
static void ensure_initialized(void)
{
    if (!initialized)
    {
        config = k_default_config;
        target = (TrackPosition){0};
        command = (TrackCommand){0};
        command.state = TRACK_STATE_NO_TARGET;
        target_age_s = 0.0f;
        initialized = true;
    }
}

// 將速度和車輪命令歸零，並記錄停止原因。
static void stop_command(TrackState state)
{
    command = (TrackCommand){0};
    command.state = state;
}

// 檢查參數有限且相互一致，避免無效除法或限幅。
static bool config_valid(const TrackConfig *cfg)
{
    if (cfg == NULL) { return false; }

    const float values[] = {
        cfg->wheel_radius_m, cfg->wheel_base_m,
        cfg->follow_distance_m, cfg->stop_tolerance_m,
        cfg->hard_stop_distance_m, cfg->max_linear_speed_mps,
        cfg->max_angular_speed_radps, cfg->max_wheel_speed_radps,
        cfg->max_linear_accel_mps2, cfg->max_angular_accel_radps2,
        cfg->distance_kp, cfg->heading_kp,
        cfg->heading_deadband_rad, cfg->turn_in_place_angle_rad,
        cfg->target_timeout_s
    };
    for (unsigned int i = 0; i < sizeof(values) / sizeof(values[0]); ++i)
    {
        if (!isfinite(values[i])) { return false; }
    }
    return cfg->wheel_radius_m > 0.0f &&
           cfg->wheel_base_m > 0.0f &&
           cfg->follow_distance_m > 0.0f &&
           cfg->stop_tolerance_m >= 0.0f &&
           cfg->hard_stop_distance_m >= 0.0f &&
           cfg->hard_stop_distance_m < cfg->follow_distance_m &&
           cfg->max_linear_speed_mps > 0.0f &&
           cfg->max_angular_speed_radps > 0.0f &&
           cfg->max_wheel_speed_radps > 0.0f &&
           cfg->max_linear_accel_mps2 > 0.0f &&
           cfg->max_angular_accel_radps2 > 0.0f &&
           cfg->distance_kp > 0.0f &&
           cfg->heading_kp > 0.0f &&
           cfg->heading_deadband_rad >= 0.0f &&
           cfg->heading_deadband_rad < cfg->turn_in_place_angle_rad &&
           cfg->turn_in_place_angle_rad < TRACK_PI &&
           cfg->target_timeout_s > 0.0f;
}

// 開機時建立車體追蹤的初始狀態。
void Track_Init(void)
{
    config = k_default_config;
    target = (TrackPosition){0};
    command = (TrackCommand){0};
    command.state = TRACK_STATE_NO_TARGET;
    target_age_s = 0.0f;
    initialized = true;
}

// 接收新人物座標並重設逾時計時，座標以車體為原點，單位為公尺。
bool Track_SetPosition(float x, float y, float z)
{
    ensure_initialized();
    if (!isfinite(x) || !isfinite(y) || !isfinite(z))
    {
        Track_Clear();
        return false;
    }

    target.x = x;
    target.y = y;
    target.z = z;
    target.valid = true;
    target_age_s = 0.0f;
    return true;
}

// 回傳最近的目標座標與有效旗標。
TrackPosition Track_GetPosition(void)
{
    ensure_initialized();
    return target;
}

// 清除目標並立即撤銷車體速度命令。
void Track_Clear(void)
{
    ensure_initialized();
    target = (TrackPosition){0};
    target_age_s = 0.0f;
    stop_command(TRACK_STATE_NO_TARGET);
}

// 回傳目前生效的車體追蹤參數。
TrackConfig Track_GetConfig(void)
{
    ensure_initialized();
    return config;
}

// 驗證後更新參數，並清除依舊參數算出的速度。
bool Track_SetConfig(const TrackConfig *new_config)
{
    ensure_initialized();
    if (!config_valid(new_config)) { return false; }
    config = *new_config;
    stop_command(TRACK_STATE_NO_TARGET); // Never retain output from old limits
    return true;
}

// 依實際經過秒數檢查目標逾時，並更新差速車的速度。
void Track_Update(float dt_s)
{
    ensure_initialized();

    // Bad update timing: fail safe instead of integrating arbitrary timing.
    if (!isfinite(dt_s) || dt_s <= 0.0f || dt_s > 0.10f)
    {
        Track_Clear();
        return;
    }

    if (!target.valid)
    {
        stop_command(TRACK_STATE_NO_TARGET);
        return;
    }

    target_age_s += dt_s;
    if (target_age_s > config.target_timeout_s)
    {
        Track_Clear();
        stop_command(TRACK_STATE_TIMEOUT);
        return;
    }

    const float x = target.x;
    const float y = target.y;
    const float distance = hypotf(x, y);
    const float angle = atan2f(y, x);
    if (!isfinite(distance) || !isfinite(angle))
    {
        Track_Clear();
        return;
    }

    // X,Y in meters in a robot-centric plane; curvature positive = left.
    float curvature = 0.0f;
    if (distance > 0.001f)
    {
        curvature = (2.0f * y) / (distance * distance);
    }

    const float abs_angle = fabsf(angle);
    const float distance_error = distance - config.follow_distance_m;
    float desired_v = 0.0f;
    float desired_w = 0.0f;
    TrackState state;
    bool immediate_linear_stop = false;

    if (distance <= config.hard_stop_distance_m)
    {
        state = TRACK_STATE_TOO_CLOSE;
        immediate_linear_stop = true;
        desired_w = 0.0f;   // Do not rotate while inside hard-stop zone
    }
    else if (abs_angle >= config.turn_in_place_angle_rad)
    {
        state = TRACK_STATE_TURNING;
        immediate_linear_stop = true;
        desired_w = config.heading_kp * angle;
    }
    else if (distance <
             config.follow_distance_m - config.stop_tolerance_m)
    {
        state = TRACK_STATE_FOLLOWING;

        desired_v = config.distance_kp * distance_error;

        desired_v = clampf_local(
            desired_v,
            -config.max_linear_speed_mps,
            0.0f
        );

        desired_v *= cosf(angle);

        desired_w = config.heading_kp * angle;
    }
    else if (distance <= config.follow_distance_m + config.stop_tolerance_m)
    {
        state = TRACK_STATE_AT_DISTANCE;
        immediate_linear_stop = true;
        if (abs_angle > config.heading_deadband_rad)
        {
            desired_w = config.heading_kp * angle;
        }
    }
    else
    {
        state = TRACK_STATE_FOLLOWING;
        desired_v = config.distance_kp * distance_error;
        desired_v = clampf_local(desired_v, 0.0f, config.max_linear_speed_mps);
        // Slow translation during steering, never reverse toward target.
        desired_v *= cosf(angle);
        // Curvature pursuit + heading correction (point target, no path).
        desired_w = desired_v * curvature;
        if (abs_angle > config.heading_deadband_rad)
        {
            desired_w += 0.5f * config.heading_kp * angle;
        }
    }

    desired_w = clampf_local(desired_w,
                             -config.max_angular_speed_radps,
                              config.max_angular_speed_radps);

    float v = immediate_linear_stop
              ? 0.0f
              : approachf_local(command.linear_vel, desired_v,
                                config.max_linear_accel_mps2 * dt_s);
    float w = (state == TRACK_STATE_TOO_CLOSE)
              ? 0.0f
              : approachf_local(command.angular_vel, desired_w,
                                config.max_angular_accel_radps2 * dt_s);

    // Differential-drive inverse kinematics; wheel outputs in rad/s.
    float left = (v - (w * config.wheel_base_m * 0.5f)) / config.wheel_radius_m;
    float right = (v + (w * config.wheel_base_m * 0.5f)) / config.wheel_radius_m;
    float largest = fmaxf(fabsf(left), fabsf(right));
    if (largest > config.max_wheel_speed_radps)
    {
        // Preserve turning radius by scaling BOTH chassis command values.
        float scale = config.max_wheel_speed_radps / largest;
        v *= scale;
        w *= scale;
        left *= scale;
        right *= scale;
    }

    command.linear_vel = v;
    command.angular_vel = w;
    command.distance = distance;
    command.target_angle = angle;
    command.curvature = curvature;
    command.left_wheel_vel = left;
    command.right_wheel_vel = right;
    command.target_valid = true;
    command.state = state;
}

// 回傳最近一輪算出的車體與車輪速度。
TrackCommand Track_GetCommand(void)
{
    ensure_initialized();
    return command;
}
