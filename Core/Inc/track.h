#ifndef TRACK_H
#define TRACK_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Coordinate convention (robot base_link, meters):
//   +X: forward, +Y: left, +Z: up.
// The caller must transform camera/world coordinates before calling
// Track_SetPosition(). This controller uses X and Y only.
typedef struct
{
    float x;
    float y;
    float z;
    bool valid;
} TrackPosition;

typedef enum
{
    TRACK_STATE_NO_TARGET = 0,
    TRACK_STATE_FOLLOWING,
    TRACK_STATE_TURNING,
    TRACK_STATE_AT_DISTANCE,
    TRACK_STATE_TOO_CLOSE,
    TRACK_STATE_TIMEOUT
} TrackState;

typedef struct
{
    float wheel_radius_m;            // Drive wheel radius, m
    float wheel_base_m;              // Distance between drive wheel centers, m
    float follow_distance_m;         // Desired distance to person, m
    float stop_tolerance_m;          // Stop if d <= follow_distance + tolerance, m
    float hard_stop_distance_m;      // Minimum safe approach distance, m

    float max_linear_speed_mps;      // Limit on forward speed, m/s
    float max_angular_speed_radps;   // Limit on yaw speed, rad/s
    float max_wheel_speed_radps;     // Absolute wheel speed limit, rad/s
    float max_linear_accel_mps2;    // Slew limit, m/s^2
    float max_angular_accel_radps2; // Slew limit, rad/s^2

    float distance_kp;              // Speed gain, (m/s)/m
    float heading_kp;               // Heading correction, 1/s
    float heading_deadband_rad;     // Ignore very small heading errors, rad
    float turn_in_place_angle_rad;  // Pivot when heading error >= this, rad
    float target_timeout_s;         // Stop if target not refreshed in this time
} TrackConfig;

typedef struct
{
    float linear_vel;       // Robot commanded forward speed, m/s
    float angular_vel;      // Robot commanded yaw rate, rad/s (+ = left)
    float distance;         // Horizontal distance to target, m
    float target_angle;     // atan2(y, x), rad (+ = left)
    float curvature;        // Point-pursuit curvature, 1/m
    float left_wheel_vel;   // Left drive wheel commanded speed, rad/s
    float right_wheel_vel;  // Right drive wheel commanded speed, rad/s
    bool target_valid;      // False for missing/stale/invalid target
    TrackState state;
} TrackCommand;

// Initializing is recommended once at startup. Defaults are provisional.
void Track_Init(void);

// Existing API retained. SetPosition must be called on every new observation.
bool Track_SetPosition(float x, float y, float z);
TrackPosition Track_GetPosition(void);
void Track_Clear(void);

// Runtime parameter editing (false if any parameter is invalid).
TrackConfig Track_GetConfig(void);
bool Track_SetConfig(const TrackConfig *config);

// Run periodically (example: 0.02f at 50 Hz). All control state is internal.
void Track_Update(float dt_s);
TrackCommand Track_GetCommand(void);

#ifdef __cplusplus
}
#endif

#endif // TRACK_H
