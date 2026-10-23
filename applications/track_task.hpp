#ifndef TRACK_TASK_HPP
#define TRACK_TASK_HPP
#include <stdint.h>
// All angles rad; motor commands N*m at the configured motor output shaft.
// Shared hard command limits: both track and CAN use the same values.
constexpr float TRACK_YAW_TORQUE_LIMIT = 0.41f;
constexpr float TRACK_FOLD_TORQUE_LIMIT = 0.122f;
struct TrackCommand
{
  float yaw;
  float fold;
  uint32_t stamp_ms;
  bool enabled;
  bool yaw_enabled;
  bool fold_enabled;
};
void track_publish_imu(float yaw_rad, float pitch_rad);
TrackCommand track_read_command();
extern volatile float debug_track_yaw_target;
extern volatile float debug_track_yaw_actual;
extern volatile float debug_track_pitch_target;
extern volatile float debug_track_pitch_actual;
extern volatile float debug_track_yaw_motor_target;
extern volatile float debug_track_fold_motor_target;
extern volatile float debug_track_yaw_command;
extern volatile float debug_track_fold_command;
extern volatile float debug_fold_target;
extern volatile float debug_fold_pid_out;
extern volatile float debug_fold_gravity;
extern volatile float debug_fold_command;
extern volatile bool debug_fold_enabled;
extern volatile float debug_fold_target_speed;
extern volatile float debug_fold_target_acceleration;
extern volatile float debug_fold_velocity_ff;
extern volatile float debug_fold_acceleration_ff;
extern volatile float debug_fold_raw_torque;
extern volatile uint32_t
  debug_track_state;  // 0 disabled, 1 tracking, 2 fault, 3 approach, 4 waiting for trajectory phase
#ifdef __cplusplus
extern "C" {
#endif
void track_task(void const * argument);
#ifdef __cplusplus
}
#endif
#endif
