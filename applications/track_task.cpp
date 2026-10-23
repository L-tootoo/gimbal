#include "track_task.hpp"

#include <cmath>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "task.h"
#include "tools/pid/pid.hpp"
#include "trajectory_task.hpp"

extern sp::DBus remote;
extern sp::RM_Motor yaw_motor;
extern sp::RM_Motor fold_motor;

namespace
{
// ==================== 总开关 ====================

constexpr bool YAW_CONTROL_ENABLED = true;
constexpr bool FOLD_CONTROL_ENABLED = true;

// ==================== 控制周期 ====================

constexpr float PID_DT = 0.01f;

static_assert(configTICK_RATE_HZ == 1000, "Tracking requires a 1 ms RTOS tick");

constexpr float PI = 3.141592654f;
constexpr float TWO_PI = 6.283185307f;

// ==================== yaw参数 ====================
// 保留你最新文件中的参数。

constexpr float YAW_KP = 0.45f;
constexpr float YAW_KI = 0.12f;
constexpr float YAW_KD = 0.015f;

constexpr float YAW_KV_FF = 0.038f;
constexpr float YAW_KA_FF = 0.0075f;

constexpr float YAW_MAX_TORQUE = TRACK_YAW_TORQUE_LIMIT;
constexpr float YAW_MAX_INTEGRAL = 0.5f;

constexpr float YAW_COMPENSATION = 0.0f;

// ==================== IMU参数 ====================

constexpr uint32_t IMU_TIMEOUT_MS = 50U;
constexpr float IMU_SPEED_FILTER_TAU = 0.01f;

// ==================== yaw预定位与同步 ====================

constexpr uint32_t APPROACH_MS = 2000U;
constexpr uint32_t APPROACH_TIMEOUT_MS = 10000U;
constexpr uint32_t SETTLE_MS = 200U;

constexpr float ARRIVAL_ERROR_RAD = 0.05f;
constexpr float ARRIVAL_SPEED_RAD_S = 0.20f;

constexpr uint32_t SYNC_TIMEOUT_MS = 3000U;
constexpr float SYNC_POSITION_ERROR_RAD = 0.003f;
constexpr float SYNC_SPEED_RAD_S = 0.25f;

// ==================== fold参数 ====================

constexpr float FOLD_TRACK_DIRECTION = 1.0f;
constexpr float FOLD_OUTPUT_DIRECTION = 1.0f;

constexpr float FOLD_KP = 0.6f;
constexpr float FOLD_KI = 0.07f;
constexpr float FOLD_KD = 0.02f;

// 前馈初始试验值，尚未辨识机构参数。
constexpr float FOLD_KV_FF = 0.0001f;  // N*m/(rad/s)
constexpr float FOLD_KA_FF = 0.00f;    // N*m/(rad/s^2)

constexpr float FOLD_MAX_TORQUE = TRACK_FOLD_TORQUE_LIMIT;
constexpr float FOLD_MAX_INTEGRAL = 0.1f;

// 初始重力补偿估算。
constexpr float FOLD_GRAVITY_MAX = 0.06f * 9.81f * 0.04f;
constexpr float FOLD_HORIZONTAL_RAD = 0.1f;
constexpr float FOLD_GRAVITY_SIGN = -0.5f;

// 保留现有摩擦补偿参数。
constexpr float FOLD_FRICTION_TORQUE = 0.02f;
constexpr float FOLD_FRICTION_DEADBAND = 0.02f;

constexpr uint32_t FOLD_BLEND_MS = 2000U;

// 原轨迹周期680ms，预瞄半周期。
constexpr uint32_t FOLD_PREVIEW_MS = 341U;

static_assert(
  FOLD_PREVIEW_MS < TRAJECTORY_PERIOD_MS, "Preview must be shorter than trajectory period");

// ==================== PID ====================

sp::PID yaw_pid(PID_DT, YAW_KP, YAW_KI, YAW_KD, YAW_MAX_TORQUE, YAW_MAX_INTEGRAL);

sp::PID fold_pid(PID_DT, FOLD_KP, FOLD_KI, FOLD_KD, FOLD_MAX_TORQUE, FOLD_MAX_INTEGRAL);

TrackCommand command{};

// ==================== IMU缓存 ====================

struct ImuFeedback
{
  float angle;
  float speed;
  uint32_t stamp_ms;
  uint32_t generation;
  bool valid;
};

ImuFeedback imu_yaw_feedback{};
ImuFeedback imu_pitch_feedback{};

bool imu_yaw_initialized = false;
bool imu_pitch_initialized = false;

float imu_previous_yaw = 0.0f;

// ==================== 辅助函数 ====================

float clamp_torque(float value, float maximum)
{
  if (value > maximum) {
    return maximum;
  }

  if (value < -maximum) {
    return -maximum;
  }

  return value;
}

// 根据当前轨迹快照匹配相位，再读取未来轨迹。
// 同时匹配角度和速度，区分经过相同角度的不同运动阶段。
static TrajectoryPoint fold_preview_point(const TrajectoryPoint & current)
{
  uint32_t best_phase_ms = 0U;
  float best_error = 1.0e30f;

  for (uint32_t phase = 0U; phase < TRAJECTORY_PERIOD_MS; ++phase) {
    const TrajectoryPoint candidate = trajectory_sample_at_ms(phase);

    const float yaw_error = candidate.yaw - current.yaw;

    const float pitch_error = candidate.pitch - current.pitch;

    // 将速度差缩放到与角度差相近的量级。
    const float yaw_speed_error = 0.05f * (candidate.yaw_vel - current.yaw_vel);

    const float pitch_speed_error = 0.05f * (candidate.pitch_vel - current.pitch_vel);

    const float error = yaw_error * yaw_error + pitch_error * pitch_error +
                        yaw_speed_error * yaw_speed_error + pitch_speed_error * pitch_speed_error;

    if (error < best_error) {
      best_error = error;
      best_phase_ms = phase;
    }
  }

  return trajectory_sample_at_ms((best_phase_ms + FOLD_PREVIEW_MS) % TRAJECTORY_PERIOD_MS);
}
// 调用时已经进入临界区。
void update_imu_yaw(float yaw_rad, uint32_t now)
{
  if (!std::isfinite(yaw_rad)) {
    imu_yaw_initialized = false;
    imu_yaw_feedback.valid = false;
    return;
  }

  if (!imu_yaw_initialized) {
    imu_previous_yaw = yaw_rad;

    imu_yaw_feedback.angle = yaw_rad;
    imu_yaw_feedback.speed = 0.0f;
    imu_yaw_feedback.stamp_ms = now;
    ++imu_yaw_feedback.generation;
    imu_yaw_feedback.valid = true;

    imu_yaw_initialized = true;
    return;
  }

  const uint32_t elapsed = now - imu_yaw_feedback.stamp_ms;

  if (elapsed == 0U) {
    return;
  }

  if (elapsed >= IMU_TIMEOUT_MS) {
    imu_yaw_initialized = false;
    imu_yaw_feedback.valid = false;
    return;
  }

  float delta = yaw_rad - imu_previous_yaw;

  // yaw跨±π展开。
  if (delta > PI) {
    delta -= TWO_PI;
  }
  else if (delta < -PI) {
    delta += TWO_PI;
  }

  const float dt = static_cast<float>(elapsed) * 0.001f;

  const float measured_speed = delta / dt;

  const float alpha = dt / (IMU_SPEED_FILTER_TAU + dt);

  const float next_angle = imu_yaw_feedback.angle + delta;

  const float next_speed =
    imu_yaw_feedback.speed + alpha * (measured_speed - imu_yaw_feedback.speed);

  if (!std::isfinite(next_angle) || !std::isfinite(next_speed)) {
    imu_yaw_initialized = false;
    imu_yaw_feedback.valid = false;
    return;
  }

  imu_yaw_feedback.angle = next_angle;
  imu_yaw_feedback.speed = next_speed;
  imu_yaw_feedback.stamp_ms = now;
  imu_yaw_feedback.valid = true;

  imu_previous_yaw = yaw_rad;
}

// 调用时已经进入临界区。
void update_imu_pitch(float pitch_rad, uint32_t now)
{
  if (!std::isfinite(pitch_rad)) {
    imu_pitch_initialized = false;
    imu_pitch_feedback.valid = false;
    return;
  }

  if (!imu_pitch_initialized) {
    imu_pitch_feedback.angle = pitch_rad;
    imu_pitch_feedback.speed = 0.0f;
    imu_pitch_feedback.stamp_ms = now;
    ++imu_pitch_feedback.generation;
    imu_pitch_feedback.valid = true;

    imu_pitch_initialized = true;
    return;
  }

  const uint32_t elapsed = now - imu_pitch_feedback.stamp_ms;

  if (elapsed == 0U) {
    return;
  }

  if (elapsed >= IMU_TIMEOUT_MS) {
    imu_pitch_initialized = false;
    imu_pitch_feedback.valid = false;
    return;
  }

  const float dt = static_cast<float>(elapsed) * 0.001f;

  // pitch使用原欧拉角，不按yaw方式累计圈数。
  const float measured_speed = (pitch_rad - imu_pitch_feedback.angle) / dt;

  const float alpha = dt / (IMU_SPEED_FILTER_TAU + dt);

  const float next_speed =
    imu_pitch_feedback.speed + alpha * (measured_speed - imu_pitch_feedback.speed);

  if (!std::isfinite(next_speed)) {
    imu_pitch_initialized = false;
    imu_pitch_feedback.valid = false;
    return;
  }

  imu_pitch_feedback.angle = pitch_rad;
  imu_pitch_feedback.speed = next_speed;
  imu_pitch_feedback.stamp_ms = now;
  imu_pitch_feedback.valid = true;
}

}  // namespace

// ==================== 调试变量 ====================

// IMU yaw目标与实际姿态。
volatile float debug_track_yaw_target = 0.0f;
volatile float debug_track_yaw_actual = 0.0f;

// Fold对应的IMU pitch目标与实际姿态。
// 不是独立pitch电机的编码器角度。
volatile float debug_track_pitch_target = 0.0f;
volatile float debug_track_pitch_actual = 0.0f;

// 保留旧接口名称，但当前表示IMU目标。
volatile float debug_track_yaw_motor_target = 0.0f;
volatile float debug_track_fold_motor_target = 0.0f;

volatile float debug_track_yaw_command = 0.0f;
volatile float debug_track_fold_command = 0.0f;

// yaw状态：
// 0失能，1追踪，2故障，3预定位，4等待轨迹相位。
volatile uint32_t debug_track_state = 0U;

// yaw故障：
// 0正常，1电机反馈异常，2预定位超时，
// 3非法数，4相位等待超时，5 IMU异常。
volatile uint32_t debug_yaw_test_fault = 0U;

volatile float debug_yaw_test_speed = 0.0f;
volatile float debug_yaw_test_p = 0.0f;
volatile float debug_yaw_test_i = 0.0f;
volatile float debug_yaw_test_d = 0.0f;
volatile float debug_yaw_test_gravity = 0.0f;

volatile float debug_yaw_target_speed = 0.0f;
volatile float debug_yaw_target_acceleration = 0.0f;
volatile float debug_yaw_velocity_ff = 0.0f;
volatile float debug_yaw_acceleration_ff = 0.0f;
volatile float debug_yaw_raw_torque = 0.0f;

volatile float debug_fold_target = 0.0f;
volatile float debug_fold_pid_out = 0.0f;
volatile float debug_fold_gravity = 0.0f;
volatile float debug_fold_command = 0.0f;
volatile bool debug_fold_enabled = false;
volatile float debug_fold_target_speed = 0.0f;
volatile float debug_fold_target_acceleration = 0.0f;
// 前馈调试量为转换到电机方向后的力矩。
volatile float debug_fold_velocity_ff = 0.0f;
volatile float debug_fold_acceleration_ff = 0.0f;
volatile float debug_fold_raw_torque = 0.0f;

// 新增诊断量，需要打印时可在plotter中extern声明。
volatile float debug_fold_imu_speed = 0.0f;

// 0正常，1电机反馈异常，2 IMU异常，3轨迹或输出非法。
volatile uint32_t debug_fold_fault = 0U;

// ==================== IMU发布接口 ====================

// 由imu_task在Mahony更新完成后调用，仅供任务调用。
void track_publish_imu(float yaw_rad, float pitch_rad)
{
  taskENTER_CRITICAL();

  const uint32_t now = osKernelSysTick();

  update_imu_yaw(yaw_rad, now);
  update_imu_pitch(pitch_rad, now);

  taskEXIT_CRITICAL();
}

// ==================== 命令读取接口 ====================

TrackCommand track_read_command()
{
  taskENTER_CRITICAL();
  const TrackCommand result = command;
  taskEXIT_CRITICAL();

  return result;
}

// ==================== fold控制 ====================

static float calculate_fold_command(bool & enabled)
{
  static bool was_enabled = false;
  static bool fault_latched = false;

  static uint32_t entry_ms = 0U;
  static uint32_t reference_generation = 0U;

  static float pitch_reference = 0.0f;

  uint32_t now;
  ImuFeedback imu;

  float encoder_angle;
  float encoder_speed;

  bool motor_alive;
  bool remote_alive;

  sp::DBusSwitchMode sw;

  taskENTER_CRITICAL();

  now = osKernelSysTick();
  imu = imu_pitch_feedback;

  encoder_angle = fold_motor.angle;
  encoder_speed = fold_motor.speed;
  motor_alive = fold_motor.is_alive(now);

  remote_alive = remote.is_alive(now);
  sw = remote.sw_r;

  taskEXIT_CRITICAL();

  const float angle = imu.angle;
  const float speed = imu.speed;

  const bool up = remote_alive && (sw == sp::DBusSwitchMode::UP);

  const bool encoder_valid =
    motor_alive && std::isfinite(encoder_angle) && std::isfinite(encoder_speed);

  const bool imu_valid = imu.valid && (now - imu.stamp_ms < IMU_TIMEOUT_MS) &&
                         std::isfinite(angle) && std::isfinite(speed);

  enabled = false;

  debug_fold_enabled = false;
  debug_fold_pid_out = 0.0f;
  debug_fold_gravity = 0.0f;
  debug_fold_command = 0.0f;
  debug_fold_target_speed = 0.0f;
  debug_fold_target_acceleration = 0.0f;
  debug_fold_velocity_ff = 0.0f;
  debug_fold_acceleration_ff = 0.0f;
  debug_fold_raw_torque = 0.0f;

  debug_track_pitch_actual = angle;
  debug_fold_imu_speed = speed;

  if (!FOLD_CONTROL_ENABLED || !up) {
    was_enabled = false;
    fold_pid.clear();

    if (remote_alive && sw != sp::DBusSwitchMode::UP) {
      fault_latched = false;
      debug_fold_fault = 0U;
    }

    debug_fold_target = angle;
    debug_track_pitch_target = angle;
    debug_track_fold_motor_target = angle;

    return 0.0f;
  }

  const bool imu_restarted = was_enabled && (imu.generation != reference_generation);

  if (!fault_latched && !encoder_valid) {
    fault_latched = true;
    debug_fold_fault = 1U;
  }

  if (!fault_latched && (!imu_valid || imu_restarted)) {
    fault_latched = true;
    debug_fold_fault = 2U;
  }

  if (fault_latched) {
    was_enabled = false;
    fold_pid.clear();
    return 0.0f;
  }

  const TrajectoryPoint point = trajectory_read();

  if (
    !std::isfinite(point.pitch) || !std::isfinite(point.pitch_vel) ||
    !std::isfinite(point.pitch_acc)) {
    fault_latched = true;
    was_enabled = false;
    debug_fold_fault = 3U;

    fold_pid.clear();
    return 0.0f;
  }

  // 上档时记录一次IMU pitch参考。
  if (!was_enabled) {
    entry_ms = now;
    pitch_reference = angle;
    reference_generation = imu.generation;

    fold_pid.clear();
    was_enabled = true;
    debug_fold_fault = 0U;
  }

  const float duration_s = static_cast<float>(FOLD_BLEND_MS) * 0.001f;
  float ratio = static_cast<float>(now - entry_ms) / static_cast<float>(FOLD_BLEND_MS);
  if (ratio > 1.0f) {
    ratio = 1.0f;
  }

  // 五次平滑接入：起止点的一、二阶导数均为0。
  const float r2 = ratio * ratio;
  const float r3 = r2 * ratio;
  const float blend = r3 * (10.0f - 15.0f * ratio + 6.0f * r2);

  float blend_vel = 0.0f;
  float blend_acc = 0.0f;
  if (now - entry_ms < FOLD_BLEND_MS) {
    const float one_minus_r = 1.0f - ratio;
    blend_vel = 30.0f * r2 * one_minus_r * one_minus_r / duration_s;
    blend_acc = 60.0f * ratio * one_minus_r * (1.0f - 2.0f * ratio) / (duration_s * duration_s);
  }

  // 位置、速度、加速度均来自同一轨迹快照。
  // 当前时刻原始目标：用于打印和评分。
  const float reference_target = pitch_reference + FOLD_TRACK_DIRECTION * blend * point.pitch;

  // 未来340ms的轨迹：用于位置控制和前馈。
  const TrajectoryPoint preview = fold_preview_point(point);

  const float target = pitch_reference + FOLD_TRACK_DIRECTION * blend * preview.pitch;

  const float target_velocity =
    FOLD_TRACK_DIRECTION * (blend_vel * preview.pitch + blend * preview.pitch_vel);

  const float target_acceleration =
    FOLD_TRACK_DIRECTION *
    (blend_acc * preview.pitch + 2.0f * blend_vel * preview.pitch_vel + blend * preview.pitch_acc);

  if (
    !std::isfinite(target) || !std::isfinite(target_velocity) ||
    !std::isfinite(target_acceleration)) {
    fault_latched = true;
    was_enabled = false;
    debug_fold_fault = 3U;
    fold_pid.clear();
    return 0.0f;
  }

  // 当前目标，提供给Plotter及grade任务。
  debug_fold_target = reference_target;
  debug_track_pitch_target = reference_target;

  // 提前后的控制目标，仅用于诊断。
  debug_track_fold_motor_target = target;

  // 单位置环，D项使用IMU pitch变化率。
  fold_pid.calc(target, angle, 0.0f, speed);

  const float error = target - angle;

  float friction = 0.0f;

  if (error > FOLD_FRICTION_DEADBAND) {
    friction = FOLD_FRICTION_TORQUE;
  }
  else if (error < -FOLD_FRICTION_DEADBAND) {
    friction = -FOLD_FRICTION_TORQUE;
  }

  // 沿用原编码器水平参考计算重力补偿。
  // 该项本来就在电机力矩坐标下。
  const float gravity =
    FOLD_GRAVITY_SIGN * FOLD_GRAVITY_MAX * std::cos(encoder_angle - FOLD_HORIZONTAL_RAD);

  const float velocity_ff = FOLD_KV_FF * target_velocity;
  const float acceleration_ff = FOLD_KA_FF * target_acceleration;

  const float raw =
    FOLD_OUTPUT_DIRECTION * (fold_pid.out + friction + velocity_ff + acceleration_ff) + gravity;

  if (!std::isfinite(raw)) {
    fault_latched = true;
    was_enabled = false;
    debug_fold_fault = 3U;

    fold_pid.clear();
    return 0.0f;
  }

  const float out = clamp_torque(raw, FOLD_MAX_TORQUE);

  enabled = true;

  debug_fold_enabled = true;
  debug_fold_target_speed = target_velocity;
  debug_fold_target_acceleration = target_acceleration;
  debug_fold_velocity_ff = FOLD_OUTPUT_DIRECTION * velocity_ff;
  debug_fold_acceleration_ff = FOLD_OUTPUT_DIRECTION * acceleration_ff;
  debug_fold_raw_torque = raw;
  debug_fold_pid_out = fold_pid.out;
  debug_fold_gravity = gravity;
  debug_fold_command = out;

  return out;
}

// ==================== 命令发布 ====================

static void publish_command(float yaw_torque, bool yaw_enabled, uint32_t now)
{
  bool fold_enabled = false;

  const float fold_torque = calculate_fold_command(fold_enabled);

  const bool yaw_ok = yaw_enabled && std::isfinite(yaw_torque);

  taskENTER_CRITICAL();

  command.yaw = yaw_ok ? yaw_torque : 0.0f;
  command.fold = fold_enabled ? fold_torque : 0.0f;

  command.stamp_ms = now;
  command.yaw_enabled = yaw_ok;
  command.fold_enabled = fold_enabled;
  command.enabled = yaw_ok || fold_enabled;

  taskEXIT_CRITICAL();

  debug_track_yaw_command = yaw_ok ? yaw_torque : 0.0f;

  debug_track_fold_command = fold_enabled ? fold_torque : 0.0f;
}

// ==================== 追踪任务 ====================

extern "C" void track_task(const void * argument)
{
  (void)argument;

  enum class State
  {
    disabled,
    approach,
    waiting,
    tracking,
    fault
  };

  State state = State::disabled;

  uint32_t approach_start = 0U;
  uint32_t sync_start = 0U;
  uint32_t settle_start = 0U;
  uint32_t yaw_reference_generation = 0U;

  bool settling = false;
  bool have_imu_reference = false;

  float entry_angle = 0.0f;
  float yaw_reference = 0.0f;
  float first_target = 0.0f;

  const TrajectoryPoint first = trajectory_sample_at_ms(0U);

  TickType_t wake_tick = xTaskGetTickCount();

  while (true) {
    uint32_t now;
    ImuFeedback imu;

    float encoder_angle;
    float encoder_speed;

    bool motor_alive;
    bool remote_alive;

    sp::DBusSwitchMode right_switch;

    taskENTER_CRITICAL();

    now = osKernelSysTick();
    imu = imu_yaw_feedback;

    encoder_angle = yaw_motor.angle;
    encoder_speed = yaw_motor.speed;
    motor_alive = yaw_motor.is_alive(now);

    remote_alive = remote.is_alive(now);
    right_switch = remote.sw_r;

    taskEXIT_CRITICAL();

    const float actual_angle = imu.angle;
    const float actual_speed = imu.speed;

    const bool switch_up = remote_alive && (right_switch == sp::DBusSwitchMode::UP);

    const bool encoder_valid =
      motor_alive && std::isfinite(encoder_angle) && std::isfinite(encoder_speed);

    const bool imu_alive = imu.valid && (now - imu.stamp_ms < IMU_TIMEOUT_MS) &&
                           std::isfinite(actual_angle) && std::isfinite(actual_speed);

    const bool imu_restarted = have_imu_reference && (imu.generation != yaw_reference_generation);

    debug_track_yaw_actual = actual_angle;
    debug_yaw_test_speed = actual_speed;

    debug_yaw_test_p = 0.0f;
    debug_yaw_test_i = 0.0f;
    debug_yaw_test_d = 0.0f;
    debug_yaw_test_gravity = 0.0f;

    debug_yaw_target_speed = 0.0f;
    debug_yaw_target_acceleration = 0.0f;
    debug_yaw_velocity_ff = 0.0f;
    debug_yaw_acceleration_ff = 0.0f;
    debug_yaw_raw_torque = 0.0f;

    // ---------- yaw关闭 ----------
    if (!YAW_CONTROL_ENABLED) {
      yaw_pid.clear();

      state = State::disabled;
      settling = false;
      have_imu_reference = false;

      debug_track_state = 0U;
      debug_yaw_test_fault = 0U;

      debug_track_yaw_target = actual_angle;
      debug_track_yaw_motor_target = actual_angle;

      publish_command(0.0f, false, now);
    }

    // ---------- 中下档或遥控离线 ----------
    else if (!switch_up) {
      yaw_pid.clear();

      settling = false;
      have_imu_reference = false;

      if (remote_alive) {
        state = State::disabled;
        debug_yaw_test_fault = 0U;
      }
      else if (state != State::fault) {
        state = State::disabled;
      }

      debug_track_yaw_target = actual_angle;
      debug_track_yaw_motor_target = actual_angle;

      debug_track_state = state == State::fault ? 2U : 0U;

      publish_command(0.0f, false, now);
    }

    // ---------- yaw异常或故障锁存 ----------
    else if (!encoder_valid || !imu_alive || imu_restarted || state == State::fault) {
      if (state != State::fault) {
        debug_yaw_test_fault = !encoder_valid ? 1U : 5U;
      }

      state = State::fault;
      yaw_pid.clear();

      debug_track_state = 2U;

      publish_command(0.0f, false, now);
    }

    // ---------- yaw姿态控制 ----------
    else {
      if (state == State::disabled) {
        yaw_reference = actual_angle;
        yaw_reference_generation = imu.generation;
        have_imu_reference = true;

        first_target = yaw_reference + first.yaw;

        entry_angle = actual_angle;
        approach_start = now;
        settling = false;

        yaw_pid.clear();
        state = State::approach;
      }

      const TrajectoryPoint point = trajectory_read();

      float target_angle = first_target;
      float target_velocity = 0.0f;
      float target_acceleration = 0.0f;

      // ---------- 靠近首点 ----------
      if (state == State::approach) {
        float ratio = static_cast<float>(now - approach_start) / static_cast<float>(APPROACH_MS);

        if (ratio > 1.0f) {
          ratio = 1.0f;
        }

        target_angle = entry_angle + ratio * (first_target - entry_angle);

        debug_track_state = 3U;

        const bool arrived = (now - approach_start >= APPROACH_MS) &&
                             (std::fabs(first_target - actual_angle) < ARRIVAL_ERROR_RAD) &&
                             (std::fabs(actual_speed) < ARRIVAL_SPEED_RAD_S);

        if (arrived) {
          if (!settling) {
            settling = true;
            settle_start = now;
          }

          if (now - settle_start >= SETTLE_MS) {
            sync_start = now;
            state = State::waiting;
          }
        }
        else {
          settling = false;
        }

        if (state == State::approach && now - approach_start >= APPROACH_TIMEOUT_MS) {
          state = State::fault;
          debug_yaw_test_fault = 2U;
        }
      }

      // ---------- 等待轨迹相位 ----------
      if (state == State::waiting) {
        target_angle = first_target;
        debug_track_state = 4U;

        const bool at_start = (std::fabs(first_target - actual_angle) < ARRIVAL_ERROR_RAD) &&
                              (std::fabs(actual_speed) < ARRIVAL_SPEED_RAD_S);

        const bool phase_at_start = std::isfinite(point.yaw) && std::isfinite(point.yaw_vel) &&
                                    (std::fabs(point.yaw - first.yaw) < SYNC_POSITION_ERROR_RAD) &&
                                    (std::fabs(point.yaw_vel) < SYNC_SPEED_RAD_S);

        if (at_start && phase_at_start) {
          state = State::tracking;
        }
        else if (now - sync_start >= SYNC_TIMEOUT_MS) {
          state = State::fault;
          debug_yaw_test_fault = 4U;
        }
      }

      // ---------- 追踪 ----------
      if (state == State::tracking) {
        target_angle = yaw_reference + point.yaw;
        target_velocity = point.yaw_vel;
        target_acceleration = point.yaw_acc;

        debug_track_state = 1U;
      }

      debug_track_yaw_target = target_angle;
      debug_track_yaw_motor_target = target_angle;

      debug_yaw_target_speed = target_velocity;
      debug_yaw_target_acceleration = target_acceleration;

      if (
        !std::isfinite(target_angle) || !std::isfinite(target_velocity) ||
        !std::isfinite(target_acceleration)) {
        state = State::fault;
        debug_yaw_test_fault = 3U;
      }

      if (state == State::fault) {
        yaw_pid.clear();
        debug_track_state = 2U;

        publish_command(0.0f, false, now);
      }
      else {
        yaw_pid.calc(target_angle, actual_angle, 0.0f, actual_speed);

        const float velocity_feedforward = YAW_KV_FF * target_velocity;

        const float acceleration_feedforward = YAW_KA_FF * target_acceleration;

        const float raw_torque =
          yaw_pid.out + velocity_feedforward + acceleration_feedforward + YAW_COMPENSATION;

        debug_yaw_test_p = yaw_pid.data.pout;
        debug_yaw_test_i = yaw_pid.data.iout;
        debug_yaw_test_d = yaw_pid.data.dout;
        debug_yaw_test_gravity = YAW_COMPENSATION;

        debug_yaw_velocity_ff = velocity_feedforward;
        debug_yaw_acceleration_ff = acceleration_feedforward;
        debug_yaw_raw_torque = raw_torque;

        if (!std::isfinite(raw_torque)) {
          state = State::fault;
          yaw_pid.clear();

          debug_track_state = 2U;
          debug_yaw_test_fault = 3U;

          publish_command(0.0f, false, now);
        }
        else {
          publish_command(clamp_torque(raw_torque, YAW_MAX_TORQUE), true, now);
        }
      }
    }

    vTaskDelayUntil(&wake_tick, pdMS_TO_TICKS(10U));
  }
}