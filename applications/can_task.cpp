#include <cmath>
#include <cstdint>

#include "can.h"
#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/pid/pid.hpp"

extern sp::DBus remote;

// ==================== CAN 与电机 ====================

sp::CAN can1(&hcan1);
sp::CAN can2(&hcan2);

// CAN1：Pitch GM6020，ID 1。
sp::RM_Motor pitch_motor(1, sp::RM_Motors::GM6020);

// CAN2：Yaw GM6020，ID 2，只读取反馈。
sp::RM_Motor yaw_motor(2, sp::RM_Motors::GM6020);

// CAN1：M2006，电调 ID 3，只读取反馈。
sp::RM_Motor fold_motor(3, sp::RM_Motors::M2006, sp::M2006_P36);

// ==================== 控制参数 ====================

static constexpr bool motor_output_enable = true;

// 正常控制目标：本次测试 -1.4 rad。
static constexpr float pitch_target_rad = -1.4f;

// 水平参考：只用于重力补偿，不随目标一起修改。
static constexpr float pitch_horizontal_rad = -1.6f;

// 正常工作范围，单位 rad。
static constexpr float pitch_min_rad = -1.95f;
static constexpr float pitch_max_rad = -0.07f;

// 轻微越界恢复范围。
static constexpr float pitch_recovery_min_rad = -1.97f;
static constexpr float pitch_recovery_max_rad = -0.05f;

// 恢复到工作边界内侧 0.01 rad。
static constexpr float pitch_recovery_margin_rad = 0.01f;

static constexpr uint32_t pitch_recovery_timeout_ms = 1500;

// 10 ms 任务周期。
static constexpr float pid_dt = 0.01f;

static constexpr float pitch_kp = 0.80f;
static constexpr float pitch_ki = 0.0f;
static constexpr float pitch_kd = 0.0f;

// PID修正项限幅。
static constexpr float pid_max_out = 0.25f;
static constexpr float pid_max_iout = 0.0f;

// PID、重力补偿、阻尼相加后的总限幅。
static constexpr float pitch_command_max = 0.25f;

// 重力补偿幅值，单位 N·m。
static constexpr float gravity_torque_max = 0.21f;

// 速度阻尼，单位 N·m/(rad/s)。
static constexpr float pitch_damping = 0.02f;

// ==================== PID ====================

sp::PID pitch_pid(pid_dt, pitch_kp, pitch_ki, pitch_kd, pid_max_out, pid_max_iout);

// ==================== 控制状态 ====================

enum class PitchState
{
  disabled,
  normal,
  recover_lower,
  recover_upper,
  fault
};

static PitchState pitch_state = PitchState::disabled;

static bool pitch_armed = false;
static uint32_t recovery_start_tick = 0;

// ==================== 调试变量 ====================

volatile uint32_t can1_rx_count = 0;
volatile uint32_t can2_rx_count = 0;

volatile uint32_t pitch_rx_count = 0;
volatile uint32_t yaw_rx_count = 0;
volatile uint32_t fold_rx_count = 0;

volatile uint32_t debug_can1_rx_id = 0;
volatile uint32_t debug_can2_rx_id = 0;

volatile bool debug_remote_alive = false;
volatile bool debug_pitch_alive = false;
volatile bool debug_pitch_enabled = false;
volatile bool debug_pitch_limit_fault = false;

volatile float debug_pitch_target = pitch_target_rad;
volatile float debug_pitch_angle = 0.0f;
volatile float debug_pitch_speed = 0.0f;
volatile float debug_pitch_torque = 0.0f;
volatile float debug_pitch_error = 0.0f;

volatile float debug_pitch_pid_out = 0.0f;
volatile float debug_pitch_gravity_comp = 0.0f;
volatile float debug_pitch_damping = 0.0f;
volatile float debug_pitch_command = 0.0f;

// 0失能，1正常，2下边界恢复，3上边界恢复，4故障。
volatile uint32_t debug_pitch_state = 0;

volatile bool debug_yaw_alive = false;
volatile float debug_yaw_angle = 0.0f;
volatile float debug_yaw_speed = 0.0f;

volatile bool debug_fold_alive = false;
volatile float debug_fold_angle = 0.0f;
volatile float debug_fold_speed = 0.0f;

// ==================== 辅助函数 ====================

static float clamp_output(float value, float limit)
{
  if (value > limit) {
    return limit;
  }

  if (value < -limit) {
    return -limit;
  }

  return value;
}

static bool angle_in_work_range(float angle)
{
  return std::isfinite(angle) && angle >= pitch_min_rad && angle <= pitch_max_rad;
}

static bool angle_in_recovery_range(float angle)
{
  return std::isfinite(angle) && angle >= pitch_recovery_min_rad && angle <= pitch_recovery_max_rad;
}

static bool configuration_ok()
{
  return std::isfinite(pitch_recovery_min_rad) && std::isfinite(pitch_min_rad) &&
         std::isfinite(pitch_max_rad) && std::isfinite(pitch_recovery_max_rad) &&
         std::isfinite(pitch_horizontal_rad) && std::isfinite(pitch_recovery_margin_rad) &&
         std::isfinite(gravity_torque_max) && std::isfinite(pitch_damping) &&
         std::isfinite(pitch_command_max) && pitch_damping >= 0.0f && pitch_command_max > 0.0f &&
         pitch_recovery_margin_rad > 0.0f && pitch_recovery_min_rad < pitch_min_rad &&
         pitch_min_rad < pitch_max_rad && pitch_max_rad < pitch_recovery_max_rad &&
         pitch_min_rad + pitch_recovery_margin_rad < pitch_max_rad - pitch_recovery_margin_rad &&
         angle_in_work_range(pitch_target_rad);
}

static bool state_is_active()
{
  return pitch_state == PitchState::normal || pitch_state == PitchState::recover_lower ||
         pitch_state == PitchState::recover_upper;
}

static void enter_fault()
{
  pitch_state = PitchState::fault;
  pitch_armed = false;
  pitch_pid.clear();
}

static void enter_recovery(PitchState new_state, uint32_t now)
{
  pitch_state = new_state;
  recovery_start_tick = now;
  pitch_pid.clear();
}

static uint32_t state_number()
{
  switch (pitch_state) {
    case PitchState::disabled:
      return 0;

    case PitchState::normal:
      return 1;

    case PitchState::recover_lower:
      return 2;

    case PitchState::recover_upper:
      return 3;

    default:
      return 4;
  }
}

// ==================== CAN 任务 ====================

extern "C" void can_task(void const * argument)
{
  (void)argument;

  can1.config();
  can1.start();

  can2.config();
  can2.start();

  const bool config_valid = configuration_ok();

  while (true) {
    const uint32_t now = osKernelSysTick();

    const bool pitch_alive = pitch_motor.is_alive(osKernelSysTick());

    const float pitch_angle = pitch_motor.angle;
    const float pitch_speed = pitch_motor.speed;

    const bool feedback_valid =
      pitch_alive && std::isfinite(pitch_angle) && std::isfinite(pitch_speed);

    const bool within_recovery = feedback_valid && angle_in_recovery_range(pitch_angle);

    const bool remote_alive = remote.is_alive(osKernelSysTick());

    const bool switch_up = remote_alive && remote.sw_r == sp::DBusSwitchMode::UP;

    const bool switch_disabled = remote_alive && (remote.sw_r == sp::DBusSwitchMode::MID ||
                                                  remote.sw_r == sp::DBusSwitchMode::DOWN);

    // ---------- 遥控失能与启动 ----------

    if (!config_valid) {
      enter_fault();
    }
    else if (!remote_alive) {
      // 重新连接后必须先经过中/下档。
      pitch_state = PitchState::disabled;
      pitch_armed = false;
      pitch_pid.clear();
    }
    else if (switch_disabled) {
      // 中/下档：零指令，并准备下一次上档启动。
      pitch_state = PitchState::disabled;
      pitch_pid.clear();

      pitch_armed = within_recovery;
    }
    else if (!switch_up) {
      pitch_state = PitchState::disabled;
      pitch_armed = false;
      pitch_pid.clear();
    }
    else if (!within_recovery) {
      // 上档期间反馈异常或超出恢复范围。
      enter_fault();
    }
    else if (pitch_state == PitchState::disabled && pitch_armed) {
      pitch_armed = false;
      pitch_pid.clear();

      if (pitch_angle < pitch_min_rad) {
        enter_recovery(PitchState::recover_lower, now);
      }
      else if (pitch_angle > pitch_max_rad) {
        enter_recovery(PitchState::recover_upper, now);
      }
      else {
        pitch_state = PitchState::normal;
      }
    }

    // ---------- 恢复模式 ----------

    if (state_is_active()) {
      if (pitch_state == PitchState::normal) {
        if (pitch_angle < pitch_min_rad) {
          enter_recovery(PitchState::recover_lower, now);
        }
        else if (pitch_angle > pitch_max_rad) {
          enter_recovery(PitchState::recover_upper, now);
        }
      }

      if (pitch_state == PitchState::recover_lower) {
        if (pitch_angle >= pitch_min_rad + pitch_recovery_margin_rad) {
          pitch_state = PitchState::normal;
          pitch_pid.clear();
        }
        else if (now - recovery_start_tick >= pitch_recovery_timeout_ms) {
          enter_fault();
        }
      }
      else if (pitch_state == PitchState::recover_upper) {
        if (pitch_angle <= pitch_max_rad - pitch_recovery_margin_rad) {
          pitch_state = PitchState::normal;
          pitch_pid.clear();
        }
        else if (now - recovery_start_tick >= pitch_recovery_timeout_ms) {
          enter_fault();
        }
      }
    }

    // ---------- 当前控制目标 ----------

    float control_target = pitch_target_rad;

    if (pitch_state == PitchState::recover_lower) {
      control_target = pitch_min_rad + pitch_recovery_margin_rad;
    }
    else if (pitch_state == PitchState::recover_upper) {
      control_target = pitch_max_rad - pitch_recovery_margin_rad;
    }

    const bool pitch_enabled = config_valid && switch_up && within_recovery && state_is_active() &&
                               angle_in_work_range(control_target);

    // ---------- PID、重力补偿、速度阻尼 ----------

    float pid_output = 0.0f;
    float gravity_comp = 0.0f;
    float damping_torque = 0.0f;
    float pitch_command = 0.0f;

    if (motor_output_enable && pitch_enabled) {
      pitch_pid.calc(control_target, pitch_angle);
      pid_output = pitch_pid.out;

      gravity_comp = gravity_torque_max * std::cos(pitch_angle - pitch_horizontal_rad);

      damping_torque = -pitch_damping * pitch_speed;

      pitch_command = pid_output + gravity_comp + damping_torque;

      if (!std::isfinite(pitch_command)) {
        pitch_command = 0.0f;
        enter_fault();
      }
      else {
        pitch_command = clamp_output(pitch_command, pitch_command_max);

        // 下边界恢复：只允许正力矩。
        if (pitch_state == PitchState::recover_lower && pitch_command < 0.0f) {
          pitch_command = 0.0f;
        }

        // 上边界恢复：只允许负力矩。
        if (pitch_state == PitchState::recover_upper && pitch_command > 0.0f) {
          pitch_command = 0.0f;
        }
      }
    }
    else {
      pitch_pid.clear();
    }

    if (!state_is_active()) {
      pitch_command = 0.0f;
    }

    // ---------- 更新调试变量 ----------

    debug_remote_alive = remote_alive;
    debug_pitch_alive = pitch_alive;

    debug_pitch_enabled = pitch_enabled && state_is_active();

    debug_pitch_target = control_target;
    debug_pitch_angle = pitch_angle;
    debug_pitch_speed = pitch_speed;
    debug_pitch_torque = pitch_motor.torque;
    debug_pitch_error = control_target - pitch_angle;

    debug_pitch_limit_fault = !config_valid || !feedback_valid ||
                              !angle_in_work_range(pitch_angle) || pitch_state == PitchState::fault;

    debug_pitch_pid_out = pid_output;
    debug_pitch_gravity_comp = gravity_comp;
    debug_pitch_damping = damping_torque;
    debug_pitch_command = pitch_command;
    debug_pitch_state = state_number();

    debug_yaw_alive = yaw_motor.is_alive(osKernelSysTick());
    debug_yaw_angle = yaw_motor.angle;
    debug_yaw_speed = yaw_motor.speed;

    debug_fold_alive = fold_motor.is_alive(osKernelSysTick());
    debug_fold_angle = fold_motor.angle;
    debug_fold_speed = fold_motor.speed;

    // ---------- Pitch CAN1 输出 ----------

    // 失能或故障时，也持续发送零指令。
    pitch_motor.cmd(pitch_command);

    for (uint8_t i = 0; i < 8; ++i) {
      can1.tx_data[i] = 0;
    }

    pitch_motor.write(can1.tx_data);
    can1.send(pitch_motor.tx_id);

    // Yaw、M2006只读取反馈。

    osDelay(10);
  }
}

// ==================== CAN 接收回调 ====================

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  const uint32_t stamp_ms = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (hcan == &hcan1) {
      can1.recv();

      ++can1_rx_count;
      debug_can1_rx_id = can1.rx_id;

      if (can1.rx_id == pitch_motor.rx_id) {
        pitch_motor.read(can1.rx_data, stamp_ms);
        ++pitch_rx_count;
      }
      else if (can1.rx_id == fold_motor.rx_id) {
        fold_motor.read(can1.rx_data, stamp_ms);
        ++fold_rx_count;
      }
    }
    else if (hcan == &hcan2) {
      can2.recv();

      ++can2_rx_count;
      debug_can2_rx_id = can2.rx_id;

      if (can2.rx_id == yaw_motor.rx_id) {
        yaw_motor.read(can2.rx_data, stamp_ms);
        ++yaw_rx_count;
      }
    }
    else {
      break;
    }
  }
}