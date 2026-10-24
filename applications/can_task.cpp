#include <cmath>
#include <cstdint>

#include "FreeRTOS.h"
#include "can.h"
#include "cmsis_os.h"
#include "distance_task.hpp"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "task.h"
#include "tools/pid/pid.hpp"
#include "track_task.hpp"

extern sp::DBus remote;
// Pitch固定位置控制，角度rad，力矩N·m。
// 以下PID及限幅为初始调试值。
static constexpr float PITCH_HOLD_RAD = -0.15f;
static constexpr float PITCH_FOLD_HOLD_RAD = -0.1f;
static constexpr float PITCH_FOLD_NEAR_RAD = -1.5f;
static constexpr uint16_t PITCH_TRIGGER_DISTANCE_CM = 30U;
static constexpr uint32_t PITCH_FOLD_HOLD_MS = 1000U;
static constexpr uint32_t PITCH_FOLD_SETTLE_MS = 100U;
static constexpr float PITCH_FOLD_ARRIVAL_ERROR_RAD = 0.05f;
static constexpr float PITCH_FOLD_ARRIVAL_SPEED_RAD_S = 0.20f;
static constexpr float PITCH_PID_DT = 0.01f;

static constexpr float PITCH_KP = 0.5f;
static constexpr float PITCH_KI = 0.0f;
static constexpr float PITCH_KD = 0.04f;

static constexpr float PITCH_MAX_TORQUE = 0.8f;
static constexpr float PITCH_MAX_INTEGRAL = 0.0f;

// 重力补偿：质量kg，重心到转轴距离m，力矩N*m。
static constexpr float PITCH_LOAD_MASS_KG = 0.08f;
static constexpr float PITCH_COM_DISTANCE_M = 0.08f;
static constexpr float PITCH_GRAVITY_MAX = PITCH_LOAD_MASS_KG * 9.81f * PITCH_COM_DISTANCE_M;
// 水平参考采用本次上传参数-1.4rad；补偿方向为正，需与实物一致。
static constexpr float PITCH_HORIZONTAL_RAD = -1.4f;
static constexpr float PITCH_GRAVITY_SIGN = 1.0f;

static sp::PID pitch_pid(
  PITCH_PID_DT, PITCH_KP, PITCH_KI, PITCH_KD, PITCH_MAX_TORQUE, PITCH_MAX_INTEGRAL);

volatile float debug_pitch_target = PITCH_HOLD_RAD;
volatile float debug_can_pitch_command = 0.0f;
volatile float debug_pitch_gravity = 0.0f;
volatile float debug_pitch_raw_torque = 0.0f;
volatile bool debug_can_pitch_enabled = false;

// 中档使用独立位置保持PID，不改变上档track任务的参数和状态。
static sp::PID fold_mode_yaw_pid(0.01f, 0.45f, 0.12f, 0.015f, TRACK_YAW_TORQUE_LIMIT, 0.5f);
static sp::PID fold_mode_fold_pid(0.01f, 0.6f, 0.07f, 0.02f, TRACK_FOLD_TORQUE_LIMIT, 0.1f);
volatile float debug_fold_mode_yaw_target = 0.0f;
volatile float debug_fold_mode_fold_target = 0.0f;
volatile bool debug_fold_mode_active = false;
volatile bool debug_pitch_distance_valid = false;
volatile float debug_pitch_distance_cm = -1.0f;
volatile bool debug_pitch_near = false;
// 0待机/返回，1前往折叠位置，2到位保持。
volatile uint32_t debug_pitch_fold_state = 0U;

// ==================== CAN与电机 ====================

sp::CAN can1(&hcan1);
sp::CAN can2(&hcan2);

// CAN1：Pitch GM6020，ID1。
sp::RM_Motor pitch_motor(1, sp::RM_Motors::GM6020);

// CAN2：Yaw GM6020，ID2，多圈反馈。
sp::RM_Motor yaw_motor(2, sp::RM_Motors::GM6020, 1.0f, true);

// CAN1：Fold M2006，ID3。
// 内部减速比由电机类处理。
sp::RM_Motor fold_motor(3, sp::RM_Motors::M2006, sp::M2006_P36);

// ==================== 接收调试变量 ====================

volatile uint32_t can1_rx_count = 0;
volatile uint32_t can2_rx_count = 0;

volatile uint32_t pitch_rx_count = 0;
volatile uint32_t yaw_rx_count = 0;
volatile uint32_t fold_rx_count = 0;

volatile uint32_t debug_can1_rx_id = 0;
volatile uint32_t debug_can2_rx_id = 0;

volatile bool debug_remote_alive = false;

volatile bool debug_pitch_alive = false;
volatile bool debug_yaw_alive = false;
volatile bool debug_fold_alive = false;

volatile float debug_pitch_angle = 0.0f;
volatile float debug_pitch_speed = 0.0f;

volatile float debug_yaw_angle = 0.0f;
volatile float debug_yaw_speed = 0.0f;

volatile float debug_fold_angle = 0.0f;
volatile float debug_fold_speed = 0.0f;
volatile float debug_fold_feedback_torque = 0.0f;

// ==================== CAN输出调试变量 ====================

volatile float debug_can_yaw_command = 0.0f;
volatile bool debug_can_yaw_enabled = false;

volatile float debug_can_fold_command = 0.0f;
volatile bool debug_can_fold_enabled = false;

// ==================== 辅助函数 ====================

static float clamp_torque(float value, float limit)
{
  if (value > limit) {
    return limit;
  }

  if (value < -limit) {
    return -limit;
  }

  return value;
}

static void clear_buffer(uint8_t * data)
{
  for (uint8_t i = 0; i < 8; ++i) {
    data[i] = 0;
  }
}

// ==================== CAN任务 ====================

extern "C" void can_task(void const * argument)
{
  (void)argument;

  can1.config();
  can1.start();

  can2.config();
  can2.start();

  enum class PitchFoldState
  {
    idle,
    approaching,
    holding
  };
  PitchFoldState pitch_fold_state = PitchFoldState::idle;
  bool pitch_trigger_armed = true;
  bool pitch_settling = false;
  uint32_t pitch_settle_start = 0U;
  uint32_t pitch_hold_start = 0U;
  bool was_middle = false;
  bool was_up = false;
  bool middle_yaw_ready = false;
  bool middle_fold_ready = false;
  float middle_yaw_target = 0.0f;
  float middle_fold_target = 0.0f;

  while (true) {
    // 上档使用track命令，中档使用CAN任务内的固定位置控制。
    const TrackCommand track = track_read_command();

    const uint32_t now = osKernelSysTick();

    float pitch_angle;
    float pitch_speed;

    float yaw_angle;
    float yaw_speed;

    float fold_angle;
    float fold_speed;
    float fold_feedback_torque;

    bool pitch_alive;
    bool yaw_alive;
    bool fold_alive;

    bool remote_alive;
    sp::DBusSwitchMode right_switch;

    // ---------- 获取反馈快照 ----------

    taskENTER_CRITICAL();

    pitch_angle = pitch_motor.angle;
    pitch_speed = pitch_motor.speed;

    yaw_angle = yaw_motor.angle;
    yaw_speed = yaw_motor.speed;

    fold_angle = fold_motor.angle;
    fold_speed = fold_motor.speed;
    fold_feedback_torque = fold_motor.torque;

    pitch_alive = pitch_motor.is_alive(now);
    yaw_alive = yaw_motor.is_alive(now);
    fold_alive = fold_motor.is_alive(now);

    remote_alive = remote.is_alive(now);
    right_switch = remote.sw_r;

    taskEXIT_CRITICAL();

    const bool switch_up = remote_alive && right_switch == sp::DBusSwitchMode::UP;

    const bool switch_middle = remote_alive && right_switch == sp::DBusSwitchMode::MID;
    const bool command_fresh = now - track.stamp_ms < 30U;
    const bool yaw_feedback_valid =
      yaw_alive && std::isfinite(yaw_angle) && std::isfinite(yaw_speed);
    const bool fold_feedback_valid =
      fold_alive && std::isfinite(fold_angle) && std::isfinite(fold_speed);

    if (switch_middle && !was_middle) {
      // 仅在进入中档时记录一次；测距触发不会重建Yaw/Fold目标。
      middle_yaw_target = yaw_angle;
      middle_fold_target = fold_angle;
      middle_yaw_ready = yaw_feedback_valid;
      middle_fold_ready = fold_feedback_valid;
      fold_mode_yaw_pid.clear();
      fold_mode_fold_pid.clear();
    }
    if (!switch_middle) {
      middle_yaw_ready = false;
      middle_fold_ready = false;
      fold_mode_yaw_pid.clear();
      fold_mode_fold_pid.clear();
    }
    // 中档反馈中断后停止该轴，重新拨档才能重新建立保持参考。
    if (!yaw_feedback_valid) middle_yaw_ready = false;
    if (!fold_feedback_valid) middle_fold_ready = false;
    const bool middle_entered = switch_middle && !was_middle;
    if (switch_middle != was_middle || switch_up != was_up) pitch_pid.clear();
    was_middle = switch_middle;
    was_up = switch_up;

    DistanceStatus distance{};
    const bool distance_valid = distance_get_status(&distance) && distance.valid;
    const bool near =
      switch_middle && distance_valid && distance.distance_cm < PITCH_TRIGGER_DISTANCE_CM;
    const bool pitch_feedback_valid =
      pitch_alive && std::isfinite(pitch_angle) && std::isfinite(pitch_speed);
    if (!switch_middle || middle_entered) {
      pitch_fold_state = PitchFoldState::idle;
      pitch_trigger_armed = true;
      pitch_settling = false;
    }
    if (switch_middle) {
      if (!pitch_feedback_valid) {
        pitch_fold_state = PitchFoldState::idle;
        pitch_settling = false;
        pitch_trigger_armed = false;
      }
      else if (pitch_fold_state == PitchFoldState::idle) {
        // 只用有效的远距离重新武装，测距丢失不会造成再次触发。
        if (distance_valid && distance.distance_cm >= PITCH_TRIGGER_DISTANCE_CM) {
          pitch_trigger_armed = true;
        }
        if (near && pitch_trigger_armed) {
          pitch_fold_state = PitchFoldState::approaching;
          pitch_trigger_armed = false;
          pitch_settling = false;
        }
      }
      else if (pitch_fold_state == PitchFoldState::approaching) {
        const bool arrived =
          std::fabs(pitch_angle - PITCH_FOLD_NEAR_RAD) < PITCH_FOLD_ARRIVAL_ERROR_RAD &&
          std::fabs(pitch_speed) < PITCH_FOLD_ARRIVAL_SPEED_RAD_S;
        if (!arrived) {
          pitch_settling = false;
        }
        else if (!pitch_settling) {
          pitch_settling = true;
          pitch_settle_start = now;
        }
        else if (now - pitch_settle_start >= PITCH_FOLD_SETTLE_MS) {
          pitch_fold_state = PitchFoldState::holding;
          pitch_hold_start = now;
        }
      }
      else if (now - pitch_hold_start >= PITCH_FOLD_HOLD_MS) {
        pitch_fold_state = PitchFoldState::idle;
        pitch_settling = false;
      }
    }
    const float pitch_target =
      switch_middle
        ? (pitch_fold_state != PitchFoldState::idle ? PITCH_FOLD_NEAR_RAD : PITCH_FOLD_HOLD_RAD)
        : PITCH_HOLD_RAD;
    debug_pitch_fold_state = static_cast<uint32_t>(pitch_fold_state);
    debug_fold_mode_active = switch_middle;
    debug_fold_mode_yaw_target = middle_yaw_target;
    debug_fold_mode_fold_target = middle_fold_target;
    debug_pitch_distance_valid = distance_valid;
    debug_pitch_distance_cm = distance_valid ? static_cast<float>(distance.distance_cm) : -1.0f;
    debug_pitch_near = near;

    // ---------- Yaw独立输出检查 ----------

    bool yaw_enabled = switch_up && command_fresh && track.yaw_enabled && yaw_alive &&
                       std::isfinite(yaw_angle) && std::isfinite(yaw_speed) &&
                       std::isfinite(track.yaw);

    float yaw_command = 0.0f;

    if (yaw_enabled) {
      yaw_command = clamp_torque(track.yaw, TRACK_YAW_TORQUE_LIMIT);
    }

    // ---------- Fold独立输出检查 ----------

    bool fold_enabled = switch_up && command_fresh && track.fold_enabled && fold_alive &&
                        std::isfinite(fold_angle) && std::isfinite(fold_speed) &&
                        std::isfinite(track.fold);

    float fold_command = 0.0f;

    if (fold_enabled) {
      fold_command = clamp_torque(track.fold, TRACK_FOLD_TORQUE_LIMIT);
    }

    // ---------- 中档：Yaw/Fold固定位置，保留实际速度阻尼 ----------
    if (switch_middle) {
      yaw_enabled = false;
      fold_enabled = false;
      yaw_command = 0.0f;
      fold_command = 0.0f;
      if (middle_yaw_ready) {
        fold_mode_yaw_pid.calc(middle_yaw_target, yaw_angle, 0.0f, yaw_speed);
        if (std::isfinite(fold_mode_yaw_pid.out)) {
          yaw_command = clamp_torque(fold_mode_yaw_pid.out, TRACK_YAW_TORQUE_LIMIT);
          yaw_enabled = true;
        }
        else
          middle_yaw_ready = false;
      }
      if (middle_fold_ready) {
        fold_mode_fold_pid.calc(middle_fold_target, fold_angle, 0.0f, fold_speed);
        if (std::isfinite(fold_mode_fold_pid.out)) {
          fold_command = clamp_torque(fold_mode_fold_pid.out, TRACK_FOLD_TORQUE_LIMIT);
          fold_enabled = true;
        }
        else
          middle_fold_ready = false;
      }
      if (!middle_yaw_ready) fold_mode_yaw_pid.clear();
      if (!middle_fold_ready) fold_mode_fold_pid.clear();
    }

    // ---------- 设置电机命令 ----------

    // 中档折叠独立运行；上档继续要求track命令新鲜。
    const bool pitch_mode_enabled = switch_middle || (switch_up && command_fresh);
    const bool pitch_enabled =
      pitch_mode_enabled && pitch_alive && std::isfinite(pitch_angle) && std::isfinite(pitch_speed);

    float pitch_command = 0.0f;
    bool pitch_output_enabled = false;
    debug_pitch_gravity = 0.0f;
    debug_pitch_raw_torque = 0.0f;

    if (pitch_enabled) {
      // 单位置环，D项使用实际速度阻尼。
      pitch_pid.calc(pitch_target, pitch_angle, 0.0f, pitch_speed);

      const float gravity =
        PITCH_GRAVITY_SIGN * PITCH_GRAVITY_MAX * std::cos(pitch_angle - PITCH_HORIZONTAL_RAD);
      const float raw_torque = pitch_pid.out + gravity;

      if (std::isfinite(raw_torque)) {
        debug_pitch_gravity = gravity;
        debug_pitch_raw_torque = raw_torque;
        pitch_command = clamp_torque(raw_torque, PITCH_MAX_TORQUE);

        pitch_output_enabled = true;
      }
      else {
        pitch_pid.clear();
      }
    }
    else {
      pitch_pid.clear();
    }

    pitch_motor.cmd(pitch_command);

    debug_pitch_target = pitch_target;
    debug_can_pitch_command = pitch_command;
    debug_can_pitch_enabled = pitch_output_enabled;

    // Yaw与fold分别使用各自命令。
    yaw_motor.cmd(yaw_command);
    fold_motor.cmd(fold_command);

    // ---------- CAN1：Pitch ----------
    // GM6020 ID1。

    clear_buffer(can1.tx_data);
    pitch_motor.write(can1.tx_data);
    can1.send(pitch_motor.tx_id);

    // ---------- CAN1：Fold ----------
    // M2006 ID3，与Pitch使用不同控制帧。

    clear_buffer(can1.tx_data);
    fold_motor.write(can1.tx_data);
    can1.send(fold_motor.tx_id);

    // ---------- CAN2：Yaw ----------
    // GM6020 ID2。

    clear_buffer(can2.tx_data);
    yaw_motor.write(can2.tx_data);
    can2.send(yaw_motor.tx_id);

    // ---------- 更新反馈调试变量 ----------

    debug_remote_alive = remote_alive;

    debug_pitch_alive = pitch_alive;
    debug_yaw_alive = yaw_alive;
    debug_fold_alive = fold_alive;

    debug_pitch_angle = pitch_angle;
    debug_pitch_speed = pitch_speed;

    debug_yaw_angle = yaw_angle;
    debug_yaw_speed = yaw_speed;

    debug_fold_angle = fold_angle;
    debug_fold_speed = fold_speed;
    debug_fold_feedback_torque = fold_feedback_torque;

    // ---------- 更新CAN输出调试变量 ----------

    debug_can_yaw_command = yaw_command;
    debug_can_yaw_enabled = yaw_enabled;

    debug_can_fold_command = fold_command;
    debug_can_fold_enabled = fold_enabled;

    osDelay(10);
  }
}

// ==================== CAN接收回调 ====================

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  const uint32_t now = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (hcan == &hcan1) {
      can1.recv();

      ++can1_rx_count;
      debug_can1_rx_id = can1.rx_id;

      if (can1.rx_id == pitch_motor.rx_id) {
        pitch_motor.read(can1.rx_data, now);
        ++pitch_rx_count;
      }
      else if (can1.rx_id == fold_motor.rx_id) {
        fold_motor.read(can1.rx_data, now);
        ++fold_rx_count;
      }
    }
    else if (hcan == &hcan2) {
      can2.recv();

      ++can2_rx_count;
      debug_can2_rx_id = can2.rx_id;

      if (can2.rx_id == yaw_motor.rx_id) {
        yaw_motor.read(can2.rx_data, now);
        ++yaw_rx_count;
      }
    }
    else {
      break;
    }
  }
}