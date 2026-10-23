#include <cmath>
#include <cstdint>

#include "FreeRTOS.h"
#include "can.h"
#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "task.h"
#include "tools/pid/pid.hpp"
#include "track_task.hpp"

extern sp::DBus remote;
// Pitch固定位置控制，角度rad，力矩N·m。
// 以下PID及限幅为初始调试值。
static constexpr float PITCH_HOLD_RAD = -0.2f;
static constexpr float PITCH_PID_DT = 0.01f;

static constexpr float PITCH_KP = 0.5f;
static constexpr float PITCH_KI = 0.0f;
static constexpr float PITCH_KD = 0.04f;

static constexpr float PITCH_MAX_TORQUE = 0.15f;
static constexpr float PITCH_MAX_INTEGRAL = 0.0f;

static sp::PID pitch_pid(
  PITCH_PID_DT, PITCH_KP, PITCH_KI, PITCH_KD, PITCH_MAX_TORQUE, PITCH_MAX_INTEGRAL);

volatile float debug_pitch_target = PITCH_HOLD_RAD;
volatile float debug_can_pitch_command = 0.0f;
volatile bool debug_can_pitch_enabled = false;

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

  while (true) {
    // 两个电机的控制结果均由track提供。
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

    const bool command_fresh = now - track.stamp_ms < 30U;

    // ---------- Yaw独立输出检查 ----------

    const bool yaw_enabled = switch_up && command_fresh && track.yaw_enabled && yaw_alive &&
                             std::isfinite(yaw_angle) && std::isfinite(yaw_speed) &&
                             std::isfinite(track.yaw);

    float yaw_command = 0.0f;

    if (yaw_enabled) {
      yaw_command = clamp_torque(track.yaw, TRACK_YAW_TORQUE_LIMIT);
    }

    // ---------- Fold独立输出检查 ----------

    const bool fold_enabled = switch_up && command_fresh && track.fold_enabled && fold_alive &&
                              std::isfinite(fold_angle) && std::isfinite(fold_speed) &&
                              std::isfinite(track.fold);

    float fold_command = 0.0f;

    if (fold_enabled) {
      fold_command = clamp_torque(track.fold, TRACK_FOLD_TORQUE_LIMIT);
    }

    // ---------- 设置电机命令 ----------

    // Pitch继续零力矩。
    // 右上档且track命令新鲜时，Pitch保持固定位置。
    // 不依赖yaw或fold是否单独使能。
    const bool pitch_enabled = switch_up && command_fresh && pitch_alive &&
                               std::isfinite(pitch_angle) && std::isfinite(pitch_speed);

    float pitch_command = 0.0f;
    bool pitch_output_enabled = false;

    if (pitch_enabled) {
      // 单位置环，D项使用实际速度阻尼。
      pitch_pid.calc(PITCH_HOLD_RAD, pitch_angle, 0.0f, pitch_speed);

      if (std::isfinite(pitch_pid.out)) {
        pitch_command = clamp_torque(pitch_pid.out, PITCH_MAX_TORQUE);

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

    debug_pitch_target = PITCH_HOLD_RAD;
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