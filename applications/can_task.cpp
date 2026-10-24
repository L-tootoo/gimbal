// Yaw nearest-turn alignment on enable; suitable only when that path is mechanically clear.
// Target angle configuration is centralized in home[] and folded[] below.
// Trigger: 3 fresh valid measurements in (0,20] cm; rearm above 25 cm.
// Start return 1 s after folding starts, without folded-arrival check.
// Return trajectory lasts 3 s; initial/return arrival checks retained.
// Right MID enable. Position tolerance: +/-0.05 rad (~2.86 deg).
// Initial tuning: increased Yaw/Fold gains, original torque limits retained.
// FOLD_MS=1000; RETURN_MS=3000. Compatible with Fold feedback Plotter.
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
volatile float debug_pitch_target = 0.0f;
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

static void clear_buffer(uint8_t * data)
{
  for (uint8_t i = 0; i < 8; ++i) {
    data[i] = 0;
  }
}

// ==================== CAN任务 ====================

#include "distance_task.hpp"
namespace
{
float home[3] = {3.08239f, -0.03978f, -0.04956f};
float folded[3] = {3.16379f, -1.66744f, 0.10955f};
constexpr uint32_t FOLD_MS = 1000U, RETURN_MS = 3000U;
sp::PID yp(0.01f, 0.90f, 0.12f, 0.015f, 0.41f, 0.1f);
sp::PID pp(0.01f, 0.5f, 0.0f, 0.04f, 0.15f, 0.0f);
sp::PID fp(0.01f, 1.20f, 0.10f, 0.02f, 0.122f, 0.1f);
sp::PID * const pid[3] = {&yp, &pp, &fp};
sp::RM_Motor * const motor[3] = {&yaw_motor, &pitch_motor, &fold_motor};
void clear_pid()
{
  for (unsigned i = 0; i < 3; ++i) pid[i]->clear();
}
}  // namespace
// State: 0 disabled,1 entering home,2 home,3 folding,4 settling folded,
// 5 returning,6 settling home,7 fault.
volatile uint32_t debug_auto_state = 0, debug_auto_fault = 0;
volatile bool debug_auto_armed = false;
volatile uint32_t debug_auto_switch = 0, debug_auto_motor_mask = 0;
volatile bool debug_auto_remote_alive = false, debug_auto_mode_active = false;
volatile float debug_auto_target[3] = {}, debug_auto_actual[3] = {}, debug_auto_torque[3] = {};

extern "C" void can_task(void const * argument)
{
  (void)argument;
  static_assert(configTICK_RATE_HZ == 1000, "Requires 1 ms tick");
  can1.config();
  can1.start();
  can2.config();
  can2.start();
  uint32_t state = 0, fault = 0, start_ms = 0, duration = 3000, settle_ms = 0, last_stamp = 0;
  unsigned near_count = 0, far_count = 0;
  bool armed = true, settling = false;
  float start[3] = {}, goal[3] = {}, target[3] = {};
  TickType_t wake = xTaskGetTickCount();
  while (true) {
    const uint32_t now = osKernelSysTick();
    float angle[3], speed[3], velocity[3] = {}, torque[3] = {};
    bool alive[3], remote_alive;
    sp::DBusSwitchMode sw;
    taskENTER_CRITICAL();
    for (unsigned i = 0; i < 3; ++i) {
      angle[i] = motor[i]->angle;
      speed[i] = motor[i]->speed;
      alive[i] = motor[i]->is_alive(now);
    }
    remote_alive = remote.is_alive(now);
    sw = remote.sw_r;
    taskEXIT_CRITICAL();
    DistanceStatus d{};
    distance_get_status(&d);
    const bool mode_active = remote_alive && sw == sp::DBusSwitchMode::MID;
    bool feedback_ok = true;
    for (unsigned i = 0; i < 3; ++i) {
      if (!alive[i] || !std::isfinite(angle[i]) || !std::isfinite(speed[i])) {
        feedback_ok = false;
        if (mode_active && state != 7) {
          state = 7;
          fault = i + 1;
        }
      }
    }
    if (!mode_active) {
      if (remote_alive) {
        state = 0;
        fault = 0;
      }
      else if (state != 7)
        state = 0;
      armed = true;
      near_count = far_count = 0;
      settling = false;
      last_stamp = d.last_valid_tick;
      clear_pid();
      for (unsigned i = 0; i < 3; ++i) target[i] = angle[i];
    }
    else if (state != 7 && feedback_ok) {
      if (state == 0) {
        // Choose the closest equivalent Yaw home angle at each enable.
        // Shift both targets by the SAME number of turns.
        constexpr float TWO_PI = 6.283185307f;
        const float shift = std::round((angle[0] - home[0]) / TWO_PI) * TWO_PI;
        home[0] += shift;
        folded[0] += shift;
        state = 1;
        start_ms = now;
        duration = 3000;
        settling = false;
        near_count = far_count = 0;
        last_stamp = d.last_valid_tick;
        clear_pid();
        for (unsigned i = 0; i < 3; ++i) {
          start[i] = angle[i];
          goal[i] = home[i];
        }
      }
      if (state == 2) {
        for (unsigned i = 0; i < 3; ++i) target[i] = home[i];
        if (!d.valid)
          near_count = far_count = 0;
        else if (last_stamp != d.last_valid_tick) {
          last_stamp = d.last_valid_tick;
          if (armed) {
            // Reject zero; it is not a confirmed usable measurement.
            near_count = (d.distance_cm > 0 && d.distance_cm <= 20) ? near_count + 1 : 0;
            if (near_count >= 1) {
              state = 3;
              armed = false;
              start_ms = now;
              duration = FOLD_MS;
              settling = false;
              near_count = far_count = 0;
              for (unsigned i = 0; i < 3; ++i) {
                start[i] = angle[i];
                goal[i] = folded[i];
              }
            }
          }
          else {
            far_count = d.distance_cm > 25 ? far_count + 1 : 0;
            if (far_count >= 3) {
              armed = true;
              far_count = 0;
            }
          }
        }
      }
      else
        last_stamp = d.last_valid_tick;
      if (state == 1 || state == 3 || state == 4 || state == 5 || state == 6) {
        const uint32_t elapsed = now - start_ms;
        const float r = elapsed >= duration ? 1.0f : float(elapsed) / float(duration);
        const float blend = r * r * r * (10.0f - 15.0f * r + 6.0f * r * r);
        const float rate = elapsed >= duration
                             ? 0.0f
                             : 30.0f * r * r * (1.0f - r) * (1.0f - r) / (float(duration) * 0.001f);
        bool reached = true;
        for (unsigned i = 0; i < 3; ++i) {
          target[i] = start[i] + blend * (goal[i] - start[i]);
          velocity[i] = rate * (goal[i] - start[i]);
          if (std::fabs(angle[i] - goal[i]) > 0.05f || std::fabs(speed[i]) > 0.1f) reached = false;
        }
        if (elapsed >= duration) {
          if (state == 3 || state == 4) {
            // Start returning exactly at the first control cycle after 1 s.
            // Do not wait for folded position arrival.
            state = 5;
            start_ms = now;
            duration = RETURN_MS;
            settling = false;
            for (unsigned i = 0; i < 3; ++i) {
              start[i] = angle[i];
              goal[i] = home[i];
              target[i] = angle[i];
              velocity[i] = 0.0f;
            }
            clear_pid();
          }
          else {
            if (state == 5) state = 6;
            if (reached) {
              if (!settling) {
                settling = true;
                settle_ms = now;
              }
              if (now - settle_ms >= 200) {
                state = 2;
                settling = false;
                near_count = far_count = 0;
              }
            }
            else
              settling = false;
            if (state != 2 && now - start_ms >= duration + 8000) {
              state = 7;
              fault = 4;
            }
          }
        }
      }
      if (state != 7) {
        for (unsigned i = 0; i < 3; ++i) {
          pid[i]->calc(target[i], angle[i], velocity[i], speed[i]);
          torque[i] = pid[i]->out;
          if (!std::isfinite(torque[i])) {
            state = 7;
            fault = 5;
          }
        }
      }
    }
    const bool enabled = mode_active && feedback_ok && state != 7;
    if (!enabled) {
      clear_pid();
      for (float & t : torque) t = 0;
    }
    for (unsigned i = 0; i < 3; ++i) motor[i]->cmd(torque[i]);
    clear_buffer(can1.tx_data);
    pitch_motor.write(can1.tx_data);
    can1.send(pitch_motor.tx_id);
    clear_buffer(can1.tx_data);
    fold_motor.write(can1.tx_data);
    can1.send(fold_motor.tx_id);
    clear_buffer(can2.tx_data);
    yaw_motor.write(can2.tx_data);
    can2.send(yaw_motor.tx_id);
    taskENTER_CRITICAL();
    debug_auto_state = state;
    debug_auto_fault = fault;
    debug_auto_armed = armed;
    debug_auto_remote_alive = remote_alive;
    debug_auto_switch = static_cast<uint32_t>(sw);
    debug_auto_mode_active = mode_active;
    debug_auto_motor_mask = (alive[0] ? 1U : 0U) | (alive[1] ? 2U : 0U) | (alive[2] ? 4U : 0U);
    for (unsigned i = 0; i < 3; ++i) {
      debug_auto_target[i] = target[i];
      debug_auto_actual[i] = angle[i];
      debug_auto_torque[i] = torque[i];
    }
    debug_remote_alive = remote_alive;
    debug_yaw_alive = alive[0];
    debug_pitch_alive = alive[1];
    debug_fold_alive = alive[2];
    debug_yaw_angle = angle[0];
    debug_pitch_angle = angle[1];
    debug_fold_angle = angle[2];
    debug_yaw_speed = speed[0];
    debug_pitch_speed = speed[1];
    debug_fold_speed = speed[2];
    debug_fold_feedback_torque = fold_motor.torque;
    debug_pitch_target = target[1];
    debug_can_yaw_command = torque[0];
    debug_can_pitch_command = torque[1];
    debug_can_fold_command = torque[2];
    debug_can_yaw_enabled = enabled;
    debug_can_pitch_enabled = enabled;
    debug_can_fold_enabled = enabled;
    taskEXIT_CRITICAL();
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(10U));
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
