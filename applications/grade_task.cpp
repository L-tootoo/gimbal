#include "grade_task.hpp"

#include <cmath>
#include <cstdint>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "task.h"
#include "track_task.hpp"

extern sp::DBus remote;

extern volatile uint32_t debug_yaw_test_fault;
extern volatile uint32_t debug_fold_fault;

volatile float debug_yaw_ratio = 0.0f;
volatile float debug_pitch_ratio = 0.0f;

// ==================== 评分函数 ====================
// 保留你提供的阈值、滤波系数和严格小于判断。

void yaw_detect(float gimbal_angle, float plan_angle, float * fire_ratio)
{
  if (fire_ratio == nullptr) {
    return;
  }

  constexpr float alpha = 0.001f;
  constexpr float threshold = 0.5f / 57.3f;

  const uint8_t fire = (std::fabs(gimbal_angle - plan_angle) < threshold) ? 1U : 0U;

  *fire_ratio = alpha * static_cast<float>(fire) + (1.0f - alpha) * (*fire_ratio);
}

void pitch_detect(float gimbal_angle, float plan_angle, float * fire_ratio)
{
  if (fire_ratio == nullptr) {
    return;
  }

  constexpr float alpha = 0.001f;
  constexpr float threshold = 1.5f / 57.3f;

  const uint8_t fire = (std::fabs(gimbal_angle - plan_angle) < threshold) ? 1U : 0U;

  *fire_ratio = alpha * static_cast<float>(fire) + (1.0f - alpha) * (*fire_ratio);
}

// ==================== 1ms评分任务 ====================

extern "C" void grade_task(void const * argument)
{
  (void)argument;

  static_assert(configTICK_RATE_HZ == 1000, "Grade task requires a 1 ms RTOS tick");

  float yaw_ratio = 0.0f;
  float pitch_ratio = 0.0f;

  TickType_t wake_tick = xTaskGetTickCount();

  while (1) {
    float yaw_actual;
    float yaw_target;

    float pitch_actual;
    float pitch_target;

    bool remote_alive;
    bool switch_up;
    bool fold_enabled;

    uint32_t yaw_state;
    uint32_t yaw_fault;
    uint32_t fold_fault;

    taskENTER_CRITICAL();

    const uint32_t now = osKernelSysTick();

    remote_alive = remote.is_alive(now);
    switch_up = remote.sw_r == sp::DBusSwitchMode::UP;

    yaw_actual = debug_track_yaw_actual;
    yaw_target = debug_track_yaw_target;

    pitch_actual = debug_track_pitch_actual;
    pitch_target = debug_track_pitch_target;

    yaw_state = debug_track_state;
    yaw_fault = debug_yaw_test_fault;

    fold_enabled = debug_fold_enabled;
    fold_fault = debug_fold_fault;

    taskEXIT_CRITICAL();

    if (!remote_alive || !switch_up) {
      // 退出上档后清零，下一轮重新评分。
      yaw_ratio = 0.0f;
      pitch_ratio = 0.0f;
    }
    else {
      // yaw只在正式追踪状态评分，排除预定位和等待阶段。
      const bool yaw_valid = yaw_state == 1U && yaw_fault == 0U && std::isfinite(yaw_actual) &&
                             std::isfinite(yaw_target);

      if (yaw_valid) {
        yaw_detect(yaw_actual, yaw_target, &yaw_ratio);
      }
      else {
        yaw_ratio = 0.0f;
      }

      // fold当前没有独立的“接入完成”状态，
      // 因此这里从使能后开始评分，包含两秒接入阶段。
      const bool pitch_valid = fold_enabled && fold_fault == 0U && std::isfinite(pitch_actual) &&
                               std::isfinite(pitch_target);

      if (pitch_valid) {
        pitch_detect(pitch_actual, pitch_target, &pitch_ratio);
      }
      else {
        pitch_ratio = 0.0f;
      }
    }

    debug_yaw_ratio = yaw_ratio;
    debug_pitch_ratio = pitch_ratio;

    vTaskDelayUntil(&wake_tick, pdMS_TO_TICKS(1U));
  }
}