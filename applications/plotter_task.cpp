#include <cstdint>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "distance_task.hpp"
#include "io/plotter/plotter.hpp"
#include "task.h"
#include "usart.h"

// 由右中档版本的 can_task.cpp 定义
extern volatile std::uint32_t debug_auto_state;
extern volatile std::uint32_t debug_auto_fault;
extern volatile std::uint32_t debug_auto_switch;
extern volatile std::uint32_t debug_auto_motor_mask;

extern volatile bool debug_auto_remote_alive;
extern volatile bool debug_auto_mode_active;

extern volatile float debug_auto_target[3];
extern volatile float debug_auto_actual[3];
extern volatile float debug_auto_torque[3];

extern volatile float debug_fold_feedback_torque;

sp::Plotter plotter(&huart1, false);

extern "C" void plotter_task(const void * argument)
{
  (void)argument;

  while (true) {
    DistanceStatus distance{};
    distance_get_status(&distance);

    float target[3]{};
    float actual[3]{};
    float torque[3]{};
    float fold_feedback_torque = 0.0f;

    std::uint32_t state = 0;
    std::uint32_t fault = 0;
    std::uint32_t switch_value = 0;
    std::uint32_t motor_mask = 0;

    bool remote_alive = false;
    bool mode_active = false;

    taskENTER_CRITICAL();

    state = debug_auto_state;
    fault = debug_auto_fault;
    switch_value = debug_auto_switch;
    motor_mask = debug_auto_motor_mask;

    remote_alive = debug_auto_remote_alive;
    mode_active = debug_auto_mode_active;

    for (unsigned i = 0; i < 3; ++i) {
      target[i] = debug_auto_target[i];
      actual[i] = debug_auto_actual[i];
      torque[i] = debug_auto_torque[i];
    }

    fold_feedback_torque = debug_fold_feedback_torque;

    taskEXIT_CRITICAL();

    plotter.plot(
      distance.valid ? static_cast<float>(distance.distance_cm) : -1.0f,  // 1：距离，cm
      distance.valid ? 1.0f : 0.0f,                                       // 2：距离有效

      static_cast<float>(state),  // 3：动作状态
      static_cast<float>(fault),  // 4：故障码

      target[0],  // 5：Yaw目标，rad
      actual[0],  // 6：Yaw实际，rad
      target[1],  // 7：Pitch目标，rad
      actual[1],  // 8：Pitch实际，rad
      target[2],  // 9：Fold目标，rad
      actual[2],  // 10：Fold实际，rad

      torque[0],             // 11：Yaw命令，N·m
      torque[1],             // 12：Pitch命令，N·m
      torque[2],             // 13：Fold命令，N·m
      fold_feedback_torque,  // 14：Fold反馈，N·m

      remote_alive ? 1.0f : 0.0f,        // 15：遥控在线
      static_cast<float>(switch_value),  // 16：右档位：0下/1中/2上
      mode_active ? 1.0f : 0.0f,         // 17：右中档使能条件
      static_cast<float>(motor_mask)     // 18：在线位掩码，7为全在线
    );

    osDelay(10);
  }
}