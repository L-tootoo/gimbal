#include <cstdint>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "distance_task.hpp"
#include "io/plotter/plotter.hpp"
#include "task.h"
#include "usart.h"
extern volatile uint32_t debug_auto_state, debug_auto_fault;
extern volatile bool debug_auto_armed;
extern volatile uint32_t debug_auto_switch, debug_auto_motor_mask;
extern volatile bool debug_auto_remote_alive, debug_auto_mode_active;
extern volatile float debug_auto_target[3], debug_auto_actual[3], debug_auto_torque[3];
sp::Plotter plotter(&huart1, false);
extern "C" void plotter_task(const void * argument)
{
  (void)argument;
  while (true) {
    DistanceStatus d{};
    distance_get_status(&d);
    float target[3], actual[3], torque[3];
    uint32_t state, fault, switch_value, motor_mask;
    bool armed, remote_alive, mode_active;
    taskENTER_CRITICAL();
    state = debug_auto_state;
    fault = debug_auto_fault;
    armed = debug_auto_armed;
    switch_value = debug_auto_switch;
    motor_mask = debug_auto_motor_mask;
    remote_alive = debug_auto_remote_alive;
    mode_active = debug_auto_mode_active;
    for (unsigned i = 0; i < 3; ++i) {
      target[i] = debug_auto_target[i];
      actual[i] = debug_auto_actual[i];
      torque[i] = debug_auto_torque[i];
    }
    taskEXIT_CRITICAL();
    plotter.plot(
      d.valid ? float(d.distance_cm) : -1.0f, d.valid ? 1.0f : 0.0f, float(state), float(fault),
      target[0], actual[0], target[1], actual[1], target[2], actual[2], torque[0], torque[1],
      torque[2], armed ? 1.0f : 0.0f, remote_alive ? 1.0f : 0.0f, float(switch_value),
      mode_active ? 1.0f : 0.0f, float(motor_mask));
    osDelay(10);
  }
}
