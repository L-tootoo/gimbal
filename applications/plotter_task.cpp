#include <cmath>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "task.h"
#include "track_task.hpp"
#include "usart.h"

extern sp::DBus remote;
extern sp::RM_Motor pitch_motor;

extern "C" void plotter_task(const void * argument)
{
  (void)argument;

  sp::Plotter plotter(&huart1, false);

  while (true) {
    const TrackCommand track = track_read_command();

    taskENTER_CRITICAL();

    const uint32_t now = osKernelSysTick();

    const bool remote_alive = remote.is_alive(now);
    const bool switch_up = remote.sw_r == sp::DBusSwitchMode::UP;
    const bool pitch_alive = pitch_motor.is_alive(now);

    const bool feedback_valid =
      std::isfinite(pitch_motor.angle) && std::isfinite(pitch_motor.speed);

    taskEXIT_CRITICAL();

    const uint32_t command_age = now - track.stamp_ms;
    const bool command_fresh = command_age < 30U;

    const bool pitch_allowed =
      remote_alive && switch_up && pitch_alive && feedback_valid && command_fresh;

    plotter.plot(
      remote_alive ? 1.0f : 0.0f, switch_up ? 1.0f : 0.0f, pitch_alive ? 1.0f : 0.0f,
      static_cast<float>(command_age), feedback_valid ? 1.0f : 0.0f, pitch_allowed ? 1.0f : 0.0f);

    osDelay(10);
  }
}