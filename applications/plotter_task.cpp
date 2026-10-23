#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"
#include "task.h"
#include "track_task.hpp"
#include "usart.h"

sp::Plotter plotter(&huart1, false);

extern "C" void plotter_task(const void * argument)
{
  (void)argument;

  while (1) {
    float yaw_ratio;
    float pitch_ratio;

    taskENTER_CRITICAL();

    taskEXIT_CRITICAL();

    plotter.plot(debug_track_pitch_target, debug_track_pitch_actual);

    osDelay(10);
  }
}