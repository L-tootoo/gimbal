#include "cmsis_os.h"
#include "distance_task.hpp"
#include "io/plotter/plotter.hpp"
#include "usart.h"

sp::Plotter plotter(&huart1, false);

extern "C" void plotter_task(void const * argument)
{
  (void)argument;

  while (true) {
    DistanceStatus status{};
    distance_get_status(&status);

    plotter.plot(static_cast<float>(status.distance_cm));

    osDelay(10);
  }
}