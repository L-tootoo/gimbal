#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "io/plotter/plotter.hpp"
#include "usart.h"

extern sp::BMI088 bmi088;

sp::Plotter plotter(&huart1, false);

extern "C" void plotter_task(void * argument)
{
  while (1) {
    plotter.plot(bmi088.gyro[0], bmi088.gyro[1], bmi088.gyro[2]);

    osDelay(10);
  }
}