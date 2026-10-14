#include "cmsis_os.h"
#include "io/led/led.hpp"

sp::LED led(&htim5);

extern "C" void led_task()
{
  led.start();

  while (1) {
    for (uint8_t r = 0; r < 10; r++) {
      led.set(0, r * 0.01f, 0);
      osDelay(100);
    }

    for (uint8_t r = 10; r > 0; r--) {
      led.set(r * 0.01f, 0, 0);
      osDelay(100);
    }
    for (uint8_t r = 10; r > 0; r--) {
      led.set(0, 0, r * 0.01f);
      osDelay(100);
    }
  }
}