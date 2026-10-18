#include <cstdint>

#include "cmsis_os.h"
#include "distance_task.hpp"
#include "io/dbus/dbus.hpp"

sp::DBus remote(&huart3);
volatile uint32_t remote_rx_count = 0;
volatile uint32_t remote_rx_size = 0;
volatile uint32_t remote_error_count = 0;

extern "C" void remote_task(void * argument)
{
  (void)argument;

  remote.request();

  while (1) {
    osDelay(10);
  }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  if (huart == &huart3) {
    ++remote_rx_count;
    remote_rx_size = Size;

    remote.update(Size, osKernelSysTick());
    remote.request();
  }
  else if (huart == &huart6) {
    distance_uart_rx_event(Size);
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart3) {
    ++remote_error_count;
    remote.request();
  }
  else if (huart == &huart6) {
    distance_uart_error_event();
  }
}