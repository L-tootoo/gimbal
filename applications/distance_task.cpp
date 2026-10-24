// GY-56：UART6，9600bps，8N1，RX DMA Normal模式。
// UART与DMA中断须满足FreeRTOS FromISR接口的优先级要求。
#include "distance_task.hpp"

#include <cstring>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "queue.h"
#include "task.h"
#include "usart.h"

// ==================== 配置 ====================

static constexpr uint16_t rx_buf_size = 64;
// 最大兼容帧长：8字节数据 + 4字节帧头及描述 + 1字节校验。
static constexpr uint16_t frame_buf_size = 13;
static constexpr uint8_t queue_length = 8;

static constexpr uint32_t distance_timeout_ticks = pdMS_TO_TICKS(500);
// 查询周期：保留原测量模式，给两次测量之间留出空闲时间。
static constexpr uint32_t distance_query_period_ticks = pdMS_TO_TICKS(400);

struct DistanceRxMessage
{
  uint16_t size;
  uint8_t data[rx_buf_size];
};

// ==================== 缓存与状态 ====================

static uint8_t distance_rx_buf[rx_buf_size];

static uint8_t frame_buf[frame_buf_size];
static uint16_t frame_len = 0;

static QueueHandle_t distance_queue = nullptr;
static DistanceStatus distance_status{};

static volatile bool recovery_requested = false;

// ==================== 诊断变量 ====================

volatile uint32_t distance_rx_byte_count = 0;
volatile uint32_t distance_valid_frame_count = 0;
volatile uint32_t distance_checksum_error_count = 0;
volatile uint32_t distance_range_error_count = 0;
volatile uint32_t distance_queue_drop_count = 0;
volatile uint32_t distance_recovery_count = 0;

volatile uint32_t distance_start_fail_count = 0;
volatile uint32_t distance_uart_error_count = 0;
volatile uint32_t distance_start_status = 0;
volatile uint32_t distance_uart_error_code = 0;
volatile uint32_t distance_dma_error_code = 0;
volatile uint32_t distance_query_fail_count = 0;

// ==================== 状态接口 ====================

bool distance_get_status(DistanceStatus * out)
{
  if (out == nullptr) {
    return false;
  }

  taskENTER_CRITICAL();

  *out = distance_status;

  if (osKernelSysTick() - out->last_valid_tick > distance_timeout_ticks) {
    out->valid = false;
  }

  taskEXIT_CRITICAL();

  return true;
}

static void invalidate_distance()
{
  taskENTER_CRITICAL();
  distance_status.valid = false;
  taskEXIT_CRITICAL();
}

// ==================== DMA 接收 ====================

static bool start_distance_receive()
{
  const HAL_StatusTypeDef result =
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, distance_rx_buf, sizeof(distance_rx_buf));

  distance_start_status = static_cast<uint32_t>(result);

  if (result != HAL_OK) {
    ++distance_start_fail_count;
    return false;
  }

  // 每次启动后关闭半传输中断。
  __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);

  return true;
}

// ==================== 拼帧缓存 ====================

static void remove_prefix(uint16_t count)
{
  if (count >= frame_len) {
    frame_len = 0;
    return;
  }

  memmove(frame_buf, frame_buf + count, frame_len - count);

  frame_len -= count;
}

// ==================== 距离范围 ====================
static bool is_distance_in_range(uint16_t value, uint8_t mode)
{
  switch (mode) {
    case 0x00:
      return value <= 200;

    case 0x01:
    case 0x02:
    case 0x03:
      return value <= 120;

    case 0x06:
      // 本机实测字段0x06：保留已验证的兼容处理。
      // 具体含义未确认，按120cm上限检查；其他未知值不接受。
      return value <= 120;

    default:
      return false;
  }
}
// ==================== 协议解析 ====================

static void parse_available_frames()
{
  while (true) {
    if (frame_len < 2) {
      return;
    }

    // 搜索帧头 5A 5A。
    uint16_t index = 0;

    while (index + 1 < frame_len) {
      if (frame_buf[index] == 0x5A && frame_buf[index + 1] == 0x5A) {
        break;
      }

      ++index;
    }

    if (index + 1 >= frame_len) {
      // 保留末尾可能属于下一帧的第一个5A。
      if (frame_buf[frame_len - 1] == 0x5A) {
        frame_buf[0] = 0x5A;
        frame_len = 1;
      }
      else {
        frame_len = 0;
      }

      return;
    }

    if (index > 0) {
      remove_prefix(index);
    }

    if (frame_len < 4) {
      return;
    }

    // 距离数据类型。
    if (frame_buf[2] != 0x15) {
      remove_prefix(1);
      continue;
    }

    const uint8_t data_count = frame_buf[3];

    // 保留8～13字节帧的兼容处理。
    if (data_count < 3 || data_count > 8) {
      remove_prefix(1);
      continue;
    }

    const uint16_t expected_len = static_cast<uint16_t>(data_count) + 5;

    if (frame_len < expected_len) {
      return;
    }

    // 累加校验：前面所有字节相加，保留低8位。
    uint8_t sum = 0;

    for (uint16_t i = 0; i < expected_len - 1; ++i) {
      sum = static_cast<uint8_t>(sum + frame_buf[i]);
    }

    if (sum != frame_buf[expected_len - 1]) {
      ++distance_checksum_error_count;

      // 丢弃一个字节，重新寻找帧头。
      remove_prefix(1);
      continue;
    }

    const uint16_t new_distance =
      (static_cast<uint16_t>(frame_buf[4]) << 8) | static_cast<uint16_t>(frame_buf[5]);

    const uint8_t mode = frame_buf[6];

    if (is_distance_in_range(new_distance, mode)) {
      taskENTER_CRITICAL();

      distance_status.distance_cm = new_distance;
      distance_status.last_valid_tick = osKernelSysTick();
      distance_status.valid = true;

      taskEXIT_CRITICAL();

      ++distance_valid_frame_count;
    }
    else {
      ++distance_range_error_count;
      invalidate_distance();
    }

    remove_prefix(expected_len);
  }
}

static void append_and_parse(const uint8_t * data, uint16_t size)
{
  for (uint16_t i = 0; i < size; ++i) {
    if (frame_len >= frame_buf_size) {
      frame_len = 0;
    }

    frame_buf[frame_len++] = data[i];

    parse_available_frames();
  }
}

// ==================== UART6 回调入口 ====================

// 由remote_task.cpp中的统一UART回调调用。
// 这里不重复定义HAL_UARTEx_RxEventCallback。
void distance_uart_rx_event(uint16_t size)
{
  if (distance_queue == nullptr) {
    return;
  }

  if (HAL_UARTEx_GetRxEventType(&huart6) == HAL_UART_RXEVENT_HT) {
    return;
  }

  if (recovery_requested) {
    return;
  }

  BaseType_t task_woken = pdFALSE;

  if (size > 0 && size <= rx_buf_size) {
    DistanceRxMessage message{};
    message.size = size;

    memcpy(message.data, distance_rx_buf, size);

    distance_rx_byte_count += size;

    if (xQueueSendFromISR(distance_queue, &message, &task_woken) != pdPASS) {
      ++distance_queue_drop_count;
      recovery_requested = true;
    }
  }
  else {
    recovery_requested = true;
  }

  if (!recovery_requested) {
    if (!start_distance_receive()) {
      recovery_requested = true;
    }
  }

  portYIELD_FROM_ISR(task_woken);
}

void distance_uart_error_event()
{
  ++distance_uart_error_count;

  distance_uart_error_code = HAL_UART_GetError(&huart6);

  if (huart6.hdmarx != nullptr) {
    distance_dma_error_code = HAL_DMA_GetError(huart6.hdmarx);
  }

  recovery_requested = true;
}

// ==================== FreeRTOS 任务 ====================

extern "C" void distance_task(void * argument)
{
  (void)argument;

  distance_queue = xQueueCreate(queue_length, sizeof(DistanceRxMessage));

  if (distance_queue == nullptr) {
    while (true) {
      osDelay(10);
    }
  }

  DistanceRxMessage message{};

  // GY-56 查询输出：A5 + 65 + 02，校验和取低8位为0C。
  const uint8_t query_command[] = {0xA5, 0x65, 0x02, 0x0C};
  uint32_t last_query_tick = osKernelSysTick() - distance_query_period_ticks;

  // 首次启动也经过统一恢复流程。
  recovery_requested = true;

  while (true) {
    if (recovery_requested) {
      invalidate_distance();

      if (HAL_UART_AbortReceive(&huart6) != HAL_OK) {
        osDelay(10);
        continue;
      }

      xQueueReset(distance_queue);
      frame_len = 0;

      recovery_requested = false;
      ++distance_recovery_count;

      if (!start_distance_receive()) {
        recovery_requested = true;

        osDelay(10);
        continue;
      }
    }

    // 非阻塞取出当前队列中所有数据。
    while (xQueueReceive(distance_queue, &message, 0) == pdPASS) {
      if (recovery_requested) {
        break;
      }

      if (message.size > 0 && message.size <= rx_buf_size) {
        append_and_parse(message.data, message.size);
      }
    }

    taskENTER_CRITICAL();

    if (
      recovery_requested ||
      osKernelSysTick() - distance_status.last_valid_tick > distance_timeout_ticks) {
      distance_status.valid = false;
    }

    taskEXIT_CRITICAL();

    osDelay(10);
  }
}