#pragma once

#include <cstdint>

struct DistanceStatus
{
  uint16_t distance_cm;
  uint32_t last_valid_tick;
  bool valid;
};

// 仅供任务调用。返回值表示参数有效，不代表测距有效；使用out->valid判断。
// 无效或超时时保留最近一次有效距离；last_valid_tick为系统tick。
bool distance_get_status(DistanceStatus * out);

// 由统一HAL UART回调在中断中转发，不在本模块重复定义HAL回调。
void distance_uart_rx_event(uint16_t size);
void distance_uart_error_event();

// 接收与解析统计
extern volatile uint32_t distance_rx_byte_count;
extern volatile uint32_t distance_valid_frame_count;
extern volatile uint32_t distance_checksum_error_count;
extern volatile uint32_t distance_range_error_count;
extern volatile uint32_t distance_queue_drop_count;
extern volatile uint32_t distance_recovery_count;

// UART / DMA 诊断
extern volatile uint32_t distance_start_fail_count;
extern volatile uint32_t distance_uart_error_count;
extern volatile uint32_t distance_start_status;
extern volatile uint32_t distance_uart_error_code;
extern volatile uint32_t distance_dma_error_code;
