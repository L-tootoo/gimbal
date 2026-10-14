#include "can.h"
#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/mahony/mahony.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "tools/pid/pid.hpp"

extern sp::Mahony mahony;
extern sp::DBus remote;

sp::CAN can1(&hcan1);

sp::RM_Motor motorA(1, sp::RM_Motors::GM6020);
sp::RM_Motor motorB(2, sp::RM_Motors::GM6020);

sp::PID motorA_pos_pid(0.01f, 0.10f, 0.0f, 0.0f, 0.30f, 0.0f);
sp::PID motorB_pos_pid(0.01f, 0.10f, 0.0f, 0.0f, 0.30f, 0.0f);

// 静摩擦补偿
static constexpr float FRICTION_COMP = 0.04f;
static constexpr float COMP_ERROR_THRESHOLD = 0.05f;

// 手动接管检测参数
static constexpr float YAW_STILL_THRESHOLD = 0.01f;
static constexpr float MANUAL_A_ERROR_THRESHOLD = 0.15f;
static constexpr float MANUAL_B_ERROR_THRESHOLD = 0.15f;
static constexpr int MANUAL_DETECT_COUNT = 8;
static constexpr float MANUAL_STOP_SPEED = 0.10f;
static constexpr int MANUAL_RELEASE_COUNT = 15;

// 调试变量
volatile uint32_t can_rx_count = 0;
volatile float debug_yaw = 0.0f;
volatile float debug_target_A = 0.0f;
volatile float debug_motor_A = 0.0f;
volatile float debug_target_B = 0.0f;
volatile float debug_motor_B = 0.0f;

// 当前联动输入源
enum class LinkSource
{
  C_BOARD,
  MOTOR_A,
  MOTOR_B
};

static LinkSource link_source = LinkSource::C_BOARD;

// 普通联动参考，可在运行过程中更新
static float yaw_ref = 0.0f;
static float motorA_ref = 0.0f;
static float motorB_ref = 0.0f;
static bool reference_initialized = false;

// HOME参考，仅用于UP复位
static float home_yaw = 0.0f;
static float home_A = 0.0f;
static float home_B = 0.0f;
static bool home_initialized = false;

// 手动接管检测状态
static float last_yaw = 0.0f;
static int manual_A_detect_count = 0;
static int manual_B_detect_count = 0;
static int manual_release_count = 0;

// 获取左开关对应的B/A联动比例
static float get_ratio()
{
  if (remote.sw_l == sp::DBusSwitchMode::DOWN) {
    return 0.5f;
  }
  else if (remote.sw_l == sp::DBusSwitchMode::MID) {
    return -1.0f;
  }
  else {
    return 3.0f;
  }
}

// 记录物理复位参考
static void set_home()
{
  home_yaw = mahony.yaw;
  home_A = motorA.angle;
  home_B = motorB.angle;
  home_initialized = true;
}

// 重建普通联动参考
static void set_reference()
{
  yaw_ref = mahony.yaw;
  motorA_ref = motorA.angle;
  motorB_ref = motorB.angle;
  last_yaw = mahony.yaw;

  motorA_pos_pid.clear();
  motorB_pos_pid.clear();

  manual_A_detect_count = 0;
  manual_B_detect_count = 0;
  manual_release_count = 0;

  link_source = LinkSource::C_BOARD;
  reference_initialized = true;
}

// 电机失能
static void stop_motors()
{
  motorA_pos_pid.clear();
  motorB_pos_pid.clear();

  motorA.cmd(0.0f);
  motorB.cmd(0.0f);
}

// 单位置环 + 静摩擦补偿
static float position_control(sp::PID & pid, float target, float feedback)
{
  pid.calc(target, feedback);

  float error = target - feedback;
  float output = pid.out;

  if (error > COMP_ERROR_THRESHOLD) {
    output += FRICTION_COMP;
  }
  else if (error < -COMP_ERROR_THRESHOLD) {
    output -= FRICTION_COMP;
  }

  if (output > 0.30f) {
    output = 0.30f;
  }

  if (output < -0.30f) {
    output = -0.30f;
  }

  return output;
}

extern "C" void can_task()
{
  can1.config();
  can1.start();

  while (1) {
    uint32_t now_ms = osKernelSysTick();

    // 遥控掉线：电机失能
    if (!remote.is_alive(now_ms)) {
      stop_motors();

      reference_initialized = false;

      manual_A_detect_count = 0;
      manual_B_detect_count = 0;
      manual_release_count = 0;

      link_source = LinkSource::C_BOARD;

      motorA.write(can1.tx_data);
      motorB.write(can1.tx_data);
      can1.send(motorA.tx_id);

      osDelay(10);
      continue;
    }
    // DOWN：失能
    if (remote.sw_r == sp::DBusSwitchMode::DOWN) {
      stop_motors();

      reference_initialized = false;

      manual_A_detect_count = 0;
      manual_B_detect_count = 0;
      manual_release_count = 0;

      link_source = LinkSource::C_BOARD;
    }

    // MID：姿态与电机联动
    else if (remote.sw_r == sp::DBusSwitchMode::MID) {
      // 第一次进入MID时记录HOME，并建立联动参考
      if (!reference_initialized) {
        if (!home_initialized) {
          set_home();
        }

        set_reference();
      }

      float ratio = get_ratio();

      float yaw_change = mahony.yaw - last_yaw;
      last_yaw = mahony.yaw;

      bool c_is_still = (yaw_change < YAW_STILL_THRESHOLD) && (yaw_change > -YAW_STILL_THRESHOLD);

      // C板作为输入源
      if (link_source == LinkSource::C_BOARD) {
        float yaw_delta = mahony.yaw - yaw_ref;

        // A:C = 1:1，B按左开关比例联动
        float target_A = motorA_ref + yaw_delta;
        float target_B = motorB_ref + ratio * yaw_delta;

        float error_A = target_A - motorA.angle;
        float error_B = target_B - motorB.angle;

        bool a_far_from_target =
          (error_A > MANUAL_A_ERROR_THRESHOLD) || (error_A < -MANUAL_A_ERROR_THRESHOLD);

        bool b_far_from_target =
          (error_B > MANUAL_B_ERROR_THRESHOLD) || (error_B < -MANUAL_B_ERROR_THRESHOLD);

        // 判断是否手动转动A
        if (c_is_still && a_far_from_target) {
          manual_A_detect_count++;
        }
        else {
          manual_A_detect_count = 0;
        }

        // 判断是否手动转动B
        if (c_is_still && b_far_from_target) {
          manual_B_detect_count++;
        }
        else {
          manual_B_detect_count = 0;
        }

        // A手动接管
        if (manual_A_detect_count >= MANUAL_DETECT_COUNT) {
          link_source = LinkSource::MOTOR_A;

          manual_A_detect_count = 0;
          manual_B_detect_count = 0;
          manual_release_count = 0;

          motorA_pos_pid.clear();
          motorA.cmd(0.0f);

          target_A = motorA.angle;

          // A作为输入，B按比例跟随
          target_B = motorB_ref + ratio * (target_A - motorA_ref);

          // 更新软件零点，防止松手后A回位
          yaw_ref = mahony.yaw - (target_A - motorA_ref);

          float output_B = position_control(motorB_pos_pid, target_B, motorB.angle);

          motorB.cmd(output_B);
        }

        // B手动接管
        else if (manual_B_detect_count >= MANUAL_DETECT_COUNT) {
          link_source = LinkSource::MOTOR_B;

          manual_A_detect_count = 0;
          manual_B_detect_count = 0;
          manual_release_count = 0;

          motorB_pos_pid.clear();
          motorB.cmd(0.0f);

          target_B = motorB.angle;

          // B作为输入，反算A目标
          target_A = motorA_ref + (target_B - motorB_ref) / ratio;

          // 更新软件零点，防止松手后B回位
          yaw_ref = mahony.yaw - (target_A - motorA_ref);

          float output_A = position_control(motorA_pos_pid, target_A, motorA.angle);

          motorA.cmd(output_A);
        }

        // 正常C板控制
        else {
          float output_A = position_control(motorA_pos_pid, target_A, motorA.angle);

          float output_B = position_control(motorB_pos_pid, target_B, motorB.angle);

          motorA.cmd(output_A);
          motorB.cmd(output_B);
        }

        debug_yaw = mahony.yaw;
        debug_target_A = target_A;
        debug_motor_A = motorA.angle;
        debug_target_B = target_B;
        debug_motor_B = motorB.angle;
      }

      // A作为输入源
      else if (link_source == LinkSource::MOTOR_A) {
        motorA_pos_pid.clear();
        motorA.cmd(0.0f);

        float target_A = motorA.angle;

        float target_B = motorB_ref + ratio * (target_A - motorA_ref);

        float output_B = position_control(motorB_pos_pid, target_B, motorB.angle);

        motorB.cmd(output_B);

        // 动态更新软件零点
        yaw_ref = mahony.yaw - (target_A - motorA_ref);

        // A停止后交还给C板
        bool a_stopped = (motorA.speed < MANUAL_STOP_SPEED) && (motorA.speed > -MANUAL_STOP_SPEED);

        if (a_stopped) {
          manual_release_count++;
        }
        else {
          manual_release_count = 0;
        }

        if (manual_release_count >= MANUAL_RELEASE_COUNT) {
          link_source = LinkSource::C_BOARD;

          manual_release_count = 0;
          manual_A_detect_count = 0;
          manual_B_detect_count = 0;

          motorA_pos_pid.clear();
        }

        debug_yaw = mahony.yaw;
        debug_target_A = target_A;
        debug_motor_A = motorA.angle;
        debug_target_B = target_B;
        debug_motor_B = motorB.angle;
      }

      // B作为输入源
      else if (link_source == LinkSource::MOTOR_B) {
        motorB_pos_pid.clear();
        motorB.cmd(0.0f);

        float target_B = motorB.angle;

        float target_A = motorA_ref + (target_B - motorB_ref) / ratio;

        float output_A = position_control(motorA_pos_pid, target_A, motorA.angle);

        motorA.cmd(output_A);

        // 动态更新软件零点
        yaw_ref = mahony.yaw - (target_A - motorA_ref);

        // B停止后交还给C板
        bool b_stopped = (motorB.speed < MANUAL_STOP_SPEED) && (motorB.speed > -MANUAL_STOP_SPEED);

        if (b_stopped) {
          manual_release_count++;
        }
        else {
          manual_release_count = 0;
        }

        if (manual_release_count >= MANUAL_RELEASE_COUNT) {
          link_source = LinkSource::C_BOARD;

          manual_release_count = 0;
          manual_A_detect_count = 0;
          manual_B_detect_count = 0;

          motorB_pos_pid.clear();
        }

        debug_yaw = mahony.yaw;
        debug_target_A = target_A;
        debug_motor_A = motorA.angle;
        debug_target_B = target_B;
        debug_motor_B = motorB.angle;
      }
    }

    // UP：箭头复位
    else {
      if (home_initialized) {
        float yaw_delta = mahony.yaw - home_yaw;

        // 恢复A/B与当前C板箭头的物理对齐
        float target_A = home_A + yaw_delta;

        float target_B = home_B + yaw_delta;

        float output_A = position_control(motorA_pos_pid, target_A, motorA.angle);

        float output_B = position_control(motorB_pos_pid, target_B, motorB.angle);

        motorA.cmd(output_A);
        motorB.cmd(output_B);

        debug_yaw = mahony.yaw;
        debug_target_A = target_A;
        debug_motor_A = motorA.angle;
        debug_target_B = target_B;
        debug_motor_B = motorB.angle;
      }
      else {
        stop_motors();
      }

      // 再次进入MID时重新建立普通联动参考
      reference_initialized = false;

      link_source = LinkSource::C_BOARD;

      manual_A_detect_count = 0;
      manual_B_detect_count = 0;
      manual_release_count = 0;
    }

    // 发送A/B控制量
    motorA.write(can1.tx_data);
    motorB.write(can1.tx_data);
    can1.send(motorA.tx_id);

    osDelay(10);
  }
}

// CAN反馈接收
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  auto stamp_ms = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (hcan == &hcan1) {
      can1.recv();
      can_rx_count++;

      if (can1.rx_id == motorA.rx_id) {
        motorA.read(can1.rx_data, stamp_ms);
      }

      if (can1.rx_id == motorB.rx_id) {
        motorB.read(can1.rx_data, stamp_ms);
      }
    }
  }
}