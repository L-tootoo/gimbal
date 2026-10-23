#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "spi.h"
#include "tools/mahony/mahony.hpp"
#include "track_task.hpp"

const float r_ab[3][3] = {{0.0f, -1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

sp::BMI088 bmi088(&hspi1, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0, r_ab);
sp::Mahony mahony(0.001f);

extern "C" void imu_task(void * argument)
{
  bmi088.init();

  while (1) {
    bmi088.update();
    mahony.update(bmi088.acc, bmi088.gyro);
    track_publish_imu(mahony.yaw, mahony.pitch);

    osDelay(1);
  }
}