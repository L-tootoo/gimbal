#ifndef GRADE_TASK_HPP
#define GRADE_TASK_HPP

void yaw_detect(float gimbal_angle, float plan_angle, float * fire_ratio);

void pitch_detect(float gimbal_angle, float plan_angle, float * fire_ratio);

// 取值0～1，乘100为百分比。
extern volatile float debug_yaw_ratio;
extern volatile float debug_pitch_ratio;

#ifdef __cplusplus
extern "C" {
#endif

void grade_task(void const * argument);

#ifdef __cplusplus
}
#endif

#endif