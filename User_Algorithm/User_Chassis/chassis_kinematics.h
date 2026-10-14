#ifndef CHASSIS_KINEMATICS_H
#define CHASSIS_KINEMATICS_H

#include "../../User_Application/chassis_config.h"

/* 轮速下标 */
#define WHEEL_FL   (0)
#define WHEEL_FR   (1)
#define WHEEL_RL   (2)
#define WHEEL_RR   (3)

/* 逆解：底盘速度 -> 各轮线速度 m/s */
void Chassis_InverseKinematics(float vx, float vy, float omega, float* wheel);

/* 正解：各轮线速度 m/s -> 底盘速度 */
void Chassis_ForwardKinematics(const float* wheel, float* vx, float* vy, float* omega);

#endif /* CHASSIS_KINEMATICS_H */
