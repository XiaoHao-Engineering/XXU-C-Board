#ifndef CHASSIS_H
#define CHASSIS_H

#include "main.h"
#include "chassis_config.h"
#include "../User_Drives/User_Motor/user_dji_motor.h"

/* 底盘：绑定 4 个驱动轮，做运动学换算 */
typedef struct {
    MOTOR_INTERFACE* wheel[4];
    int8_t  reverse[4];      /* 装反了填 -1 */
    float   wheel_r;         /* 轮半径 m */
    float   ratio;           /* 减速比 */
    float   power_limit;     /* 单轮功率上限 W */
    float   vx, vy, omega;   /* 目标速度 */
    float   speed[4];        /* 各轮目标线速度 m/s */
    float   rpm[4];          /* 各轮目标转子转速 RPM */
    float   rpm_fdb[4];      /* 各轮实际转子转速 RPM */
} Chassis;

void Chassis_Init(Chassis* c,
                  MOTOR_INTERFACE* fl, MOTOR_INTERFACE* fr,
                  MOTOR_INTERFACE* rl, MOTOR_INTERFACE* rr);

/* 设定底盘目标速度 */
void Chassis_Set_Velocity(Chassis* c, float vx, float vy, float omega);

/* 逆解并写电机目标，需周期调用 */
void Chassis_Update(Chassis* c);

/* 正解：读电机反馈算底盘实际速度 */
void Chassis_Get_Velocity(Chassis* c, float* vx, float* vy, float* omega);

#endif /* CHASSIS_H */
