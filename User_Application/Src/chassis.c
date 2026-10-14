#include "../chassis.h"
#include "../../User_Algorithm/User_Chassis/chassis_kinematics.h"
#include <math.h>

#define RAD_TO_RPM(rad)     ((rad) * 9.54929659f)   /* 60 / 2pi */
#define RPM_TO_RAD(rpm)     ((rpm) * 0.10471976f)   /* 2pi / 60 */

void Chassis_Init(Chassis* c,
                  MOTOR_INTERFACE* fl, MOTOR_INTERFACE* fr,
                  MOTOR_INTERFACE* rl, MOTOR_INTERFACE* rr) {
    c->wheel[WHEEL_FL] = fl;
    c->wheel[WHEEL_FR] = fr;
    c->wheel[WHEEL_RL] = rl;
    c->wheel[WHEEL_RR] = rr;

    c->reverse[WHEEL_FL] = CHASSIS_REV_FL;
    c->reverse[WHEEL_FR] = CHASSIS_REV_FR;
    c->reverse[WHEEL_RL] = CHASSIS_REV_RL;
    c->reverse[WHEEL_RR] = CHASSIS_REV_RR;

    c->wheel_r = CHASSIS_WHEEL_R;
    c->ratio   = CHASSIS_RATIO;
    c->power_limit = CHASSIS_WHEEL_POWER_MAX;

    c->vx = c->vy = c->omega = 0.0f;
    for (int i = 0; i < 4; i++) {
        c->speed[i] = 0.0f;
        c->rpm[i] = 0.0f;
        c->rpm_fdb[i] = 0.0f;
    }
}

void Chassis_Set_Velocity(Chassis* c, float vx, float vy, float omega) {
    if (vx >  CHASSIS_MAX_VX) vx =  CHASSIS_MAX_VX;
    if (vx < -CHASSIS_MAX_VX) vx = -CHASSIS_MAX_VX;
    if (vy >  CHASSIS_MAX_VY) vy =  CHASSIS_MAX_VY;
    if (vy < -CHASSIS_MAX_VY) vy = -CHASSIS_MAX_VY;
    if (omega >  CHASSIS_MAX_W) omega =  CHASSIS_MAX_W;
    if (omega < -CHASSIS_MAX_W) omega = -CHASSIS_MAX_W;

    c->vx = vx;
    c->vy = vy;
    c->omega = omega;
}

void Chassis_Update(Chassis* c) {
    Chassis_InverseKinematics(c->vx, c->vy, c->omega, c->speed);

    for (int i = 0; i < 4; i++) {
        /* 轮线速度 m/s -> 转子 RPM */
        c->rpm[i] = RAD_TO_RPM(c->speed[i] / c->wheel_r) * c->ratio * (float)c->reverse[i];
        c->rpm_fdb[i] = c->wheel[i]->Get_Motor_Speed(c->wheel[i]);

        c->wheel[i]->Set_Power_Limit(c->wheel[i], c->power_limit);
        c->wheel[i]->Set_Motor_State(c->wheel[i], c->rpm[i]);
    }
}

void Chassis_Get_Velocity(Chassis* c, float* vx, float* vy, float* omega) {
    float speed[4];
    for (int i = 0; i < 4; i++) {
        const float rpm = c->wheel[i]->Get_Motor_Speed(c->wheel[i]) / c->ratio;
        speed[i] = RPM_TO_RAD(rpm) * c->wheel_r * (float)c->reverse[i];
    }
    Chassis_ForwardKinematics(speed, vx, vy, omega);
}
