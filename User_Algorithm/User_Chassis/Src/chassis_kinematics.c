#include "../chassis_kinematics.h"
#include <math.h>

/* 约定：x 向前 y 向左 omega 逆时针为正，单位 m/s、rad/s */
#define INV_SQRT2   (0.70710678f)
#define INV_2SQRT2  (0.35355339f)

void Chassis_InverseKinematics(const float vx, const float vy, const float omega, float* wheel) {
#if CHASSIS_TYPE == CHASSIS_MECANUM
    const float k = (CHASSIS_WHEELBASE + CHASSIS_TRACK) * 0.5f;
    wheel[WHEEL_FL] = vx - vy - k * omega;
    wheel[WHEEL_FR] = vx + vy + k * omega;
    wheel[WHEEL_RL] = vx + vy - k * omega;
    wheel[WHEEL_RR] = vx - vy + k * omega;
#elif CHASSIS_TYPE == CHASSIS_OMNI4
    /* 四轮全向轮辊子切向，r 为轮心到底盘中心距离 */
    const float r = 0.5f * sqrtf(CHASSIS_WHEELBASE * CHASSIS_WHEELBASE
                               + CHASSIS_TRACK * CHASSIS_TRACK);
    wheel[WHEEL_FL] = -INV_SQRT2 * vx - INV_SQRT2 * vy + r * omega;
    wheel[WHEEL_FR] = -INV_SQRT2 * vx + INV_SQRT2 * vy + r * omega;
    wheel[WHEEL_RL] =  INV_SQRT2 * vx - INV_SQRT2 * vy + r * omega;
    wheel[WHEEL_RR] =  INV_SQRT2 * vx + INV_SQRT2 * vy + r * omega;
#else
#error "未知的底盘类型 CHASSIS_TYPE"
#endif
}

void Chassis_ForwardKinematics(const float* wheel, float* vx, float* vy, float* omega) {
#if CHASSIS_TYPE == CHASSIS_MECANUM
    const float k = (CHASSIS_WHEELBASE + CHASSIS_TRACK) * 0.5f;
    *vx = 0.25f * (wheel[WHEEL_FL] + wheel[WHEEL_FR] + wheel[WHEEL_RL] + wheel[WHEEL_RR]);
    *vy = 0.25f * (-wheel[WHEEL_FL] + wheel[WHEEL_FR] + wheel[WHEEL_RL] - wheel[WHEEL_RR]);
    *omega = (-wheel[WHEEL_FL] + wheel[WHEEL_FR] - wheel[WHEEL_RL] + wheel[WHEEL_RR]) / (4.0f * k);
#elif CHASSIS_TYPE == CHASSIS_OMNI4
    const float r = 0.5f * sqrtf(CHASSIS_WHEELBASE * CHASSIS_WHEELBASE
                               + CHASSIS_TRACK * CHASSIS_TRACK);
    *vx = INV_2SQRT2 * (-wheel[WHEEL_FL] - wheel[WHEEL_FR] + wheel[WHEEL_RL] + wheel[WHEEL_RR]);
    *vy = INV_2SQRT2 * (-wheel[WHEEL_FL] + wheel[WHEEL_FR] - wheel[WHEEL_RL] + wheel[WHEEL_RR]);
    *omega = (wheel[WHEEL_FL] + wheel[WHEEL_FR] + wheel[WHEEL_RL] + wheel[WHEEL_RR]) / (4.0f * r);
#else
#error "未知的底盘类型 CHASSIS_TYPE"
#endif
}
