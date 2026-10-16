#ifndef USER_AHRS_H
#define USER_AHRS_H

/* 包含头文件 */
#include "main.h"

/* ---------------------------------------------------------------------------
 * Mahony 互补滤波姿态解算（AHRS）
 *
 * 为什么自己写、而不用参考工程那个 QuaternionEKF：
 *   · 不用 CMSIS 矩阵库（不需要 arm_mat_*，构建更快）
 *   · 完全不 malloc（没有堆风险）
 *   · 只有四则运算 + 少量 sqrtf/atan2f，执行时间确定，不可能越界写内存
 *   · 同样输出 wxyz 四元数 + 欧拉角，并能估计陀螺零偏
 *   （参考工程那个 EKF 实测一开就让系统跑飞：静态初始化 6 状态与它内部按 7 状态
 *     使用不匹配 -> 越界写内存 -> 启动即 HardFault。）
 *
 * 算法来源：Mahony R., et al. "Nonlinear Complementary Filters on the Special
 *           Orthogonal Group"（即开源 MahonyAHRS 的标准形式），公式标准、可逐行核对。
 *
 * 原理（三句话）：
 *   ① 陀螺积分四元数（高频准，但会漂）—— 负责动态
 *   ② 加速度计测到的重力方向 与 四元数估计的重力方向 求叉积 = 误差
 *   ③ 用这个误差去【修正陀螺】(比例项 kp) 并【估计陀螺零偏】(积分项 ki)
 *   => 加速度计管低频绝对基准、陀螺管高频，正好互补。
 *
 * ⚠️ 局限：没有磁力计 -> 偏航(yaw)不可观测 -> yaw 仍会漂。
 *    本工程的 yaw 零偏由 gimbal.c 的"零偏在线跟踪"另外处理，两者互不冲突。
 * ------------------------------------------------------------------------- */

/* 默认增益（在线可调）
   kp 越大越信加速度计 -> 收敛快但机动/振动时会引入误差；越小越信陀螺 -> 平滑但漂移恢复慢
   ki 用来估计陀螺零偏；=0 则完全不估零偏（静态漂移会一直存在）*/
#define AHRS_KP_DEF        (1.0f)
#define AHRS_KI_DEF        (0.005f)
#define AHRS_ACC_LPF_DEF   (0.2f)     /* 加速度计一阶低通系数 0~1，0=不滤 */

/* 类型定义 ----------------------------------------------------------------*/

typedef struct {
    float q[4];                 /* 四元数 w,x,y,z（机体 -> 世界）*/
    float roll, pitch, yaw;     /* 欧拉角 deg（ZYX 内旋）*/

    float gyro_bias[3];         /* 陀螺零偏估计 rad/s */
    float kp;                   /* 加速度计校正比例增益（可在线调）*/
    float ki;                   /* 零偏积分增益（可在线调，0=不估）*/
    float acc_lpf;              /* 加速度计低通系数（可在线调）*/
    float acc_f[3];             /* 加速度计低通状态 */
    uint8_t is_init;
} AHRS_DRIVES;

/* 函数声明 ----------------------------------------------------------------*/

void AHRS_Init(AHRS_DRIVES* ahrs, float kp, float ki, float acc_lpf);

/* 每个 IMU 采样调用一次
   gx/gy/gz : 陀螺 rad/s（与 imu->gyro 同单位）
   ax/ay/az : 加速度计（单位不限，内部会归一化）
   dt       : 采样周期 s */
void AHRS_Update(AHRS_DRIVES* ahrs, float gx, float gy, float gz,
                 float ax, float ay, float az, float dt);

#endif // USER_AHRS_H
