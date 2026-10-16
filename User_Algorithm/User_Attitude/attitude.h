#ifndef ATTITUDE_H
#define ATTITUDE_H

/* 包含头文件 */
#include "main.h"
#ifdef HAL_SPI_MODULE_ENABLED

#include "../../User_Drives/User_Sensor/user_bmi088.h"
#include "user_ahrs.h"
#include "../../User_Algorithm/user_cf.h"
#include "../../Core/Inc/bsp_config.h"

/* ★★ 四元数 EKF 总开关。
   隔离排查用：=0 时完全不初始化、不调用 EKF（连头文件都不包含），
   这样"没有 EKF"的这条路径绝对干净。
   参考工程用的是 Kalman_Filter_Init_Static_6_0_3（静态初始化），
   若它给的矩阵尺寸与 7 状态不匹配，会在初始化时越界写内存 -> 启动直接跑飞。
   确认 EKF 安全后再改成 1。 */
/* ⚠️ 2026-10 实测结论：参考工程移植来的 QuaternionEKF 【开启就会让系统跑飞】——
   V3.6(关) 时串口绘图 100sps 正常；V3.8(开) 后所有串口一个字节都没有。
   原因基本确定：它用的是 Kalman_Filter_Init_Static_6_0_3（静态初始化 6 状态），
   而 QuaternionEKF 内部按 7 状态（四元数4 + 陀螺零偏3）使用，
   矩阵尺寸不匹配 -> 越界写内存 -> 启动即跑飞。
   => 结论：这个 EKF 不能用，除非重写它的矩阵尺寸。
   替代方案：自己写一个 Mahony/Madgwick 互补滤波（无矩阵库、不 malloc、~80 行），
   同样输出 wxyz 四元数，且能顺带估计陀螺零偏。 */
#define ATTITUDE_EKF_ENABLE     (0)
/* ★ Mahony 互补滤波（替代那个会跑飞的 EKF）：=1 时输出 qe_q[4] 四元数给视觉 */
#define ATTITUDE_AHRS_ENABLE    (1)

/* 可调参数 --------------------------------------------------------------*/

/* 互补滤波时间常数 s：越大越信陀螺、越抗震动，零偏漂移暴露得越多 */
#define ATTITUDE_CF_TC       (1.0f)

/* 装机轴向映射：C 板 IMU 焊在板上，按实际倾斜方向改符号 */
#define ATTITUDE_SIGN_X      (1.0f)
#define ATTITUDE_SIGN_Y      (1.0f)
#define ATTITUDE_SIGN_Z      (1.0f)

#define ATTITUDE_RAD2DEG     (57.29578f)
#define ATTITUDE_DEG2RAD     (0.0174532925f)

/* 类型定义 --------------------------------------------------------------*/

/* 只输出云台需要的两个轴：俯仰 Pitch、偏航 Yaw */
typedef struct {
    BMI088_DRIVES* user_bmi088;     /* 数据来源 */
    float dt;                       /* 采样周期 s */

    CF_Filter cf_pitch;             /* 俯仰互补滤波 */

    float pitch_acc;                /* 加速度解算俯仰 deg */
    float pitch_gyro;               /* 陀螺积分俯仰 deg */
    float yaw_gyro;                 /* 陀螺积分偏航 deg */

    float pitch, yaw;               /* 输出姿态角 deg */
    float yaw_rate;                 /* 世界系 yaw 角速度 deg/s（已按俯仰投影补偿），给云台做阻尼 */
    float qe_roll, qe_pitch, qe_yaw;   /* 四元数 EKF 解算的欧拉角 deg（与互补滤波并行对比）*/
    float qe_q[4];                     /* 四元数 EKF 的 q = w,x,y,z（回传给视觉用）*/
    /* ★ Mahony AHRS 状态（真正在跑的四元数解算）*/
    AHRS_DRIVES ahrs;
    float yaw_offset;               /* 偏航零位偏移 */
    uint32_t update_cnt;            /* 更新次数 */
} ATTITUDE_DRIVES;

/* 函数声明 --------------------------------------------------------------*/
void  Attitude_Init(ATTITUDE_DRIVES* attitude, BMI088_DRIVES* user_bmi088, float dt);

/* 把当前偏航角置为 0 */
void  Attitude_Reset_Yaw(ATTITUDE_DRIVES* attitude);

float Attitude_Get_Pitch(const ATTITUDE_DRIVES* attitude);
float Attitude_Get_Yaw(const ATTITUDE_DRIVES* attitude);
float Attitude_Get_Yaw_Rate(const ATTITUDE_DRIVES* attitude);

#endif /* HAL_SPI_MODULE_ENABLED */
#endif // ATTITUDE_H
