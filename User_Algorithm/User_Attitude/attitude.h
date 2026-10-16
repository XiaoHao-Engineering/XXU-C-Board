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
   ★ 2026-10 重新查证（逐行 diff + 子代理分析）后的更正：
   本段早先写的"6状态/7状态矩阵尺寸不匹配导致越界"是【错误推断】，已作废。
   事实：QuaternionEKF 就是 6 状态 / 3 量测，静态缓冲区 P/F/Q[36]、H/HT/K[18]、R[9] 尺寸精确匹配，
        且 user_quaternion_ekf.c / user_kalman_filter.c 与参考工程【逐行完全一致】。
   真正的跑飞原因：本工程是【软浮点】+ SPI 只有 328kHz（读 IMU 花 463us）+ SysTick 优先级最低，
        三者叠加使 1ms 任务超时 -> SysTick 中断背靠背重入 -> 主循环饿死 -> 所有串口无输出。
   对策：开硬件 FPU、提高 SPI 分频、把 EKF 移出中断（见 main.c / CMakeLists）。 */
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
