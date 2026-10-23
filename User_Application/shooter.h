#ifndef SHOOTER_H
#define SHOOTER_H

#include "main.h"
#include "shooter_config.h"
#include "../User_Architect/user_systick.h"
#include "../User_Drives/User_Motor/user_dji_motor.h"
#include "../User_Algorithm/User_Controller/user_pid.h"
#include "../User_Algorithm/User_Controller/user_cascade.h"
#include "../User_Algorithm/user_ballistics.h"
#include "../User_Algorithm/user_fir.h"

/* 摩擦轮 / 拨弹轮的大状态（观察用，state_f 镜像这个）*/
typedef enum {
    SHOOTER_STATE_STOP = 0,     /* 停火：总闸关 / 摩擦轮停转 */
    SHOOTER_STATE_SPINUP,       /* 摩擦轮起转，等待转速到位 */
    SHOOTER_STATE_FIRE,         /* 摩擦轮已起转：按 sw2 档位单发 / 连发 / 仅摩擦轮 */
    SHOOTER_STATE_REVERSE,      /* 卡弹：反转退弹（只在连发里判）*/
} Shooter_State;

/* ★ 停火有限状态机（照 PPT）：失能 -> 激活 -> 处理 -> 失能
   只用于【单发】档；1kHz 后台每拍都跑，防双发。 */
typedef enum {
    SHOOTER_STOPPER_DISABLED = 0,   /* 失能：默认；拨弹轮不出力（目标角跟随实际角）*/
    SHOOTER_STOPPER_ACTIVE,         /* 激活：保持"一发"的目标角继续走，等电流/1s 超时 */
    SHOOTER_STOPPER_PROCESS,        /* 处理：锁住当前角 -> 发射机构停火 -> 回失能 */
} Shooter_Stopper_State;

/* sw2 档位 -> 发射模式（mode_f 镜像这个）*/
typedef enum {
    SHOOTER_MODE_FRICTION_ONLY = 0, /* 中档：只转摩擦轮 */
    SHOOTER_MODE_SINGLE = 1,        /* 上档：单发（位控 + 停火 FSM）*/
    SHOOTER_MODE_CONTINUOUS = 2,    /* 下档：连发（速控，按 fire_int 节奏）*/
} Shooter_Fire_Mode;

typedef struct {
    /* 底层：三个电机（实例在 bsp.c，由 main.c 注入）*/
    DJI_MOTOR_DRIVES* friction_left;
    DJI_MOTOR_DRIVES* friction_right;
    DJI_MOTOR_DRIVES* feeder;

    /* 中层：拨弹轮串级（位置外环 + 速度内环）*/
    PID_Controller     feeder_pid_angle;
    PID_Controller     feeder_pid_speed;
    Cascade_Controller feeder_cascade;

    /* 摩擦轮速度环：输入 RPM，输出电流刻度 ±16384 */
    PID_Controller     friction_left_pid;
    PID_Controller     friction_right_pid;

    /* FIR 滤波：拨弹轮电流（卡弹判定）+ 摩擦轮转速（弹速估计）*/
    FIR_Filter fir_current;
    FIR_Filter fir_friction_rpm;
    float feeder_current_filt;
    float friction_rpm_filt;

    BALLISTICS_DRIVES ballistics;

    /* 上层：状态机 */
    Shooter_State state;
    uint32_t state_ms;
    uint32_t fire_timer_ms;
    uint32_t jam_timer_ms;
    uint8_t  jam_count;
    uint8_t  is_online;
    uint8_t  is_ready;               /* 1=摩擦轮已到转速，允许拨弹 */

    float friction_target;           /* 摩擦轮目标转速 RPM（已含方向）*/
    float friction_rpm;              /* 摩擦轮实测转速（两轮绝对值平均）*/
    float feeder_angle;              /* 拨弹轮转子角 deg */
    float feeder_target_deg;         /* 拨弹轮转子目标角 deg（累加，内部用）*/
    float feeder_current;            /* 拨弹轮电流 mA */
    float feeder_err;
    float feeder_dir;                /* 拨盘方向 +1/-1，可在线改（转反了翻这个）*/
    /* 状态机 float 镜像：枚举进不了串口绘图的 float* 通道表
       （0=STOP 1=SPINUP 2=FIRE 3=REVERSE）*/
    float state_f;
    /* 观察用镜像（排查"电机到底有没有反馈"）：
       online_mask 的 bit0=摩擦轮左 bit1=摩擦轮右 bit2=拨弹轮，对应位为 1 表示该电机有反馈；
       ready_f 是 is_ready 的 float 镜像。
       ★ 排查方法：fstate 会变(1/2) 但 online_mask=0 -> 电机真掉线；
                    fstate 恒为 0 -> 总闸 yaw_auto 没开，或遥控 sw2 档位没被识别。*/
    float online_mask;
    float ready_f;

    /* ---- 单发 / 连发 / 停火 FSM 的状态与镜像 ---- */
    Shooter_Stopper_State stopper_state;  /* 停火 FSM 当前状态 */
    Shooter_Fire_Mode     fire_mode;      /* 本拍 sw2 解出来的模式 */
    uint32_t stopper_ms_acc;              /* 激活/处理已持续时间 ms */
    uint16_t stopper_cur_ms;              /* 摩擦轮电流连续超阈值的时长 ms */
    uint8_t  wheel_armed;                 /* 1=拨轮已回中位，下一次越过死区可发弹 */
    float friction_current;               /* 摩擦轮电流 mA（两轮绝对值的较大者）*/
    float stop_f;                         /* FSM float 镜像：0=失能 1=激活 2=处理 */
    float mode_f;                         /* 模式 float 镜像：0=仅摩擦轮 1=单发 2=连发 */
    float fric_cur_f;                     /* 摩擦轮电流 float 镜像（mA，观察用）*/

    float fire_rpm;                  /* 摩擦轮目标转速 RPM（在线 fire_rpm / friction_rpm）*/
    float step_dial_deg;             /* 一步走多少【拨盘】角度 deg（内部×减速比换算成转子角）*/
    float fire_interval_ms;
    float distance_m;
    float height_m;
    float auto_aim;

    /* ---- 在线可调（chassis_control.c 调参表）---- */
    float fire_dead;                 /* 拨轮触发死区（DBUS ±660），在线 fire_dead */
    float stopper_cur;               /* 停火电流阈值 mA（8A=8000），在线 stopper_cur */
    float stopper_ms;                /* 激活超时 ms（PPT 1s），在线 stopper_ms */

    SysTick_Task task;               /* 1kHz 控制任务 */
    SysTick_Task ballistics_task;    /* ★ 低速率弹道解算（SHOOTER_BALLISTICS_PERIOD_MS）*/
} SHOOTER_DRIVES;

void Shooter_Init(SHOOTER_DRIVES* shooter,
                  DJI_MOTOR_DRIVES* friction_left,
                  DJI_MOTOR_DRIVES* friction_right,
                  DJI_MOTOR_DRIVES* feeder);

uint8_t Shooter_Is_Online(const SHOOTER_DRIVES* shooter);
uint8_t Shooter_Is_Ready(const SHOOTER_DRIVES* shooter);
float   Shooter_Get_Friction_Rpm(const SHOOTER_DRIVES* shooter);
float   Shooter_Get_Ballistics_Pitch(const SHOOTER_DRIVES* shooter);

/* 上层接口：设置目标距离/高度（接视觉/裁判系统就调这个）*/
void Shooter_Set_Target_Distance(SHOOTER_DRIVES* shooter, float distance_m, float height_m);

#endif // SHOOTER_H
