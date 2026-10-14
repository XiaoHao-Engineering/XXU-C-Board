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

typedef enum {
    SHOOTER_STATE_STOP = 0,     /* 停火：摩擦轮停转 */
    SHOOTER_STATE_SPINUP,       /* 摩擦轮起转，等待转速到位 */
    SHOOTER_STATE_FIRE,         /* 可发射 */
    SHOOTER_STATE_REVERSE,      /* 卡弹：反转退弹 */
} Shooter_State;

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
    /* 状态机 float 镜像：枚举进不了串口绘图的 float* 通道表
       （0=STOP 1=SPINUP 2=FIRE 3=REVERSE）*/
    float state_f;
    /* 观察用镜像（排查"电机到底有没有反馈"）：
       online_mask 的 bit0=摩擦轮左 bit1=摩擦轮右 bit2=拨弹轮，对应位为 1 表示该电机有反馈；
       ready_f 是 is_ready 的 float 镜像。
       ★ 排查方法：fstate 会变(1/2) 但 online_mask=0 -> 电机真掉线；
                   fstate 恒为 0 -> 遥控 sw2 没被识别（档位不在单发/连发位）。*/
    float online_mask;
    float ready_f;

    float fire_rpm;
    float step_dial_deg;             /* 一步走多少【拨盘】角度 deg（内部×减速比换算成转子角）*/
    float fire_interval_ms;
    float distance_m;
    float height_m;
    float auto_aim;

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
