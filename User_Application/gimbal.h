#ifndef GIMBAL_H
#define GIMBAL_H

#include "main.h"
#include "../User_Architect/user_systick.h"
#include "../User_Drives/User_Motor/user_dji_motor.h"
#include "../User_Algorithm/User_Controller/user_pid.h"
#include "../User_Algorithm/User_Controller/user_pid_adv.h"
#include "../User_Algorithm/User_Controller/user_cascade.h"

/* 云台 yaw 配置 */
/* 锁世界方向（用 Rotor_speed 模式，见 main.c）：推杆 = 世界系里恒速转，松杆 = 锁当前方向、
   推开自动回位；底盘自转靠前馈抵消，云台不跟底盘转。反馈用板载 IMU 的 yaw（A 板装在 yaw 上）。*/
#define GIMBAL_LOCK_WORLD       (1)      /* 0=退回纯速度控制（IMU 有问题时用）*/

#define GIMBAL_YAW_MOTOR_ID     (1)          /* GM6020 的 CAN ID (1~7) */
#define GIMBAL_YAW_ZERO_DEG     (0.0f)       /* 云台正对底盘前方时的电机多圈角度 */
#define GIMBAL_YAW_DIR          (1.0f)       /* 角度增大方向：+1 或 -1 现场核对 */

#define GIMBAL_YAW_POWER_LIMIT  (90.0f)      /* 90W 对应 GM6020 电流满量程 */

/* 速度环 PID（Rotor_speed：目标/反馈都是转子转速 RPM，输出是电流刻度，量程 3000）。
   Ki 原来给 0：纯 P 速度环有转速跌落、反向匀速跟踪有静差，给一点消掉。在线调 gimbal_kp/ki/kd */
#define GIMBAL_YAW_KP           (30.0f)
#define GIMBAL_YAW_KI           (20.0f)
#define GIMBAL_YAW_KD           (1.0f)
#define GIMBAL_YAW_MAX_OUT      (3000.0f)    /* GM6020 电流帧量程 */
#define GIMBAL_YAW_MAX_IOUT     (1000.0f)

/* 云台 yaw 遥控：1=开启。DBUS 通道号：0=右手水平 1=右手垂直 2=左手水平 3=左手垂直 */
#define GIMBAL_RC_ENABLE        (1)
#define GIMBAL_RC_CH            (1)          /* ★ 用哪个通道：1 = 右摇杆上下 */
#define GIMBAL_YAW_MAX_W        (6.0f)       /* 满杆角速度 rad/s */

/* ---- 锁世界位置环（抗惯性三件套）------------------------------------------
   外环是"角度误差 -> 转速目标"，1 RPM = 6 deg/s，所以开环时间常数 τ ≈ 6/world_kp 秒。
   原来 world_kp = 1.0 -> τ ≈ 6 秒 —— 这就是"云台偏软、走起来被惯性拖着"的根因。
   ★ 整定顺序必须【先加阻尼 world_kd，再提增益 world_kp】，反过来必振荡。 */
#define GIMBAL_WORLD_KP         (3.0f)       /* τ 6s -> 2s；稳了可以再试 5.0 */
/* 陀螺阻尼：用板载 IMU 的 yaw 角速度做阻尼，量纲 RPM/(deg/s)，纯运动学取值 = 1/6 ≈ 0.167。
   偏软就加到 0.33（2 倍），一抖就往下调。在线调 gimbal_world_kd */
#define GIMBAL_WORLD_KD         (0.17f)
/* 阻尼源一阶低通系数 0~1：陀螺噪声会直接进转速指令，越小滤得越狠、相位滞后越大 */
#define GIMBAL_WORLD_D_ALPHA    (0.25f)
/* 底盘角速度前馈系数：运动学上精确抵消底盘自转就是 1.0。
   原来 0.3 = 只补 30%，剩 70% 让位置环去追 -> 稳态滞后 + 摆动。抖就 0.8，迟钝就 1.0 */
#define GIMBAL_FF_GAIN          (1.0f)
/* 外环输出限幅 RPM —— 原来完全没有限幅，跨 ±180° 或 IMU 异常时会一脚踹满速度环 */
#define GIMBAL_WORLD_MAX_RPM    (60.0f)

/* ★★ IMU 装在哪：1 = 装在云台上（imu_yaw 就是云台世界朝向，直接用）
                  0 = 装在底盘上（云台世界朝向 = 底盘 yaw + 云台相对底盘的编码器角）
   ⚠️ 必须按实车填对，填错会让锁世界锁错对象（云台跟着底盘转）。
   10 秒判定法：① 固定底盘、手动转云台，看串口 gimu/yaw 变不变；② 固定云台、手动转底盘，再看。
   哪个有反应，IMU 就在哪。 */
#define GIMBAL_IMU_ON_GIMBAL    (1)
#define GIMBAL_RC_DEAD          (10)

/* 掉线保护：连续多少 ms 收不到 GM6020 反馈就判定掉线 */
/* PID 测试波形注入（调试用）：打开后波形直接当【转速指令】驱动电机、绕过外环，用串口绘图的 gspd
   跟随来调 gimbal_kp / gimbal_ki。建议幅值 20 RPM、2Hz。在线可调：wave_en/type/amp/freq/duty */
#define GIMBAL_WAVE_ENABLE      (1)      /* 0=完全编译掉，1=允许用 wave_en 打开 */
#define GIMBAL_WAVE_AMP_DEF     (20.0f)  /* 默认幅值 RPM */
#define GIMBAL_WAVE_FREQ_DEF    (2.0f)   /* 默认频率 Hz */
#define GIMBAL_WAVE_MAX_RPM     (150.0f) /* 波形输出限幅，保护电机 */

#define GIMBAL_OFFLINE_MS       (100)

/* 云台 pitch 配置 */
/* 控制方式：GIMBAL_PITCH_CASCADE=1 真串级 —— 内外环两个 PID 组成 Cascade 注入电机，电机跑
   Rotor_angle 模式，自己用外环(角度)算转速目标、内环(速度)算电流，外部只喂目标角；=0 退回单环 ——
   电机跑 Rotor_speed 只跑速度环，外环角度 PID 改到 gimbal.c 里算（两种模式用同两个 PID）。*/
#define GIMBAL_PITCH_CASCADE    (1)
#define GIMBAL_PITCH_ENABLE     (1)      /* 1=启用 pitch 轴，0=整轴关闭（排错用）*/

#define GIMBAL_PITCH_MOTOR_ID   (4)          /* GM6020 的 CAN ID (1~7) */
/* ★ pitch 挂 CAN2 而不是 CAN1：CAN1 已有 4 轮 + yaw ≈78% 负载，再加会超出 CAN 可靠区间，
   挂 CAN2 后两条总线都留有余量。yaw(id1)/pitch(id4) 都映射 0x1FE，但分属两总线、不会互相干扰。*/
#define GIMBAL_PITCH_CAN        user_can_2
#define GIMBAL_PITCH_DIR        (1.0f)       /* 角度增大方向：+1 或 -1，现场核对 */
#define GIMBAL_PITCH_POWER_LIMIT (90.0f)

/* 软限位（度）：作用在"逻辑角"上，到限位就夹住目标，不会硬顶机构 */
#define GIMBAL_PITCH_MIN_DEG    (-30.0f)
#define GIMBAL_PITCH_MAX_DEG    ( 15.0f)

/* 上电时枪管的真实俯仰角：上电前摆到水平就填 0，否则量出实际角度填这里（pitch 读数才是真角度，
   软限位才落在正确位置）。⚠️ 每次上电必须是同一姿态，否则零点会漂（同 yaw 的上电对准前方）*/
#define GIMBAL_PITCH_ZERO_BIAS_DEG (0.0f)

/* 外环（角度环）：输入 deg，输出【转速目标 RPM】。用增强 PID（user_pid_adv），ki/kd 量纲带 s
   （ki = 输出/(误差·s)、kd = 输出/(误差/s)），数值比 user_pid 隐含 dt 的写法大，是正常的。 */
#define GIMBAL_PITCH_ANGLE_KP       (12.0f)
#define GIMBAL_PITCH_ANGLE_KI       (2.0f)
#define GIMBAL_PITCH_ANGLE_KD       (0.2f)
#define GIMBAL_PITCH_ANGLE_KF       (0.0f)    /* 目标速度前馈（自动瞄准时可给一点）*/
#define GIMBAL_PITCH_ANGLE_MAX_OUT  (200.0f)  /* 外环输出上限 = 内环转速目标上限 RPM */
#define GIMBAL_PITCH_ANGLE_MAX_IOUT (60.0f)
#define GIMBAL_PITCH_ANGLE_MAX_DOUT (60.0f)
#define GIMBAL_PITCH_ANGLE_DEAD     (0.15f)   /* 死区 deg：压住零位附近抖动 */
#define GIMBAL_PITCH_ANGLE_D_ALPHA  (0.4f)    /* D 项一阶低通 */
#define GIMBAL_PITCH_ANGLE_IVAR_A   (6.0f)    /* 变速积分：6° 以内正常积分 */
#define GIMBAL_PITCH_ANGLE_IVAR_B   (24.0f)   /* 6°~30° 线性衰减到 0 */
#define GIMBAL_PITCH_ANGLE_ISEP     (30.0f)   /* 积分分离阈值 deg（★ 衰减，不硬清零）*/

/* 内环（速度环）：输入 RPM，输出电流刻度 */
/* ★★ ki 【绝对不能是 0】：稳态时外环误差 0、内环目标 0 RPM，顶重力的恒力矩只能靠【内环】积分维持。
   ki 单位 电流刻度/(RPM·s)，300 起步（满量程 16384）；松手后枪管慢慢下沉 = ki 太小 / max_iout 太小 */
#define GIMBAL_PITCH_SPEED_KP       (30.0f)
#define GIMBAL_PITCH_SPEED_KI       (300.0f)
#define GIMBAL_PITCH_SPEED_KD       (0.0f)
#define GIMBAL_PITCH_SPEED_KF       (0.0f)
#define GIMBAL_PITCH_SPEED_MAX_OUT  (8000.0f)  /* 约 1.46A / 1.08N·m，不够就往上加 */
#define GIMBAL_PITCH_SPEED_MAX_IOUT (6000.0f)
#define GIMBAL_PITCH_SPEED_MAX_DOUT (1500.0f)
#define GIMBAL_PITCH_SPEED_DEAD     (0.0f)
#define GIMBAL_PITCH_SPEED_D_ALPHA  (0.5f)   /* ★ RPM 反馈是整数、差分噪声大，必须滤 */
#define GIMBAL_PITCH_SPEED_IVAR_A   (0.0f)
#define GIMBAL_PITCH_SPEED_IVAR_B   (0.0f)
#define GIMBAL_PITCH_SPEED_ISEP     (0.0f)

/* pitch 遥控：★ 用遥控器【左上角拨轮 roll】控制俯仰（yaw 占了右摇杆、底盘占了左摇杆，拨轮空着）。
   拨轮偏转 → 目标角以恒定角速度走，拨轮回中即停在当前目标角；行程由 MIN/MAX_DEG 软限位兜住。 */
#define GIMBAL_PITCH_RC_ENABLE   (1)
#define GIMBAL_PITCH_WHEEL_DIR   (1.0f)       /* 拨轮方向：+1 或 -1，现场核对 */
#define GIMBAL_PITCH_MAX_W       (120.0f)     /* 满拨轮角速度 deg/s，可在线改 */
#define GIMBAL_PITCH_WHEEL_DEAD  (30)         /* 拨轮死区（DBUS roll 范围 ±660）*/
#define GIMBAL_PITCH_AUTO_ENABLE_DEF (0.0f)

/* 类型定义 */

typedef struct {
    DJI_MOTOR_DRIVES* yaw_motor;   /* GM6020，Rotor_speed（转速）模式 */
    PID_Controller    yaw_pid;
    float zero_deg;                /* 云台正对底盘前方时的多圈角度 */
    float yaw;                     /* 云台相对底盘角度 deg，±180 */
    float target_yaw;              /* 世界系目标角 deg，±180（锁世界用）*/
    float angle_raw;               /* 电机多圈角度 deg（观察用）*/
    float speed;                   /* 电机转速 RPM（观察用）*/
    float rc_ch;                   /* 遥控通道 0~3，可在线改 */
    float max_w;                   /* 满杆角速度 rad/s，可在线改 */
    float world_kp;                /* 锁世界位置环增益 RPM/deg，可在线改 */
    float world_kd;                /* 陀螺阻尼系数 RPM/(deg/s)，可在线改 */
    float yaw_rate_lpf;            /* 陀螺 yaw 角速度低通状态（RPM）*/
    float target_rate;             /* 摇杆给的目标角速度 deg/s（外环前馈用）*/
    float ff_gain;                 /* 底盘角速度前馈系数 0~1，可在线改 */
    uint32_t last_dbus_count;      /* 上一周期见到的遥控帧计数 */
    uint16_t dbus_offline_ms;      /* 遥控断线累计 ms */
    float imu_yaw;                 /* IMU 实测的云台世界朝向 deg（观察用）*/
    float target_speed;            /* 目标转子转速 RPM（观察用）*/
    SysTick_Task task;

    uint32_t last_rx_count;        /* 上一周期收到的反馈累计次数 */
    uint16_t offline_ms;           /* 连续未收到反馈的时间 ms */
    uint8_t  is_online;            /* 1=反馈正常 */
    uint8_t  is_aligned;           /* 1=已完成首次对齐（以当前位置为零点） */
    uint8_t  has_ever_online;      /* 1=本次上电曾经收到过反馈 */

    /* PID 测试波形（在线可调）*/
    float wave_en;                 /* 1=波形注入打开 */
    float wave_type;               /* WAVE_TYPE 枚举值 */
    float wave_amp;                /* 幅值 RPM */
    float wave_freq;               /* 频率 Hz */
    float wave_duty;               /* 占空比 0~1 */

    /* ---- pitch 轴：真串级（外环角度 + 内环速度），见本文件顶部说明。两环都用增强 PID
       （user_pid_adv）：user_pid 的 ki/kd 隐含吸收了 dt，改成显式 dt 会让底盘/yaw 参数失配。 */
    DJI_MOTOR_DRIVES*    pitch_motor;      /* GM6020，Rotor_angle（多圈角度）模式 */
    PID_Adv_Controller   pitch_pid_angle;  /* 外环：deg -> RPM */
    PID_Adv_Controller   pitch_pid_speed;  /* 内环：RPM -> 电流刻度 */
    Cascade_Controller   pitch_cascade;    /* 由上面两个 PID 组成，注入给电机 */
    float pitch;                           /* 当前俯仰角 deg */
    float target_pitch;                    /* 目标俯仰角 deg（拨轮积分 / 弹道解算写入）*/
    float pitch_speed;                     /* 电机转速 RPM（观察用）*/
    float pitch_current;                   /* 电机电流 mA（观察用）*/
    float pitch_wheel;                     /* 归一化拨轮量 -1~1（观察用）*/
    float pitch_wheel_dead;                /* 拨轮死区（DBUS roll 原始值），可在线改 */
    float pitch_max_w;                     /* 满拨轮角速度 deg/s，可在线改 */
    float pitch_auto;                      /* 1=目标由上层写入（弹道解算），可在线改 */
    float pitch_speed_target;              /* CASCADE=0 时外环算出的转速目标（观察用）*/
    uint32_t last_pitch_rx_count;          /* 上一周期 pitch 反馈计数 */
    uint16_t pitch_offline_ms;             /* pitch 连续未收到反馈时间 ms */
    uint8_t  pitch_is_online;              /* 1=pitch 反馈正常 */
    uint8_t  pitch_has_ever_online;        /* 1=本次上电 pitch 曾经在线 */
    uint8_t  pitch_is_aligned;             /* 1=已完成 pitch 角度标定 */
    uint32_t last_pitch_dbus_count;        /* 遥控帧计数 */
    uint16_t pitch_dbus_offline_ms;        /* 遥控断线累计 ms */
} GIMBAL_DRIVES;

void  Gimbal_Init(GIMBAL_DRIVES* gimbal, DJI_MOTOR_DRIVES* yaw_motor);

/* 把当前位置记成云台正前方（零点对齐）；上电时 Gimbal_Update 自动调用一次，掉线重连后也会重新对齐 */
void  Gimbal_Set_Zero(GIMBAL_DRIVES* gimbal);

/* 云台相对底盘的角度 deg，逆时针为正，范围 ±180 */
float Gimbal_Get_Yaw(const GIMBAL_DRIVES* gimbal);

/* GM6020 反馈是否正常，掉线返回 0 */
uint8_t Gimbal_Is_Online(const GIMBAL_DRIVES* gimbal);

/* 是否已完成首次对齐（对齐前不会下发任何指令，避免上电猛转） */
uint8_t Gimbal_Is_Aligned(const GIMBAL_DRIVES* gimbal);

/* 云台角度是否【可信】给底盘当航向源用：从没连上过 → 可信（裸车调试，角度恒 0）；
   连上过又掉线 → 不可信（角度是冻结值会跑偏）；在线但没对齐 → 不可信；在线且已对齐 → 可信 */
uint8_t Gimbal_Is_Reliable(const GIMBAL_DRIVES* gimbal);

/* 把 user_gimbal 里的波形参数应用到 user_wave（在线改参数后调用） */
void Gimbal_Wave_Apply(void);

/* pitch 轴初始化。★ 必须在 DJI_Motor_Init(&user_gimbal_pitch_motor, ...) 【之前】调用：电机控制器
   指针指向 pitch_cascade，之后 CAN 回调随时会调 cascade->Calculate，内外环指针为 NULL 就空指针解引用 */
void  Gimbal_Pitch_Init(GIMBAL_DRIVES* gimbal, DJI_MOTOR_DRIVES* pitch_motor);

/* 当前俯仰角 deg（相对"上电标定位姿 + 偏置"）*/
float Gimbal_Get_Pitch(const GIMBAL_DRIVES* gimbal);

/* 上层（弹道解算 / 自动瞄准）写入 pitch 目标角 deg。写完自动把 gimbal_pitch_auto 置 1，切到
   "外部给目标"模式；该模式下遥控拨轮不再改变目标（避免和外部目标互相打架）。 */
void  Gimbal_Set_Pitch_Target(GIMBAL_DRIVES* gimbal, float deg);

/* 切换 pitch 目标来源：1=外部（弹道/自瞄），0=纯遥控积分；切回 0 时目标角重置到当前实际角 */
void  Gimbal_Set_Pitch_Auto(GIMBAL_DRIVES* gimbal, uint8_t enable);

/* pitch 反馈是否正常，掉线返回 0 */
uint8_t Gimbal_Pitch_Is_Online(const GIMBAL_DRIVES* gimbal);

/* pitch 是否已完成角度标定（标定前不下发任何指令，避免上电猛打）*/
uint8_t Gimbal_Pitch_Is_Aligned(const GIMBAL_DRIVES* gimbal);

#endif // GIMBAL_H
