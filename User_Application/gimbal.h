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

#define GIMBAL_YAW_POWER_LIMIT  (90.0f)      /* 单电机功率上限 W（GM6020 满量程 3A×24V=72W，所以 90W 不会成为瓶颈）*/

/* 速度环 PID（Rotor_speed：目标/反馈都是转子转速 RPM，输出是电流刻度，满量程 ±16384）。
   ⚠️⚠️ Ki 必须保持 0 —— 这条是踩过坑的：
     这个环用的是老 user_pid.c，Iout += ki*err 里【没有 dt】，而它被 1kHz 的 CAN 反馈调用，
     所以等效积分增益 = ki×1000。再叠加 GM6020 的 rotor_speed 是【整数】（静止时读数在
     0/±1 之间跳），Ki>0 会立刻形成静止极限环 —— 表现为"刚上电云台就慢慢抖"。
     想要刚度请用下面的 MAX_OUT（电流权限）+ 外环 world_kp/world_kd，不要动这里的 Ki。 */
#define GIMBAL_YAW_KP           (48.0f)
#define GIMBAL_YAW_KI           (0.0f)
#define GIMBAL_YAW_KD           (1.0f)
/* ★★★ 电流权限 —— 这是"yaw 一直不硬"的根因，不是增益问题：
   原来这里是 3000，只有满量程 ±16384 的 18%！
   换算：3000/16384 × 3A = 0.55A × 0.741N·m/A ≈ 0.41 N·m，而 GM6020 满量程能出 2.2 N·m。
   即"再怎么调增益，最多只有 5.5 分之一 的扭矩可用"，所以怎么调都发软、被惯性拖着停不下来。
   12000 ≈ 2.2A ≈ 1.6 N·m，功率 2.2A×24V = 53W 仍在 90W 预算内。
   想再激进可以给到 16384（满量程 3A / 72W），但先确认机械和发热没问题。 */
#define GIMBAL_YAW_MAX_OUT      (12000.0f)
#define GIMBAL_YAW_MAX_IOUT     (1000.0f)    /* Ki=0 时不起作用，留着以防以后开积分 */

/* 云台 yaw 遥控：1=开启。DBUS 通道号：0=右摇杆左右 1=右摇杆上下 2=左摇杆左右 3=左摇杆上下 */
#define GIMBAL_RC_ENABLE        (1)
#define GIMBAL_RC_CH            (0)          /* ★ yaw = 右摇杆【左右】 */
#define GIMBAL_YAW_MAX_W        (3.0f)       /* 满杆角速度 rad/s：6.0=344deg/s 太快，高速下"停不下来"
                                                与超调都随速度线性变差，降到 3.0=172deg/s。在线调 gimbal_max_w */

/* ★★ 摇杆速率前馈增益 KF —— "跟手"的主旋钮
   摇杆给出的目标角速度会以 `target_rate × KF / 6` RPM 直接前馈进速度环。
   运动学上 KF = 1.0 就是精确值（1 RPM = 6 deg/s），也就是"理论最优"。
   KF > 1 = 略微"超喂"：起步时让云台先快一点点，抵消摩擦、齿槽、惯性的滞后，手感更利落。
   ⚠️ 代价：运动过程中云台会略微"领着"目标，松杆时要把这点引量还回来 —— KF 太大会有橡皮筋感。
   ★ 它是【前馈】、不在反馈回路里，所以改它【不影响稳定性】—— 这正是它比加 kp 安全的原因。
   在线调 gimbal_kf，建议范围 1.0 ~ 1.3。 */
#define GIMBAL_YAW_KF           (1.30f)
/* ---- 自瞄（视觉）接管配置 ---- */
#define GIMBAL_YAW_AUTO_DEF     (0.0f)     /* 0=摇杆 1=视觉接管（在线调 yaw_auto）*/
#define GIMBAL_CAM_YAW_OFF      (0.0f)     /* 相机光轴与枪管的水平零位偏置 deg */
#define GIMBAL_AIM_FF_GAIN      (1.0f)     /* 视觉角速度前馈增益：1.0=直接用视觉给的角速度 */
#define GIMBAL_AIM_DELAY_MS     (20.0f)    /* 链路延迟估计 ms，需实测（先给 20）*/
#define GIMBAL_AIM_TIMEOUT_MS   (120.0f)   /* 丢帧超时 ms */

/* ★★★ 陀螺零偏在线跟踪 —— 根治"摇杆不动时每 5~6 秒飘一次"-----------------
   机理：IMU 的 yaw 是【纯陀螺积分、没有绝对参考】，零偏（实测约 0.05deg/s）会让 imu_yaw
   自己缓慢漂移；位置环为了把 imu_yaw 钉在固定目标上，就必须让云台【物理上】以同样速率
   反向爬行 —— 那个周期性小修正就是这么来的。死区只能把它量化成"隔几秒跳一下"，治不了根；
   而"把目标挪过去跟着漂"更糟：云台会跟着一起漂。

   ★ 正确做法是【把零偏从反馈里减掉】：
     摇杆居中 + 编码器说云台没动  ->  真实角速度≈0  ->  此时测到的 yaw_rate 就是零偏
       ->  低通成零偏估计 yaw_bias，累积成 bias_accum
       ->  位置反馈改用 (imu_yaw - bias_accum)
     反馈量不再漂，位置环就不用再爬，云台才能真正停住；同时位置环保持满刚度，
     死区也就可以缩到 0.05（顺带治"到位后轻轻飘"）。
   ⚠️ 门控必须同时满足"摇杆居中"和"编码器没动"，否则真实的慢速转动会被当成零偏吃掉。 */
#define GIMBAL_BIAS_TRACK_ALPHA     (0.0002f)  /* 零偏估计一阶低通：α=dt/τ，1kHz 下 τ=5s。
                                                  在线调 gimbal_bias_k（填 0 = 关闭，便于 A/B 对比）*/
#define GIMBAL_BIAS_ENC_MAX_RATE    (0.5f)     /* 认为"没动"的编码器角速度阈值 deg/s。
                                                  ⚠️ 必须远大于零偏爬行速率(0.05)，否则门控把自己关死 */
#define GIMBAL_ENC_RATE_ALPHA       (0.002f)   /* 编码器角速度低通（1kHz 下 τ≈0.5s）：
                                                  压掉整数计数 0.044deg/count 的量化噪声 */

/* ---- 锁世界位置环（抗惯性三件套）------------------------------------------
   外环是"角度误差 -> 转速目标"，1 RPM = 6 deg/s，所以开环时间常数 τ ≈ 6/world_kp 秒。
   原来 world_kp = 1.0 -> τ ≈ 6 秒 —— 这就是"云台偏软、走起来被惯性拖着"的根因。
   ★ 整定顺序必须【先加阻尼 world_kd，再提增益 world_kp】，反过来必振荡。 */
#define GIMBAL_WORLD_KP         (3.0f)       /* τ 6s -> 2.0s */
/* 陀螺阻尼：用板载 IMU 的 yaw 角速度做阻尼，量纲 RPM/(deg/s)，纯运动学取值 = 1/6 ≈ 0.167。
   ★ "转快了冲出去再回来"= 阻尼不够。这一项就是刹车，按 ζ（阻尼比）配：kd ≈ 2ζ√(6·kp·τ内)/6 - 1/6。
     在线整定口诀：冲过头 -> 加 gimbal_world_kd；发飘/迟钝 -> 加 gimbal_world_kp。 */
#define GIMBAL_WORLD_KD         (0.55f)
/* 阻尼源一阶低通系数 0~1：陀螺噪声会直接进转速指令，越小滤得越狠、相位滞后越大。
   0.25 时滞后较大，把阻尼在高频处削弱了 -> 提到 0.40，阻尼能更早开始刹车（噪声大再往下调）*/
#define GIMBAL_WORLD_D_ALPHA    (0.40f)
/* 底盘角速度前馈系数：运动学上精确抵消底盘自转就是 1.0。
   原来 0.3 = 只补 30%，剩 70% 让位置环去追 -> 稳态滞后 + 摆动。抖就 0.8，迟钝就 1.0 */
#define GIMBAL_FF_GAIN          (1.0f)
/* ★ 摇杆速率限斜率 deg/s² —— 治"转快了冲出去再回来"的另一半：
   松杆瞬间前馈从 344deg/s（满杆 6rad/s）直接掉到 0，而云台还在以 344deg/s 走，
   位置环只能事后去追 -> 先冲出去再被拉回来。限斜率让前馈"先减下来"，
   云台跟着平滑减速，超调显著变小。1500deg/s² -> 满杆到零约 0.23s。
   ★ 但它有代价：限斜率期间【目标角还在往前走】，所以"停不下来"会变明显一点。
     满杆速度已降到 3.0rad/s，这里提到 2500deg/s²，让减速段缩到约 0.07s（多走约 6°），
     既保住"不冲出去"，又不至于"停不下来"。太小会跟手变钝，太大等于没加。在线调 gimbal_rate_slew */
#define GIMBAL_RATE_SLEW        (2500.0f)

/* 修正量限幅 RPM —— ★ 只夹【位置修正部分】，摇杆前馈在限幅之后再相加（见 gimbal.c）。
   原来 60 太小：满杆前馈本身就 57 RPM，等于把阻尼余量挤没了 -> 冲一下再回来。
   它的职责只是拦住跨 ±180° 的一脚踹和 IMU 异常，150 RPM 足够宽松又仍能兜底。 */
#define GIMBAL_WORLD_MAX_RPM    (150.0f)

/* ★★ 外环死区 deg —— 这是"停着不动也慢慢抖"的关键一条：
   本工程 IMU 的 yaw 是【纯陀螺积分、没有绝对参考】（互补滤波只融合了 pitch），
   静止时 yaw 会缓慢漂移；外环去追这个漂移，就表现为静止时慢慢抖/慢慢转。
   死区内把角度误差清零（阻尼与前馈仍然保留），静止就真的静止。
   pitch 外环早就有同样的死区（pitch_ang_dz），yaw 这里补上以保持一致。
   0.2~0.5 都合适；调太大跟踪会有静差。在线调 gimbal_world_dead */
#define GIMBAL_WORLD_DEAD_DEG   (0.05f)

/* ★★ IMU 装在哪：1 = 装在云台上（imu_yaw 就是云台世界朝向，直接用）
                  0 = 装在底盘上（云台世界朝向 = 底盘 yaw + 云台相对底盘的编码器角）
   ✅ 本车已确认 = 1：C 板装在 yaw 与 pitch 的连接处（相当于"头部"），
      所以板载 BMI088 的 yaw 就是云台的世界朝向，imu_yaw 可直接当云台世界角用。
      ⚠️ 换车/挪板子必须重新确认，填错会让锁世界锁错对象（云台跟着底盘转）。
      判定法：① 固定底盘、手动转云台，看串口 gimu/yaw 变不变；② 固定云台、手动转底盘，再看。 */
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
/* 软件限位（度）—— ★ 按【物理方向】命名，避免正负号搞反：
   你要的是"向前俯最多 15°、向后仰最多 30°"。
   这里假定【向前俯 = 负角度、向后仰 = 正角度】；
   若实测软限位方向还是反的，把下面两个数值对调即可（不用改别的）。 */
#define GIMBAL_PITCH_FORWARD_MAX_DEG   (15.0f)   /* 向前俯的最大角度 */
#define GIMBAL_PITCH_BACKWARD_MAX_DEG  (30.0f)   /* 向后仰的最大角度 */
#define GIMBAL_PITCH_MIN_DEG    (-GIMBAL_PITCH_FORWARD_MAX_DEG)
#define GIMBAL_PITCH_MAX_DEG    ( GIMBAL_PITCH_BACKWARD_MAX_DEG)

/* 上电时枪管的真实俯仰角：上电前摆到水平就填 0，否则量出实际角度填这里（pitch 读数才是真角度，
   软限位才落在正确位置）。⚠️ 每次上电必须是同一姿态，否则零点会漂（同 yaw 的上电对准前方）*/
#define GIMBAL_PITCH_ZERO_BIAS_DEG (0.0f)

/* 外环（角度环）：输入 deg，输出【转速目标 RPM】。用增强 PID（user_pid_adv），ki/kd 量纲带 s
   （ki = 输出/(误差·s)、kd = 输出/(误差/s)），数值比 user_pid 隐含 dt 的写法大，是正常的。 */
#define GIMBAL_PITCH_ANGLE_KP       (12.0f)
#define GIMBAL_PITCH_ANGLE_KI       (2.0f)
#define GIMBAL_PITCH_ANGLE_KD       (0.2f)
/* ★★★ 目标速度前馈 —— 这是"pitch 轻微推动响应慢/不跟手"的正解：
   外环是"角度误差 -> 转速目标"，纯 P 要产生转速就必须先有误差：
   要跟上 120deg/s 的目标，输出需要 20RPM，就得先攒 20/12 = 1.67° 的滞后 -> 手感"慢半拍"。
   KF 把【目标的角速度】直接前馈进去，滞后直接归零（理论上 KF = 1/6 ≈ 0.167 就把
   "deg/s 的目标速率" 精确换算成 "RPM 的输出"，因为 1RPM = 6deg/s）。
   ⚠️ 前馈不参与死区，所以小推杆也会立刻动 —— 死区只影响 P 项，响应不再被死区拖慢。 */
#define GIMBAL_PITCH_ANGLE_KF       (0.167f)
#define GIMBAL_PITCH_ANGLE_MAX_OUT  (200.0f)  /* 外环输出上限 = 内环转速目标上限 RPM */
#define GIMBAL_PITCH_ANGLE_MAX_IOUT (60.0f)
#define GIMBAL_PITCH_ANGLE_MAX_DOUT (60.0f)
/* 死区 deg：压住零位附近抖动。原来 0.15 偏大 —— 死区内 P 项为 0，
   到位后会在这个范围内"自由飘"，表现为"动过去后还会轻轻动一点角度"。改成 0.08。 */
#define GIMBAL_PITCH_ANGLE_DEAD     (0.08f)
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
/* pitch 遥控：★ 改用【右摇杆上下】（原来在左上角拨轮；拨轮已空出来，
   若想改回拨轮或换成别的通道，只要改 GIMBAL_PITCH_RC_CH 或在线上发 pitch_ch=N#）*/
#define GIMBAL_PITCH_RC_ENABLE   (1)
#define GIMBAL_PITCH_RC_CH       (1)          /* ★ pitch = 右摇杆【上下】 */
#define GIMBAL_PITCH_RC_DIR      (1.0f)       /* 推杆方向：+1 或 -1，现场核对 */
#define GIMBAL_PITCH_MAX_W       (200.0f)     /* 满杆角速度 deg/s：120 时小幅推杆目标走得太慢（10% 杆=12deg/s），
                                                 提到 200 让"轻微推动"也跟得上。在线调 pitch_max_w */
/* 摇杆死区（DBUS ±660）。★ 必须大于遥控器松手后的实际静息偏移，否则目标角会一直
   被残余杆量缓慢积分 -> "动过去后还会轻轻动一点角度"。
   ⚠️ 它和"轻微推动的响应"是直接对立的：死区越大越不爬、但小杆量越迟钝。
      12 是折中；先按 12 试，若还爬就 pitch_dead=20/25，若嫌迟钝就 pitch_dead=8。 */
#define GIMBAL_PITCH_RC_DEAD     (12)
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
    float world_dead;              /* 外环角度死区 deg，可在线改 */
    float rate_slew;               /* 摇杆速率限斜率 deg/s²，可在线改 */
    float yaw_rate_lpf;            /* 陀螺 yaw 角速度低通状态（RPM）*/
    float target_rate;             /* 摇杆给的目标角速度 deg/s（外环前馈用）*/
    float ff_gain;                 /* 底盘角速度前馈系数 0~1，可在线改 */
    float stick_kf;                /* 摇杆速率前馈增益（跟手主旋钮），可在线改 */
    float yaw_bias;                /* 陀螺 yaw 零偏估计 deg/s */
    float bias_accum;              /* 零偏累积角 deg，从位置反馈里减掉 */
    float bias_k;                  /* 零偏跟踪系数，可在线改（0=关闭）*/
    float enc_rate_lpf;            /* 编码器角速度低通状态 deg/s */
    float enc_last;                /* 上一拍编码器角度 deg */
    /* ---- 自瞄（视觉）接管 ---- */
    float    yaw_auto;        /* ≠0 = 目标角由视觉给，摇杆不参与。在线调 yaw_auto */
    float    yaw_auto_pre;    /* 上一拍标志，用于检测"接管瞬间" */
    float    yaw_target_ext;  /* 视觉给的目标角 deg */
    float    cam_yaw_off;     /* 相机光轴 <-> 枪管零位偏置 deg（打靶标定）*/
    float    aim_ff_gain;     /* 视觉角速度前馈增益（前馈不在回路里，改它不影响稳定性）*/
    float    aim_delay_ms;    /* 链路延迟补偿 ms */
    float    aim_timeout_ms;  /* 丢帧超时 ms，超时就冻结目标角 */
    float    aim_err;         /* 观察用：视觉目标 - 实际 */
    float    aim_ok_f;        /* 观察用镜像：1=数据有效 */
    uint8_t  aim_valid;
    uint32_t aim_ms;
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
    float pitch_stick;                     /* 归一化摇杆量 -1~1（观察用）*/
    float pitch_rc_ch;                     /* pitch 遥控通道 0~3，可在线改 */
    float pitch_dead;                      /* pitch 摇杆死区（DBUS 原始值），可在线改 */
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
