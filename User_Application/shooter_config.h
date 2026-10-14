#ifndef SHOOTER_CONFIG_H
#define SHOOTER_CONFIG_H

/* 三个电机都挂 CAN2：1/2 = 摩擦轮 M3508+C620，3 = 拨弹轮 M2006+C610 */
#define SHOOTER_FRICTION_LEFT_ID    (1)
#define SHOOTER_FRICTION_RIGHT_ID   (2)
#define SHOOTER_FEEDER_ID           (3)

/* 摩擦轮对装、转向相反（装反把对应项改成 -1）；喂弹方向反了改 FEEDER_DIR */
#define SHOOTER_FRICTION_LEFT_DIR   ( 1.0f)
#define SHOOTER_FRICTION_RIGHT_DIR  (-1.0f)

#define SHOOTER_FEEDER_DIR          ( 1.0f)

/* 摩擦轮转速（转子 RPM）：待发 / 发射 */
#define SHOOTER_FRICTION_RPM_READY  (3500.0f)
#define SHOOTER_FRICTION_RPM_FIRE   (6000.0f)
/* 就绪容差 RPM：达到目标 ± 这个值才允许拨弹（没起速就拨会把弹卡在两轮之间）；起转超时后不再等，未就绪仍不拨 */
#define SHOOTER_FRICTION_READY_TOL  (600.0f)
#define SHOOTER_SPINUP_TIMEOUT_MS   (3000)

/* 摩擦轮速度环 PID：输入 RPM，输出电流刻度 ±16384；POWER_MAX 单位 W */
#define SHOOTER_FRICTION_KP         (20.0f)
#define SHOOTER_FRICTION_KI         (0.5f)
#define SHOOTER_FRICTION_KD         (0.0f)
#define SHOOTER_FRICTION_MAX_OUT    (16384.0f)
#define SHOOTER_FRICTION_MAX_IOUT   (4000.0f)
#define SHOOTER_FRICTION_POWER_MAX  (60.0f)

/* 拨弹轮：位置式，走固定角度一步一发。M2006 减速比 36:1：反馈是【转子角】，除以 36 才是【拨盘角】；对外参数用拨盘角 */
#define SHOOTER_FEEDER_GEAR         (36.0f)
#define SHOOTER_FEEDER_SLOTS        (8)         /* 拨盘一圈几个弹位 */
#define SHOOTER_FEEDER_STEP_DIAL_DEG (360.0f / (float)SHOOTER_FEEDER_SLOTS)

/* 拨弹轮串级 PID：外环 角度->转速（输出即内环转速目标 RPM），内环 转速->电流（C610 刻度 ±16384）*/
#define SHOOTER_FEEDER_ANGLE_KP       (6.0f)
#define SHOOTER_FEEDER_ANGLE_KI       (0.0f)
#define SHOOTER_FEEDER_ANGLE_KD       (0.2f)
#define SHOOTER_FEEDER_ANGLE_MAX_OUT  (300.0f)
#define SHOOTER_FEEDER_ANGLE_MAX_IOUT (100.0f)

#define SHOOTER_FEEDER_SPEED_KP       (8.0f)
#define SHOOTER_FEEDER_SPEED_KI       (0.3f)
#define SHOOTER_FEEDER_SPEED_KD       (0.0f)
#define SHOOTER_FEEDER_SPEED_MAX_OUT  (10000.0f)
#define SHOOTER_FEEDER_SPEED_MAX_IOUT (2000.0f)
#define SHOOTER_FEEDER_POWER_MAX      (30.0f)

/* 遥控 sw2 档位（与底盘用的 sw1 分工，互不冲突）：上(1)=单发 中(3)=停火 下(2)=连发 */
#define SHOOTER_SW2_SINGLE          (1)
#define SHOOTER_SW2_AUTO            (2)
#define SHOOTER_SW2_STOP            (3)
#define SHOOTER_SW_VALID(v)         ((v) == SHOOTER_SW2_SINGLE || (v) == SHOOTER_SW2_AUTO)

/* ⚠️ 拨轮 roll 已被 pitch 轴占用（见 gimbal.h），所以默认 0，直接用 sw2 档位当扳机：
   单发档取进入边沿（挂起后等就绪再打），连发档取电平；roll 范围 ±660，超阈值算扣下 */
#define SHOOTER_TRIGGER_USE_WHEEL   (0)
#define SHOOTER_TRIGGER_THRESHOLD   (300)

/* 连发间隔 ms；单发一次走几步；连发是否需要按住扳机（1=按住才连发，更安全）*/
#define SHOOTER_FIRE_INTERVAL_MS    (120)
#define SHOOTER_SINGLE_SHOTS        (1)
#define SHOOTER_AUTO_NEED_TRIGGER   (1)

/* 卡弹判据：位置没跟上 且 电流大，两条同时持续 JAM_MS 才判 ——
   只看电流会被机构本身的机械冲击误触发。ERR 是转子角 deg，CURRENT 是 C610 刻度 */
#define SHOOTER_JAM_ERR_DEG         (30.0f)
#define SHOOTER_JAM_CURRENT         (6000.0f)
#define SHOOTER_JAM_MS              (150)
#define SHOOTER_REVERSE_MS          (120)
#define SHOOTER_REVERSE_SCALE       (-0.5f)     /* 反转步长比例（负号=往回走）*/

/* FIR 滤波（拨弹轮电流 + 摩擦轮转速）。⚠️ order = 抽头数 - 1（FIR_Init 按 order+1 当 numTaps）；
   8 抽头滑动平均，1kHz 下群延迟 3.5ms，够用且几乎不占 CPU */
#define SHOOTER_FIR_ORDER           (7)
#define SHOOTER_FIR_TAPS            (SHOOTER_FIR_ORDER + 1)

#define SHOOTER_PERIOD_MS           (1)

/* 总开关：0 = 整个射击模块不初始化、不建任务（排错/二分定位时用）*/
#define SHOOTER_ENABLE              (1)

/* ★ 弹道解算周期 ms。绝对不要放进 1kHz 控制任务：牛顿迭代要算 tanf/cosf，本工程是【软件浮点】，
   一次求解可能上千个周期，而所有任务都在 1ms 的 SysTick 中断里跑，放进去会把整套控制饿死；
   弹道不需要快，20ms（50Hz）绰绰有余 */
#define SHOOTER_BALLISTICS_PERIOD_MS (20)

#endif /* SHOOTER_CONFIG_H */
