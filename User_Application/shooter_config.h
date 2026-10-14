#ifndef SHOOTER_CONFIG_H
#define SHOOTER_CONFIG_H

/* 三个电机都挂 CAN2：1/2 = 摩擦轮 M3508+C620，3 = 拨弹轮 M2006+C610 */
#define SHOOTER_FRICTION_LEFT_ID    (1)
#define SHOOTER_FRICTION_RIGHT_ID   (2)
#define SHOOTER_FEEDER_ID           (3)

/* 摩擦轮对装、转向相反（装反把对应项改成 -1）；喂弹方向反了改 FEEDER_DIR */
#define SHOOTER_FRICTION_LEFT_DIR   ( 1.0f)
#define SHOOTER_FRICTION_RIGHT_DIR  (-1.0f)

/* ★ 拨盘旋转方向：转反了就把这个改成 -1.0（现在已按实车设为 -1，即原来 +1 时是反的）。
   安全性说明：拨盘是【位置环】，方向不能靠把电流输出取反（那会变成正反馈直接飞车）。
   本工程把方向作用在【目标角】上（step_rotor_deg = DIR × 拨盘步长 × 减速比），
   目标往回退 -> 位置环自然反向驱动，反馈符号不变，所以不会自激。
   ★ 也可以在线改：feed_dir=-1# / feed_dir=1#（不用重新烧录就能试两个方向）。 */
#define SHOOTER_FEEDER_DIR          (-1.0f)

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

/* ★★★ 拨盘转速的理论（"拨盘太慢"该调哪里看这里）---------------------------
   一发要走的角度 = STEP_DIAL_DEG × GEAR = 45° × 36 = 1620°【转子角】= 4.5 圈转子。

   它是位置串级：外环(角度->转速) 的输出就是【内环的转速目标 RPM】，而这个输出
   被 ANGLE_MAX_OUT 限幅 —— 所以拨盘的最高转速 = ANGLE_MAX_OUT RPM（转子转速）：

       一步最快耗时 = 1620 / (ANGLE_MAX_OUT × 6)   秒      （RPM × 6 = deg/s）

   原来 ANGLE_MAX_OUT = 300：最高 300 RPM -> 一步 0.9 秒（≈1.1 发/秒）。
   而 fire_interval_ms = 120ms 想要 8.3 发/秒 —— 伺服比节奏慢了 7.5 倍，
   所以瓶颈完全在这个限幅上，fire_int 调多小都没用。

   ⚠️ 因此：拨盘慢【不要去调 KP/KI】。外环在 1620° 的大误差下一直处于饱和状态
      （kp=6 × 1620 = 9720 RPM 的需求，远超限幅），KP 大小根本体现不出来。
      要快就抬 ANGLE_MAX_OUT（软件），或改减速比/减少槽数（机械）。

   改成 2500：最高 2500 RPM -> 一步 0.108 秒，120ms 的节奏才真正生效
   （实际还要算加减速，约 0.12~0.15s/发）。想更快就继续加，上限看电机发热与卡弹。 */
#define SHOOTER_FEEDER_ANGLE_MAX_OUT  (2500.0f)
#define SHOOTER_FEEDER_ANGLE_MAX_IOUT (100.0f)

/* 拨弹轮串级 PID：外环 角度->转速（输出即内环转速目标 RPM），内环 转速->电流（C610 刻度 ±16384）*/
#define SHOOTER_FEEDER_ANGLE_KP       (6.0f)
#define SHOOTER_FEEDER_ANGLE_KI       (0.0f)
#define SHOOTER_FEEDER_ANGLE_KD       (0.2f)

#define SHOOTER_FEEDER_SPEED_KP       (8.0f)
#define SHOOTER_FEEDER_SPEED_KI       (0.3f)
#define SHOOTER_FEEDER_SPEED_KD       (0.0f)
#define SHOOTER_FEEDER_SPEED_MAX_OUT  (10000.0f)
#define SHOOTER_FEEDER_SPEED_MAX_IOUT (2000.0f)
/* 30W -> 45W：原来 30W/24V = 1.25A，C610 满量程 10A，拨盘加速时电流被掐得很死。
   45W ≈ 1.9A，给加速留余量；仍然是保守值（C610 能到 10A），发热不对就往下调。 */
#define SHOOTER_FEEDER_POWER_MAX      (45.0f)

/* 遥控 sw2 档位（右拨杆，与底盘用的 sw1 分工）——【从上到下】：
     上(1) = 仅摩擦轮：摩擦轮起转到 fire_rpm，但【不拨弹】
     中(3) = 无：全停
     下(2) = 摩擦轮 + 拨弹盘：摩擦轮起转，并按 fire_interval_ms 连续拨弹
   ⚠️ DBUS 的档位编码是 上=1 / 下=2 / 中=3（和底盘的 sw1 同一套约定，别想当然按 1/2/3 顺序）*/
#define SHOOTER_SW2_FRICTION_ONLY   (1)      /* 上：只转摩擦轮 */
#define SHOOTER_SW2_FIRE            (2)      /* 下：摩擦轮 + 拨弹盘 */
#define SHOOTER_SW2_STOP            (3)      /* 中：全停 */
#define SHOOTER_SW_VALID(v)         ((v) == SHOOTER_SW2_FRICTION_ONLY || (v) == SHOOTER_SW2_FIRE)

/* 拨弹触发：现在【下档】直接就拨弹，不需要额外扳机。
   ★ 左上角拨轮 roll 因为 pitch 改到右摇杆而空出来了：想改成"按住拨轮才拨弹"，
     只要在 shooter.c 的 feed_enable 定义里与上 (user_dbus.roll > 阈值) 即可。 */

/* 拨弹间隔 ms（下档连续拨弹的节奏）*/
#define SHOOTER_FIRE_INTERVAL_MS    (120)

/* 卡弹判据：位置没跟上 且 电流大，两条同时持续 JAM_MS 才判 ——
   只看电流会被机构本身的机械冲击误触发。ERR 是转子角 deg，CURRENT 是 C610 刻度。
   ⚠️ CURRENT 必须【低于】功率限幅换算出来的可达到电流，否则卡弹永远判不出来：
     45W / 24V = 1.9A，C610 满量程 10A -> 1.9/10×16384 ≈ 3100 刻度。
     原来写 6000（=3.7A），已经超过功率限幅能达到的电流 -> 这个判据是死的，已改成 2500。 */
#define SHOOTER_JAM_ERR_DEG         (30.0f)
#define SHOOTER_JAM_CURRENT         (2500.0f)
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
