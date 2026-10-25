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

/* ---- 摩擦轮转速：保守弹速 20 m/s --------------------------------------------
   换算公式（用户/PPT 给的就是这个）：
       bullet_speed(m/s) = rpm × 0.10471976 × 0.03 = rpm × 0.0031415928
   于是：20 / 0.0031415928 = 6366.2 RPM -> 取整 6400 RPM（≈ 20.1 m/s）。
   ★ 用户要求"弹丸不要太快、保守"，所以 FIRE 就用 6400，不要凭手感往上加。
   在线可改：friction_rpm=6400#（它与 fire_rpm 是同一个变量，两个名字都能改）。 */
#define SHOOTER_FRICTION_RPM_READY        (3500.0f)
#define SHOOTER_FRICTION_RPM_CONSERVATIVE (6400.0f)
#define SHOOTER_FRICTION_RPM_FIRE         SHOOTER_FRICTION_RPM_CONSERVATIVE
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

/* ---- 拨弹轮：位置式，走固定角度 ------------------------------------------------
   M2006 减速比 36:1：反馈是【转子角】，除以 36 才是【拨盘角】；对外参数一律用拨盘角。

   ★★ 一发的角度（用户定案 + PPT）：
        拨盘齿轮 7 齿 -> 一齿 = 360/7 = 51.4286°
        再 +5°（PPT："适当小角度"防空程/齿隙）
        一发【拨盘角】= 51.4286 + 5 = 56.4286°      <- 对外的 step_dial_deg（在线 step_deg）
        一发【电机转子角】= 56.4286 × GEAR(36) = 2031.43°（≈ 5.64 圈转子）
   ⚠️ 7 是【拨盘】的齿数/弹位数，不是电机内的齿轮；减速比 GEAR=36 保持不变。 */
#define SHOOTER_FEEDER_GEAR         (36.0f)
#define SHOOTER_FEEDER_TEETH        (7)
#define SHOOTER_FEEDER_EXTRA_DEG    (5.0f)
#define SHOOTER_FEEDER_STEP_DIAL_DEG (360.0f / (float)SHOOTER_FEEDER_TEETH + SHOOTER_FEEDER_EXTRA_DEG)

/* ★★★ 拨盘转速的理论（"拨盘太慢"该调哪里看这里）---------------------------
   一发要走的角度 = 56.43° × 36 = 2031.4°【转子角】= 5.64 圈转子。

   它是位置串级：外环(角度->转速) 的输出就是【内环的转速目标 RPM】，而这个输出
   被 ANGLE_MAX_OUT 限幅 —— 所以拨盘的最高转速 = ANGLE_MAX_OUT RPM（转子转速）：

       一步最快耗时 = 2031.4 / (ANGLE_MAX_OUT × 6)   秒      （RPM × 6 = deg/s）

   ANGLE_MAX_OUT = 2500：最高 2500 RPM -> 一步约 0.135 秒（≈7.4 发/秒的理论上限）。
   而 fire_interval_ms = 120ms 想要 8.3 发/秒 —— 已经接近这个限幅的极限，
   所以连发节奏不够快时，瓶颈仍然在这个限幅上，fire_int 调多小都没用。

   ⚠️ 因此：拨盘慢【不要去调 KP/KI】。外环在 2031° 的大误差下一直处于饱和状态
      （kp=6 × 2031 = 12186 RPM 的需求，远超限幅），KP 大小根本体现不出来。
      要快就抬 ANGLE_MAX_OUT（软件），或改减速比/减少齿数（机械）。 */
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

/* ---- ★★ 遥控 sw2 档位（右拨杆）——【新语义】----------------------------------
     上(1) = 单发（位控）：拨动【左上角拨轮】越过死区打一发，必须回中位才能再打
     下(2) = 连发（速控）：按 fire_interval_ms 连续拨弹（沿用原实现）
     中(3) = 只转摩擦轮（不拨弹）
   ⚠️ DBUS 档位编码是 上=1 / 下=2 / 中=3（和底盘的 sw1 同一套约定，别想当然按 1/2/3 顺序）
   ★★ 总闸有两道（详见下面 sw1 那一节）：yaw_auto=1# 且 sw1 拨到【上】才允许摩擦轮/拨弹；
      任一不满足时不管 sw2 在哪，摩擦轮和拨弹盘【全部失能】。 */
#define SHOOTER_SW2_SINGLE          (1)      /* 上：单发（位控 + 停火 FSM）*/
#define SHOOTER_SW2_CONTINUOUS      (2)      /* 下：连发（速控，按 fire_int 节奏）*/
#define SHOOTER_SW2_FRICTION_ONLY   (3)      /* 中：只转摩擦轮，不拨弹 */

/* ---- ★★ 摩擦轮开关：左上角拨杆 sw1（新语义）----------------------------------
   sw1 现在是【双语义】拨杆，语义由 yaw_auto 决定：

     yaw_auto = 0（摇杆）：sw1 归【底盘】用 —— 上=跟随 / 中=手动 / 下=小陀螺
     yaw_auto = 1（自瞄）：sw1 归【射击】用 —— 上=摩擦轮开；中/下=摩擦轮关
                           （此时底盘模式不再由 sw1 决定，跟随模式不可用）

   ⚠️ 所以"跟随模式"和"摩擦轮"虽然都挂在 sw1 的【上】档，但不会同时生效：
      yaw_auto=0 时上档是跟随，yaw_auto=1 时上档才是摩擦轮。

   ★ 总闸变成【两道，缺一不可】：
       ① yaw_auto >= 0.5（在线 yaw_auto=1#）
       ② sw1 == 上（本宏）
     任一不满足 -> 摩擦轮 + 拨弹盘全部失能。 */
#define SHOOTER_SW1_FRICTION_ON     (1)      /* sw1 上档 = 摩擦轮开（DBUS 上=1 / 下=2 / 中=3）*/
#define SHOOTER_SW1_GATE_ENABLE     (0)      /* 1=启用 sw1 这道闸；0=退回旧行为（只看 yaw_auto，排错用）*/

/* ---- ★★ 遥控(DBUS)掉线保护：失联 -> 强制失能（安全关键，第三道总闸）---------------
   判据【复用工程已有机制，不新造一套】：
     user_dji_bus.c:24 的 rc_callback_handler() 每收到一帧 18 字节有效 DBUS 包就
     user_dbus.rx_count++；本模块在 1kHz 的 Shooter_Update 里比较这个计数是否还在增长
     （与 gimbal.c:333 检测遥控在线完全同一套做法）。不用 is_update —— 那个标志会被
     底盘任务清零，多模块抢会丢数据（user_dji_bus.h:16 已注明）。
   行为：连续超过本宏的时长没有新帧 -> 判定失联，arm 恒为 0：
         · 摩擦轮目标 0、拨弹盘目标跟随实际角（不出力）
         · 状态机回 STOP、停火 FSM 回 DISABLED
         · 期间不解析 sw1/sw2/roll（按 0 处理），且拨轮不重新武装
     链路恢复后【不会自动补发】上一帧的档位/拨轮手势：必须先收到新帧，再按正常的
     sw1==上 + yaw_auto 判定起转；单发/连发还要等拨轮回中位后的下一次拨动。
   取值 100ms 的理由：
     ① DT7 遥控约 71Hz（14ms/帧），100ms ≈ 连续丢 7 帧才判掉线，足以滤掉偶发丢帧；
     ② 摩擦轮 6400RPM，真失联时 100ms 内切断动力，来得及，不会"一直转"；
     ③ 与 gimbal.h 的 GIMBAL_OFFLINE_MS(100) 取同一时间尺度，全车遥控/电机掉线判据一致。
   想更保守可降到 50（≈3.5 帧），但射频环境差时会频繁误判失能。 */
#define SHOOTER_DBUS_OFFLINE_MS     (100)

/* ---- ★ 单发触发：遥控【左上角拨轮】--------------------------------------------
   拨轮 = DBUS 的 roll 字段（帧第 16~17 字节，11 bit，松手中位 0，范围约 ±660）。
   ★ 不是 ch0~ch3，也不是 mouse_x —— 见 user_dji_bus.c 的 rc_callback_handler()：
       dbus_drive->roll = ((dbus_buf[16] | dbus_buf[17] << 8) & 0x07FF) - 1024;
     docs/上机调试流程.md 也写明："左上角拨轮 = DBUS 的 roll 字段，不是 ch0~ch3"。
   触发：|roll| >= fire_dead（在线 fire_dead=...）打一发；
   重新武装：必须回到 |roll| <= fire_dead × RELEASE_RATIO（50% 回滞），
             所以"一直推着拨轮不放"只会打一发 —— 这就是用户要的防连发。 */
#define SHOOTER_FIRE_DEAD           (100.0f)
#define SHOOTER_FIRE_RELEASE_RATIO  (0.5f)

/* ---- ★★ 停火有限状态机（照 PPT）----------------------------------------------
   失能(默认) -> 激活 -> 处理 -> 失能；跑在 1kHz 的 Shooter_Update 里，每拍都过一遍，
   只做几个比较和计时（没有三角函数/浮点重活），不阻塞。

     失能：拨弹轮不出力（目标角跟随实际角 -> 位置误差≈0 -> 不主动出力）
     激活：保持"一发"的角度环目标继续走；摩擦轮电流 > stopper_cur 或 激活 > stopper_ms -> 处理
     处理：拨弹轮切角度闭环锁住【当前角】，发射机构停火，保持一小会儿后回失能

   ★ 电流单位：本工程 DJI_Motor_Get_Current() 返回【mA】
       （= torque_current/16384 × 电调满量程A × 1000；M3508+C620 满量程 20A）。
       所以 PPT 的 8A 阈值 = 8000 mA，默认就是 8000。
   ⚠️ 功率限幅的坑：摩擦轮 SHOOTER_FRICTION_POWER_MAX = 60W，
       60W/24V = 2.5A 就是电调实际能给的最大电流（≈2500 mA），
       【8A 在现有功率限幅下永远到不了】。想让电流判据真正起作用：
         ① 在线把 stopper_cur 调到 2000~2500（会"提前锁角"，见报告），或
         ② 抬 SHOOTER_FRICTION_POWER_MAX。
       默认 8000 时，激活态总是被 stopper_ms = 1s 结束（单发节奏最快 1 秒/发）。 */
#define SHOOTER_STOPPER_CUR_MA      (8000.0f)
#define SHOOTER_STOPPER_MS          (1000.0f)
/* 电流超阈值要连续保持这么多 ms 才认（滤掉单拍噪声；1kHz 下 10 拍）*/
#define SHOOTER_STOPPER_CUR_MS      (10)
/* "处理"状态至少保持多久（锁角停火窗口），然后回失能 */
#define SHOOTER_STOPPER_PROCESS_MS  (20)

/* 拨弹间隔 ms（下档连续拨弹的节奏）*/
#define SHOOTER_FIRE_INTERVAL_MS    (120)

/* 卡弹判据：位置没跟上 且 电流大，两条同时持续 JAM_MS 才判 ——
   只看电流会被机构本身的机械冲击误触发。ERR 是转子角 deg，CURRENT 是 C610 刻度。
   ⚠️ 注意：CURRENT 实际比较的是 DJI_Motor_Get_Current() 的返回值【mA】，
      而 45W / 24V = 1.9A = 1875 mA 就是功率限幅下的电流上限 ——
      所以 2500（mA）这个判据在现有 45W 限幅下也永远到不了（历史遗留，本次不改）。
      真要启用卡弹反转，先把它降到 1500 左右。 */
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
