#ifndef GIMBAL_H
#define GIMBAL_H

#include "main.h"
#include "../User_Architect/user_systick.h"
#include "../User_Drives/User_Motor/user_dji_motor.h"
#include "../User_Algorithm/User_Controller/user_pid.h"
#include "../User_Algorithm/User_Controller/user_pid_adv.h"
#include "../User_Algorithm/User_Controller/user_cascade.h"    //实测625S左右零飘偏一度（10分钟）

/* 云台 yaw 配置 */
/* 锁世界方向（用 Rotor_speed 模式，见 main.c）：推杆 = 世界系里恒速转，松杆 = 锁当前方向、
   推开自动回位；底盘自转靠前馈抵消，云台不跟底盘转。反馈用板载 IMU 的 yaw（A 板装在 yaw 上）。*/
#define GIMBAL_LOCK_WORLD       (1)      /* 0=退回纯速度控制（IMU 有问题时用）*/

/* ★ 角度换算常量：度 -> 弧度（π/180）。
   ★ 只给【观测/绘图镜像】用；云台控制内部（target_yaw / 编码器角 / PID / 限位 / 回位 /
     cam_yaw_off / cam_pitch_off）全部按【度】标定，不要拿它去改任何控制量。 */
#define GIMBAL_DEG2RAD          (0.017453292519943295f)

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
/* ---- 自瞄（视觉）接管配置 ----
   语义按参考工程 crt_gimbal.cpp / ita_robot.cpp:_Gimbal_Control()：
     ① 视觉给的目标角是【世界绝对角】，以【覆盖】的方式给云台（不是叠加到摇杆积分上）；
     ② 视觉给的目标角速度当作【速度前馈】，乘一个系数后叠加到【速度环】的目标上；
     ③ 接管条件 = 使能 && 视觉在线 && mode != NO_CONTROL。
   与参考的差别（本工程比参考完整）：保留了下面的 aim_delay_ms 延迟外推，
   并额外加了一道【数据有效性闸门】（参考工程几乎没有无效值保护，见 GIMBAL_AIM_* 一段）。 */
#define GIMBAL_YAW_AUTO_DEF     (0.0f)  /* 0=摇杆 1=视觉接管（在线调 yaw_auto）。★上电即开：六道保护闸同版已在，视觉离线/mode=0/NaN/越界/跳变 时完全不接管，云台照旧听摇杆 */
#define GIMBAL_CAM_YAW_OFF       (-10.0f)  /* ★ 实测标定(2026-10-19)：准心水平居中；视觉 yaw + 本值 = 云台世界目标 */
#define GIMBAL_CAM_PITCH_OFF     (-3.0f)   /* ★ 实测标定：准心与装甲板同高度 */
/* ★★ 前馈系数与参考工程逐项对齐：
   crt_gimbal.cpp:40  Set_Yaw_Feedforward_Omega_Gain(0.5f)
   crt_gimbal.cpp:47  Set_Pitch_Feedforward_Omega_Gain(0.6f)
   ita_robot.cpp:644/727  feedforward = 系数 × 视觉目标角速度
   ⚠️ 它们只是"用视觉角速度的百分之多少去喂速度环"的无量纲系数（前馈不在反馈回路里，
      改它不影响稳定性，只影响跟踪移动目标时的滞后/超调）。在线可调：aim_ff_gain / aim_ff_pitch */
#define GIMBAL_AIM_FF_GAIN_YAW   (0.0f)   /* ★ 实测(2026-10-17)关掉才不抖：视觉角速度噪声大(±16deg/s)，开0.5会把噪声灌进速度环。要打提前量只能在线 0.1/0.2 小步加 */
#define GIMBAL_AIM_FF_GAIN_PITCH (0.0f)   /* ★ 同上，pitch 视觉速度前馈关掉；之前 0.6 导致 pitch 电机嗡嗡震 */
/* 兼容旧名：原来只有一个 GIMBAL_AIM_FF_GAIN(1.0)，语义就是 yaw 的角速度前馈系数 */
#define GIMBAL_AIM_FF_GAIN       (GIMBAL_AIM_FF_GAIN_YAW)
#define GIMBAL_AIM_DELAY_MS     (20.0f)    /* 链路延迟估计 ms，需实测（先给 20）*/
#define GIMBAL_AIM_TIMEOUT_MS   (120.0f)   /* 丢帧/离线超时 ms：超过就判"视觉真的没了" */

/* ---- ★★★ 视觉数据有效性闸门（安全关键；参考工程没有，本工程必须有）--------------
   实测：视觉在 tracker=lost 时会给出 raw_ctrl_yaw_deg = -9518.6、ctrl_pitch_deg = 197.7
   这类【离谱值】。这些值直接拿去控制，云台会猛甩（撞软限位、丢目标、甚至打机构）。
   所以视觉数据在写进 yaw_target_ext / pitch_target_ext 之前要过下面几道闸：

     闸① NaN/Inf      —— 一旦混进来，后面所有比较都会失效（NaN 与任何数比较都是 false），
                          Gimbal_Wrap180 里的 float->int 转换更是未定义行为，必须先拦。
     闸② 未使能/离线/mode==0 —— 不接管，回摇杆，连算都不算。
     闸③ yaw 量程     —— 归一化到 ±180 后，与【当前世界角】的偏差 > 本阈值 = 无效。
                          注意：-9518.6° 归一化后是 -158.6°，只靠这一道闸不一定拦得住，
                          必须和闸⑤（跳变）一起用。
     闸④ pitch 量程   —— 必须落在【工程已有】的机械软限位（GIMBAL_PITCH_MIN/MAX_DEG）内，
                          超一点（留 MARGIN）就拒绝。
     闸⑤ 跳变门限     —— 与上一帧【有效】目标角的差 > 本阈值 = 单帧野值，本帧拒。
     闸⑥ 连续无效计数 —— 连续 GIMBAL_AIM_INVALID_LIMIT 帧都被拒 -> 退出接管、回摇杆。
   单位都是【度】（工程内部单位）。拒绝时【绝不写】目标角，继续用上一帧的有效值。 */
 #define GIMBAL_AIM_MAX_ERR_DEG       (45.0f)   /* ★ 实测调整：原120度时目标在侧后方也会立刻全力转过去(猛甩)。改45度：只接管近处目标，远处要自己手动转过去，进入45度后自动接管 */
#define GIMBAL_AIM_JUMP_DEG          (60.0f)   /* 闸⑤：单帧目标角跳变门限 */
#define GIMBAL_AIM_PITCH_MARGIN_DEG  (1.0f)    /* 闸④：pitch 有效性判据比软限位松 1° */
#define GIMBAL_AIM_INVALID_LIMIT     (5U)      /* 闸⑥：连续多少帧被拒就退出接管 */

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
/* ★ 降过冲：3.0 -> 2.2。kp 是"每差 1 度给多少 RPM"，它同时也是【弹簧刚度】：
   刚度越大，同样的转动惯量下越容易冲过头。τ = 6/kp 从 2.0s 变到约 2.7s（慢一点点），
   换来的是过冲幅度明显变小。它和下面的 kd 必须【成对】改 —— 只降 kp 会变"软/迟钝"，
   只加 kd 会变"粘"。在线整定：gimbal_world_kp */
#define GIMBAL_WORLD_KP         (3.2f)   /* ★ 实测有效：2.2 幅值衰减(目标-13只到-8.5)；3.2 后峰值差<1° */
/* 陀螺阻尼：用板载 IMU 的 yaw 角速度做阻尼，量纲 RPM/(deg/s)，纯运动学取值 = 1/6 ≈ 0.167。
   ★ "转快了冲出去再回来"= 阻尼不够。这一项就是刹车，按 ζ（阻尼比）配：kd ≈ 2ζ√(6·kp·τ内)/6 - 1/6。
     在线整定口诀：冲过头 -> 加 gimbal_world_kd；发飘/迟钝 -> 加 gimbal_world_kp。 */
/* ★ 加阻尼抑过冲：0.55 -> 0.85。kd 是"按当前转速扣多少 RPM"的刹车项：
   云台转得越快、扣得越多，所以它专门吃"冲过头再被拉回来"的那一下。
   kd 越大 -> 刹车越重 -> 过冲越小、回稳越快；但给太多会"发粘、跟手变慢"（像泡在糖浆里）。
   它和 kp 的关系：kp 决定"多想去目标"，kd 决定"快到目标时多早松油门"。 */
#define GIMBAL_WORLD_KD         (0.95f)  /* ★ 实测定案：0.85->0.95 后过冲受控、不迟钝 */
/* 阻尼源一阶低通系数 0~1：陀螺噪声会直接进转速指令，越小滤得越狠、相位滞后越大。
   ★ 0.40 -> 0.55：alpha 提高 = 微分低通更"快"、滞后更小，阻尼能【更早】开始刹车
     （原来 0.40 的滞后把 kd 的效果推迟了几十 ms，刹车踩晚了，所以过冲压不住）。
   ⚠️ 代价：滤得少 -> 陀螺噪声更多地进转速指令，云台可能听出细微"沙沙"感/轻微发热。
     噪声明显就把它往 0.45 降，然后 kd 补一点。在线调 gimbal_world_da */
#define GIMBAL_WORLD_D_ALPHA    (0.55f)
/* 底盘角速度前馈系数：运动学上精确抵消底盘自转就是 1.0。
   原来 0.3 = 只补 30%，剩 70% 让位置环去追 -> 稳态滞后 + 摆动。抖就 0.8，迟钝就 1.0 */
/* ★ 1.0 -> 0.4 —— 这是"调过去后会晃一下再回来"的三个共犯之一：
   摇杆给的是【阶跃式】目标（一秒内目标角直接跳几十度），满前馈等于在起步瞬间
   给速度环塞一个满幅的转速指令脉冲，云台被推着先冲到目标【前面】去，位置环再把它拉回来
   -> 看起来就是"晃一下"。降到 0.4 只保留抵消底盘自转/匀速跟踪需要的那部分。
   ★ 它仍然有 1.0 的正确用法：跟踪【匀速移动】的目标（视觉自瞄那种连续移动的目标）时，
     满前馈能把"跟踪滞后"清零，那时可以在线调回 0.8~1.0。在线调 gimbal_ff */
#define GIMBAL_FF_GAIN          (1.0f)   /* ★ 实测定案(2026-10-17)：0.4 滞后6度、0.9 偏慢、1.0 轻微提前约2度=可接受。★ 注意在线调参表里 gimbal_ff 上限是1.0，写 3 会被夹成 1.0 */
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

/* ★★★ 目标角一阶平滑（时间常数 ms）—— 治"调过去晃一下再回来"最有效的一条。
   ---------------------------------------------------------------------------
   问题本质：摇杆给的是【阶跃式】目标角，不是连续轨迹。每 1ms 把 target_yaw 加上
   rate×dt，松杆那一刻目标角就【突然】停在某处不再动了 —— 这个"折角"里含了
   全频段的能量（数学上阶跃 = 所有频率的正弦叠加），会把位置环推过冲；
   云台冲过目标后位置环反向拉回来 -> 就是你看到的"晃一下再回来"。

   做法：不直接拿 target_yaw 去算误差，而是让它先过一个一阶低通 target_yaw_smooth
   再用（见 gimbal.c）。效果 = 把那个"折角"磨圆，目标变成一条平滑的轨迹去追，
   位置环从"救火"变成"跟随"，过冲自然就没了。它比降 kp / 加 kd 都温和：
   降 kp 是"整体变软"（所有动作都迟钝），目标平滑只削"突变"，匀速跟随时【零滞后】。

   ★ 单位 ms，物理含义 = 这个一阶滤波器的时间常数 τ（约 63% 到位时间）。
     系数按 α = dt/(τ+dt) 在代码里实时算（dt = 1ms），不是写死的魔数：
       τ = 25ms -> α = 0.001/(0.025+0.001) ≈ 0.0385
     ★ 填 0 = 关闭平滑（退化成直接用 target_yaw，和改之前行为一致），方便 A/B 对比。

   ★ 副作用（都是想要的）：视觉接管/切回摇杆那一瞬间的"阶跃式目标切换"也会被磨圆，
     所以接管瞬间不会甩一下；代价是目标本身快速移动时云台会"慢半个 τ"，
     视觉跟快速移动目标觉得跟不上就把这个值调小（0~10ms）。在线调 gimbal_tgt_smooth。

   ★★ 分模式（用户要求：摇杆要快、自瞄要稳）：拆成两套时间常数，运行时按
      gimbal->aim_takeover 自动二选一（见 gimbal.c 算 alpha_s 那一段）：
        · aim_takeover == 0（摇杆 / 没识别到装甲板 / 视觉未接管）-> RC  = 12ms（小 = 跟手）
        · aim_takeover != 0（识别到装甲板、自瞄接管）            -> AIM = 40ms（大 = 不抖）
      ⚠️ 两套都保留在线可调：gimbal_tgt_smooth 改【摇杆】那套，gimbal_aim_smooth 改【自瞄】那套。
         （结构体字段名 tgt_smooth_ms 现在表示"摇杆值"，保留是为了不破坏已有的串口命令。） */
#define GIMBAL_TGT_SMOOTH_RC_MS   (12.0f)   /* 摇杆/无目标时：小=跟手 */
 #define GIMBAL_TGT_SMOOTH_AIM_MS  (12.0f)   /* ★ 实测(2026-10-19)：40ms 响应太慢，12ms 明显变快且不抖(前馈已关，抖源已消除) */
/* 兼容旧引用：= 摇杆值；运行时不再直接用这个宏（按 aim_takeover 二选一） */
#define GIMBAL_TGT_SMOOTH_MS      GIMBAL_TGT_SMOOTH_RC_MS

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
   向前俯(向下) = 负角度、向后仰(向上) = 正角度。
   ★ 依据（2026-10-17 用户实测，绝对单圈编码器读数）：
       枪管最上（仰到底）绝对 120.718°、枪管最下（俯到底）绝对 78.2606°；
       按"关节角 = 绝对角 − GIMBAL_PITCH_ABS_TO_JOINT_OFFSET（90.5716）"换算，
       真实机械行程 = 关节 −12.311° ~ +30.146°。
     旧值 −15° ~ +30° 的下限比机械行程多出 2.7°，会顶机械限位，故按实测收窄：
       向下留 1.5° 安全余量 -> −10.8°（对应绝对 79.7716°），
       向上留 1.5° 安全余量 -> +28.6°（对应绝对 119.1716°）。
     GIMBAL_PITCH_MIN_DEG / MAX_DEG 由下面两个正值取负/正派生，不用单独改。
   若实测软限位方向还是反的，把下面两个数值对调即可（不用改别的）。 */
#define GIMBAL_PITCH_FORWARD_MAX_DEG   (10.8f)   /* 向前俯的最大角度。★ 实测机械行程: 绝对 78.2606(最下)~120.718(最上) = 关节 -12.311~+30.146, 留1.5度余量故取 -10.8 (原15会顶机械限位2.7度) */
#define GIMBAL_PITCH_BACKWARD_MAX_DEG  (28.6f)   /* 向后仰的最大角度。★ 同理留1.5度余量取 +28.6 (原30只剩0.15度余量) */
#define GIMBAL_PITCH_MIN_DEG    (-GIMBAL_PITCH_FORWARD_MAX_DEG)
#define GIMBAL_PITCH_MAX_DEG    ( GIMBAL_PITCH_BACKWARD_MAX_DEG)

/* ★ 已不再参与 pitch 零点计算（保留宏仅为兼容旧配置/旧代码引用）：
   首次标定现在按【绝对单圈编码器】换算关节零点，见下面的 GIMBAL_PITCH_ABS_TO_JOINT_OFFSET，
   上电姿态不再要求固定。原语义：上电时枪管的真实俯仰角，上电前摆到水平就填 0，否则量出
   实际角度填这里。⚠️ 每次上电必须是同一姿态，否则零点会漂（同 yaw 的上电对准前方）*/
#define GIMBAL_PITCH_ZERO_BIAS_DEG (0.0f)

/* ★ 上电默认回到的 pitch 关节角（度）。实测：绝对编码器 99.7119° 对应关节角 9.1403°，
   即 关节角 = 绝对角 - 90.5716（90.5716 见下面的 GIMBAL_PITCH_ABS_TO_JOINT_OFFSET）。
   要改回零位或别的角度，改这个值即可（范围必须在
   GIMBAL_PITCH_MIN_DEG ~ GIMBAL_PITCH_MAX_DEG 之内，即 -10.8 ~ +28.6；
   目标 9.1403 落在范围内，合法）。
   1 = 启用"上电驱到该角度"；0 = 恢复旧行为（停在哪儿保持哪儿）。
   ⚠️ 安全：上电瞬间若枪管离 9.1403° 较远，位置环会在 GIMBAL_PITCH_ANGLE_MAX_OUT
      （200 RPM）限幅内直接转过去 —— 第一次测试请【手扶枪管】或确认行程内无障碍。
   ★ 首次标定已改为按【绝对单圈编码器】换算零点（见 GIMBAL_PITCH_ABS_TO_JOINT_OFFSET），
      所以这个目标角与上电姿态无关：target=9.1403° 永远等于绝对 99.7119°。 */
#define GIMBAL_PITCH_BOOT_TO_FIXED     (1)
#define GIMBAL_PITCH_BOOT_TARGET_DEG   (9.1403f)

/* ★ pitch 关节角零点换算：关节角 = 绝对单圈编码器角 - 本偏移（单位：度）。
   实测一对值：绝对 99.7119° <-> 关节 9.1403°，所以偏移 = 99.7119 - 9.1403 = 90.5716。
   重新标定：把枪摆到已知绝对角 A、读出当时的关节角 J，本宏 = A - J，并同步改
   GIMBAL_PITCH_BOOT_TARGET_DEG（若要回同一个物理位置）。
   ★ 按当前软限位：GIMBAL_PITCH_MIN_DEG(-10.8) 对应绝对 79.7716°、MAX_DEG(+28.6)
     对应绝对 119.1716°，均落在用户实测机械行程 78.2606° ~ 120.718° 之内。 */
#define GIMBAL_PITCH_ABS_TO_JOINT_OFFSET  (90.5716f)

/* ---- ★ 上电回位（boot homing）状态机参数（只在 GIMBAL_PITCH_BOOT_TO_FIXED=1 时生效）----
   为什么需要状态机、而不是"首次对齐那一拍写一次目标角"：
   原来 target_pitch 只在【首次对齐的那一拍】被赋成 9.1403，之后没有任何地方再声明它。
   而 pitch 的首次对齐/重新对齐由"电机是否在线"驱动：上电初期 CAN 一抖动（>GIMBAL_OFFLINE_MS）
   就会重新对齐一次，重新对齐走的是"掉线重连 → 保持原位"分支，会把 target_pitch 直接改成
   当前角 —— 9.1403 就这样被永久覆盖，表现成"只有烧录后第一次上电会回位"。
   现在改成：上电后第一次对齐就把 pitch_boot_homing 置 1，并【每个控制周期】重新写回目标角，
   直到满足下面的退出条件之一才交回正常控制（达成/超时/人为动作）。
   观测量：user_gimbal.pitch_boot_homing（1=还在回位）、user_gimbal.pitch_boot_homed（1=本次上电已结束回位）。 */
#define GIMBAL_PITCH_BOOT_HOMING_TOL_DEG    (0.5f)   /* 到位判据 deg：|实际角 - 9.1403| 小于它算到位 */
#define GIMBAL_PITCH_BOOT_HOMING_HOLD_MS    (120U)   /* 到位后要连续保持这么久才退出（防抖/防过冲时误判）*/
#define GIMBAL_PITCH_BOOT_HOMING_TIMEOUT_MS (3000U)  /* 超时兜底 ms：堵转/机械卡死时不能永远霸占目标角。
                                                        想让回位更"执着"就加大，想更早交权就减小 */

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
 #define GIMBAL_PITCH_ANGLE_KF       (0.167f) /* ★ 恢复！这是【摇杆目标】的微分前馈(让推杆更跟手)，不是视觉前馈。之前误设 0 导致 pitch 推杆发飘。★ 视觉路径的跳变尖峰要靠 aim_takeover 时旁路它来解决(见 gimbal.c) */
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
    /* ★ 平滑后的目标角（一阶低通，时间常数 = tgt_smooth_ms）—— 位置环只用它，
       不直接用 target_yaw。加它的原因和 τ 的取法见 gimbal.h 里 GIMBAL_TGT_SMOOTH_MS。
       ⚠️ 跟踪它必须用 Wrap180 求差，否则跨 ±180° 会绕一整圈（见 gimbal.c）*/
    float target_yaw_smooth;
    float tgt_smooth_ms;           /* 目标角平滑 τ ms【摇杆/无目标时用】，在线改 gimbal_tgt_smooth（0=关闭）*/
    float tgt_smooth_aim_ms;       /* 目标角平滑 τ ms【自瞄接管时用】，在线改 gimbal_aim_smooth（大=不抖）*/
    float angle_raw;               /* 电机多圈角度 deg（观察用）*/
    float speed;                   /* 电机转速 RPM（观察用）*/
    float rc_ch;                   /* 遥控通道 0~3，可在线改 */
    float max_w;                   /* 满杆角速度 rad/s，可在线改 */
    float world_kp;                /* 锁世界位置环增益 RPM/deg，可在线改 */
    float world_kd;                /* 陀螺阻尼系数 RPM/(deg/s)，可在线改 */
    float world_d_alpha;           /* 阻尼源一阶低通系数 0~1，可在线改（大=滞后小、噪声大）*/
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
    /* ---- 自瞄（视觉）接管：yaw ---- */
    float    yaw_auto;        /* ≠0 = 使能视觉接管（操作手开关）。在线调 yaw_auto */
    float    yaw_auto_pre;    /* 上一拍使能标志（观察/兼容用）*/
    float    yaw_target_ext;  /* 视觉给的 yaw 目标角 deg（世界绝对角，覆盖式）*/
    float    cam_yaw_off;     /* 相机光轴 <-> 枪管零位偏置 deg（打靶标定）*/
    float    aim_ff_gain;     /* yaw 视觉角速度前馈增益（参考 0.5；前馈不在回路里，不影响稳定性）*/
    float    aim_delay_ms;    /* 链路延迟补偿 ms */
    float    aim_timeout_ms;  /* 丢帧/离线超时 ms，超时就判"视觉真的没了" */
    float    aim_err;         /* 观察用：视觉目标 - 实际 */
    float    aim_ok_f;        /* 观察用镜像：1=本帧视觉数据有效 */
    uint8_t  aim_valid;       /* 1=视觉目标可用（含短丢帧的保持期）*/
    uint32_t aim_ms;          /* 最近一次【有效帧】的时刻（离线超时判定用）*/

    /* ---- 自瞄（视觉）接管：pitch（与 yaw 完全对称，见 gimbal.c 的 Gimbal_Aim_Update）---- */
    float    pitch_target_ext;  /* 视觉给的 pitch 目标角 deg（绝对角，覆盖式）*/
    float    cam_pitch_off;     /* 相机光轴 <-> 枪管俯仰零位偏置 deg */
    float    aim_ff_gain_pitch; /* pitch 视觉角速度前馈增益（参考 0.6）*/
    float    pitch_aim_err;     /* 观察用：pitch 视觉目标 - 实际 */
    uint8_t  pitch_aim_valid;   /* 1=pitch 视觉目标可用（与 aim_valid 同源）*/
    float    aim_rate;          /* yaw 视觉角速度前馈 deg/s（= 系数 × 视觉角速度）*/
    float    pitch_aim_rate;    /* pitch 视觉角速度前馈 deg/s（= 0.6 × 视觉角速度）*/
    float    aim_takeover;      /* 观察用：1=本拍由视觉接管（yaw/pitch 共用同一帧判定）*/
    uint16_t aim_bad_frames;    /* 连续被闸门拒绝的视觉帧数（到限就退出接管）*/
    uint32_t aim_reject_cnt;    /* 累计被闸门拒绝的帧数（排错用，只增不减）*/
    uint32_t last_vision_frame; /* 上一拍见到的视觉有效帧计数（用于"是不是新的一帧"判定）*/
    uint32_t last_dbus_count;      /* 上一周期见到的遥控帧计数 */
    uint16_t dbus_offline_ms;      /* 遥控断线累计 ms */
    float imu_yaw;                 /* IMU 实测的云台世界朝向 deg（观察用）*/
    /* ★ 串口绘图 ch7：位置环【实际使用】的 yaw 反馈，= imu_yaw − bias_accum，单位 deg。
       只读镜像（在 gimbal.c 里紧跟 world_yaw 赋值），不参与任何控制运算。
       它和 target_yaw / target_yaw_smooth 同属【世界系】，两者相减才是真实跟随误差；
       ch2 的 yaw 是编码器【相对角】（另一个坐标系），不能直接拿来算误差。 */
    float yaw_fb;
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
    /* ---- 上电回位（boot homing）状态，见 gimbal.h 里 GIMBAL_PITCH_BOOT_HOMING_* ---- */
    uint8_t  pitch_boot_homing;            /* 1=正在执行上电回位（此时 target_pitch 每拍被强制为 9.1403）*/
    uint8_t  pitch_boot_homed;             /* 1=本次上电的回位已结束（到达/摇杆接管/视觉接管/超时），掉线重连不再回位 */
    uint16_t pitch_homing_ms;              /* 回位已进行的时间 ms（超时判据，1kHz 下每拍 +1）*/
    uint16_t pitch_homing_in_pos_ms;       /* 连续到位的时间 ms（到位判据的防抖计数）*/

    /* ---- ★★ 串口绘图 ch7/ch8 的弧度镜像（只读观测量，加在结构体最后、不影响已有布局）----
       ★ 云台控制内部（target_yaw / 编码器角 / PID / 限位 / 回位 / cam_yaw_off / cam_pitch_off）
         仍然全部按【度】标定；这里只是给绘图/上位机看的一份弧度副本，绝不参与任何控制计算。
       每拍在 gimbal.c 里紧跟 yaw_fb / yaw 的赋值更新（1kHz）。 */
    float tx_qw;        /* 发给视觉的四元数（已补 -bias_accum 旋转，与 yaw 字段同基准）*/
    float tx_qx;
    float tx_qy;
    float tx_qz;
    float yaw_fb_rad;    /* ch7 的弧度镜像 = yaw_fb × GIMBAL_DEG2RAD（仅供绘图观测）*/
    float yaw_rad;       /* ch8 的弧度镜像 = yaw    × GIMBAL_DEG2RAD（仅供绘图观测）*/
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
   "外部给目标"模式；该模式下遥控拨轮不再改变目标（避免和外部目标互相打架）。
   ⚠️ 优先级：视觉自瞄接管中（gimbal->aim_takeover）时，视觉目标角【覆盖】本函数写入的值 ——
      与参考工程一致（视觉的目标角写在最后）。 */
void  Gimbal_Set_Pitch_Target(GIMBAL_DRIVES* gimbal, float deg);

/* 切换 pitch 目标来源：1=外部（弹道/自瞄），0=纯遥控积分；切回 0 时目标角重置到当前实际角 */
void  Gimbal_Set_Pitch_Auto(GIMBAL_DRIVES* gimbal, uint8_t enable);

/* pitch 反馈是否正常，掉线返回 0 */
uint8_t Gimbal_Pitch_Is_Online(const GIMBAL_DRIVES* gimbal);

/* pitch 是否已完成角度标定（标定前不下发任何指令，避免上电猛打）*/
uint8_t Gimbal_Pitch_Is_Aligned(const GIMBAL_DRIVES* gimbal);

#endif // GIMBAL_H
