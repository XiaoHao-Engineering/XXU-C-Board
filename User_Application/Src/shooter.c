#include "../../Core/Inc/bsp.h"

#include <math.h>
#include "../shooter.h"

/* 私有变量 */

/* 8 抽头滑动平均 FIR 系数（低通）；⚠️ FIR_Init 会把系数 memcpy 进自己 malloc 的数组，用 static const 即可 */
static const float shooter_fir_coeffs[SHOOTER_FIR_TAPS] = {
    1.0f / (float)SHOOTER_FIR_TAPS, 1.0f / (float)SHOOTER_FIR_TAPS,
    1.0f / (float)SHOOTER_FIR_TAPS, 1.0f / (float)SHOOTER_FIR_TAPS,
    1.0f / (float)SHOOTER_FIR_TAPS, 1.0f / (float)SHOOTER_FIR_TAPS,
    1.0f / (float)SHOOTER_FIR_TAPS, 1.0f / (float)SHOOTER_FIR_TAPS,
};

static SHOOTER_DRIVES* s_shooter = NULL;

/* ★ 遥控(DBUS)掉线保护的状态。本模块是单实例（s_shooter），且 shooter.h 不在本次改动范围内，
   所以放在文件作用域：判据复用 user_dbus.rx_count（见 user_dji_bus.c:24 / gimbal.c:333），
   本任务 1kHz 比较计数是否还在增长，不新增定时器、不占用串口绘图通道。 */
static uint32_t s_shooter_last_dbus_count = 0;
static uint16_t s_shooter_dbus_offline_ms = 0;

static void  Shooter_Update(void* arg);
static void  Shooter_Ballistics_Update(void* arg);
static void  Shooter_Enter_State(SHOOTER_DRIVES* shooter, Shooter_State state);
static void  Shooter_Stopper_Set(SHOOTER_DRIVES* shooter, Shooter_Stopper_State state);
static void  Shooter_Feed_One_Step(SHOOTER_DRIVES* shooter, float scale);
static float Shooter_Step_Rotor_Deg(const SHOOTER_DRIVES* shooter);
static void  Shooter_Single_Update(SHOOTER_DRIVES* shooter, uint8_t feed_ok, uint8_t fire_request);
static void  Shooter_Continuous_Update(SHOOTER_DRIVES* shooter, uint8_t feed_ok);

/* 函数体 */

static float Shooter_Step_Rotor_Deg(const SHOOTER_DRIVES* shooter) {
    /* 方向作用在【目标角】上，不是电流输出 —— 位置环这样反向才安全（见 shooter_config.h）
       一发拨盘角（默认 56.43°）× 减速比 36 = 2031.43° 转子角 */
    return shooter->feeder_dir * shooter->step_dial_deg * SHOOTER_FEEDER_GEAR;
}

static void Shooter_Feed_One_Step(SHOOTER_DRIVES* shooter, const float scale) {
    shooter->feeder_target_deg += Shooter_Step_Rotor_Deg(shooter) * scale;
}

/* 停火 FSM 状态切换：进入时把两个计时清零。
   同一个状态重复调用是空操作（否则每拍都会把计时清零，永远到不了 1s）。 */
static void Shooter_Stopper_Set(SHOOTER_DRIVES* shooter, const Shooter_Stopper_State state) {
    if (shooter->stopper_state == state) {
        return;
    }
    shooter->stopper_state = state;
    shooter->stopper_ms_acc = 0;
    shooter->stopper_cur_ms = 0;
}

static void Shooter_Enter_State(SHOOTER_DRIVES* shooter, const Shooter_State state) {
    shooter->state = state;
    shooter->state_ms = 0;

    if (state == SHOOTER_STATE_FIRE) {
        /* ★ 进 FIRE 时目标角对齐当前位置：否则机构被手动拨过或目标角停在 0，一进 FIRE 会猛追一段直接卡弹 */
        shooter->feeder_target_deg = shooter->feeder_angle;
        /* 让拨弹的第一步可以立刻走出来 */
        shooter->fire_timer_ms = (uint32_t)shooter->fire_interval_ms;
    } else if (state == SHOOTER_STATE_SPINUP) {
        shooter->is_ready = 0;
    } else if (state == SHOOTER_STATE_REVERSE) {
        Shooter_Feed_One_Step(shooter, SHOOTER_REVERSE_SCALE);
        shooter->jam_timer_ms = 0;
    }
}

void Shooter_Init(SHOOTER_DRIVES* shooter,
                  DJI_MOTOR_DRIVES* friction_left,
                  DJI_MOTOR_DRIVES* friction_right,
                  DJI_MOTOR_DRIVES* feeder) {
#if !SHOOTER_ENABLE
    /* 整个模块关掉：什么都不做、也不建任务（排错时二分定位用）*/
    (void)shooter; (void)friction_left; (void)friction_right; (void)feeder;
    return;
#else
    shooter->friction_left = friction_left;
    shooter->friction_right = friction_right;
    shooter->feeder = feeder;

    PID_Init(&shooter->friction_left_pid, SHOOTER_FRICTION_KP, SHOOTER_FRICTION_KI,
             SHOOTER_FRICTION_KD, SHOOTER_FRICTION_MAX_OUT, SHOOTER_FRICTION_MAX_IOUT);
    PID_Init(&shooter->friction_right_pid, SHOOTER_FRICTION_KP, SHOOTER_FRICTION_KI,
             SHOOTER_FRICTION_KD, SHOOTER_FRICTION_MAX_OUT, SHOOTER_FRICTION_MAX_IOUT);

    PID_Init(&shooter->feeder_pid_angle, SHOOTER_FEEDER_ANGLE_KP, SHOOTER_FEEDER_ANGLE_KI,
             SHOOTER_FEEDER_ANGLE_KD, SHOOTER_FEEDER_ANGLE_MAX_OUT, SHOOTER_FEEDER_ANGLE_MAX_IOUT);
    PID_Init(&shooter->feeder_pid_speed, SHOOTER_FEEDER_SPEED_KP, SHOOTER_FEEDER_SPEED_KI,
             SHOOTER_FEEDER_SPEED_KD, SHOOTER_FEEDER_SPEED_MAX_OUT, SHOOTER_FEEDER_SPEED_MAX_IOUT);
    Cascade_Controller_Init(&shooter->feeder_cascade,
                            (CONTROLLER_INTERFACE*)&shooter->feeder_pid_angle,
                            (CONTROLLER_INTERFACE*)&shooter->feeder_pid_speed,
                            SHOOTER_FEEDER_SPEED_MAX_OUT);

    FIR_Init(&shooter->fir_current, shooter_fir_coeffs, SHOOTER_FIR_ORDER);
    FIR_Init(&shooter->fir_friction_rpm, shooter_fir_coeffs, SHOOTER_FIR_ORDER);
    shooter->feeder_current_filt = 0.0f;
    shooter->friction_rpm_filt = 0.0f;

    Ballistics_Init(&shooter->ballistics);

    shooter->state = SHOOTER_STATE_STOP;
    shooter->state_ms = 0;
    shooter->fire_timer_ms = 0;
    shooter->jam_timer_ms = 0;
    shooter->jam_count = 0;
    shooter->is_online = 0;
    shooter->is_ready = 0;
    shooter->friction_target = 0.0f;
    shooter->friction_rpm = 0.0f;
    shooter->feeder_angle = 0.0f;
    shooter->feeder_target_deg = 0.0f;
    shooter->feeder_current = 0.0f;
    shooter->feeder_err = 0.0f;
    shooter->state_f = 0.0f;
    shooter->online_mask = 0.0f;
    shooter->ready_f = 0.0f;

    /* 停火 FSM：上电默认失能；wheel_armed 先给 0，等拨轮真的回到中位才武装
       （防止上电时拨轮已经推着 -> 起转完立刻自己打一发）*/
    shooter->stopper_state = SHOOTER_STOPPER_DISABLED;
    shooter->fire_mode = SHOOTER_MODE_FRICTION_ONLY;
    shooter->stopper_ms_acc = 0;
    shooter->stopper_cur_ms = 0;
    shooter->wheel_armed = 0;
    shooter->friction_current = 0.0f;
    shooter->stop_f = 0.0f;
    shooter->mode_f = 0.0f;
    shooter->fric_cur_f = 0.0f;

    shooter->fire_rpm = SHOOTER_FRICTION_RPM_FIRE;
    shooter->step_dial_deg = SHOOTER_FEEDER_STEP_DIAL_DEG;
    shooter->feeder_dir = SHOOTER_FEEDER_DIR;
    shooter->fire_interval_ms = (float)SHOOTER_FIRE_INTERVAL_MS;
    shooter->distance_m = 3.0f;
    shooter->height_m = 0.0f;
    shooter->auto_aim = 0.0f;

    shooter->fire_dead = SHOOTER_FIRE_DEAD;
    shooter->stopper_cur = SHOOTER_STOPPER_CUR_MA;
    shooter->stopper_ms = SHOOTER_STOPPER_MS;

    s_shooter = shooter;

    /* 遥控掉线检测状态复位：重新初始化后先给满一个 SHOOTER_DBUS_OFFLINE_MS 的宽限期 */
    s_shooter_last_dbus_count = 0;
    s_shooter_dbus_offline_ms = 0;

    /* 1kHz 控制任务：只做采样/滤波/状态机/下发，绝不做弹道解算 */
    SysTick_InitTask(&shooter->task, (void*)shooter, 100, SHOOTER_PERIOD_MS,
                     Task_REPEAT, Shooter_Update);
    SysTick_StartTask(&shooter->task);

    /* ★ 弹道解算单独挂一个低速率任务：牛顿迭代里的 tanf/cosf 是软件浮点，很慢，
       而任务都在 1ms 的 SysTick 中断里跑；单开一个任务也避免叠在 Shooter_Update 的 1KB 栈上 */
    SysTick_InitTask(&shooter->ballistics_task, (void*)shooter, 150,
                     SHOOTER_BALLISTICS_PERIOD_MS, Task_REPEAT, Shooter_Ballistics_Update);
    SysTick_StartTask(&shooter->ballistics_task);
#endif /* SHOOTER_ENABLE */
}

/* 低速任务（默认 20ms / 50Hz）：弹丸初速估计 + 弹道解算 + 自动瞄准 */
static void Shooter_Ballistics_Update(void* arg) {
    SHOOTER_DRIVES* shooter = (SHOOTER_DRIVES*)arg;

    /* 滤波后的摩擦轮转速 RPM -> 弹丸初速 m/s */
    Ballistics_Set_V0_From_Rpm(&shooter->ballistics, shooter->friction_rpm_filt);

    /* 牛顿迭代解俯仰角；打不到时靠可解性判别式立刻返回（跑不满迭代），这是"不允许在中断里久算"的前提 */
    (void)Ballistics_Solve(&shooter->ballistics, shooter->distance_m, shooter->height_m);

    /* 自动瞄准：把解算结果写给云台 pitch（Gimbal_Set_Pitch_Target 内部做软限位）*/
    if (shooter->auto_aim >= 0.5f) {
        Gimbal_Set_Pitch_Target(&user_gimbal, Ballistics_Get_Pitch(&shooter->ballistics));
    }
}

uint8_t Shooter_Is_Online(const SHOOTER_DRIVES* shooter) {
    return (shooter != NULL) ? shooter->is_online : 0;
}

uint8_t Shooter_Is_Ready(const SHOOTER_DRIVES* shooter) {
    return (shooter != NULL) ? shooter->is_ready : 0;
}

float Shooter_Get_Friction_Rpm(const SHOOTER_DRIVES* shooter) {
    return (shooter != NULL) ? shooter->friction_rpm_filt : 0.0f;
}

float Shooter_Get_Ballistics_Pitch(const SHOOTER_DRIVES* shooter) {
    return (shooter != NULL) ? Ballistics_Get_Pitch(&shooter->ballistics) : 0.0f;
}

void Shooter_Set_Target_Distance(SHOOTER_DRIVES* shooter, const float distance_m, const float height_m) {
    if (shooter == NULL) {
        return;
    }
    shooter->distance_m = distance_m;
    shooter->height_m = height_m;
}

/* ==================== 单发：位控 + 停火 FSM ====================
 * sw2 上档。一发 = 当前反馈角 + 一发的转子角（step_dial_deg × GEAR）。
 * 拨轮（DBUS roll）越过死区触发；必须回中位（死区×回滞比）才重新武装。
 * 触发在 Shooter_Update 里统一"消费"成 fire_request（每拍最多一次），所以哪怕
 * 冷却期间又拨了一下，也不会排到冷却结束后补发 —— 一次手势最多一发。
 *
 *  失能 --触发--> 激活 --电流>stopper_cur 或 1s--> 处理 --20ms--> 失能
 *   ^                                                                    |
 *   +--------------------------------------------------------------------+
 *
 * ⚠️ 这个函数每 1ms 被调用一次，内部只做整数计时 + 几个浮点比较，没有阻塞、没有三角函数。
 */
static void Shooter_Single_Update(SHOOTER_DRIVES* shooter, const uint8_t feed_ok,
                                  const uint8_t fire_request) {
    switch (shooter->stopper_state) {
        case SHOOTER_STOPPER_DISABLED:
            /* 失能：拨弹轮不出力 —— 目标角跟随实际角，位置误差恒 0 -> 位置环不输出 */
            shooter->feeder_target_deg = shooter->feeder_angle;
            if (fire_request) {
                /* 触发已经消费掉：没起速/没就绪时拨动不会"排队"到之后就绪时补发 */
                if (feed_ok) {
                    /* 单发是位控：目标 = 当前【反馈】角 + 一发 */
                    shooter->feeder_target_deg = shooter->feeder_angle
                                               + Shooter_Step_Rotor_Deg(shooter);
                    Shooter_Stopper_Set(shooter, SHOOTER_STOPPER_ACTIVE);
                }
            }
            break;

        case SHOOTER_STOPPER_ACTIVE:
            /* 激活：保持"一发"的目标角不动（继续往目标走），不在这里改写 target */
            shooter->stopper_ms_acc += SHOOTER_PERIOD_MS;

            if (fabsf(shooter->friction_current) > shooter->stopper_cur) {
                shooter->stopper_cur_ms += SHOOTER_PERIOD_MS;
            } else {
                shooter->stopper_cur_ms = 0;
            }

            /* 电流超阈值（持续 CUR_MS）或 激活超时（PPT：1s）-> 处理 */
            if (shooter->stopper_cur_ms >= SHOOTER_STOPPER_CUR_MS
                || shooter->stopper_ms_acc >= (uint32_t)shooter->stopper_ms) {
                Shooter_Stopper_Set(shooter, SHOOTER_STOPPER_PROCESS);
            }
            break;

        case SHOOTER_STOPPER_PROCESS:
            /* 处理：角度闭环【锁住当前角】-> 发射机构停火；保持 PROCESS_MS 后回失能 */
            shooter->feeder_target_deg = shooter->feeder_angle;
            shooter->stopper_ms_acc += SHOOTER_PERIOD_MS;
            if (shooter->stopper_ms_acc >= SHOOTER_STOPPER_PROCESS_MS) {
                Shooter_Stopper_Set(shooter, SHOOTER_STOPPER_DISABLED);
            }
            break;

        default:
            Shooter_Stopper_Set(shooter, SHOOTER_STOPPER_DISABLED);
            break;
    }
}

/* ==================== 连发：速控（沿用原实现）====================
 * sw2 下档。每 fire_interval_ms 走一步；拨盘内环本身就是速度环
 * （外环角度->转速，内环转速->电流），节奏由 fire_int 决定、上限由 feed_ang_mo 决定。
 * 连发不经过停火 FSM（那是单发防双发用的）。
 */
static void Shooter_Continuous_Update(SHOOTER_DRIVES* shooter, const uint8_t feed_ok) {
    if (feed_ok) {
        if (shooter->fire_timer_ms >= (uint32_t)shooter->fire_interval_ms) {
            shooter->fire_timer_ms = 0;
            Shooter_Feed_One_Step(shooter, 1.0f);
        } else {
            shooter->fire_timer_ms += SHOOTER_PERIOD_MS;
        }
    } else {
        shooter->fire_timer_ms = 0;
        /* 不拨弹时让目标角跟随实际角，避免积累误差、松档时突然追一大段 */
        shooter->feeder_target_deg = shooter->feeder_angle;
    }
    Shooter_Stopper_Set(shooter, SHOOTER_STOPPER_DISABLED);
}

/* 1kHz 射击任务：采样滤波 / 总闸 / 模式 / 状态机 / 下发 */
static void Shooter_Update(void* arg) {
    SHOOTER_DRIVES* shooter = (SHOOTER_DRIVES*)arg;

    /* ---------------- 1kHz 采样 ---------------- */
    const float rpm_left = DJI_Motor_Get_Speed(shooter->friction_left);
    const float rpm_right = DJI_Motor_Get_Speed(shooter->friction_right);
    const float cur_left = DJI_Motor_Get_Current(shooter->friction_left);
    const float cur_right = DJI_Motor_Get_Current(shooter->friction_right);

    /* 两个摩擦轮对装、转向相反，所以取绝对值再平均 */
    shooter->friction_rpm = (fabsf(rpm_left) + fabsf(rpm_right)) * 0.5f;
    shooter->friction_rpm_filt = FIR_Update(&shooter->fir_friction_rpm, shooter->friction_rpm);

    /* 摩擦轮电流（mA）：停火 FSM 判据。取两轮【绝对值较大者】——任一轮过流就认 */
    shooter->friction_current = fmaxf(fabsf(cur_left), fabsf(cur_right));
    shooter->fric_cur_f = shooter->friction_current;

    shooter->feeder_angle = DJI_Motor_Get_Angle(shooter->feeder);
    shooter->feeder_current = DJI_Motor_Get_Current(shooter->feeder);
    /* 电流取绝对值再滤波：卡弹判据只关心大小，不关心方向 */
    shooter->feeder_current_filt = FIR_Update(&shooter->fir_current, fabsf(shooter->feeder_current));
    shooter->feeder_err = shooter->feeder_target_deg - shooter->feeder_angle;

    shooter->is_online = (DJI_Motor_Is_Online(shooter->friction_left)
                       && DJI_Motor_Is_Online(shooter->friction_right)
                       && DJI_Motor_Is_Online(shooter->feeder)) ? 1 : 0;

    /* ★ 弹道解算已挪到 Shooter_Ballistics_Update()（低速任务），绝对不要放回这个 1kHz 函数：
       tanf/cosf 在软件浮点下很慢，而本函数跑在 1ms 的 SysTick 中断里，会把整套控制饿死 */

    /* ---------------- 输入解析 ---------------- */
    /* ★★ 遥控(DBUS)在线检测（安全关键，判据复用工程既有机制）：
       user_dji_bus.c 的 rc_callback_handler() 每收到一帧 18 字节有效包就 user_dbus.rx_count++，
       本任务 1kHz，计数不涨就累加离线时长 —— 与 gimbal.c 判遥控/电机在线完全同一套做法。
       ★ 不用 is_update：它会被底盘任务清零，多模块抢会丢数据（见 user_dji_bus.h:16）。
       时间基数 = SHOOTER_PERIOD_MS（本任务周期，默认 1ms），宏是 ms。 */
    if (user_dbus.rx_count != s_shooter_last_dbus_count) {
        s_shooter_last_dbus_count = user_dbus.rx_count;
        s_shooter_dbus_offline_ms = 0;
    } else if (s_shooter_dbus_offline_ms < SHOOTER_DBUS_OFFLINE_MS) {
        s_shooter_dbus_offline_ms =
            (uint16_t)(s_shooter_dbus_offline_ms + SHOOTER_PERIOD_MS);
    }
    const uint8_t dbus_online =
        (s_shooter_dbus_offline_ms < SHOOTER_DBUS_OFFLINE_MS) ? 1 : 0;

    /* ★★ 失联时【不解析】遥控字段（一律按 0）：
       否则 sw1/sw2/roll 会一直保持最后一帧的值，sw1 停在"上"就会让摩擦轮一直转。
       0 档对 sw1/sw2 都是"非上档"，对 roll 是"中位" -> 后面所有判定自然全部走失能分支。 */
    const uint8_t sw2 = dbus_online ? user_dbus.sw2 : (uint8_t)0;
    const float wheel = dbus_online ? (float)user_dbus.roll : 0.0f;

    /* ★★ 总闸（三道，缺一不可）：
         ① dbus_online    ：遥控链路在线（失联 -> 强制失能，见 shooter_config.h 的
                            SHOOTER_DBUS_OFFLINE_MS；这是本次新增的安全闸）
         ② yaw_auto >= 0.5：自瞄已开启（在线 yaw_auto=1#）
         ③ sw1 == 上      ：左上角拨杆拨到"上" = 允许摩擦轮起转
       任一不满足 -> 摩擦轮 + 拨弹盘全部失能，不管 sw2 在哪。
       ⚠️ yaw_auto=0 时 sw1 归底盘做模式选择（上=跟随），此时这道闸必然关死，
          所以"跟随模式"和"摩擦轮"共用 sw1 的上档，但永不同时生效。 */
    const uint8_t sw1 = dbus_online ? user_dbus.sw1 : (uint8_t)0;
#if SHOOTER_SW1_GATE_ENABLE
    const uint8_t sw1_friction_on = (sw1 == SHOOTER_SW1_FRICTION_ON) ? 1 : 0;
#else
    const uint8_t sw1_friction_on = 1;   /* 排错：绕开 sw1 这道闸，回到只看 yaw_auto 的旧行为 */
    (void)sw1;
#endif
    const uint8_t arm = ((SHOOTER_DBUS_GATE_ENABLE ? dbus_online : (uint8_t)1)
                         && (user_gimbal.yaw_auto >= 0.5f)
                         && sw1_friction_on) ? 1 : 0;

    /* sw2 -> 模式：上(1)=单发 下(2)=连发 中(3)=仅摩擦轮。
       0/其它（遥控没数据/野值）按"仅摩擦轮"处理，绝不乱拨弹 */
    Shooter_Fire_Mode mode = SHOOTER_MODE_FRICTION_ONLY;
    if (sw2 == SHOOTER_SW2_SINGLE) {
        mode = SHOOTER_MODE_SINGLE;
    } else if (sw2 == SHOOTER_SW2_CONTINUOUS) {
        mode = SHOOTER_MODE_CONTINUOUS;
    }

    /* 拨轮回中位 -> 重新武装（50% 回滞，防止拨轮停在死区附近抖成连发）。
       ★ 失联期间【不武装】并清掉武装位：否则断线时 wheel 被当 0（=中位）会把它武装起来，
         而操作手可能正推着拨轮 —— 链路一恢复就会立刻打出去，不是操作手这一拍的本意。
         恢复后必须先看到"正常的回中位帧"，再拨动才发弹。 */
    if (!dbus_online) {
        shooter->wheel_armed = 0;
    } else if (fabsf(wheel) <= shooter->fire_dead * SHOOTER_FIRE_RELEASE_RATIO) {
        shooter->wheel_armed = 1;
    }

    /* 拨轮越过死区 = 一次发射请求（每拍最多一次）。
       ★ 不管停火 FSM 当前在哪个状态都【消费掉】这次手势：这样"冷却期间又拨一下"
         不会被排到冷却结束后补发 —— 一次拨动最多一发。*/
    const uint8_t fire_request = (shooter->wheel_armed && fabsf(wheel) >= shooter->fire_dead) ? 1 : 0;
    if (fire_request) {
        shooter->wheel_armed = 0;
    }

    /* 摩擦轮是否已到转速：用【滤波后】的转速判断，避免噪声误判 */
    shooter->is_ready = (fabsf(shooter->friction_rpm_filt - shooter->fire_rpm)
                         < SHOOTER_FRICTION_READY_TOL) ? 1 : 0;

    shooter->state_ms += SHOOTER_PERIOD_MS;
    if (shooter->state_ms > 60000U) {
        shooter->state_ms = 60000U;
    }

    shooter->fire_mode = mode;

    /* ---------------- 总闸 + 状态机 ---------------- */
    if (!arm) {
        /* 总闸关（含 ★遥控失联：dbus_online=0 时 arm 恒为 0）：
           摩擦轮目标 0、拨弹盘目标跟随实际角（不出力）、状态机回 STOP、停火 FSM 回 DISABLED */
        if (shooter->state != SHOOTER_STATE_STOP) {
            Shooter_Enter_State(shooter, SHOOTER_STATE_STOP);
        }
        shooter->friction_target = 0.0f;
        shooter->feeder_target_deg = shooter->feeder_angle;
        shooter->fire_timer_ms = 0;
        shooter->jam_timer_ms = 0;
        shooter->fire_mode = SHOOTER_MODE_FRICTION_ONLY;
        Shooter_Stopper_Set(shooter, SHOOTER_STOPPER_DISABLED);
    } else {
        switch (shooter->state) {
            case SHOOTER_STATE_STOP:
                /* 总闸一开就进起转；摩擦轮目标在 SPINUP/FIRE 里恒为 fire_rpm */
                shooter->friction_target = shooter->fire_rpm;
                Shooter_Enter_State(shooter, SHOOTER_STATE_SPINUP);
                break;

            case SHOOTER_STATE_SPINUP:
                shooter->friction_target = shooter->fire_rpm;
                shooter->feeder_target_deg = shooter->feeder_angle;
                Shooter_Stopper_Set(shooter, SHOOTER_STOPPER_DISABLED);
                /* 转速到位或超时都进 FIRE，但真正拨弹仍要 is_ready，不会没起速就乱拨 */
                if (shooter->is_ready || shooter->state_ms > SHOOTER_SPINUP_TIMEOUT_MS) {
                    Shooter_Enter_State(shooter, SHOOTER_STATE_FIRE);
                }
                break;

            case SHOOTER_STATE_FIRE: {
                /* ★ 总闸开着 -> 摩擦轮【始终】转，目标就是保守弹速对应的 fire_rpm */
                shooter->friction_target = shooter->fire_rpm;

                /* 卡弹判定只在【连发】做：单发由停火 FSM 兜底，而且 7 齿一步 2031.5°
                   在 feed_ang_mo=2500RPM 下要约 0.135s，已经贴近 JAM_MS=150ms，容易误判 */
                if (mode == SHOOTER_MODE_CONTINUOUS && shooter->is_ready
                    && fabsf(shooter->feeder_err) > SHOOTER_JAM_ERR_DEG
                    && shooter->feeder_current_filt > SHOOTER_JAM_CURRENT) {
                    shooter->jam_timer_ms += SHOOTER_PERIOD_MS;
                    if (shooter->jam_timer_ms >= SHOOTER_JAM_MS) {
                        shooter->jam_count++;
                        Shooter_Enter_State(shooter, SHOOTER_STATE_REVERSE);
                        break;
                    }
                } else {
                    shooter->jam_timer_ms = 0;
                }

                if (mode == SHOOTER_MODE_SINGLE) {
                    Shooter_Single_Update(shooter, shooter->is_ready, fire_request);
                } else if (mode == SHOOTER_MODE_CONTINUOUS) {
                    Shooter_Continuous_Update(shooter, shooter->is_ready);
                } else {
                    /* 中档：只转摩擦轮，拨盘不出力 */
                    shooter->feeder_target_deg = shooter->feeder_angle;
                    shooter->fire_timer_ms = 0;
                    Shooter_Stopper_Set(shooter, SHOOTER_STOPPER_DISABLED);
                }
                break;
            }

            case SHOOTER_STATE_REVERSE:
                /* 退弹期间保持摩擦轮转速，退完回 FIRE */
                shooter->friction_target = shooter->fire_rpm;
                Shooter_Stopper_Set(shooter, SHOOTER_STOPPER_DISABLED);
                if (shooter->state_ms >= SHOOTER_REVERSE_MS) {
                    Shooter_Enter_State(shooter, SHOOTER_STATE_FIRE);
                }
                break;

            default:
                Shooter_Enter_State(shooter, SHOOTER_STATE_STOP);
                break;
        }
    }

    /* 状态机镜像给串口绘图看（0=STOP 1=SPINUP 2=FIRE 3=REVERSE）*/
    shooter->state_f = (float)shooter->state;
    /* 停火 FSM / 发射模式镜像（0/1/2），留给以后加绘图通道或用调试器看 */
    shooter->stop_f = (float)shooter->stopper_state;
    shooter->mode_f = (float)shooter->fire_mode;

    /* 三个电机的反馈状态打包成一个数给上位机看：bit0=左轮 bit1=右轮 bit2=拨弹轮 */
    shooter->online_mask = (float)((DJI_Motor_Is_Online(shooter->friction_left) ? 1 : 0)
                                 | (DJI_Motor_Is_Online(shooter->friction_right) ? 2 : 0)
                                 | (DJI_Motor_Is_Online(shooter->feeder) ? 4 : 0));
    shooter->ready_f = (float)shooter->is_ready;

    /* ★ 必须判"这个电机曾经上过线"再下发：Set_Motor_State 是电机结构体里的【函数指针】，
       只有 DJI_Motor_Init 才会赋值；本模块任务在它之前就注册（顺序见 main.c），早跑会空指针调用 -> HardFault。
       用 has_ever_online 而非 is_online：它只在收到过 CAN 反馈后才为 1；掉线时内部会强制清零输出，跳过不影响安全。 */
    if (DJI_Motor_Has_Ever_Online(shooter->friction_left)) {
        shooter->friction_left->Set_Motor_State(shooter->friction_left,
                                                shooter->friction_target * SHOOTER_FRICTION_LEFT_DIR);
    }
    if (DJI_Motor_Has_Ever_Online(shooter->friction_right)) {
        shooter->friction_right->Set_Motor_State(shooter->friction_right,
                                                 shooter->friction_target * SHOOTER_FRICTION_RIGHT_DIR);
    }
    if (DJI_Motor_Has_Ever_Online(shooter->feeder)) {
        shooter->feeder->Set_Motor_State(shooter->feeder, shooter->feeder_target_deg);
    }
}
