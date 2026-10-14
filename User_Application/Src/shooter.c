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

static void  Shooter_Update(void* arg);
static void  Shooter_Ballistics_Update(void* arg);
static void  Shooter_Enter_State(SHOOTER_DRIVES* shooter, Shooter_State state);
static void  Shooter_Feed_One_Step(SHOOTER_DRIVES* shooter, float scale);
static float Shooter_Step_Rotor_Deg(const SHOOTER_DRIVES* shooter);

/* 函数体 */

static float Shooter_Step_Rotor_Deg(const SHOOTER_DRIVES* shooter) {
    return SHOOTER_FEEDER_DIR * shooter->step_dial_deg * SHOOTER_FEEDER_GEAR;
}

static void Shooter_Feed_One_Step(SHOOTER_DRIVES* shooter, const float scale) {
    shooter->feeder_target_deg += Shooter_Step_Rotor_Deg(shooter) * scale;
}

static void Shooter_Enter_State(SHOOTER_DRIVES* shooter, const Shooter_State state) {
    shooter->state = state;
    shooter->state_ms = 0;

    if (state == SHOOTER_STATE_FIRE) {
        /* ★ 进 FIRE 时目标角对齐当前位置：否则机构被手动拨过或目标角停在 0，一进 FIRE 会猛追一段直接卡弹 */
        shooter->feeder_target_deg = shooter->feeder_angle;
        shooter->shots_left = 0;
        /* 让连发的第一次可以立刻打出来 */
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
    shooter->shots_left = 0;
    shooter->was_triggered = 0;
    shooter->fire_pending = 0;
    shooter->last_sw2 = 0;
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

    shooter->fire_rpm = SHOOTER_FRICTION_RPM_FIRE;
    shooter->step_dial_deg = SHOOTER_FEEDER_STEP_DIAL_DEG;
    shooter->fire_interval_ms = (float)SHOOTER_FIRE_INTERVAL_MS;
    shooter->distance_m = 3.0f;
    shooter->height_m = 0.0f;
    shooter->auto_aim = 0.0f;

    s_shooter = shooter;

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

/* 1kHz 射击任务：采样滤波 / 状态机 / 下发 */
static void Shooter_Update(void* arg) {
    SHOOTER_DRIVES* shooter = (SHOOTER_DRIVES*)arg;

    const float rpm_left = DJI_Motor_Get_Speed(shooter->friction_left);
    const float rpm_right = DJI_Motor_Get_Speed(shooter->friction_right);

    /* 两个摩擦轮对装、转向相反，所以取绝对值再平均 */
    shooter->friction_rpm = (fabsf(rpm_left) + fabsf(rpm_right)) * 0.5f;
    shooter->friction_rpm_filt = FIR_Update(&shooter->fir_friction_rpm, shooter->friction_rpm);

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

    const uint8_t sw2 = user_dbus.sw2;

    /* 单发档的【进入边沿】：拨到单发档的那一下算一次触发 */
    const uint8_t single_edge =
        (sw2 == SHOOTER_SW2_SINGLE && shooter->last_sw2 != SHOOTER_SW2_SINGLE) ? 1 : 0;

#if SHOOTER_TRIGGER_USE_WHEEL
    /* 拨轮当扳机（当前不可用：拨轮被 pitch 占用）*/
    const uint8_t trigger_level = (user_dbus.roll > SHOOTER_TRIGGER_THRESHOLD) ? 1 : 0;
    const uint8_t trigger_edge = (trigger_level != 0 && shooter->was_triggered == 0) ? 1 : 0;
#else
    const uint8_t trigger_level = (sw2 == SHOOTER_SW2_AUTO) ? 1 : 0;
    const uint8_t trigger_edge = single_edge;
#endif

    shooter->was_triggered = trigger_level;
    shooter->last_sw2 = sw2;

    /* ★ 单发请求做成"挂起"而不是当场消费：从停火拨到单发那一下往往发生在 SPINUP 期间，
       当场消费的话进 FIRE 已经没得打，第一发永远打不出去；进 FIRE 且就绪后再消费 */
    if (trigger_edge) {
        shooter->fire_pending = 1;
    }

    /* 摩擦轮是否已到转速：用【滤波后】的转速判断，避免噪声误判 */
    shooter->is_ready = (fabsf(shooter->friction_rpm_filt - shooter->fire_rpm)
                         < SHOOTER_FRICTION_READY_TOL) ? 1 : 0;

    shooter->state_ms += SHOOTER_PERIOD_MS;
    if (shooter->state_ms > 60000U) {
        shooter->state_ms = 60000U;
    }

    switch (shooter->state) {
        case SHOOTER_STATE_STOP:
            shooter->friction_target = 0.0f;
            shooter->shots_left = 0;
            /* 停火时目标角跟随实际角：位置误差恒 0 -> 位置环不输出，拨盘不发力也不发热 */
            shooter->feeder_target_deg = shooter->feeder_angle;
            if (SHOOTER_SW_VALID(sw2)) {
                Shooter_Enter_State(shooter, SHOOTER_STATE_SPINUP);
            } else {
                /* 停在停火档：丢掉挂起的单发请求，免得以后莫名打一发。不能无条件清 ——
                   拨到单发那一拍的 pending 正是本拍置上的，无条件清会导致第一发永远打不出去 */
                shooter->fire_pending = 0;
            }
            break;

        case SHOOTER_STATE_SPINUP:
            shooter->friction_target = shooter->fire_rpm;
            shooter->feeder_target_deg = shooter->feeder_angle;
            /* 转速到位或超时都进 FIRE，但真正拨弹仍要 is_ready，不会没起速就乱拨 */
            if (shooter->is_ready || shooter->state_ms > SHOOTER_SPINUP_TIMEOUT_MS) {
                Shooter_Enter_State(shooter, SHOOTER_STATE_FIRE);
            }
            if (!SHOOTER_SW_VALID(sw2)) {
                Shooter_Enter_State(shooter, SHOOTER_STATE_STOP);
            }
            break;

        case SHOOTER_STATE_FIRE:
            shooter->friction_target = shooter->fire_rpm;

            if (!SHOOTER_SW_VALID(sw2)) {
                Shooter_Enter_State(shooter, SHOOTER_STATE_STOP);
                break;
            }

            /* 卡弹判定：位置没跟上 + 电流大，两个条件同时持续 JAM_MS 才判。
               只看电流会被机构本身的机械冲击误触发。 */
            if (shooter->is_ready
                && fabsf(shooter->feeder_err) > SHOOTER_JAM_ERR_DEG
                && shooter->feeder_current_filt > SHOOTER_JAM_CURRENT) {
                if (shooter->jam_timer_ms < SHOOTER_JAM_MS) {
                    shooter->jam_timer_ms += SHOOTER_PERIOD_MS;
                }
                if (shooter->jam_timer_ms >= SHOOTER_JAM_MS) {
                    shooter->jam_count++;
                    Shooter_Enter_State(shooter, SHOOTER_STATE_REVERSE);
                    break;
                }
            } else {
                shooter->jam_timer_ms = 0;
            }

            /* 发射。没到转速就什么都不做，并把连发计时清零 */
            if (shooter->is_ready) {
                if (sw2 == SHOOTER_SW2_AUTO) {
#if SHOOTER_AUTO_NEED_TRIGGER
                    if (trigger_level) {
#else
                    if (1) {
#endif
                        if (shooter->fire_timer_ms >= (uint32_t)shooter->fire_interval_ms) {
                            shooter->fire_timer_ms = 0;
                            Shooter_Feed_One_Step(shooter, 1.0f);
                        } else {
                            shooter->fire_timer_ms += SHOOTER_PERIOD_MS;
                        }
                    } else {
                        shooter->fire_timer_ms = 0;
                    }
                } else if (sw2 == SHOOTER_SW2_SINGLE) {
                    shooter->fire_timer_ms = 0;
                    /* 消费挂起的单发请求（可能是在 SPINUP 期间拨的档）*/
                    if (shooter->fire_pending) {
                        shooter->fire_pending = 0;
                        shooter->shots_left = SHOOTER_SINGLE_SHOTS;
                    }
                    if (shooter->shots_left > 0) {
                        shooter->shots_left--;
                        Shooter_Feed_One_Step(shooter, 1.0f);
                    }
                }
            } else {
                shooter->fire_timer_ms = 0;
            }
            break;

        case SHOOTER_STATE_REVERSE:
            /* 退弹期间保持摩擦轮转速，退完回 FIRE */
            shooter->friction_target = shooter->fire_rpm;
            if (shooter->state_ms >= SHOOTER_REVERSE_MS) {
                Shooter_Enter_State(shooter, SHOOTER_STATE_FIRE);
            }
            break;

        default:
            Shooter_Enter_State(shooter, SHOOTER_STATE_STOP);
            break;
    }

    /* 状态机镜像给串口绘图看（0=STOP 1=SPINUP 2=FIRE 3=REVERSE）*/
    shooter->state_f = (float)shooter->state;

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
