#include "../../Core/Inc/bsp.h"
#include "../gimbal.h"

#define YAW_STICK_MAX   (660.0f)
#define RAD_S_TO_RPM    (9.54929659f)   /* 60 / 2pi */
#define RAD_TO_DEG      (57.29578f)
#define GIMBAL_DT_S     (0.001f)        /* Gimbal_Update 跑在 1kHz */

static GIMBAL_DRIVES* s_gimbal = NULL;

static void  Gimbal_Update(void* arg);
static void  Gimbal_Pitch_Update(GIMBAL_DRIVES* gimbal);
static float Gimbal_Wrap180(float deg);
static float Gimbal_Stick(int16_t value);
static float Gimbal_Wheel(int16_t value, float dead);
static int16_t Gimbal_RC_Channel(uint8_t ch);

void Gimbal_Init(GIMBAL_DRIVES* gimbal, DJI_MOTOR_DRIVES* yaw_motor) {
    gimbal->yaw_motor = yaw_motor;
    gimbal->zero_deg = GIMBAL_YAW_ZERO_DEG;
    gimbal->yaw = 0.0f;
    gimbal->target_yaw = 0.0f;
    gimbal->last_rx_count = 0;
    gimbal->offline_ms = 0;
    gimbal->is_online = 0;
    gimbal->is_aligned = 0;
    gimbal->has_ever_online = 0;
    gimbal->rc_ch = (float)GIMBAL_RC_CH;
    gimbal->max_w = GIMBAL_YAW_MAX_W;
    gimbal->world_kp = GIMBAL_WORLD_KP;
    gimbal->world_kd = GIMBAL_WORLD_KD;
    gimbal->yaw_rate_lpf = 0.0f;
    gimbal->ff_gain = GIMBAL_FF_GAIN;
    gimbal->target_rate = 0.0f;
    gimbal->last_dbus_count = 0;
    gimbal->dbus_offline_ms = 0;

    gimbal->wave_en = 0.0f;
    gimbal->wave_type = (float)WAVE_SQUARE;
    gimbal->wave_amp = GIMBAL_WAVE_AMP_DEF;
    gimbal->wave_freq = GIMBAL_WAVE_FREQ_DEF;
    gimbal->wave_duty = 0.5f;

    Waveform_Init(&user_wave, GIMBAL_DT_S);
    Gimbal_Wave_Apply();
    gimbal->imu_yaw = 0.0f;
    gimbal->target_speed = 0.0f;

    PID_Init(&gimbal->yaw_pid, GIMBAL_YAW_KP, GIMBAL_YAW_KI, GIMBAL_YAW_KD,
             GIMBAL_YAW_MAX_OUT, GIMBAL_YAW_MAX_IOUT);

    s_gimbal = gimbal;

    SysTick_InitTask(&gimbal->task, (void*)gimbal, 100, 1, Task_REPEAT, Gimbal_Update);
    SysTick_StartTask(&gimbal->task);
}

void Gimbal_Set_Zero(GIMBAL_DRIVES* gimbal) {
    gimbal->zero_deg = DJI_Motor_Get_Angle(gimbal->yaw_motor);
    gimbal->yaw = 0.0f;
    gimbal->target_yaw = 0.0f;
}

float Gimbal_Get_Yaw(const GIMBAL_DRIVES* gimbal) {
    const float rel = (DJI_Motor_Get_Angle(gimbal->yaw_motor) - gimbal->zero_deg)
                    * GIMBAL_YAW_DIR;
    return Gimbal_Wrap180(rel);
}

uint8_t Gimbal_Is_Online(const GIMBAL_DRIVES* gimbal) {
    return (gimbal != NULL) ? gimbal->is_online : 0;
}

uint8_t Gimbal_Is_Aligned(const GIMBAL_DRIVES* gimbal) {
    return (gimbal != NULL) ? gimbal->is_aligned : 0;
}

uint8_t Gimbal_Is_Reliable(const GIMBAL_DRIVES* gimbal) {
    if (gimbal == NULL) {
        return 0;
    }
    if (gimbal->is_online) {
        return gimbal->is_aligned;                        /* 在线且已对齐 */
    }
    /* 掉线状态：从没连上过算"裸车调试"，连上过又掉才算真掉线 */
    return (gimbal->has_ever_online != 0) ? 0 : 1;
}

void Gimbal_Pitch_Init(GIMBAL_DRIVES* gimbal, DJI_MOTOR_DRIVES* pitch_motor) {
    gimbal->pitch_motor = pitch_motor;

    gimbal->pitch = 0.0f;
    gimbal->target_pitch = 0.0f;
    gimbal->pitch_speed = 0.0f;
    gimbal->pitch_current = 0.0f;
    gimbal->pitch_wheel = 0.0f;
    gimbal->pitch_wheel_dead = (float)GIMBAL_PITCH_WHEEL_DEAD;
    gimbal->pitch_max_w = GIMBAL_PITCH_MAX_W;
    gimbal->pitch_auto = GIMBAL_PITCH_AUTO_ENABLE_DEF;
    gimbal->pitch_speed_target = 0.0f;
    gimbal->last_pitch_rx_count = 0;
    gimbal->pitch_offline_ms = 0;
    gimbal->pitch_is_online = 0;
    gimbal->pitch_has_ever_online = 0;
    gimbal->pitch_is_aligned = 0;
    gimbal->last_pitch_dbus_count = 0;
    gimbal->pitch_dbus_offline_ms = 0;

    /* 外环：角度 deg -> 转速目标 RPM（增强 PID）*/
    PID_Adv_Init(&gimbal->pitch_pid_angle);
    PID_Adv_Set_DT(&gimbal->pitch_pid_angle, GIMBAL_DT_S);
    PID_Adv_Set_K_P(&gimbal->pitch_pid_angle, GIMBAL_PITCH_ANGLE_KP);
    PID_Adv_Set_K_I(&gimbal->pitch_pid_angle, GIMBAL_PITCH_ANGLE_KI);
    PID_Adv_Set_K_D(&gimbal->pitch_pid_angle, GIMBAL_PITCH_ANGLE_KD);
    PID_Adv_Set_K_F(&gimbal->pitch_pid_angle, GIMBAL_PITCH_ANGLE_KF);
    PID_Adv_Set_Max_Out(&gimbal->pitch_pid_angle, GIMBAL_PITCH_ANGLE_MAX_OUT);
    PID_Adv_Set_Max_IOut(&gimbal->pitch_pid_angle, GIMBAL_PITCH_ANGLE_MAX_IOUT);
    PID_Adv_Set_Max_DOut(&gimbal->pitch_pid_angle, GIMBAL_PITCH_ANGLE_MAX_DOUT);
    PID_Adv_Set_Dead_Zone(&gimbal->pitch_pid_angle, GIMBAL_PITCH_ANGLE_DEAD);
    PID_Adv_Set_D_Filter_Alpha(&gimbal->pitch_pid_angle, GIMBAL_PITCH_ANGLE_D_ALPHA);
    PID_Adv_Set_I_Variable_Speed(&gimbal->pitch_pid_angle,
                                 GIMBAL_PITCH_ANGLE_IVAR_A, GIMBAL_PITCH_ANGLE_IVAR_B);
    /* ★ 积分分离用"衰减"而不是硬清零：pitch 靠积分顶重力，硬清零会让枪管在大阶跃瞬间沉一下 */
    PID_Adv_Set_I_Separate(&gimbal->pitch_pid_angle, GIMBAL_PITCH_ANGLE_ISEP,
                           PID_ADV_I_SEP_DECAY);
    /* ★ 微分先行：模式切换 / 自动瞄准更新目标时会有阶跃，不能让 D 项跟着跳 */
    PID_Adv_Set_D_First(&gimbal->pitch_pid_angle, 1);
    PID_Adv_Set_Direction(&gimbal->pitch_pid_angle, PID_ADV_DIRECT);
    /* ★★ 零位积分泄放必须关闭：pitch 静止时也要靠积分顶住重力，泄放掉枪管就会慢慢下沉 */
    PID_Adv_Set_Zero_Bleed(&gimbal->pitch_pid_angle, 0);

    /* 内环：转速 RPM -> 电流刻度（增强 PID）*/
    PID_Adv_Init(&gimbal->pitch_pid_speed);
    PID_Adv_Set_DT(&gimbal->pitch_pid_speed, GIMBAL_DT_S);
    PID_Adv_Set_K_P(&gimbal->pitch_pid_speed, GIMBAL_PITCH_SPEED_KP);
    PID_Adv_Set_K_I(&gimbal->pitch_pid_speed, GIMBAL_PITCH_SPEED_KI);
    PID_Adv_Set_K_D(&gimbal->pitch_pid_speed, GIMBAL_PITCH_SPEED_KD);
    PID_Adv_Set_K_F(&gimbal->pitch_pid_speed, GIMBAL_PITCH_SPEED_KF);
    PID_Adv_Set_Max_Out(&gimbal->pitch_pid_speed, GIMBAL_PITCH_SPEED_MAX_OUT);
    PID_Adv_Set_Max_IOut(&gimbal->pitch_pid_speed, GIMBAL_PITCH_SPEED_MAX_IOUT);
    PID_Adv_Set_Max_DOut(&gimbal->pitch_pid_speed, GIMBAL_PITCH_SPEED_MAX_DOUT);
    PID_Adv_Set_Dead_Zone(&gimbal->pitch_pid_speed, GIMBAL_PITCH_SPEED_DEAD);
    PID_Adv_Set_D_Filter_Alpha(&gimbal->pitch_pid_speed, GIMBAL_PITCH_SPEED_D_ALPHA);
    PID_Adv_Set_I_Variable_Speed(&gimbal->pitch_pid_speed,
                                 GIMBAL_PITCH_SPEED_IVAR_A, GIMBAL_PITCH_SPEED_IVAR_B);
    PID_Adv_Set_I_Separate(&gimbal->pitch_pid_speed, GIMBAL_PITCH_SPEED_ISEP,
                           PID_ADV_I_SEP_DECAY);
    PID_Adv_Set_D_First(&gimbal->pitch_pid_speed, 1);
    PID_Adv_Set_Direction(&gimbal->pitch_pid_speed, PID_ADV_DIRECT);
    PID_Adv_Set_Zero_Bleed(&gimbal->pitch_pid_speed, 0);

    /* ★ 串级必须在 DJI_Motor_Init(..., &gimbal->pitch_cascade) 之前初始化：电机的控制器指针
       指向这里，之后任何一帧反馈都可能调用 cascade->Calculate，内外环指针为 NULL 就是空指针解引用。 */
    Cascade_Controller_Init(&gimbal->pitch_cascade,
                            (CONTROLLER_INTERFACE*)&gimbal->pitch_pid_angle,
                            (CONTROLLER_INTERFACE*)&gimbal->pitch_pid_speed,
                            GIMBAL_PITCH_SPEED_MAX_OUT);
}

float Gimbal_Get_Pitch(const GIMBAL_DRIVES* gimbal) {
    return (gimbal != NULL) ? gimbal->pitch : 0.0f;
}

void Gimbal_Set_Pitch_Target(GIMBAL_DRIVES* gimbal, const float deg) {
    if (gimbal == NULL) {
        return;
    }
    gimbal->target_pitch = deg;
    gimbal->pitch_auto = 1.0f;
}

void Gimbal_Set_Pitch_Auto(GIMBAL_DRIVES* gimbal, const uint8_t enable) {
    if (gimbal == NULL) {
        return;
    }

    if (enable == 0) {
        gimbal->target_pitch = gimbal->pitch;
        gimbal->pitch_auto = 0.0f;
    } else {
        gimbal->pitch_auto = 1.0f;
    }
}

uint8_t Gimbal_Pitch_Is_Online(const GIMBAL_DRIVES* gimbal) {
    return (gimbal != NULL) ? gimbal->pitch_is_online : 0;
}

uint8_t Gimbal_Pitch_Is_Aligned(const GIMBAL_DRIVES* gimbal) {
    return (gimbal != NULL) ? gimbal->pitch_is_aligned : 0;
}

static void Gimbal_Update(void* arg) {
    GIMBAL_DRIVES* gimbal = (GIMBAL_DRIVES*)arg;

    /* 掉线检测：反馈计数不涨就累计超时 */
    if (gimbal->yaw_motor->rx_count != gimbal->last_rx_count) {
        gimbal->last_rx_count = gimbal->yaw_motor->rx_count;
        gimbal->offline_ms = 0;
        gimbal->is_online = 1;
        gimbal->has_ever_online = 1;
    } else if (gimbal->offline_ms < GIMBAL_OFFLINE_MS) {
        gimbal->offline_ms++;
        if (gimbal->offline_ms >= GIMBAL_OFFLINE_MS) {
            gimbal->is_online = 0;
            gimbal->is_aligned = 0;      /* 掉线就丢掉零点，恢复后重新对齐 */
        }
    }

    /* 观察用镜像 + 遥控目标：故意放在对齐判断【之前】，云台没接通也能从串口看 graw/gspd/gtgt */
    gimbal->angle_raw = DJI_Motor_Get_Angle(gimbal->yaw_motor);
    gimbal->speed = DJI_Motor_Get_Speed(gimbal->yaw_motor);

    /* ★ pitch 轴在这里更新：必须放在 yaw 的"未对齐就 return"之前，否则 yaw 掉线会把 pitch 一起跳过 */
    Gimbal_Pitch_Update(gimbal);

#if GIMBAL_LOCK_WORLD
    /* A 板在云台 yaw 上，所以 IMU 的 yaw 就是【云台的世界朝向】 */
    gimbal->imu_yaw = Gimbal_Wrap180(Attitude_Get_Yaw(&user_attitude));
#endif

#if GIMBAL_RC_ENABLE
    /* 遥控在线检测用 rx_count 而不是 is_update：① is_update 会被底盘任务清零，两个模块抢同一个
       标志会丢数据；② DT7 只有 ~71Hz 而本任务 1kHz，拿它当积分门控会让积分速度只剩 1/14 */
    if (user_dbus.rx_count != gimbal->last_dbus_count) {
        gimbal->last_dbus_count = user_dbus.rx_count;
        gimbal->dbus_offline_ms = 0;
    } else if (gimbal->dbus_offline_ms < GIMBAL_OFFLINE_MS) {
        gimbal->dbus_offline_ms++;
    }

    /* 遥控断线就把杆量当 0：锁世界模式会停住不动，速度模式会主动刹车 */
    const float stick = (gimbal->dbus_offline_ms < GIMBAL_OFFLINE_MS)
                      ? Gimbal_Stick(Gimbal_RC_Channel((uint8_t)gimbal->rc_ch))
                      : 0.0f;

#if GIMBAL_LOCK_WORLD
    /* ★ 每周期都积分（本任务固定 1kHz，DT = 1ms）：stick 对应角速度 deg/s，既是积分增量也是外环前馈 */
    gimbal->target_rate = stick * gimbal->max_w * RAD_TO_DEG * GIMBAL_YAW_DIR;

    gimbal->target_yaw += gimbal->target_rate * GIMBAL_DT_S;
    gimbal->target_yaw = Gimbal_Wrap180(gimbal->target_yaw);
#else
    /* 纯速度控制：推着匀速转、松杆主动刹车停在当前位置 */
    gimbal->target_speed = stick * gimbal->max_w * RAD_S_TO_RPM * GIMBAL_YAW_DIR;
#endif
#else
#if !GIMBAL_LOCK_WORLD
    gimbal->target_speed = 0.0f;
#endif
#endif

    /* 首次对齐：绝对编码器只知道"转子在哪"，上电时 zero_deg 还是旧值，直接下发会把云台猛拉向编码器
       零位；所以等第一帧反馈到了，以【当前位置】为零点，对齐完成前不发任何指令 */
    if (!gimbal->is_aligned) {
        if (!gimbal->is_online) {
            return;
        }
        Gimbal_Set_Zero(gimbal);
#if GIMBAL_LOCK_WORLD
        gimbal->target_yaw = gimbal->imu_yaw;   /* 世界目标 = 当前世界朝向 */
#endif
        gimbal->is_aligned = 1;
    }

    gimbal->yaw = Gimbal_Get_Yaw(gimbal);

#if GIMBAL_WAVE_ENABLE
    if (gimbal->wave_en > 0.5f) {
        /* ★ PID 测试模式：方波直接当转速指令、绕过外环；串口绘图看 gspd 跟随来调 kp/ki */
        gimbal->target_speed = Waveform_Update(&user_wave);
        gimbal->yaw_motor->Set_Motor_State(gimbal->yaw_motor, gimbal->target_speed);
        return;
    }
#endif

#if GIMBAL_LOCK_WORLD
    /* ★ 锁世界 = 位置环 + 三路抗惯性：
       ① 陀螺阻尼 —— 用 IMU 的 yaw 角速度做阻尼，抑制"被惯性拖着走/回摆"
       ② 底盘角速度前馈 —— 底盘自转时云台须相对底盘反向同速，才能在世界系里不动
       ③ 摇杆前馈 —— 消除跟踪斜坡输入的固有滞后
       ⚠️ 符号约定：陀螺阻尼与底盘前馈都是【世界系物理量】，和位置环同号放进 DIR 里；
          若把 GIMBAL_YAW_DIR 改成 -1，这三项的符号要一起重新核对。 */
#if GIMBAL_IMU_ON_GIMBAL
    const float world_yaw = gimbal->imu_yaw;
#else
    /* IMU 装在底盘上：云台世界朝向 = 底盘世界朝向 + 云台相对底盘的编码器角 */
    const float world_yaw = Gimbal_Wrap180(gimbal->imu_yaw + gimbal->yaw);
#endif
    const float error = Gimbal_Wrap180(gimbal->target_yaw - world_yaw);

    /* 陀螺 yaw 角速度 deg/s -> RPM（/360*60 = /6），先一阶低通压噪声再进阻尼 */
    const float gyro_rate_rpm = user_attitude.yaw_rate * (1.0f / 6.0f);
    gimbal->yaw_rate_lpf += GIMBAL_WORLD_D_ALPHA * (gyro_rate_rpm - gimbal->yaw_rate_lpf);

    /* deg/s -> RPM */
    const float rate_rpm = gimbal->target_rate * (1.0f / 6.0f);

    float target_speed = GIMBAL_YAW_DIR *
                         (gimbal->world_kp * error
                          - gimbal->world_kd * gimbal->yaw_rate_lpf
                          - user_chassis.omega * RAD_S_TO_RPM * gimbal->ff_gain)
                         + rate_rpm;

    /* 外环限幅：跨 ±180° 或 IMU 异常时不让外环一脚踹满速度环 */
    if (target_speed > GIMBAL_WORLD_MAX_RPM) {
        target_speed = GIMBAL_WORLD_MAX_RPM;
    } else if (target_speed < -GIMBAL_WORLD_MAX_RPM) {
        target_speed = -GIMBAL_WORLD_MAX_RPM;
    }
    gimbal->target_speed = target_speed;

    gimbal->yaw_motor->Set_Motor_State(gimbal->yaw_motor, gimbal->target_speed);
#else
    gimbal->yaw_motor->Set_Motor_State(gimbal->yaw_motor, gimbal->target_speed);
#endif
}

/* 1kHz：pitch 角度环。串级模式下外环在电机内部跑，这里只管掉线保护 + 角度标定 + 遥控给目标 + 软限位 */
static void Gimbal_Pitch_Update(GIMBAL_DRIVES* gimbal) {
#if !GIMBAL_PITCH_ENABLE
    /* pitch 轴整个关掉（排错/二分定位用）：一句话都不说，也不下发指令 */
    (void)gimbal;
    return;
#else
    DJI_MOTOR_DRIVES* motor = gimbal->pitch_motor;

    /* Gimbal_Pitch_Init 还没跑（pitch_motor 仍为 NULL）—— 直接跳过 */
    if (motor == NULL) {
        return;
    }

    /* 掉线检测：与 yaw 同一套逻辑 */
    if (motor->rx_count != gimbal->last_pitch_rx_count) {
        gimbal->last_pitch_rx_count = motor->rx_count;
        gimbal->pitch_offline_ms = 0;
        gimbal->pitch_is_online = 1;
        gimbal->pitch_has_ever_online = 1;
    } else if (gimbal->pitch_offline_ms < GIMBAL_OFFLINE_MS) {
        gimbal->pitch_offline_ms++;
        if (gimbal->pitch_offline_ms >= GIMBAL_OFFLINE_MS) {
            gimbal->pitch_is_online = 0;
            gimbal->pitch_is_aligned = 0;    /* 掉线就丢掉标定，恢复后重新标 */
        }
    }

    /* 观察用镜像：放在标定判断【之前】，电机没标定时也能从串口看出 pitch/pitch_spd/pitch_cur 有没有值 */
    gimbal->pitch = DJI_Motor_Get_Angle(motor);
    gimbal->pitch_speed = DJI_Motor_Get_Speed(motor);
    gimbal->pitch_current = DJI_Motor_Get_Current(motor);

    /* ★ 首次角度标定：收到第一帧后把【当前位置】定义成 GIMBAL_PITCH_ZERO_BIAS_DEG，此后
       Get_Angle() 返回的才是真实俯仰角，软限位才落在正确角度上（标定这一拍不下发指令）。 */
    if (!gimbal->pitch_is_aligned) {
        if (!gimbal->pitch_is_online) {
            return;
        }
        DJI_Motor_Set_Angle_Here(motor, GIMBAL_PITCH_ZERO_BIAS_DEG);
        gimbal->pitch = DJI_Motor_Get_Angle(motor);
        gimbal->target_pitch = gimbal->pitch;
        gimbal->pitch_is_aligned = 1;
        return;
    }

    /* 遥控：★ 用遥控器左上角【拨轮 roll】控制俯仰 —— 拨轮偏转 → 目标角以恒定角速度走，回中即停 */
    float wheel = 0.0f;

#if GIMBAL_PITCH_RC_ENABLE
    if (user_dbus.rx_count != gimbal->last_pitch_dbus_count) {
        gimbal->last_pitch_dbus_count = user_dbus.rx_count;
        gimbal->pitch_dbus_offline_ms = 0;
    } else if (gimbal->pitch_dbus_offline_ms < GIMBAL_OFFLINE_MS) {
        gimbal->pitch_dbus_offline_ms++;
    }

    /* 遥控断线就把拨轮当 0：目标角停在原地，不会乱动 */
    if (gimbal->pitch_dbus_offline_ms < GIMBAL_OFFLINE_MS) {
        wheel = Gimbal_Wheel(user_dbus.roll, gimbal->pitch_wheel_dead);
    }
#endif

    gimbal->pitch_wheel = wheel;

    /* pitch_auto != 0 时目标角由上层（弹道解算）给，拨轮不参与，避免互相打架 */
    if (gimbal->pitch_auto < 0.5f) {
        gimbal->target_pitch += wheel * gimbal->pitch_max_w * GIMBAL_PITCH_WHEEL_DIR * GIMBAL_DT_S;
    }

    /* 软限位：夹住目标角；目标固定后误差不再增大、积分也不再涨，等于顺带做了抗积分饱和 */
    if (gimbal->target_pitch > GIMBAL_PITCH_MAX_DEG) {
        gimbal->target_pitch = GIMBAL_PITCH_MAX_DEG;
    } else if (gimbal->target_pitch < GIMBAL_PITCH_MIN_DEG) {
        gimbal->target_pitch = GIMBAL_PITCH_MIN_DEG;
    }

#if GIMBAL_PITCH_CASCADE
    /* ★ 串级在电机内部跑（Rotor_angle 模式）：只把目标【角度】交给电机，它自己走角度环→速度环→电流 */
    motor->Set_Motor_State(motor, gimbal->target_pitch);
#else
    /* 退回单环：外环角度环在这里算，输出当转速目标（电机停在 Rotor_speed 模式），用的是同一个 PID */
    gimbal->pitch_pid_angle.Set_Target(&gimbal->pitch_pid_angle, gimbal->target_pitch);
    gimbal->pitch_speed_target =
        gimbal->pitch_pid_angle.Calculate(&gimbal->pitch_pid_angle, gimbal->pitch, 0.0f);
    motor->Set_Motor_State(motor, gimbal->pitch_speed_target);
#endif
#endif /* GIMBAL_PITCH_ENABLE */
}

void Gimbal_Wave_Apply(void) {
    const int t = (int)user_gimbal.wave_type;

    switch (t) {
        case WAVE_STEP:
            Waveform_Set_Step(&user_wave, -user_gimbal.wave_amp,
                              user_gimbal.wave_amp, 0.5f);
            break;
        case WAVE_SINE:
            Waveform_Set_Sine(&user_wave, user_gimbal.wave_amp,
                              user_gimbal.wave_freq, 0.0f);
            break;
        case WAVE_TRIANGLE:
            Waveform_Set_Triangle(&user_wave, user_gimbal.wave_amp,
                                  user_gimbal.wave_freq, 0.0f);
            break;
        case WAVE_SAW:
            Waveform_Set_Saw(&user_wave, user_gimbal.wave_amp,
                             user_gimbal.wave_freq, 0.0f);
            break;
        case WAVE_SQUARE:
        default:
            Waveform_Set_Square(&user_wave, user_gimbal.wave_amp,
                                user_gimbal.wave_freq, 0.0f, user_gimbal.wave_duty);
            break;
    }

    Waveform_Set_Limit(&user_wave, -GIMBAL_WAVE_MAX_RPM, GIMBAL_WAVE_MAX_RPM);
}

static int16_t Gimbal_RC_Channel(const uint8_t ch) {
    switch (ch) {
        case 0:  return user_dbus.ch0;
        case 1:  return user_dbus.ch1;
        case 2:  return user_dbus.ch2;
        case 3:  return user_dbus.ch3;
        default: return 0;
    }
}

static float Gimbal_Wrap180(float deg) {
    /* 先用整数除法一次折叠到 ±360 以内，再 while 微调：云台一直转，deg 可能几万度，纯 while 太慢 */
    deg -= 360.0f * (float)((int32_t)(deg / 360.0f));

    while (deg >= 180.0f) {
        deg -= 360.0f;
    }
    while (deg < -180.0f) {
        deg += 360.0f;
    }
    return deg;
}

static float Gimbal_Stick(const int16_t value) {
    if (value > -GIMBAL_RC_DEAD && value < GIMBAL_RC_DEAD) {
        return 0.0f;
    }
    return (float)value / YAW_STICK_MAX;
}

/* 拨轮（DBUS 的 roll，范围也是 ±660）归一化到 -1~1，带死区 */
static float Gimbal_Wheel(const int16_t value, const float dead) {
    if (value > -dead && value < dead) {
        return 0.0f;
    }
    return (float)value / YAW_STICK_MAX;
}
