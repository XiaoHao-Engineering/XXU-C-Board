#include "../../Core/Inc/bsp.h"
#include "../gimbal.h"
#include <math.h>       /* isnan / isinf：视觉数据有效性闸门要用（工程可用标准 math.h）*/

#define YAW_STICK_MAX   (660.0f)
#define RAD_S_TO_RPM    (9.54929659f)   /* 60 / 2pi */
#define RAD_TO_DEG      (57.29578f)
#define GIMBAL_DT_S     (0.001f)        /* Gimbal_Update 跑在 1kHz */

static GIMBAL_DRIVES* s_gimbal = NULL;

static void  Gimbal_Update(void* arg);
static void  Gimbal_Pitch_Update(GIMBAL_DRIVES* gimbal);
static void  Gimbal_Aim_Update(GIMBAL_DRIVES* gimbal);
static uint8_t Gimbal_Aim_Accept_Frame(GIMBAL_DRIVES* gimbal);
static void  Gimbal_Aim_Release(GIMBAL_DRIVES* gimbal);
static void  Gimbal_Aim_Abort(GIMBAL_DRIVES* gimbal);
static float Gimbal_Wrap180(float deg);
static float Gimbal_Stick(int16_t value, float dead);
static int16_t Gimbal_RC_Channel(uint8_t ch);

void Gimbal_Init(GIMBAL_DRIVES* gimbal, DJI_MOTOR_DRIVES* yaw_motor) {
    gimbal->yaw_motor = yaw_motor;
    gimbal->zero_deg = GIMBAL_YAW_ZERO_DEG;
    gimbal->yaw = 0.0f;
    gimbal->target_yaw = 0.0f;
    gimbal->target_yaw_smooth = 0.0f;   /* 上电时平滑值与目标角一致（真正对齐在后面 target_yaw = imu_yaw 处）*/
    /* ★ 目标角平滑分两套（按 aim_takeover 二选一，见 Gimbal_Update 里算 alpha_s 那一段）：
       摇杆/无目标用 RC（小=跟手），自瞄接管用 AIM（大=不抖），两个都可在线上改。 */
    gimbal->tgt_smooth_ms     = GIMBAL_TGT_SMOOTH_RC_MS;   /* 摇杆：gimbal_tgt_smooth */
    gimbal->tgt_smooth_aim_ms = GIMBAL_TGT_SMOOTH_AIM_MS;  /* 自瞄：gimbal_aim_smooth */
    gimbal->last_rx_count = 0;
    gimbal->offline_ms = 0;
    gimbal->is_online = 0;
    gimbal->is_aligned = 0;
    gimbal->has_ever_online = 0;
    gimbal->rc_ch = (float)GIMBAL_RC_CH;
    gimbal->max_w = GIMBAL_YAW_MAX_W;
    gimbal->world_kp = GIMBAL_WORLD_KP;
    gimbal->world_kd = GIMBAL_WORLD_KD;
    gimbal->world_d_alpha = GIMBAL_WORLD_D_ALPHA;   /* 可在线改：gimbal_world_da */
    gimbal->world_dead = GIMBAL_WORLD_DEAD_DEG;
    gimbal->rate_slew = GIMBAL_RATE_SLEW;
    gimbal->yaw_rate_lpf = 0.0f;
    gimbal->ff_gain = GIMBAL_FF_GAIN;
    gimbal->stick_kf = GIMBAL_YAW_KF;
    gimbal->yaw_bias = 0.0f;
    gimbal->bias_accum = 0.0f;
    gimbal->bias_k = GIMBAL_BIAS_TRACK_ALPHA;
    gimbal->enc_rate_lpf = 0.0f;
    gimbal->enc_last = 0.0f;
    gimbal->yaw_auto = GIMBAL_YAW_AUTO_DEF;
    gimbal->yaw_auto_pre = 0.0f;
    gimbal->yaw_target_ext = 0.0f;
    gimbal->cam_yaw_off = GIMBAL_CAM_YAW_OFF;
    gimbal->aim_ff_gain = GIMBAL_AIM_FF_GAIN_YAW;
    gimbal->aim_delay_ms = GIMBAL_AIM_DELAY_MS;
    gimbal->aim_timeout_ms = GIMBAL_AIM_TIMEOUT_MS;
    gimbal->aim_err = 0.0f;
    gimbal->aim_ok_f = 0.0f;
    gimbal->aim_valid = 0;
    gimbal->aim_ms = 0;
    gimbal->target_rate = 0.0f;

    /* 自瞄 pitch 侧（与 yaw 对称）+ 数据闸门的计数器 */
    gimbal->pitch_target_ext = 0.0f;
    gimbal->cam_pitch_off = GIMBAL_CAM_PITCH_OFF;
    gimbal->aim_ff_gain_pitch = GIMBAL_AIM_FF_GAIN_PITCH;
    gimbal->pitch_aim_err = 0.0f;
    gimbal->pitch_aim_valid = 0;
    gimbal->aim_rate = 0.0f;
    gimbal->pitch_aim_rate = 0.0f;
    gimbal->aim_takeover = 0.0f;
    gimbal->aim_bad_frames = 0;
    gimbal->aim_reject_cnt = 0;
    gimbal->last_vision_frame = 0;
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
    gimbal->yaw_fb   = 0.0f;      /* 位置环实际反馈镜像（串口绘图 ch7）*/
    gimbal->yaw_fb_rad = 0.0f;    /* ch7 弧度镜像（只读观测，见 gimbal.h）*/
    gimbal->yaw_rad    = 0.0f;    /* ch8 弧度镜像（只读观测，见 gimbal.h）*/
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
    gimbal->target_yaw_smooth = 0.0f;   /* 平滑目标一起归零，否则重连会从旧值慢慢爬过来 */
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
    gimbal->pitch_stick = 0.0f;
    gimbal->pitch_rc_ch = (float)GIMBAL_PITCH_RC_CH;
    gimbal->pitch_dead = (float)GIMBAL_PITCH_RC_DEAD;
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

    /* ★ 上电回位（boot homing）状态：每次上电（Gimbal_Pitch_Init 只在上电时调用一次）清零，
       所以 pitch_boot_homed == 0 就是"本次上电还没回过位"，比 pitch_has_ever_online 可靠：
       后者在第一帧反馈就被置 1，上电初期一掉线就会把回位目标覆盖成"保持原位"。 */
    gimbal->pitch_boot_homing = 0;
    gimbal->pitch_boot_homed = 0;
    gimbal->pitch_homing_ms = 0;
    gimbal->pitch_homing_in_pos_ms = 0;

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

/* ★ 把目标角夹在软限位内（全文件唯一一份通用 clamp 实现）：
   否则摇杆推到底时 target_pitch 会一路累积超出机械行程，导致误差持续增大、云台顶在
   限位上（实测会冲到 +33/−13，限位是 +28.6/−10.8）。
   调用点只有两处，都是同一个函数：
     ① Gimbal_Set_Pitch_Target：弹道/自瞄的 50Hz 外部写入点，写完立即夹 —— 本函数跑在
        1kHz 云台任务之外，不夹的话字段会以原始解算角一直停到下一拍；
     ② Gimbal_Pitch_Update：摇杆积分/视觉/回位全部写完之后、下发之前的每拍最终闸门。 */
static void Gimbal_Clamp_Pitch_Target(GIMBAL_DRIVES* gimbal) {
    if (gimbal->target_pitch > GIMBAL_PITCH_MAX_DEG) {
        gimbal->target_pitch = GIMBAL_PITCH_MAX_DEG;
    }
    if (gimbal->target_pitch < GIMBAL_PITCH_MIN_DEG) {
        gimbal->target_pitch = GIMBAL_PITCH_MIN_DEG;
    }
}

void Gimbal_Set_Pitch_Target(GIMBAL_DRIVES* gimbal, const float deg) {
    if (gimbal == NULL) {
        return;
    }
    gimbal->target_pitch = deg;

    /* ★ 写入即合法：本函数被 50Hz 的弹道/自瞄任务在【1kHz 云台任务之外】调用，
       不在这里夹的话，字段会以原始解算角一直停到下一拍才被最终闸门夹住；
       这期间任何读取者（串口绘图 ch6 直接读本字段）都可能读到 +33/−13 这种越界值，
       而实际角被软限位挡在 +29/−10，看起来就像"目标越过限位还在累积"。
       这里调用的是与最终闸门相同的函数，不是第二份 clamp 逻辑。 */
    Gimbal_Clamp_Pitch_Target(gimbal);

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

#if GIMBAL_LOCK_WORLD
    /* A 板在云台 yaw 上，所以 IMU 的 yaw 就是【云台的世界朝向】。
       ★ 提到 pitch / 自瞄之前：视觉数据闸门要用它做 yaw 量程判定（见 Gimbal_Aim_Update）*/
    gimbal->imu_yaw = Gimbal_Wrap180(Attitude_Get_Yaw(&user_attitude));
#endif

    /* ★★ 视觉自瞄：数据有效性闸门 + 目标解算。
       必须放在两个轴的控制之前：视觉一包里 yaw/pitch 是同一帧，两个轴要用
       同一份"有效/无效"结论（要么都信，要么都不信）。它只写结构体里的
       目标角/前馈/标志，不下发任何电机指令。 */
    Gimbal_Aim_Update(gimbal);

    /* ★ pitch 轴在这里更新：必须放在 yaw 的"未对齐就 return"之前，否则 yaw 掉线会把 pitch 一起跳过 */
    Gimbal_Pitch_Update(gimbal);

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
                      ? Gimbal_Stick(Gimbal_RC_Channel((uint8_t)gimbal->rc_ch), (float)GIMBAL_RC_DEAD)
                      : 0.0f;

#if GIMBAL_LOCK_WORLD
    /* ★ 每周期都积分（本任务固定 1kHz，DT = 1ms）：stick 对应角速度 deg/s，既是积分增量也是外环前馈 */
    /* ★ 摇杆速率限斜率：松杆瞬间前馈从满杆速率直接掉到 0，而云台还在以那个速度走，
       位置环只能事后去追 -> "冲出去再回来"。限斜率让前馈慢慢减下来，云台跟着平滑减速。
       积分用的也是限斜率之后的 target_rate，所以目标角不会跳。 */
    const float rate_cmd = stick * gimbal->max_w * RAD_TO_DEG * GIMBAL_YAW_DIR;
    if (gimbal->aim_takeover > 0.5f) {
        /* ★★★ 自瞄接管：目标角来自视觉，摇杆完全不参与。
           参考 ita_robot.cpp:639-650 —— "视觉下，覆盖设置云台的目标角度"（世界绝对角）。 */
        if (gimbal->aim_valid != 0) {
            /* ① 接管瞬间的目标初始化在 Gimbal_Aim_Update 里做（目标角从当前角度起），
                  否则云台会朝一个陈旧目标猛甩一下（撞软限位、丢目标）*/
            gimbal->target_rate = 0.0f;      /* 摇杆积分停用，避免和视觉目标打架 */
            /* ② 延迟补偿：视觉看到的是【过去】，用云台自身角速度外推到"现在"。
                  age 用 aim_delay_ms 估计（需实测）；姿态漂移在几十毫秒内可忽略 */
            gimbal->target_yaw = Gimbal_Wrap180(gimbal->yaw_target_ext
                                 + gimbal->yaw_rate_lpf * 6.0f
                                   * (gimbal->aim_delay_ms * 0.001f));
        } else {
            /* ③ 丢帧冻结：视觉数据暂时不可用时【冻结】目标角 ——
                  既不继续外推（会越推越偏），也不回中（会让操作手措手不及）；
                  角速度前馈同时清零，不许带着上一帧的速度继续冲 */
            gimbal->target_rate = 0.0f;
            gimbal->aim_rate = 0.0f;
        }
        gimbal->aim_err = Gimbal_Wrap180(gimbal->yaw_target_ext - gimbal->imu_yaw);
        gimbal->aim_ok_f = (float)gimbal->aim_valid;
    } else {
    const float rate_step = gimbal->rate_slew * GIMBAL_DT_S;
    if (rate_cmd > gimbal->target_rate + rate_step) {
        gimbal->target_rate += rate_step;
    } else if (rate_cmd < gimbal->target_rate - rate_step) {
        gimbal->target_rate -= rate_step;
    } else {
        gimbal->target_rate = rate_cmd;
    }

    gimbal->target_yaw += gimbal->target_rate * GIMBAL_DT_S;
    gimbal->target_yaw = Gimbal_Wrap180(gimbal->target_yaw);
    gimbal->aim_rate = 0.0f;     /* 摇杆模式：视觉前馈必须清零，避免残留 */
    }
    gimbal->yaw_auto_pre = gimbal->yaw_auto;
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
        /* ★ 平滑目标角必须一起对齐：否则这一拍 smooth 还是旧值（0 或掉线前的值），
           位置环会把云台往那儿拉一下，表现成"上电/重连后甩一下再回来" */
        gimbal->target_yaw_smooth = gimbal->target_yaw;
#endif
        gimbal->is_aligned = 1;
    }

    gimbal->yaw = Gimbal_Get_Yaw(gimbal);
    /* ★ 串口绘图 ch8 的弧度镜像（只读观测，不参与控制）：每拍同步一次 */
    gimbal->yaw_rad = gimbal->yaw * GIMBAL_DEG2RAD;
#if GIMBAL_LOCK_WORLD
    /* 回传给视觉：四元数(EKF) + 当前角度/角速度 + 弹速。
       视觉需要这些才能把"像素偏差"换成"目标角度"，并做完整三维变换 */
    /* ★ pitch 字段：见 user_vision.h 里 VISION_PITCH_USE_IMU_ONLY 的说明。
       本车 IMU 在 yaw 框架上 -> 枪口世界仰角 = IMU俯仰 + 编码器俯仰 */
#if VISION_PITCH_USE_IMU_ONLY
    /* ★ 与参考工程一致：参考的 Gimbal.IMU_Gimbal.Get_Angle_Pitch() 返回 Data_Euler.pitch,
       而 dvc_bmi088.cpp:524-527 写明"QEKF_INS.Roll 实际是绕 Y 轴的俯仰角"
       -> Data_Euler.pitch = QEKF_INS.Roll。所以发给视觉的 pitch 要用 qe_roll 而不是 qe_pitch。
       实测两个独立样本都验证了这一点：包内 pitch ≈ 四元数解出的 roll。 */
    const float tx_pitch = user_attitude.qe_roll;
#else
    const float tx_pitch = user_attitude.qe_pitch + gimbal->pitch;
#endif

    /* ★ 回传的工作模式用【使能开关 yaw_auto】而不是 aim_takeover：
       这是和视觉的握手 —— 只有先告诉视觉"AUTOAIM"，它才会开始发 mode=1/2 的控制包；
       若这里改成看 aim_takeover（依赖 mode!=0），就会形成"等对方先动"的死锁。 */
    Vision_Set_Tx_Mode(&user_vision,
                       (gimbal->yaw_auto > 0.5f) ? VISION_TX_MODE_AUTOAIM : VISION_TX_MODE_IDLE);
    /* ★ yaw 发【扣掉零偏后】的值(bias_accum 跟踪到的零偏)，而不是未修正的 imu_yaw：
       这样视觉的世界系跟的是和闭环同一个 yaw_fb，漂移速度一致（原来发 imu_yaw 会漂得更快）*/
    Vision_Set_Tx_Attitude(&user_vision,
                           user_attitude.qe_q[0], user_attitude.qe_q[1],
                           user_attitude.qe_q[2], user_attitude.qe_q[3],
                           /* ★ 按包内单位换算（VISION_ANGLE_UNIT_RAD=1 时 -> 弧度/弧度每秒，与参考工程一致）*/
                           Gimbal_Wrap180(gimbal->imu_yaw - gimbal->bias_accum) * VISION_DEG2PKG,
                           user_attitude.yaw_rate * VISION_DEG2PKG,
                           tx_pitch * VISION_DEG2PKG,
                           gimbal->pitch_speed * 6.0f * VISION_DEG2PKG);
    /* ★ 弹速：按参考工程的算法用【摩擦轮转速】换算（rad/s × 系数），
       不再用 ballistics.v0 那个占位值（它恒为 5.0，会让视觉算错弹道）。
       bullet_count 仍为 0：真实弹量要接裁判系统，本车没接。 */
    Vision_Set_Tx_Bullet(&user_vision,
                         user_shooter.friction_rpm_filt * VISION_RPM_TO_RAD_S
                           * VISION_BULLET_SPEED_COEF,
                         0U);
#endif

#if GIMBAL_LOCK_WORLD && GIMBAL_IMU_ON_GIMBAL
    /* ★★ 陀螺零偏在线跟踪（根治"摇杆不动时每 5~6 秒飘一次"，原理见 gimbal.h）：
       摇杆居中 + 编码器说云台没动 -> 真实角速度≈0 -> 测到的 yaw_rate 就是零偏，
       低通后累积成 bias_accum，位置反馈用 imu_yaw - bias_accum。反馈量不再漂，
       位置环就不用为了追零偏而让云台物理爬行。 */
    const float enc_rate = (gimbal->yaw - gimbal->enc_last) / GIMBAL_DT_S;
    gimbal->enc_last = gimbal->yaw;
    gimbal->enc_rate_lpf += GIMBAL_ENC_RATE_ALPHA * (enc_rate - gimbal->enc_rate_lpf);

    if (gimbal->bias_k > 0.0f
        && stick == 0.0f
        && gimbal->enc_rate_lpf <  GIMBAL_BIAS_ENC_MAX_RATE
        && gimbal->enc_rate_lpf > -GIMBAL_BIAS_ENC_MAX_RATE) {
        gimbal->yaw_bias += gimbal->bias_k * (user_attitude.yaw_rate - gimbal->yaw_bias);
    }
    gimbal->bias_accum += gimbal->yaw_bias * GIMBAL_DT_S;
#endif

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
    /* 减去累积的零偏估计：这样反馈量不会自己漂，位置环不用爬、也能保持满刚度 */
    const float world_yaw = Gimbal_Wrap180(gimbal->imu_yaw - gimbal->bias_accum);
#else
    /* IMU 装在底盘上：云台世界朝向 = 底盘世界朝向 + 云台相对底盘的编码器角 */
    const float world_yaw = Gimbal_Wrap180(gimbal->imu_yaw + gimbal->yaw);
#endif

    /* ★ 串口绘图 ch7：把位置环【本拍实际使用】的 yaw 反馈镜像出去（只读，不参与控制）。
       直接用 world_yaw 这个变量，就是为了保证"画出来的"和"位置环用的"是同一个值、
       同一种算法（含 bias_accum 扣除），不会各算一遍再对不上。
       ★ 它和 ch5 target_yaw / 平滑后的 target_yaw_smooth 都在【世界系】，
         ch7 - ch5 = 真实跟随误差；ch2 的编码器相对角是另一个坐标系，不能这样相减。
       ★ 单位 deg，±180；对齐前（未进位置环）保持上次值/初值 0。 */
    gimbal->yaw_fb = world_yaw;
    /* ★ 串口绘图 ch7 的弧度镜像（只读观测，不参与控制）：与 yaw_fb 同一拍、同一个 world_yaw */
    gimbal->yaw_fb_rad = world_yaw * GIMBAL_DEG2RAD;

    /* ★★★ 目标角一阶平滑（治"调过去晃一下再回来"最有效的一条）--------------------
       摇杆给的是【阶跃式】目标角：松杆那一刻 target_yaw 突然停住/拐弯，位置环看到的是
       一个折角 -> 冲过头再由位置环拉回来。这里让 target_yaw 先过一个一阶低通，
       位置环只追平滑后的值，折角被磨圆，过冲就没了。
       系数按 τ 实时算（不写死魔数）：α = dt/(τ+dt)，dt = GIMBAL_DT_S = 1ms。
       τ 分两套、按 aim_takeover 选：摇杆 tgt_smooth_ms(12ms) / 自瞄 tgt_smooth_aim_ms(40ms)，
       两者都可在线上改；填 0 -> α = 1（等价于关闭平滑）。
       ★ 必须用 Gimbal_Wrap180 求差：target 与 smooth 分别在 ±180 两侧时，
         直接相减会得到 ±360 的假误差，云台会朝反方向绕一整圈。
       ★ 放在这里（而不是 target_yaw 赋值处）：视觉接管分支 / 摇杆分支 / 上电对齐
         都会写 target_yaw，只有这一处能拿到【本拍最终】的目标角。 */
    {
        /* ★ 分模式（用户要求）：摇杆要快（12ms，跟手）、自瞄要稳（40ms，不抖）。
           aim_takeover 在 Gimbal_Aim_Update 里本拍已更新（本函数之前调用），读到的就是最新值。
           两个时间常数都保留在线可调：gimbal_tgt_smooth = RC，gimbal_aim_smooth = AIM。 */
        const float smooth_ms = (gimbal->aim_takeover > 0.5f)
                              ? gimbal->tgt_smooth_aim_ms
                              : gimbal->tgt_smooth_ms;

        if (smooth_ms > 0.0f) {
            const float alpha_s = GIMBAL_DT_S / (smooth_ms * 0.001f + GIMBAL_DT_S);

            gimbal->target_yaw_smooth +=
                alpha_s * Gimbal_Wrap180(gimbal->target_yaw - gimbal->target_yaw_smooth);
            gimbal->target_yaw_smooth = Gimbal_Wrap180(gimbal->target_yaw_smooth);
        } else {
            gimbal->target_yaw_smooth = gimbal->target_yaw;   /* 关闭平滑：直通 */
        }
    }

    /* ★ 位置环的输入用【平滑后】的目标角，不用 target_yaw（否则前面那段平滑等于白做）*/
    float error = Gimbal_Wrap180(gimbal->target_yaw_smooth - world_yaw);

    /* ★ 死区：IMU 的 yaw 是纯陀螺积分、没有绝对参考，静止时会缓慢漂移。
       外环不设死区就会一直去追这个漂移，表现为"停着不动也慢慢抖/慢慢转"。
       死区内把误差清零 —— 只清 P 项，阻尼与前馈仍然工作。
       （pitch 外环早就有同样的死区 pitch_ang_dz，这里保持一致）*/
    if (error < gimbal->world_dead && error > -gimbal->world_dead) {
        error = 0.0f;
    }

    /* 陀螺 yaw 角速度 deg/s -> RPM（/360*60 = /6），先一阶低通压噪声再进阻尼。
       ★ alpha 走结构体字段（gimbal_world_da 可在线上改）：alpha 大 = 微分低通更"快"、
         滞后更小、阻尼更早开始刹车；代价是陀螺噪声更多地进转速指令。 */
    const float gyro_rate_rpm = user_attitude.yaw_rate * (1.0f / 6.0f);
    gimbal->yaw_rate_lpf += gimbal->world_d_alpha * (gyro_rate_rpm - gimbal->yaw_rate_lpf);

    /* deg/s -> RPM（1 RPM = 6 deg/s）= 摇杆速率前馈 + 视觉自瞄角速度前馈。
       ★ 摇杆前馈乘"超喂"增益 stick_kf（>1 起步更利落，前馈不在反馈回路里，不影响稳定性）；
       ★ 自瞄前馈【不】乘 stick_kf —— 参考工程给的 0.5/0.6 就是最终系数，
         再乘 1.3 的摇杆超喂就不是 0.5 了（在线调 aim_ff_gain/aim_ff_pitch 才是正确旋钮）。*/
    const float rate_rpm = (gimbal->target_rate * gimbal->stick_kf + gimbal->aim_rate)
                         * (1.0f / 6.0f);

    /* 位置修正 = 位置环 + 陀螺阻尼 + 底盘前馈（这三项都是"修正量"）*/
    float correction = GIMBAL_YAW_DIR *
                       (gimbal->world_kp * error
                        - gimbal->world_kd * gimbal->yaw_rate_lpf
                        - user_chassis.omega * RAD_S_TO_RPM * gimbal->ff_gain);

    /* ★ 只对【修正量】限幅，摇杆前馈不参与 —— 这是"冲一下再回来、不跟手"的结构性原因：
       原来把两者一起限幅（目标 = 限幅(修正 + 前馈)），而满杆前馈就有
       6 rad/s = 57 RPM，限幅却只有 60 RPM，等于把阻尼和修正的余量全挤没了，
       松杆瞬间前馈掉到 0、阻尼却被限幅卡住 -> 冲过头再被拉回来。
       限幅的真正目的只是拦住跨 ±180° 的一脚踹和 IMU 异常，不是限制正常控制。 */
    if (correction > GIMBAL_WORLD_MAX_RPM) {
        correction = GIMBAL_WORLD_MAX_RPM;
    } else if (correction < -GIMBAL_WORLD_MAX_RPM) {
        correction = -GIMBAL_WORLD_MAX_RPM;
    }

    /* 摇杆前馈单独相加：它是"该以多快转"的命令，不该被修正限幅吃掉 */
    gimbal->target_speed = correction + rate_rpm;

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

    /* ★ 首次角度标定：收到第一帧后把【当前绝对单圈角】换算成关节角零点
       （关节角 = 绝对角 - GIMBAL_PITCH_ABS_TO_JOINT_OFFSET），此后 Get_Angle() 返回的就是
       绝对姿态对应的关节角，软限位也落在绝对行程上（标定这一拍不下发指令）。 */
    if (!gimbal->pitch_is_aligned) {
        if (!gimbal->pitch_is_online) {
            return;
        }
        /* ★ 用【绝对单圈编码器】换算关节零点（关节角 = 绝对角 - 90.5716），
           这样 target_pitch = 9.1403° 永远等于绝对 99.7119°，与上电姿态无关。
           ⚠️ 必须在 Set_Angle_Here 之前读 rotor_angle：它会改 total_angle/零点。
           GM6020 反馈里的 rotor_angle 是 0~8191 的单圈计数 -> 0~360°（与 chassis_control.c
           的 Chassis_Encoder_SingleTurn_Deg 同一换算；那个函数是 static，不能跨文件调用）。
           注：Set_Angle_Here 内部按 8192 计数（≈0.0439°）量化零点，误差 < 1 个计数。 */
        const float abs_deg_before = (float)motor->rotor_angle / 8192.0f * 360.0f;
        DJI_Motor_Set_Angle_Here(motor, abs_deg_before - GIMBAL_PITCH_ABS_TO_JOINT_OFFSET);
        gimbal->pitch = DJI_Motor_Get_Angle(motor);
#if GIMBAL_PITCH_BOOT_TO_FIXED
        /* ★★ 本次上电还没回过位 -> 进入/继续"上电回位"（由下面的状态机每拍强制目标角）。
           ⚠️ 判据从原来的 first_align_after_boot（= pitch_has_ever_online==0 的快照）改成
           pitch_boot_homed==0，这是"只有烧录后第一次上电会回位"的修复关键：
             · pitch_has_ever_online 在第一帧反馈就被置 1（上面"本次在线"分支），而上电初期
               CAN 只要抖一下超过 GIMBAL_OFFLINE_MS，pitch_is_aligned 就会被清掉、随后重新对齐；
               那次重新对齐看到 pitch_has_ever_online==1，于是走 else 把 target_pitch 覆盖成
               【当前角】—— 9.1403 的启动目标只写过一次，被覆盖后就再也没人写回来。
             · pitch_boot_homed 只在【本次上电的回位真正结束】后才置 1（到位/超时/人为接管），
               所以上电初期的掉线/重新对齐不会再取消回位，重连也继续回位到 9.1403。 */
        if (gimbal->pitch_boot_homed == 0) {
            /* ★ 上电主动驱到固定关节角（用户要求"回到绝对编码器 99.7119°"= 关节角 9.1403°）。
               零点已按绝对角换算，所以这个目标与上电姿态无关。
               ⚠️ 安全：若上电位置离该角较远，位置环会在 GIMBAL_PITCH_ANGLE_MAX_OUT（200 RPM）
                  限幅内直接转过去 —— 第一次测试请【手扶枪管】或确认行程内无障碍。 */
            gimbal->target_pitch = GIMBAL_PITCH_BOOT_TARGET_DEG;
            /* 安全检查：必须落在机械限位内，否则退回"保持原位"，避免撞限位 */
            if (gimbal->target_pitch > GIMBAL_PITCH_MAX_DEG) { gimbal->target_pitch = GIMBAL_PITCH_MAX_DEG; }
            if (gimbal->target_pitch < GIMBAL_PITCH_MIN_DEG) { gimbal->target_pitch = GIMBAL_PITCH_MIN_DEG; }
            /* 进入/重启回位：计时清零（重新给 3s），到位防抖计数也清零 */
            gimbal->pitch_boot_homing = 1;
            gimbal->pitch_homing_ms = 0;
            gimbal->pitch_homing_in_pos_ms = 0;
        } else {
            /* ★ 掉线重连/重新标定：保持当前位置，绝不自己跑回固定角 */
            gimbal->target_pitch = gimbal->pitch;
        }
#else
        gimbal->target_pitch = gimbal->pitch;
#endif
        gimbal->pitch_is_aligned = 1;
        return;
    }

    /* 遥控：★ pitch 用【右摇杆上下】（通道可在线改 pitch_ch）——
       推杆 → 目标角以恒定角速度走，松杆即停在当前目标角 */
    float stick = 0.0f;

#if GIMBAL_PITCH_RC_ENABLE
    if (user_dbus.rx_count != gimbal->last_pitch_dbus_count) {
        gimbal->last_pitch_dbus_count = user_dbus.rx_count;
        gimbal->pitch_dbus_offline_ms = 0;
    } else if (gimbal->pitch_dbus_offline_ms < GIMBAL_OFFLINE_MS) {
        gimbal->pitch_dbus_offline_ms++;
    }

    /* 遥控断线就把杆量当 0：目标角停在原地，不会乱动 */
    if (gimbal->pitch_dbus_offline_ms < GIMBAL_OFFLINE_MS) {
        stick = Gimbal_Stick(Gimbal_RC_Channel((uint8_t)gimbal->pitch_rc_ch), gimbal->pitch_dead);
    }
#endif

    gimbal->pitch_stick = stick;

    /* ★★★ 视觉自瞄接管 pitch（覆盖式，与 yaw 完全对称）----------------------------
       参考工程 ita_robot.cpp:721-733：使能 && 在线 && mode!=0 时
         "视觉下，覆盖设置云台的目标角度"：目标角 = 视觉给的世界绝对角（不是叠加到摇杆积分上），
         速度前馈   = 系数(0.6) × 视觉目标角速度 -> 叠加到【速度环】的目标上。
       ★ 与 yaw 的差别：pitch 这里【没有】"摇杆积分被覆盖"的问题，因为目标本身就是给角度，
         所以直接覆盖 target_pitch 即可；数据有效性/野值拦截已经在 Gimbal_Aim_Update 里做完。
       ★ 优先级：自瞄接管 > 弹道解算(pitch_auto) > 摇杆。参考工程也是视觉覆盖在最后。
       ★ 开火链路先不做：接法见参考 ita_robot.cpp:424-434（在线 && mode==CONTROL_AND_FIRE
         且左键按下 -> 请求开火），本工程还没有确认扳机/使能通道，不要自己发明通道号。 */
    if (gimbal->aim_takeover > 0.5f) {
        /* 世界绝对角 + 延迟外推：视觉看到的是【过去】，用云台自身角速度外推到"现在"
           （pitch_speed 是电机转速 RPM，×6 = deg/s；1 RPM = 6 deg/s）*/
        gimbal->target_pitch = gimbal->pitch_target_ext
                             + gimbal->pitch_speed * 6.0f
                               * (gimbal->aim_delay_ms * 0.001f);
        gimbal->pitch_aim_err = gimbal->pitch_target_ext - gimbal->pitch;
    } else if (gimbal->pitch_auto < 0.5f) {
        /* pitch_auto != 0 时目标角由上层（弹道解算）给，摇杆不参与，避免互相打架 */
        gimbal->target_pitch += stick * gimbal->pitch_max_w * GIMBAL_PITCH_RC_DIR * GIMBAL_DT_S;
    }

#if GIMBAL_PITCH_BOOT_TO_FIXED
    /* ★★ 上电回位（boot homing）状态机 ----------------------------------------------
       位置：视觉/弹道/摇杆分支【之后】、软限位与下发【之前】。所以这一拍再没有任何分支能
       在它之后改 target_pitch —— 回位期间目标角的写入优先级最高（摇杆积分、视觉目标、
       弹道解算写的值都会被这一拍覆盖掉）。
       退出条件（任一满足即交回正常控制，之后不再进入 homing）：
         ① 到位：|实际角 - GIMBAL_PITCH_BOOT_TARGET_DEG| <= 0.5° 且连续保持 120ms；
                  （保持是为了防"冲过头路过目标"被误判成到位）
         ② 人为动作：摇杆出了死区（Gimbal_Stick 在死区内返回 0，所以 stick != 0 即"有意推杆"），
                  或视觉自瞄已经接管 pitch —— 操作手/上层一要控制就立刻交权，绝不和用户抢；
         ③ 超时 3s：堵转、机械卡住、目标进不去时不能永远霸占目标角。
       ★ 与掉线重连的关系：回位【结束】才置 pitch_boot_homed=1，所以之后的重连保持原位
         （与原来一致）；回位【进行中】遇到掉线则保持 pitch_boot_homing，重新对齐后继续回位，
         不会因为上电初期一次 CAN 抖动就把回位取消掉。 */
    if (gimbal->pitch_boot_homing) {
        if (stick != 0.0f || gimbal->aim_takeover > 0.5f) {
            /* ② 人为动作 / 视觉接管：立即交权。本拍上面分支写好的目标角保持不动 */
            gimbal->pitch_boot_homing = 0;
            gimbal->pitch_boot_homed = 1;
        } else if (gimbal->pitch_homing_ms >= GIMBAL_PITCH_BOOT_HOMING_TIMEOUT_MS) {
            /* ③ 超时兜底 */
            gimbal->pitch_boot_homing = 0;
            gimbal->pitch_boot_homed = 1;
        } else {
            gimbal->pitch_homing_ms++;      /* 本任务固定 1kHz，+1 = +1ms */
            /* 强制目标角。9.1403 在软限位内，下面的软限位夹紧对它不起作用 */
            gimbal->target_pitch = GIMBAL_PITCH_BOOT_TARGET_DEG;

            /* ① 到位判定：实际角进带并保持住 -> 退出。目标角就留在 9.1403，交回正常控制 */
            if (fabsf(gimbal->pitch - GIMBAL_PITCH_BOOT_TARGET_DEG)
                <= GIMBAL_PITCH_BOOT_HOMING_TOL_DEG) {
                if (gimbal->pitch_homing_in_pos_ms < GIMBAL_PITCH_BOOT_HOMING_HOLD_MS) {
                    gimbal->pitch_homing_in_pos_ms++;
                }
                if (gimbal->pitch_homing_in_pos_ms >= GIMBAL_PITCH_BOOT_HOMING_HOLD_MS) {
                    gimbal->pitch_boot_homing = 0;
                    gimbal->pitch_boot_homed = 1;
                }
            } else {
                gimbal->pitch_homing_in_pos_ms = 0;
            }
        }
    }
#endif

    /* 软限位：夹住目标角；目标固定后误差不再增大、积分也不再涨，等于顺带做了抗积分饱和。
       ★ 自瞄目标在闸门里已经用同一对 MIN/MAX（留 1° 余量）判过有效性，这里是最终夹紧。
       ★ 位置：摇杆积分 / 视觉接管 / 弹道(pitch_auto) / 上电回位 全部写完之后、下发之前；
         调用唯一实现 Gimbal_Clamp_Pitch_Target()，与外部写入点共用同一份限位逻辑。 */
    Gimbal_Clamp_Pitch_Target(gimbal);

    /* ★★ 目标速度前馈 KF 分模式（用户要求：摇杆要有前馈/跟手，自瞄不要前馈/别抖）----------
       摇杆给的目标角是【连续斜坡】（每 1ms += rate*dt），KF 把目标角速度前馈进外环，
       消掉"先攒误差才有转速"的滞后 -> 推杆跟手；
       视觉接管时目标角是【每帧阶跃刷新】的（~100Hz），KF 对"目标角差分"会每帧打一个
       尖峰（fout = kf × Δtarget / 1ms），表现就是 pitch 跟着每一帧抖一下 —— 所以接管时置 0，
       真正需要的目标角速度由视觉通过 ff_ext 单独给（aim_ff_pitch，默认也是 0）。
       ⚠️ PID_Adv_Set_K_F 的实现就是 pid->kf = kf; （user_pid_adv.c:235），纯字段写、
          无副作用、无额外开销，放在 1kHz 周期里每拍写一次没问题。
       ⚠️ 唯一副作用：在线命令 pitch_ang_kf 改的值会在下一拍被这里覆盖，等于失效；
          现在要改摇杆 KF 请改宏 GIMBAL_PITCH_ANGLE_KF 后重新烧录。 */
    PID_Adv_Set_K_F(&gimbal->pitch_pid_angle,
                    (gimbal->aim_takeover > 0.5f) ? 0.0f : GIMBAL_PITCH_ANGLE_KF);

    /* ★★ 视觉角速度前馈 -> 【速度环目标】--------------------------------------------
       参考工程走的是 Motor_Pitch.Set_Feedforward_Omega(0.6 × 视觉角速度)
       （ita_robot.cpp:727/731）。本工程 pitch 是【外环角度(deg->RPM) + 内环速度(RPM->电流)】
       的串级，外环的输出就是内环（速度环）的目标 —— 因此把前馈(deg/s -> RPM)
       加到外环输出上（PID_Adv 的 ff_ext）就等价于"叠加到速度环的目标上"。
       ⚠️ 摇杆模式必须写回 0，否则会一直带着上一次自瞄的残留前馈。
       ⚠️ 外环自带的 pitch_ang_kf 是把【目标角差分】当目标速度用的；视觉每帧是阶跃更新，
          会因此产生一个每帧一次的脉冲 —— 这个问题已由上面"自瞄接管时把 KF 置 0"解决；
          真正的目标角速度由视觉通过 ff_ext 提供（aim_ff_pitch，默认 0）。 */
    PID_Adv_Set_Feedforward(&gimbal->pitch_pid_angle,
                            (gimbal->aim_takeover > 0.5f)
                            ? (gimbal->pitch_aim_rate * (1.0f / 6.0f)) : 0.0f);

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

/* ================= 视觉自瞄：数据闸门 + 目标解算（安全关键） =================
   为什么集中放一个函数：视觉一包同时带 yaw 和 pitch，两轴必须共用同一帧的
   "有效/无效"结论；而且放行只能有【一处】，才能保证"拒绝时绝不把野值写进目标变量"。

   六道闸（阈值都是 gimbal.h 里的 GIMBAL_AIM_* 宏，单位度，方便现场调参）：
     ① NaN/Inf      —— 先于一切算术；否则后面比较全部失效、Wrap180 的 float->int 是 UB
     ② 不接管       —— 未使能 / 离线 / mode==NO_CONTROL：回摇杆，一个目标变量都不写
     ③ yaw 量程     —— 归一化后与当前世界角偏差 > GIMBAL_AIM_MAX_ERR_DEG -> 无效
     ④ pitch 量程   —— 超出现有机械软限位（留 GIMBAL_AIM_PITCH_MARGIN_DEG）-> 无效
     ⑤ 跳变门限     —— 与上一帧有效目标角差 > GIMBAL_AIM_JUMP_DEG -> 本帧无效
     ⑥ 连续无效计数 —— 连续 GIMBAL_AIM_INVALID_LIMIT 帧无效 -> 退出接管、回摇杆
   前因后果与实测野值例子见 gimbal.h 里 GIMBAL_AIM_MAX_ERR_DEG 那一段注释。 */

/* 放行一帧视觉数据：返回 1 = 六道闸全过，并且已经把目标角/前馈写进结构体；
   返回 0 = 本帧无效（调用者负责计数、保持上一帧目标、必要时退出接管）。*/
static uint8_t Gimbal_Aim_Accept_Frame(GIMBAL_DRIVES* gimbal) {
    /* 闸① NaN / Inf —— 必须放在换算与 Gimbal_Wrap180 之前 */
    const float raw_yaw     = user_vision.target_yaw;
    const float raw_yaw_vel = user_vision.target_yaw_vel;
    const float raw_pitch   = user_vision.target_pitch;
    const float raw_pitch_vel = user_vision.target_pitch_vel;

    if (isnan(raw_yaw) || isinf(raw_yaw)
        || isnan(raw_yaw_vel) || isinf(raw_yaw_vel)
        || isnan(raw_pitch) || isinf(raw_pitch)
        || isnan(raw_pitch_vel) || isinf(raw_pitch_vel)) {
        return 0;
    }

    /* 单位换算：包内是【弧度】（与参考工程一致，见 user_vision.h），工程内部是【度】 */
    const float yaw_ext = Gimbal_Wrap180(raw_yaw * VISION_PKG2DEG + gimbal->cam_yaw_off);
    const float pitch_ext = raw_pitch * VISION_PKG2DEG + gimbal->cam_pitch_off;
    /* TODO(需实测标定): 视觉给的 pitch 与工程内部的 pitch（上电标定后的【关节角】）之间
       是否存在随姿态变化的差？参考工程是直接把 Get_Target_Pitch() 当关节角目标用
       （ita_robot.cpp:725），本工程照做；若实测发现"底盘俯仰变化后 pitch 目标整体偏了"，
       就在这里加/减 user_attitude.pitch（或用 cam_pitch_off 补掉那个常量差）。
       注意本工程发给视觉的 pitch 只含 IMU 俯仰（VISION_PITCH_USE_IMU_ONLY=1），
       这个约定要和视觉侧对齐。 */

    /* 闸③ yaw 量程：视觉给的是世界绝对角，正常跟踪时与云台当前世界角很接近；
       差得离谱说明这一帧不是可信目标（例如 197.7° 这种 tracker 丢失时的输出）。
       ⚠️ 单靠这道闸拦不住 -9518.6°（归一化后是 -158.6°，可能刚好落在范围内），
          所以必须配合闸⑤的跳变门限一起用。 */
    if (fabsf(Gimbal_Wrap180(yaw_ext - gimbal->imu_yaw)) > GIMBAL_AIM_MAX_ERR_DEG) {
        return 0;
    }

    /* 闸④ pitch 量程：必须落在【工程已有】的机械软限位内（留一点余量兼容标定误差）*/
    if (pitch_ext > (GIMBAL_PITCH_MAX_DEG + GIMBAL_AIM_PITCH_MARGIN_DEG)
        || pitch_ext < (GIMBAL_PITCH_MIN_DEG - GIMBAL_AIM_PITCH_MARGIN_DEG)) {
        return 0;
    }

    /* 闸⑤ 跳变门限：和上一帧【有效】目标角比。跨 ±180° 求差要先用 Wrap180，
       否则目标从 +179 走到 -179（实际只动 2°）会被误判成 358° 的大跳变。 */
    if (fabsf(Gimbal_Wrap180(yaw_ext - gimbal->yaw_target_ext)) > GIMBAL_AIM_JUMP_DEG
        || fabsf(pitch_ext - gimbal->pitch_target_ext) > GIMBAL_AIM_JUMP_DEG) {
        return 0;
    }

    /* 全部通过：这是【唯一】写视觉目标的地方（拒绝路径绝不写，见函数头说明）*/
    gimbal->yaw_target_ext   = yaw_ext;
    gimbal->pitch_target_ext = pitch_ext;
    /* 角速度前馈：系数 × 视觉目标角速度（参考 0.5 / 0.6），到速度环前再 /6 换算成 RPM */
    gimbal->aim_rate       = raw_yaw_vel * VISION_PKG2DEG * gimbal->aim_ff_gain;
    gimbal->pitch_aim_rate = raw_pitch_vel * VISION_PKG2DEG * gimbal->aim_ff_gain_pitch;
    return 1;
}

/* 本拍不接管，但保留操作手的使能开关：离线 / mode==NO_CONTROL 时用（回摇杆控制）。
   ★ 不清 aim_ms：离线超时（aim_timeout_ms）正是靠它判"视觉是真的没了"。 */
static void Gimbal_Aim_Release(GIMBAL_DRIVES* gimbal) {
    gimbal->aim_takeover = 0.0f;
    gimbal->aim_valid = 0;
    gimbal->pitch_aim_valid = 0;
    gimbal->aim_rate = 0.0f;
    gimbal->pitch_aim_rate = 0.0f;
    gimbal->aim_ok_f = 0.0f;
}

/* 退出接管：放掉使能开关、回摇杆控制，要求操作手重新打开。
   ★ 不自动恢复是故意的：野值/丢失通常是持续性的（目标真的没了、或跟踪错了），
     自动恢复会形成"接管→甩一下→退出→再接管"的抖动，比干脆交还给操作手更危险。
     排错看 aim_reject_cnt（累计拒绝帧数）与 aim_bad_frames（连续无效帧数）。*/
static void Gimbal_Aim_Abort(GIMBAL_DRIVES* gimbal) {
    Gimbal_Aim_Release(gimbal);
    gimbal->yaw_auto = 0.0f;
    gimbal->aim_bad_frames = 0;
}

static void Gimbal_Aim_Update(GIMBAL_DRIVES* gimbal) {
    const uint32_t now = HAL_GetTick();

    /* 闸②-a 未使能：视觉一概不参与，摇杆/编码器控制保持原样 */
    if (gimbal->yaw_auto < 0.5f) {
        Gimbal_Aim_Release(gimbal);
        gimbal->aim_bad_frames = 0;
        gimbal->last_vision_frame = user_vision.frame_count;  /* 关着的时候不累计"无效帧" */
        gimbal->yaw_auto_pre = gimbal->yaw_auto;
        return;
    }

    /* 使能【刚打开】这一拍：给"最近一次有效帧时刻"打起点。
       ★ 必须放在这里、而不是下面那个"接管瞬间"里：离线时每拍都会走"接管瞬间"
         （因为上一拍被 Release 掉了），时间戳就会被一直刷新 ->
         "离线超过 aim_timeout_ms 就退出接管"这条永远触发不了。 */
    if (gimbal->yaw_auto_pre < 0.5f) {
        gimbal->aim_ms = now;
    }
    gimbal->yaw_auto_pre = gimbal->yaw_auto;

    /* 接管瞬间（从摇杆/离线/mode==0 切回视觉控制）：目标角从【当前角度】起。
       不这么做，第一次接管会朝一个陈旧目标猛甩（撞软限位、丢目标）。 */
    if (gimbal->aim_takeover < 0.5f) {
        gimbal->yaw_target_ext = gimbal->imu_yaw;
        gimbal->pitch_target_ext = gimbal->pitch;
        gimbal->aim_bad_frames = 0;
        gimbal->aim_rate = 0.0f;
        gimbal->pitch_aim_rate = 0.0f;
    }

    /* 闸②-b 视觉离线：不接管，回摇杆（保持原来的摇杆/编码器控制）。
       短丢帧（< aim_timeout_ms）只是这一拍交还摇杆，云台目标不会跳（接管时重新以
       当前角度为起点）；连续超过 aim_timeout_ms 仍无有效帧 -> 判"视觉真的没了"，退出接管。*/
    if (Vision_Is_Online(&user_vision) == 0) {
        Gimbal_Aim_Release(gimbal);
        if ((uint32_t)(now - gimbal->aim_ms) > (uint32_t)gimbal->aim_timeout_ms) {
            Gimbal_Aim_Abort(gimbal);
        }
        return;
    }

    /* 闸②-c mode == NO_CONTROL：视觉主动要求"别控制"，同样交还摇杆。
       ★ 这里【不】动 yaw_auto：操作手的使能开关不能被视觉的一帧悄悄改掉。 */
    if (Vision_Get_Rx_Mode(&user_vision) == VISION_RX_MODE_NO_CONTROL) {
        Gimbal_Aim_Release(gimbal);
        gimbal->last_vision_frame = user_vision.frame_count;
        return;
    }

    /* 接管条件齐了：使能 && 在线 && mode != NO_CONTROL（参考 ita_robot.cpp:639 同款）*/
    gimbal->aim_takeover = 1.0f;

    /* ★ 只有【新的一帧】才做校验与计数：控制任务是 1kHz，视觉约 100Hz，
       否则一帧之间会被误判成"连续多帧无效"而误退出。没有新帧就沿用上一帧的结论
       （目标角零阶保持，Gimbal_Pitch_Update / yaw 那两处都会用它）。 */
    if (gimbal->last_vision_frame == user_vision.frame_count) {
        return;
    }
    gimbal->last_vision_frame = user_vision.frame_count;

    if (Gimbal_Aim_Accept_Frame(gimbal) != 0) {
        gimbal->aim_valid = 1;
        gimbal->pitch_aim_valid = 1;
        gimbal->aim_bad_frames = 0;
        gimbal->aim_ms = now;
        gimbal->aim_ok_f = 1.0f;
        return;
    }

    /* 本帧被闸门拒绝：保持上一帧的有效目标（绝不写野值），前馈立刻清零 */
    gimbal->aim_reject_cnt++;
    if (gimbal->aim_bad_frames < 0xFFFFU) {
        gimbal->aim_bad_frames++;
    }
    gimbal->aim_valid = 0;
    gimbal->pitch_aim_valid = 0;
    gimbal->aim_rate = 0.0f;
    gimbal->pitch_aim_rate = 0.0f;
    gimbal->aim_ok_f = 0.0f;

    /* 闸⑥ 连续若干帧都无效 -> 退出接管、回摇杆控制（要求操作手重新打开自瞄）*/
    if (gimbal->aim_bad_frames >= GIMBAL_AIM_INVALID_LIMIT) {
        Gimbal_Aim_Abort(gimbal);
    }
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

/* 摇杆归一化到 -1~1，带死区（yaw 与 pitch 共用；两个轴死区可各自在线调）*/
static float Gimbal_Stick(const int16_t value, const float dead) {
    if (value > -dead && value < dead) {
        return 0.0f;
    }
    return (float)value / YAW_STICK_MAX;
}
