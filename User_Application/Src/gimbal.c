#include "../../Core/Inc/bsp.h"
#include "../gimbal.h"
#include <math.h>       /* isnan / isinf：视觉数据有效性闸门要用（工程可用标准 math.h）*/

#define YAW_STICK_MAX   (660.0f)
#define RAD_S_TO_RPM    (9.54929659f)   /* 60 / 2pi */
#define RAD_TO_DEG      (57.29578f)
#define GIMBAL_DT_S     (0.001f)        /* Gimbal_Update 跑在 1kHz */

static GIMBAL_DRIVES* s_gimbal = NULL;

/* ★ pitch 自瞄目标角的一阶平滑状态（与 yaw 的 gimbal->target_yaw_smooth 对称）。
   gimbal.h 的约束：只允许在 gimbal.c 里改，不为它新增结构体字段，所以放文件静态。
   只有 aim_takeover 分支使用/推进它，摇杆路径完全不碰（aim 为假时行为一个字不变）。
   在 Gimbal_Pitch_Init 里清零，在接管瞬间（Gimbal_Aim_Update）对齐到当前角度。 */
static float s_pitch_cmd_smooth = 0.0f;

static void  Gimbal_Update(void* arg);
static void  Gimbal_Pitch_Update(GIMBAL_DRIVES* gimbal);
static void  Gimbal_Aim_Update(GIMBAL_DRIVES* gimbal);
static uint8_t Gimbal_Aim_Accept_Frame(GIMBAL_DRIVES* gimbal);
static void  Gimbal_Aim_Release(GIMBAL_DRIVES* gimbal);
static void  Gimbal_Aim_Abort(GIMBAL_DRIVES* gimbal);
static float Gimbal_Wrap180(float deg);
static float Gimbal_Stick(int16_t value, float dead);
static int16_t Gimbal_RC_Channel(uint8_t ch);
#if GIMBAL_LOCK_WORLD && GIMBAL_IMU_ON_GIMBAL
static void  Gimbal_Calib_Update(GIMBAL_DRIVES* gimbal, float stick);
static void  Gimbal_Calib_Clear_Window(GIMBAL_DRIVES* gimbal);
#endif

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

    /* 上电静态零偏标定（见 gimbal.h 的 GIMBAL_CALIB_* 和本文件的 Gimbal_Calib_Update）：
       状态机从 IDLE 起，每次上电只成功标定一次；calib_restart=1 可随时重来。 */
    gimbal->calib_state = GIMBAL_CALIB_IDLE;
    gimbal->calib_ms = 0U;
    gimbal->calib_gyro_avg = 0.0f;
    gimbal->calib_gyro_pp = 0.0f;
    gimbal->calib_gyro_std = 0.0f;
    gimbal->calib_ms_cfg = (float)GIMBAL_CALIB_MS;
    gimbal->calib_restart = 0.0f;
    gimbal->calib_target_ms = (uint16_t)GIMBAL_CALIB_MS;
    gimbal->calib_timeout_ms = 0U;
    gimbal->calib_stable_ms = 0U;
    gimbal->calib_n = 0U;
    gimbal->calib_sum = 0.0f;
    gimbal->calib_sum2 = 0.0f;
    gimbal->calib_min = 0.0f;
    gimbal->calib_max = 0.0f;
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
    gimbal->aim_need_ok = 0;
    gimbal->aim_need_ok_f = 0.0f;      /* aim_need_ok 的 float 绘图镜像（只读观测）*/
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
    gimbal->sw1_f      = 0.0f;    /* ch2 拨杆档位镜像（只读观测，见 gimbal.h）*/
    gimbal->sw2_f      = 0.0f;    /* ch5 拨杆档位镜像（只读观测，见 gimbal.h）*/
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
    s_pitch_cmd_smooth = 0.0f;   /* pitch 自瞄目标平滑状态（文件静态，只服务 aim 分支）*/
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

    /* ★★ 拨杆原始档位只读镜像（1kHz，给串口绘图 ch2/ch5）：
       ① user_dbus.sw1 / sw2 是 uint8_t，而 Serialplot_Set_Data() 按 sizeof(float)
          逐 4 字节 memcpy，直接把它们的地址传进去会读到相邻字段 -> 图上 NaN/1e38，
          所以这里每拍同步成 float；
       ② 只读观测：不解析、不判断、不参与任何控制，档位语义的唯一使用者仍是 shooter.c；
       ③ 放在所有 early-return 之前，云台掉线/未对齐时也照常刷新。 */
    gimbal->roll_f = (float)user_dbus.roll;
    gimbal->sw1_f = (float)user_dbus.sw1;
    gimbal->sw2_f = (float)user_dbus.sw2;

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
    /* ★ 串口绘图 ch6 的只读镜像（aim_need_ok 是 uint8_t，绘图层按 float 取 4 字节，
       直接绑它的地址会读到相邻填充字节；这里每拍同步成 float）。不参与任何控制运算。*/
    gimbal->aim_need_ok_f = (float)gimbal->aim_need_ok;

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
    if (gimbal->aim_takeover > 0.5f) {
        /* ★★★ 自瞄接管：目标角 100% 来自视觉，摇杆完全不参与。
           参考 ita_robot.cpp:639-650 —— "视觉下，覆盖设置云台的目标角度"（目标角 = 当前姿态 + 视觉偏差，见 Gimbal_Aim_Accept_Frame）。 */
        /* ★ 硬约束（不是"把某个增益设 0"）：本拍先把摇杆角速度前馈显式清零，
           下面只允许由 yaw_target_ext 写 target_yaw；这个分支里不再出现 stick / rate_cmd，
           所以摇杆怎么推都进不了位置环（target_rate 是速度前馈里唯一的摇杆项）。*/
        gimbal->target_rate = 0.0f;      /* 摇杆积分 + stick_kf 前馈一并停用 */
        if (gimbal->aim_valid != 0) {
            /* ① 接管瞬间的目标初始化在 Gimbal_Aim_Update 里做（目标角从当前角度起），
                  否则云台会朝一个陈旧目标猛甩一下（撞软限位、丢目标）*/
            /* ② 延迟补偿：视觉看到的是【过去】，用云台自身角速度外推到"现在"。
                  age 用 aim_delay_ms 估计（需实测）；姿态漂移在几十毫秒内可忽略。
                  ★ target_yaw 只由【视觉目标 + 外推】得到，不叠加任何摇杆量 */
            gimbal->target_yaw = Gimbal_Wrap180(gimbal->yaw_target_ext
                                 + gimbal->yaw_rate_lpf * 6.0f
                                   * (gimbal->aim_delay_ms * 0.001f));
        } else {
            /* ③ 丢帧冻结：视觉数据暂时不可用时【冻结】目标角 ——
                  既不继续外推（会越推越偏），也不回中（会让操作手措手不及）；
                  角速度前馈同时清零，不许带着上一帧的速度继续冲 */
            gimbal->aim_rate = 0.0f;
        }
        gimbal->aim_err = Gimbal_Wrap180(gimbal->yaw_target_ext - gimbal->imu_yaw);
        gimbal->aim_ok_f = (float)gimbal->aim_valid;
    } else {
    /* ★ 只有摇杆模式才计算/使用摇杆角速度：自瞄分支里连 rate_cmd 都不存在 */
    const float rate_cmd = stick * gimbal->max_w * RAD_TO_DEG * GIMBAL_YAW_DIR;
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
    /* ★★★ TX pitch 字段的语义（2026-10-26 第三轮修正：和 yaw 对齐，协议语义有变！）---
       以前这里发 user_attitude.qe_roll（IMU / yaw 框架的世界俯仰），而【控制用】的是
       gimbal->pitch（编码器关节角）。两者不是同一个量：
         · qe_roll 里没有 pitch 电机的关节角（IMU 装在 yaw 框架上，看不到枪管自己的俯仰）；
         · gimbal->pitch 里有 9.1403° 的装配零点。
       视觉拿 qe_roll 当"当前枪管姿态"算偏差，我们却把偏差加在关节角上 —— 这条链天生
       差一个"yaw 框架俯仰 + 9.14 零点"，而且这个差随关节角/姿态变化（不是常数），
       表现就是 pitch 目标来回游走、不收敛。yaw 没有这个问题：发出去的 yaw
       = imu_yaw − bias_accum，正是控制用的 yaw_fb，收发同一个量。

       现在 pitch 也发【控制用的那个量】，和 yaw 完全对称：
           yaw   TX = yaw_fb    = wrap(imu_yaw − bias_accum)   （世界角，上电≈0）
           pitch TX = pitch_fb  = gimbal->pitch − 9.1403       （关节角，上电=0）
       ★ 视觉侧会看到什么变化（必须转告视觉组）：
         · q[4] 不变，仍然只描述【yaw 框架】的姿态（不含 pitch 电机）；
         · pitch 字段从"IMU 世界俯仰 qe_roll"变成"枪管相对 yaw 框架/上电姿态的俯仰"，
           零点 = 上电时的枪管姿态（= 水平，关节角 9.1403°）。做三维变换时正确的组合是
           R_barrel = R_q * R_pitch(pitch)，不能把 pitch 再当世界角用；
         · 单位仍是 rad（VISION_ANGLE_UNIT_RAD=1）。
       ★ VISION_PITCH_USE_IMU_ONLY 在本字段上【作废】：宏还留在 user_vision.h 里（本轮
         不动那个文件），但 gimbal.c 不再按它分支，今后也不要再按它理解这个字段。
       ⚠️ cam_pitch_off(+3) 是在旧语义下标定的，改完必须重标（见报告）。
       ⚠️ pitch_vel 不用改：d(pitch_fb)/dt ≡ d(gimbal->pitch)/dt（常量 9.1403 的导数为 0），
          下面 pitch_speed × 6 那套换算仍然正确。 */
    const float tx_pitch = gimbal->pitch - GIMBAL_PITCH_BOOT_TARGET_DEG;
    gimbal->tx_pitch_deg = tx_pitch;   /* ★只读镜像：这就是 TX 真正发出去的 pitch(度) */

    /* ★ 回传的工作模式用【使能开关 yaw_auto】而不是 aim_takeover：
       这是和视觉的握手 —— 只有先告诉视觉"AUTOAIM"，它才会开始发 mode=1/2 的控制包；
       若这里改成看 aim_takeover（依赖 mode!=0），就会形成"等对方先动"的死锁。 */
    Vision_Set_Tx_Mode(&user_vision,
                       (gimbal->yaw_auto > 0.5f) ? VISION_TX_MODE_AUTOAIM : VISION_TX_MODE_IDLE);
    /* ★ yaw 发【扣掉零偏后】的值(bias_accum 跟踪到的零偏)，而不是未修正的 imu_yaw：
       这样视觉的世界系跟的是和闭环同一个 yaw_fb，漂移速度一致（原来发 imu_yaw 会漂得更快）*/
    /* ★ 发给视觉的四元数也要和 yaw 字段【同一个基准】：把 q 绕世界 Z 轴旋转 -bias_accum，
       否则 q 里含着未修正的 yaw 漂移，视觉用它定坐标系就漂得更快。
       q_rot=(c,0,0,-s)，c=cos(b/2)、s=sin(b/2)、b=bias_accum(wrap到±180)；q_out=q_rot⊗q_in。 */
    {
        const float b_half = 0.5f * Gimbal_Wrap180(gimbal->bias_accum) * GIMBAL_DEG2RAD;
        const float qc = cosf(b_half);
        const float qs = sinf(b_half);
        const float qw0 = user_attitude.qe_q[0];
        const float qx0 = user_attitude.qe_q[1];
        const float qy0 = user_attitude.qe_q[2];
        const float qz0 = user_attitude.qe_q[3];
        gimbal->tx_qw = qc * qw0 + qs * qz0;
        gimbal->tx_qx = qc * qx0 + qs * qy0;
        gimbal->tx_qy = qc * qy0 - qs * qx0;
        gimbal->tx_qz = qc * qz0 - qs * qw0;
    }

    Vision_Set_Tx_Attitude(&user_vision,
        gimbal->tx_qw, gimbal->tx_qx,
        gimbal->tx_qy, gimbal->tx_qz,
                           /* ★ 按包内单位换算（VISION_ANGLE_UNIT_RAD=1 时 -> 弧度/弧度每秒，与参考工程一致）*/
                           Gimbal_Wrap180(gimbal->imu_yaw - gimbal->bias_accum) * VISION_DEG2PKG,
                           (user_attitude.yaw_rate - gimbal->yaw_bias) * VISION_DEG2PKG,   /* ★ 扣零偏，和 yaw 字段同基准 */
                           tx_pitch * VISION_DEG2PKG,   /* = pitch_fb（上电=0 的关节角，rad）*/
                           gimbal->pitch_speed * 6.0f * VISION_DEG2PKG);  /* = d(pitch_fb)/dt，与关节角速度相同 */
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

    /* ★★ 上电静态零偏标定：放在在线跟踪 + 积分【之后】，这样标定完成的那一拍
       可以把 bias_accum 精确清 0（紧接着的下一拍才重新开始积分）。
       ★ 它只写 yaw_bias / bias_accum 和 calib_* 字段，不碰 PID / 限位 / 回位 / 视觉参数。 */
    Gimbal_Calib_Update(gimbal, stick);
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

/* ================= ★★ 上电静态零偏标定（yaw） =================
   状态机（每拍在 1kHz 的 Gimbal_Update 里调用一次，不新建任务）：

     IDLE --(user_heater 预热到 47℃ / 或加热被关掉)--> WAIT
     WAIT --(静止条件连续满足 GIMBAL_CALIB_STABLE_MS)--> COLLECTING
     COLLECTING --(采够 calib_ms)--> DONE：yaw_bias = 平均零偏(deg/s)，bias_accum = 0
     COLLECTING --(静止条件被破坏 / 云台掉线)--> WAIT：清空累加，重新等 200ms
     WAIT/COLLECTING --(从加热就绪起累计 10s)--> TIMEOUT：放弃，在线跟踪兜底

   ★ 进入采样的前置条件（全部满足）：
     ① 加热：user_heater.preheat_done != 0（预热到 47℃ 以上）；若 user_heater.enabled == 0
        （台架关加热）则跳过这一条；
     ② 云台：gimbal->is_online && gimbal->is_aligned（yaw GM6020 在线且已对齐）；
     ③ 传感器：BMI088 在线且它自己的上电 1000 点标定已完成（否则 gyro[] 还没更新，采到的是 0）；
     ④ 静止：stick == 0、|enc_rate_lpf| < GIMBAL_BIAS_ENC_MAX_RATE、|gyro_z| < GIMBAL_CALIB_GYRO_MAX_DPS。
   ★ 单位：gyro_z 是 rad/s，乘 GIMBAL_RAD2DEG 变成 deg/s 再累加/写入 yaw_bias；
     yaw_bias(deg/s) × 0.001 = bias_accum(deg)，位置环用 wrap(imu_yaw(deg) − bias_accum(deg))。
   ★ bias_k 不关：标定只给它一个更准的起点，之后仍按原来的"摇杆居中 + 编码器没动"在线微调。 */
#if GIMBAL_LOCK_WORLD && GIMBAL_IMU_ON_GIMBAL

/* 清空"这一次采样"的累加量（不动 calib_state / calib_timeout_ms / calib_ms_cfg）*/
static void Gimbal_Calib_Clear_Window(GIMBAL_DRIVES* gimbal) {
    gimbal->calib_ms = 0U;
    gimbal->calib_stable_ms = 0U;
    gimbal->calib_n = 0U;
    gimbal->calib_sum = 0.0f;
    gimbal->calib_sum2 = 0.0f;
    gimbal->calib_min = 0.0f;
    gimbal->calib_max = 0.0f;
}

static void Gimbal_Calib_Update(GIMBAL_DRIVES* gimbal, const float stick) {
    /* ① 加热条件：预热完成，或人为关掉加热（台架调试直接允许标定）*/
    const uint8_t heater_ok = (user_heater.enabled == 0.0f) || (user_heater.preheat_done != 0);

    /* ② 云台在线且已对齐；③ IMU 在线且完成它自己的上电标定（见 user_bmi088.c）*/
    const uint8_t gimbal_ok = (gimbal->is_online != 0U) && (gimbal->is_aligned != 0U);
    const uint8_t sensor_ok = (user_bmi088.is_online != 0U) && (user_bmi088.calibrated != 0U);

    /* ④ 静止：摇杆居中 + 编码器角速度小 + 陀螺角速度小。
       ★ gyro_z 先换成 deg/s 再和阈值比：阈值宏是按 deg/s 标定的。 */
    const float gz_dps = user_bmi088.gyro[2] * GIMBAL_RAD2DEG;
    const uint8_t still = (stick == 0.0f)
                       && (fabsf(gimbal->enc_rate_lpf) < GIMBAL_BIAS_ENC_MAX_RATE)
                       && (fabsf(gz_dps) < GIMBAL_CALIB_GYRO_MAX_DPS);
    const uint8_t ready = (heater_ok != 0U) && (gimbal_ok != 0U)
                       && (sensor_ok != 0U) && (still != 0U);

    /* ---- 在线调参 calib_restart：写 1 触发重新标定（内部消费后清零）----
       不清 yaw_bias / bias_accum：新标定成功前，继续用旧的零偏和在线跟踪，绝不突然失去修正。 */
    if (gimbal->calib_restart > 0.5f) {
        gimbal->calib_restart = 0.0f;
        gimbal->calib_state = GIMBAL_CALIB_IDLE;
        gimbal->calib_timeout_ms = 0U;
        gimbal->calib_gyro_avg = 0.0f;
        gimbal->calib_gyro_pp = 0.0f;
        gimbal->calib_gyro_std = 0.0f;
        Gimbal_Calib_Clear_Window(gimbal);
    }

    /* DONE / TIMEOUT 是终态：不再自动重来（否则每次摇杆回中都会重新标一遍）。
       要再标一次就把 calib_restart 写 1。 */
    if ((gimbal->calib_state == GIMBAL_CALIB_DONE)
        || (gimbal->calib_state == GIMBAL_CALIB_TIMEOUT)) {
        return;
    }

    /* ---- 预热阶段：保持 IDLE，也不计 10s 超时 ----
       ★ 超时必须从"加热就绪"起算：IMU 从室温烧到 47℃ 通常远超 10s，
         若从上电就计时，真实车上会 100% 走到 TIMEOUT，这个功能等于没用。 */
    if (heater_ok == 0U) {
        gimbal->calib_state = GIMBAL_CALIB_IDLE;
        Gimbal_Calib_Clear_Window(gimbal);
        return;
    }

    /* 加热就绪：第一次进入 WAIT 时开始 10s 计时 */
    if (gimbal->calib_state == GIMBAL_CALIB_IDLE) {
        gimbal->calib_state = GIMBAL_CALIB_WAIT;
        gimbal->calib_timeout_ms = 0U;
        gimbal->calib_stable_ms = 0U;
    }

    /* ---- 总超时：10s 内没成功就放弃，靠已有的在线跟踪兜底 ----
       ★ 期间被碰/云台掉线导致的重来也算在这 10s 里（需求：总共等 10 秒）。*/
    if (gimbal->calib_timeout_ms < GIMBAL_CALIB_TIMEOUT_MS) {
        gimbal->calib_timeout_ms++;
        if (gimbal->calib_timeout_ms >= GIMBAL_CALIB_TIMEOUT_MS) {
            gimbal->calib_state = GIMBAL_CALIB_TIMEOUT;
            Gimbal_Calib_Clear_Window(gimbal);
            return;
        }
    }

    if (gimbal->calib_state == GIMBAL_CALIB_WAIT) {
        /* 静止条件要连续满足 200ms 才开始采样，避开刚上电/刚加热完的抖动 */
        if (ready != 0U) {
            if (gimbal->calib_stable_ms < GIMBAL_CALIB_STABLE_MS) {
                gimbal->calib_stable_ms++;
            }
            if (gimbal->calib_stable_ms >= GIMBAL_CALIB_STABLE_MS) {
                /* 锁存本次采样时长：采样中途在线改 calib_ms 只影响下一次，不会让本次突然结束 */
                float target_ms = gimbal->calib_ms_cfg;
                if (!(target_ms >= GIMBAL_CALIB_MS_MIN)) {   /* 同时挡住 NaN */
                    target_ms = GIMBAL_CALIB_MS_MIN;
                }
                if (target_ms > GIMBAL_CALIB_MS_MAX) {
                    target_ms = GIMBAL_CALIB_MS_MAX;
                }
                gimbal->calib_target_ms = (uint16_t)target_ms;

                Gimbal_Calib_Clear_Window(gimbal);
                gimbal->calib_state = GIMBAL_CALIB_COLLECTING;
            }
        } else {
            /* 静止条件被破坏（有人碰/推杆/云台掉线）：重新等 200ms */
            gimbal->calib_stable_ms = 0U;
        }
        return;
    }

    /* ---- COLLECTING ----
       中途只要"静止条件"或在线状态被破坏，就清空累加、退回 WAIT 重新等 200ms。
       这样平均窗口里绝不会混进碰云台/转底盘的样本。 */
    if (ready == 0U) {
        gimbal->calib_state = GIMBAL_CALIB_WAIT;
        Gimbal_Calib_Clear_Window(gimbal);
        return;
    }

    gimbal->calib_sum += gz_dps;
    gimbal->calib_sum2 += gz_dps * gz_dps;
    if ((gimbal->calib_n == 0U) || (gz_dps < gimbal->calib_min)) {
        gimbal->calib_min = gz_dps;
    }
    if ((gimbal->calib_n == 0U) || (gz_dps > gimbal->calib_max)) {
        gimbal->calib_max = gz_dps;
    }
    gimbal->calib_n++;
    gimbal->calib_ms++;

    if (gimbal->calib_ms >= gimbal->calib_target_ms) {
        const float n_f = (float)gimbal->calib_n;
        const float avg = gimbal->calib_sum / n_f;
        float var = (gimbal->calib_sum2 / n_f) - (avg * avg);
        if (var < 0.0f) {
            var = 0.0f;      /* 浮点误差可能让方差差一点点变负 */
        }
        const float std = sqrtf(var);

        /* 传感器读飞（NaN/Inf）时绝不能把野值写进 yaw_bias —— 那会直接毁掉位置环反馈。
           丢掉这次窗口、回 WAIT 重来；若一直坏，10s 超时会把它变成 TIMEOUT。 */
        if (!isfinite(avg) || !isfinite(std) || !isfinite(gimbal->calib_min)
            || !isfinite(gimbal->calib_max)) {
            gimbal->calib_state = GIMBAL_CALIB_WAIT;
            Gimbal_Calib_Clear_Window(gimbal);
            return;
        }

        gimbal->calib_gyro_avg = avg;                              /* deg/s */
        gimbal->calib_gyro_pp  = gimbal->calib_max - gimbal->calib_min;  /* deg/s */
        gimbal->calib_gyro_std = std;                              /* deg/s */

        /* ★★★ 写入零偏 + bias_accum 归零 ---------------------------------------------
           ★ 为什么还要平移目标角：bias_accum 是【位置环反馈】的一部分，
             world_yaw = wrap(imu_yaw − bias_accum)。把它直接清 0，反馈会瞬间增大
             old_accum（预热几分钟里在线跟踪可能已经积了 1~5°），位置环就会让云台
             物理上转一个小角度去"追"这个阶跃 —— 表现成"标定一完成云台自己动一下"。
             世界系整体平移 +old_accum、目标角同步平移，误差不变 ⇒ 没有任何物理动作。
             推导：W_old = imu − A，W_new = imu = W_old + A；
                   要 (T_new − W_new) == (T_old − W_old)，得 T_new = T_old + A。
           ★ 平移的只是"世界系角度的基准"，不动 PID/增益/限位/回位/cam_*_off。 */
        const float old_accum = gimbal->bias_accum;
        gimbal->target_yaw = Gimbal_Wrap180(gimbal->target_yaw + old_accum);
        gimbal->target_yaw_smooth = Gimbal_Wrap180(gimbal->target_yaw_smooth + old_accum);
        gimbal->yaw_target_ext = Gimbal_Wrap180(gimbal->yaw_target_ext + old_accum);

        gimbal->yaw_bias = avg;       /* deg/s：与 user_attitude.yaw_rate / imu_yaw 同一单位链 */
        gimbal->bias_accum = 0.0f;    /* 从现在开始重新积分；bias_k 保持不变，继续在线微调 */
        gimbal->calib_state = GIMBAL_CALIB_DONE;
    }
}

#endif /* GIMBAL_LOCK_WORLD && GIMBAL_IMU_ON_GIMBAL */

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

    gimbal->pitch_stick = stick;    /* 只读观测镜像：保留真实杆量，供绘图标定死区 */

    /* ★ 参与 target_pitch 积分的摇杆量：自瞄接管时显式置 0（不是把增益设 0）。
       观测镜像 pitch_stick 仍保留真实杆量；这里只决定"写进目标角的杆量"，
       所以即使以后有人把积分行挪进 aim 分支，命中的也是 0。*/
    const float pitch_stick_cmd = (gimbal->aim_takeover > 0.5f) ? 0.0f : stick;

    /* ★★★ 视觉自瞄接管 pitch（覆盖式，与 yaw 同结构）--------------------------------
       参考工程 ita_robot.cpp:721-733：使能 && 在线 && mode!=0 时
         "视觉下，覆盖设置云台的目标角度"：目标角 = 当前姿态 + 视觉偏差（不是叠加到摇杆积分上），
         速度前馈   = 系数(0.6) × 视觉目标角速度 -> 叠加到【速度环】的目标上。
       ★ 与 yaw 同结构：覆盖式写目标角 / 同一套 2ms 目标平滑 / 无效帧冻结目标。
       ★ 与 yaw 唯一刻意不同的地方：pitch【不做】"测量转速 × aim_delay" 的转速外推 ——
         这是稳定性硬约束，不是省事，推导见本分支末尾的长注释。
       ★ 优先级：自瞄接管 > 弹道解算(pitch_auto) > 摇杆。参考工程也是视觉覆盖在最后。
       ★ 开火链路先不做：接法见参考 ita_robot.cpp:424-434（在线 && mode==CONTROL_AND_FIRE
         且左键按下 -> 请求开火），本工程还没有确认扳机/使能通道，不要自己发明通道号。 */
    if (gimbal->aim_takeover > 0.5f) {
        /* ★ 目标角 100% 来自视觉解算值 pitch_target_ext，不叠加摇杆（摇杆量已在上面的
           pitch_stick_cmd 里按 aim_takeover 显式清零）。
           ★ 这里【不再】做 "pitch_target_ext + pitch_speed*6*aim_delay" 的转速外推，
             原因见本分支末尾的长注释（kp·τ > 1，会把位置环翻成正反馈）。 */
        if (gimbal->pitch_aim_valid != 0) {
            /* ★★ 只有本帧视觉数据有效才刷新目标；被闸门拒绝时保持上一拍 target_pitch。
               原来这里没有判断，会拿上一帧的旧目标继续按转速外推。 */
            const float pitch_cmd = gimbal->pitch_target_ext;

            /* ★★ 目标角一阶平滑（和 yaw 的 target_yaw_smooth 同一套、同一个 τ 字段）：
               τ = GIMBAL_TGT_SMOOTH_AIM_MS(2ms)，α = dt/(τ+dt)，dt = 1ms。
               pitch 原来完全没有平滑，视觉 100Hz 的目标阶跃直接进串级外环。 */
            const float smooth_ms = gimbal->tgt_smooth_aim_ms;
            if (smooth_ms > 0.0f) {
                const float alpha_s = GIMBAL_DT_S / (smooth_ms * 0.001f + GIMBAL_DT_S);
                s_pitch_cmd_smooth += alpha_s * (pitch_cmd - s_pitch_cmd_smooth);
            } else {
                s_pitch_cmd_smooth = pitch_cmd;
            }
            gimbal->target_pitch = s_pitch_cmd_smooth;

            gimbal->pitch_aim_err = pitch_cmd - gimbal->pitch;
        }

        /* ★★★ 为什么删掉原来的 "pitch_target_ext + pitch_speed*6*aim_delay" 外推 -----
           它把【实测关节转速】直接加进位置目标：target = ext + τ·v（τ = aim_delay = 30ms）。
           串级外环是 "角度误差(deg) -> 转速目标(RPM)"，kp = GIMBAL_PITCH_ANGLE_KP = 12 RPM/deg
           = 72 (deg/s)/deg，于是闭环满足
                v = kp·(ext − θ + τ·v)   =>   v·(1 − kp·τ) = kp·(ext − θ)
           本工程 kp·τ = 72 × 0.030 = 2.16 > 1 -> (1 − kp·τ) < 0：
           速度项压过位置项、整个位置环符号翻转 = 正反馈。数值仿真（同样的
           kp/ki/kd + 一阶速度环）里，只要带这个 30ms 外推，pitch 就发散/极限环；
           去掉它以后各种速度环滞后下都收敛。
           yaw 为什么没事：yaw 外环 kp = GIMBAL_WORLD_KP = 3.2 RPM/deg = 19.2 (deg/s)/deg，
           kp·τ = 0.576 < 1，同一个公式在 yaw 上是稳定的小超前。**同一条外推公式在
           pitch 上不等价**：pitch 外环比 yaw 快 3.75 倍，30ms 的提前量对它是过补偿
           （稳定边界只有 1/(6·kp) = 14ms）。
           另外，偏差语义下 pitch_ext 已经含【当前关节角】，链路延迟里云台自身的运动
           已经由那一项补偿过，再加 τ·v 是重复补偿。
           视觉【目标自身】的运动仍然有前馈：pitch_aim_rate（视觉角速度 × aim_ff_pitch）
           在下面被送进速度环，那是不在位置回路里的前馈，不影响稳定性。
           ⚠️ GIMBAL_AIM_DELAY_MS(30) 本身没动，yaw 继续用它；只是 pitch 不再用
              "测量转速 × 30ms" 这种会自激的形式。 */
    } else if (gimbal->pitch_auto < 0.5f) {
        /* pitch_auto != 0 时目标角由上层（弹道解算）给，摇杆不参与，避免互相打架 */
        gimbal->target_pitch += pitch_stick_cmd * gimbal->pitch_max_w * GIMBAL_PITCH_RC_DIR * GIMBAL_DT_S;
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
       尖峰（fout = kf × Δtarget / 1ms），表现就是 pitch 跟着每一帧抖一下 —— 所以接管时置 0。
       （本轮给 aim 目标补了 2ms 一阶平滑，阶跃尖峰已经被压低；但 KF 仍然置 0：
         位置回路里再叠一个"目标微分"只会把视觉噪声放大成转速指令。）
       真正需要的目标角速度由视觉通过 ff_ext 单独给（aim_ff_pitch）。
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

   ★★ 数据语义（2026-10-26 视觉组确认）：视觉 yaw/pitch 是【相对当前姿态的偏差量】，
   不是绝对目标角。本函数负责把它解算成绝对目标：目标 = 当前姿态 + 偏差。

   六道闸（阈值都是 gimbal.h 里的 GIMBAL_AIM_* 宏，单位度，方便现场调参）：
     ① NaN/Inf      —— 先于一切算术；否则后面比较全部失效、Wrap180 的 float->int 是 UB
     ② 不接管       —— 未使能 / 离线 / mode==NO_CONTROL：回摇杆，一个目标变量都不写
     ③ 偏差量程     —— |视觉偏差| > GIMBAL_AIM_MAX_ERR_DEG -> 无效（yaw、pitch 同判）
     ④ pitch 量程   —— "当前关节角 + 偏差"超出现有机械软限位（留 MARGIN）-> 无效
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

    /* ★★★ 语义修正（2026-10-26）：视觉发回来的是【偏差量（纠偏量）】，不是绝对目标角。
       旧实现当绝对角用 -> 正反馈：云台一低头，视觉看到的偏差更大 -> 目标更低 -> 一路
       低头撞到 pitch 下限卡死。正确关系是：
           目标 = 当前姿态 + 偏差
       · yaw 基准取 gimbal->yaw_fb = wrap(imu_yaw − bias_accum)：它就是位置环实际反馈，
         也是 target_yaw / 标定平移（Gimbal_Calib_Update 里的 +old_accum）所在的世界系；
       · pitch 基准取 gimbal->pitch（上电标定后的编码器【关节角】）：软限位/回位/下发都用它。
       ⚠️ Gimbal_Aim_Update 在 Gimbal_Update 里跑在 yaw_fb / pitch 本拍刷新【之前】，
          所以这里读到的是上一拍（1 ms 前）的姿态；1 kHz 任务下可忽略。 */
    const float vision_yaw_deg   = raw_yaw   * VISION_PKG2DEG;   /* 弧度 -> 度 */
    const float vision_pitch_deg = raw_pitch * VISION_PKG2DEG;

    /* 闸③ 偏差量程（原"yaw 绝对角量程"闸，偏差语义下【复用】同一道）：
       |偏差| > GIMBAL_AIM_MAX_ERR_DEG(45°) = 野值 / 目标在视场外 -> 直接拒帧。
       ★ 判【视觉偏差本身】，不拿 yaw_ext 反推：yaw_ext 已含 cam_yaw_off，反推会把
         标定偏置算进"偏差"，45° 阈值就不准了；偏差量本身也不需要 Wrap180。
       ★ 必须放在下面 Wrap180 之前：-9518.6（rad 或 deg）这类 tracker 丢失输出换算后
         绝对值都远超 45°，在此拦下，也就没有"巨大浮点 -> int32"的 UB 风险。
       ★ pitch 也一起判：需求是"单帧偏差绝对值 > 45° 直接拒帧"，两轴同判语义才一致。
         正常情况下闸④已经能覆盖 pitch（关节总行程 -11.8~29.6 约 41° < 45°），
         这里保留显式判断是刻意的冗余，不是新增闸门。 */
    if (fabsf(vision_yaw_deg) > GIMBAL_AIM_MAX_ERR_DEG
        || fabsf(vision_pitch_deg) > GIMBAL_AIM_MAX_ERR_DEG) {
        return 0;
    }

    /* 偏差 -> 绝对目标：偏差是相对【当前姿态】的，所以每一帧都在当前姿态上叠加。
       · yaw 结果归一化到 ±180（世界角）；pitch 是关节角、不绕圈，直接加。
       ★ cam_yaw_off(-10) / cam_pitch_off(+3) 先按原值保留 —— 它们是旧绝对角语义下
         打的靶，偏差语义下含义变成"相机光轴 <-> 枪管零位"的常量差，待用户重标。
       ★ 收敛判据（供重标）：稳态时 偏差 = −cam_off。cam_off 校准正确时枪管落点
         居中，但 ch1/ch4 会停在 −相机安装偏角（不一定是 0，当前参数下理论
         ch1 ≈ +10°、ch4 ≈ −3°）。实测重标：落点偏 M 就把 cam_*_off 减 M，
         直到落点居中。 */
    const float yaw_ext   = Gimbal_Wrap180(gimbal->yaw_fb + vision_yaw_deg
                                          + gimbal->cam_yaw_off);
    /* ★★★ pitch 侧和 yaw 完全对称：偏差叠加在【发给视觉的那个姿态量】上 ---------
       yaw：TX 发 yaw_fb = wrap(imu_yaw − bias_accum)（世界角，上电≈0），
            它同时也是 yaw 位置环的反馈；RX 就直接 target = yaw_fb + 偏差 + cam_yaw_off。
       pitch：TX 发 pitch_fb = gimbal->pitch − 9.1403（关节角，上电=0，见 Gimbal_Update
              里 tx_pitch 那一段）；RX 同样 target_fb = pitch_fb + 偏差 + cam_pitch_off。
       下发前再把 9.1403 加回去，因为软限位 / 回位 / PID / Set_Motor_State 全在
       【关节角】域（−10.8 ~ +28.6，上电回位 9.1403，一个都不改）。
       ★ 为什么这样就收敛（这是本轮真正的根因修复）：视觉算偏差时用的"当前姿态"和
         我们叠加偏差用的"当前姿态"现在是同一个量 pitch_fb —— 云台一动，我们 100Hz
         把新的 pitch_fb 发给视觉，视觉下一帧的偏差就跟着减小，形成真正的负反馈闭环。
         改之前 TX 发的是 qe_roll（不含 pitch 关节角，云台怎么动它都不变），
         视觉的偏差不会随关节角减小，target = 当前 + 偏差 就退化成"每帧 +偏差"的积分，
         目标只能来回游走/一路顶到限位。
       ★ 9.1403 在这里只是"把上电=0 的姿态量变回编码器关节角"的坐标变换，
         出现一次、不影响闭环；软限位/回位/关节零点全部保持原样。
       ⚠️ cam_pitch_off(+3) 是旧 TX 语义（qe_roll）下标定的，本轮改了 TX 的 pitch 语义，
          必须重标：落点偏 M 就把 cam_pitch_off 减 M，直到准心落在装甲板上。 */
    const float pitch_fb     = gimbal->pitch - GIMBAL_PITCH_BOOT_TARGET_DEG;  /* 上电 = 0 */
    const float pitch_ext_fb = pitch_fb + GIMBAL_VISION_PITCH_SIGN * vision_pitch_deg + gimbal->cam_pitch_off;   /* ★ 视觉 pitch 偏差方向与我们相反，取 -1 */
    const float pitch_ext    = pitch_ext_fb + GIMBAL_PITCH_BOOT_TARGET_DEG;   /* 回关节角域 */

    /* 闸④ pitch 量程：必须落在【工程已有】的机械软限位内（留一点余量兼容标定误差）
       ★ 偏差语义下这道闸仍然【必须保留且正确】：pitch_ext 依旧是关节角目标，
         "当前角 + 偏差"叠加后可以超出机械行程，靠它拦。 */
    if (pitch_ext > (GIMBAL_PITCH_MAX_DEG + GIMBAL_AIM_PITCH_MARGIN_DEG)
        || pitch_ext < (GIMBAL_PITCH_MIN_DEG - GIMBAL_AIM_PITCH_MARGIN_DEG)) {
        return 0;
    }

    /* 闸⑤ 跳变门限：和上一帧【有效】目标角比。跨 ±180° 求差要先用 Wrap180，
       否则目标从 +179 走到 -179（实际只动 2°）会被误判成 358° 的大跳变。
       ★ 偏差语义下这道闸仍然正确、而且更稳：目标 ≈ 装甲板的世界角/关节角，云台往目标
         转的时候 yaw_fb/pitch 在动、偏差在反向缩小，两者相加几乎不动；真正的大跳变
         只可能来自 tracker 换目标 / 丢帧野值 —— 正是要拦的。
       ⚠️ 极端情况：大角度快速调转时 |Δyaw_fb| 会叠加进来，但 100Hz 帧率下云台一帧
          最多转 ~19°（按 GM6020 320RPM 估），远小于 60° 门限，不会误拒。 */
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
    /* ★ 一帧通过全部闸门的有效数据 = "视觉恢复正常"的证据：解除闸⑥的重新接管锁存。
       注意只清这个锁存，不在这里动 aim_takeover —— 是否接管由 Gimbal_Aim_Update 的
       统一入口判定（下一拍起允许），避免绕开闸②的在线/mode 判断。 */
    gimbal->aim_need_ok   = 0;
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

/* 退出接管：只【放掉接管】、把云台交回摇杆 —— 绝不动操作手的使能开关 yaw_auto。
   ★★★ 为什么【不】清 yaw_auto（BUG 修复，2026-10-25）：
     ① yaw_auto 是操作手的开关（在线 yaw_auto=1#），不是视觉的状态位。视觉一掉线/一给野值
        就把它清 0，操作手每次都得重新发命令，手感像"自瞄自己会掉"；
     ② 它还是发射机构的【总闸之一】（shooter.c: arm = (yaw_auto >= 0.5) && sw1 上），
        清掉 yaw_auto 会连累摩擦轮/拨弹一起失能 —— 视觉抖一帧就把枪断电，风险更大；
     ③ 它同时是给视觉回传的握手（gimbal.c 里 Vision_Set_Tx_Mode 看 yaw_auto）：
        清 0 会回 IDLE，视觉侧可能就不再发 mode=1/2 的控制包，反而更难恢复。
   ★ 现在视觉侧的一切异常（离线超时 / 连续野值）只影响【接管】：aim_takeover=0，
     云台本拍就交回摇杆；yaw_auto 保持操作手设定的值，视觉恢复有效帧即自动重新接管。
   ★ 重新接管时目标角以【当前角度】为起点（Gimbal_Aim_Update 的"接管瞬间"那一段），
     所以不会朝一个陈旧目标猛甩 —— 自动恢复是安全的。
   ★ 真正"关掉自瞄"只剩操作手自己的动作：yaw_auto=0#（参数表直写，见 chassis_control.c）。
   排错看 aim_reject_cnt（累计拒绝帧数）与 aim_bad_frames（连续无效帧数）。*/
static void Gimbal_Aim_Abort(GIMBAL_DRIVES* gimbal) {
    Gimbal_Aim_Release(gimbal);     /* aim_takeover = 0：本拍立刻交回摇杆（安全行为，必须保留）*/
    gimbal->aim_bad_frames = 0;     /* 重新计连续无效帧，避免同一帧野值反复触发 */
    /* ★★ 锁存"必须再来一帧通过全部闸门的有效数据才允许重新接管"。
       没有它：Abort 只 Release，下一拍 yaw_auto && 在线 && mode!=0 会立刻重新置
       aim_takeover=1，而 aim_valid=0 又走"丢帧冻结"分支 —— 视觉持续发野值时摇杆只有
       每 ~50ms 中的 1ms 能控制 yaw（操作手抢不动）。锁存后野值期间云台一直听摇杆，
       视觉恢复给出一帧合格数据时由 Gimbal_Aim_Accept_Frame 清 0，自动重新接管。 */
    gimbal->aim_need_ok = 1;
}

static void Gimbal_Aim_Update(GIMBAL_DRIVES* gimbal) {
    const uint32_t now = HAL_GetTick();

    /* ★★ BUG 修复（2026-10-26）：操作手重开自瞄 = 给视觉【一次新的接管机会】。
       在 yaw_auto 的 0→1 上升沿清掉闸⑥留下的"重新接管许可"锁存 aim_need_ok 和
       连续无效帧计数。原来的锁只有"再来一帧通过全部闸门的有效视觉数据"才解得掉：
       视觉一旦连续野值/超时触发过 Gimbal_Aim_Abort()，操作手把 yaw_auto 关掉再打开
       也清不掉 -> 下面 aim_need_ok==0 的判断永远不成立 -> 云台再也不跟视觉。
       ★ 位置：放在 yaw_auto<0.5 的早退分支【之前】，更在本函数更新 yaw_auto_pre
         之前 —— ① 早退分支里也要把 pre 归 0（否则关自瞄期间 pre 一直停在 1，下次
         0→1 就检测不到）；② pre 一旦先被更新成 1，这个比较就永远为假。
       ★ 只放一次"许可"，保护语义不变：重开后若视觉仍然持续野值，
         GIMBAL_AIM_INVALID_LIMIT 帧后 Gimbal_Aim_Abort() 会再次上锁并抢回摇杆。
       ★ aim_reject_cnt 不清（累计统计，排错用）；aim_ms 也不清（离线超时判据要用，
         它在本函数下面"刚打开"那个分支里重新打点）。*/
    if (gimbal->yaw_auto_pre < 0.5f && gimbal->yaw_auto >= 0.5f) {
        gimbal->aim_need_ok    = 0;
        gimbal->aim_bad_frames = 0;
    }

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
        /* ★ pitch 目标平滑状态也对齐到当前角度：否则接管第一拍会从上一轮的旧平滑值
           慢慢爬过来（和 target_pitch 的初始化同一个理由）。 */
        s_pitch_cmd_smooth = gimbal->pitch;
        gimbal->aim_bad_frames = 0;
        gimbal->aim_rate = 0.0f;
        gimbal->pitch_aim_rate = 0.0f;
    }

    /* 闸②-b 视觉离线：不接管，回摇杆（保持原来的摇杆/编码器控制）。
       短丢帧（< aim_timeout_ms）只是这一拍交还摇杆，云台目标不会跳（接管时重新以
       当前角度为起点）；连续超过 aim_timeout_ms 仍无有效帧 -> 判"视觉真的没了"，退出接管。
       ★ 两条路径都【只释放接管】，不碰 yaw_auto：操作手的开关不能被视觉的掉线改掉
         （BUG 修复，见 Gimbal_Aim_Abort 上面那段说明）。*/
    if (Vision_Is_Online(&user_vision) == 0) {
        Gimbal_Aim_Release(gimbal);
        if ((uint32_t)(now - gimbal->aim_ms) > (uint32_t)gimbal->aim_timeout_ms) {
            Gimbal_Aim_Abort(gimbal);   /* 与 Release 的区别只剩"清连续无效帧计数"*/
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

    /* 接管条件齐了：使能 && 在线 && mode != NO_CONTROL（参考 ita_robot.cpp:639 同款）
       ★★ 新增一道"重新接管许可"：被闸⑥（连续野值）中止过时 aim_need_ok=1，此时即使
          在线/mode 已恢复也【不】接管，必须等一帧真正通过全部闸门的有效视觉数据
          （Gimbal_Aim_Accept_Frame 成功末尾清 0）。这样野值期间云台老老实实听摇杆，
          视觉恢复后自动重新接管，不需要操作手干预。其余释放路径（未使能/离线/mode==0）
          仍走各自的 Gimbal_Aim_Release，不经过这个锁存。 */
    if (gimbal->aim_need_ok == 0) {
        gimbal->aim_takeover = 1.0f;
    }

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

    /* 闸⑥ 连续若干帧都无效 -> 退出接管、回摇杆控制。
       ★ 同样【不清 yaw_auto】：野值只说明"这一段时间的视觉数据不能信"，不代表
         操作手要关自瞄。清掉会形成"操作手重发 → 再被野值清掉"的死循环（BUG 修复）。*/
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
