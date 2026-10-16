#include "../../Core/Inc/bsp.h"
#include "../chassis_control.h"
#include <math.h>

#define DBUS_STICK_MAX   (660.0f)        /* DBUS 摇杆满量程 */
#define DBUS_OFFLINE_MS  (100)           /* 断线停车阈值 ms */
#define DEG2RAD          (0.0174532925f)

/* ---- 在线可调参数，初值取自 chassis_config.h ---- */
static float s_spin_w   = CHASSIS_SPIN_W;
static float s_dead     = (float)CHASSIS_RC_DEAD;
static float s_head_kp  = CHASSIS_FOLLOW_KP;
static float s_head_ki  = CHASSIS_FOLLOW_KI;
static float s_head_kd  = CHASSIS_FOLLOW_KD;
static float s_wheel_kp = CHASSIS_SPEED_KP;
static float s_wheel_ki = CHASSIS_SPEED_KI;
static float s_wheel_kd = CHASSIS_SPEED_KD;

static float s_omega_dir = CHASSIS_OMEGA_DIR;

/* 航向源保护开关（在线可调）：1=启用（云台掉线就禁自转），0=关闭 */
static float s_src_protect = 1.0f;

/* 单轮测试：0=正常，1~4=只驱动对应轮子，负值反转 */
static float s_test_wheel = 0.0f;
static float s_test_rpm    = 500.0f;

static float s_rev_fl = CHASSIS_REV_FL;
static float s_rev_fr = CHASSIS_REV_FR;
static float s_rev_rl = CHASSIS_REV_RL;
static float s_rev_rr = CHASSIS_REV_RR;

static Chassis*       s_chassis = NULL;
static DBUS_DRIVES*   s_dbus = NULL;
static PID_Controller s_heading_pid = {0};
static Chassis_Mode   s_mode = CHASSIS_MODE_MANUAL;
static uint8_t        s_offline_ms = 0;
static uint8_t        s_prev_sw1 = 0;
static Chassis_HeadingSource s_heading_src = ChassisControl_NoHeading;
static Chassis_SourceValid   s_source_valid = ChassisControl_SourceAlwaysValid;

/* 串口绘图通道：上电 = CHASSIS_SERIALPLOT_MODE，也能在线改（发 plot_mode=3#）*/
static float s_plot_mode = (float)CHASSIS_SERIALPLOT_MODE;

/* 小陀螺转向（带死区保持，避免摇杆回中时 1kHz 反复换向）+ 自转 omega 的速率限制状态 */
static int8_t s_spin_dir = 1;
static float  s_w_cmd = 0.0f;

static void  Chassis_Apply_Wheel_Gain(void);
static void  Chassis_Apply_Reverse(void);
static void  Chassis_Apply_Friction_Gain(void);
static void  Chassis_Apply_Plot_Mode(void);
static float Chassis_Slew_Omega(float target);
static int16_t RC_Channel(uint8_t ch);
static float Chassis_Spin_Omega(void);
static float Stick(int16_t value);
static float Heading(void);
static void  Heading_Pid_Reset(void);

float ChassisControl_NoHeading(void) {
    return 0.0f;
}

uint8_t ChassisControl_SourceAlwaysValid(void) {
    return 1;
}

static void Chassis_Apply_Wheel_Gain(void) {
    PID_Controller* pids[4] = {&user_wheel_pid_fl, &user_wheel_pid_fr,
                               &user_wheel_pid_rl, &user_wheel_pid_rr};
    for (uint8_t i = 0; i < 4; i++) {
        pids[i]->kp = s_wheel_kp;
        pids[i]->ki = s_wheel_ki;
        pids[i]->kd = s_wheel_kd;
    }
}

static void Chassis_Apply_Reverse(void) {
    if (s_chassis == NULL) {
        return;
    }
    s_chassis->reverse[0] = (s_rev_fl < 0.0f) ? -1 : 1;
    s_chassis->reverse[1] = (s_rev_fr < 0.0f) ? -1 : 1;
    s_chassis->reverse[2] = (s_rev_rl < 0.0f) ? -1 : 1;
    s_chassis->reverse[3] = (s_rev_rr < 0.0f) ? -1 : 1;
}

/* 两轮同增益但必须独立 PID 实例（各自维护积分与误差历史）；调参表只挂左轮，回调同步右轮 */
static void Chassis_Apply_Friction_Gain(void) {
    user_shooter.friction_right_pid.kp = user_shooter.friction_left_pid.kp;
    user_shooter.friction_right_pid.ki = user_shooter.friction_left_pid.ki;
    user_shooter.friction_right_pid.kd = user_shooter.friction_left_pid.kd;
}

static int16_t RC_Channel(const uint8_t ch) {
    switch (ch) {
        case 0:  return s_dbus->ch0;
        case 1:  return s_dbus->ch1;
        case 6:
            /* 四元数 / 姿态调试（8 通道）：
               qw qx qy qz  <- ★ 这 4 个就是通过 UART1 发给视觉的四元数（w,x,y,z）
               qroll qpitch qyaw <- EKF 解算的欧拉角 deg
               cf_pitch     <- 互补滤波的 pitch deg（与 EKF 对比用）
               ★ 静止时 qw≈1、qx/qy/qz≈0，且 qw^2+qx^2+qy^2+qz^2 应恒为 1
               ★ 转动云台时四个都会变；若范数不是 1，说明 EKF 有问题 */
            Serialplot_Set_Data(&user_serialplot, 8,
                                &user_attitude.qe_q[0], &user_attitude.qe_q[1],
                                &user_attitude.qe_q[2], &user_attitude.qe_q[3],
                                &user_attitude.qe_roll, &user_attitude.qe_pitch,
                                &user_attitude.qe_yaw, &user_attitude.pitch);
            break;
        case 2:  return s_dbus->ch2;
        case 3:  return s_dbus->ch3;
        default: return 0;
    }
}

static float Chassis_Spin_Omega(void) {
    if (s_mode == CHASSIS_MODE_SPIN) {
        /* ★★ 必须带死区 + 磁滞保持上次方向。
           原来写的是 `(RC_Channel(...) < 0) ? -s_spin_w : s_spin_w` —— 没有死区，
           摇杆回中/漂移时 ch2 在 0 附近抖一下，omega 就会在 ±spin_w 之间
           【1kHz 反复翻转】。底盘指令跟着正负横跳，表现就是小陀螺"卡一下"。 */
        const int16_t dir_ch = RC_Channel(CHASSIS_SPIN_DIR_CH);
        const int16_t dead = (int16_t)s_dead;
        if (dir_ch > dead) {
            s_spin_dir = 1;
        } else if (dir_ch < -dead) {
            s_spin_dir = -1;
        }
        return (float)s_spin_dir * s_spin_w;
    }
#if CHASSIS_RC_W_ENABLE
    return Stick(RC_Channel(CHASSIS_RC_W_CH)) * CHASSIS_MAX_W;
#else
    /* 右摇杆左右已经给云台 yaw 用了，手动模式不再从摇杆取自转 */
    return 0.0f;
#endif
}

/* 自转角速度速率限制：模式切换/换向时不让 omega 阶跃，避免"顿一下" */
static float Chassis_Slew_Omega(const float target) {
    /* 标准 C 的 #if 不允许浮点常量，所以这里用运行期判断（step<=0 即关闭限速）*/
    const float step = CHASSIS_W_SLEW * (float)CHASSIS_PERIOD_MS * 0.001f;
    if (step <= 0.0f) {
        return target;
    }

    if (target > s_w_cmd + step) {
        s_w_cmd += step;
    } else if (target < s_w_cmd - step) {
        s_w_cmd -= step;
    } else {
        s_w_cmd = target;
    }
    return s_w_cmd;
}

static float Stick(const int16_t value) {
    if (value > -(int16_t)s_dead && value < (int16_t)s_dead) {
        return 0.0f;
    }
    return (float)value / DBUS_STICK_MAX;
}

static float Heading(void) {
    return (s_heading_src != NULL) ? s_heading_src() : 0.0f;
}

static void Heading_Pid_Reset(void) {
    PID_Init(&s_heading_pid, s_head_kp, s_head_ki, s_head_kd,
             CHASSIS_FOLLOW_MAX, CHASSIS_FOLLOW_MAX);
}

/* 按模式登记上报通道（集中在这里而非 main.c 的 #if 链，才能在线切换）*/
static void Chassis_Apply_Plot_Mode(void) {
    const uint8_t mode = (uint8_t)(s_plot_mode + 0.5f);

    switch (mode) {
        case 3:
            /* pitch 轴调试（8 通道）：俯仰角 | ptgt 目标角 | pspd 转速 | 外环积分 pid_i |
               内环积分 psd_i | pcur 电流 mA | pacc 加速度计解算角 | pgyro 陀螺积分角
               ★ 顶重力的是内环积分 psd_i（稳态非零，pid_i 接近 0）；
                 松手下沉且 psd_i 顶不上去 -> 加大 pitch_spd_ki/mi
               ★★ 后两通道是【互补滤波的两个输入】，用来直接观察滤波效果：
                 pacc  抖动大（吃振动）但不漂
                 pgyro 很平滑但会缓慢漂走
                 pitch（第 1 通道）应该【既平滑又不漂】= 互补滤波在起作用 */
            Serialplot_Set_Data(&user_serialplot, 8,
                                &user_gimbal.pitch, &user_gimbal.target_pitch,
                                &user_gimbal.pitch_speed,
                                &user_gimbal.pitch_pid_angle.iout,
                                &user_gimbal.pitch_pid_speed.iout,
                                &user_gimbal.pitch_current,
                                &user_attitude.pitch_acc, &user_attitude.pitch_gyro);
            break;

        case 4:
            /* 射击/总线排错（12 通道）：frpm 摩擦轮实测 | frpm_f 滤波后 | frpm_t 目标转速 |
               ferr 拨弹轮位置误差 | fcur 滤波后电流 | bpitch 解算俯仰角 | bv0 弹丸初速 |
               fstate 状态机(0=STOP 1=SPINUP 2=FIRE 3=REVERSE) |
               online 三电机反馈 bit0=左轮 bit1=右轮 bit2=拨弹轮 | ready 摩擦轮是否到转速 |
               can1_rx / can2_rx 两条总线的【原始收帧数】累计值（约 +5000/s / +1000~4000/s）
               ★ fstate 会变(1/2) 但 online=0 -> 电机真没反馈（用 can2_rx 判断：
                 只按 pitch 的 ~1000/s 涨 = 那 3 个电调根本没在发，查接线/供电/ESC ID）*/
            Serialplot_Set_Data(&user_serialplot, 12,
                                &user_shooter.friction_rpm, &user_shooter.friction_rpm_filt,
                                &user_shooter.fire_rpm,
                                &user_shooter.feeder_err, &user_shooter.feeder_current_filt,
                                &user_shooter.ballistics.pitch_deg, &user_shooter.ballistics.v0,
                                &user_shooter.state_f,
                                &user_shooter.online_mask, &user_shooter.ready_f,
                                &user_can_1.rx_count_f, &user_can_2.rx_count_f);
            break;

        case 5:
            /* 自瞄调试（6 通道）：vtgt 视觉目标角 | vyaw 实际世界角 | verr 误差 |
               vdelay 延迟补偿 ms | vok 数据是否有效 | vfps 视觉帧数（看帧率）*/
            Serialplot_Set_Data(&user_serialplot, 6,
                                &user_gimbal.yaw_target_ext, &user_gimbal.imu_yaw,
                                &user_gimbal.aim_err, &user_gimbal.aim_delay_ms,
                                &user_gimbal.aim_ok_f, &user_vision.frame_count_f);
            break;

        case 6:
            /* 四元数 / 姿态调试（8 通道）：
               qw qx qy qz  <- ★ 这 4 个就是通过 UART1 发给视觉的四元数（w,x,y,z）
               qroll qpitch qyaw <- EKF 解算的欧拉角 deg
               cf_pitch     <- 互补滤波的 pitch deg（与 EKF 对比用）
               ★ 静止时 qw≈1、qx/qy/qz≈0，且 qw^2+qx^2+qy^2+qz^2 应恒为 1
               ★ 转动云台时四个都会变；若范数不是 1，说明 EKF 有问题 */
            Serialplot_Set_Data(&user_serialplot, 8,
                                &user_attitude.qe_q[0], &user_attitude.qe_q[1],
                                &user_attitude.qe_q[2], &user_attitude.qe_q[3],
                                &user_attitude.qe_roll, &user_attitude.qe_pitch,
                                &user_attitude.qe_yaw, &user_attitude.pitch);
            break;
        case 7:
            /* ★ 视觉发送包【镜像】（10 通道）：绘图上显示的就是 tx_packet 里真正要发出去的字节内容。
               所以"这里在变 + tx_frames 在涨" = 电控确实把四元数和欧拉角发给了视觉。
               tx_qw tx_qx tx_qy tx_qz  <- packet.q[0..3]，四元数 w,x,y,z
               tx_yaw  tx_yawv          <- packet.yaw / yaw_vel (deg, deg/s)
               tx_pitch tx_pitchv       <- packet.pitch / pitch_vel (deg, deg/s)
               tx_bspd                  <- packet.bullet_speed (m/s)
               tx_frames                <- 已发送帧数，应持续增长（100/秒）
               ★ 判据：tx_frames 在涨 + 转动云台时 tx_qx/qy/qz 跟着变 -> 发送链路完全正常 */
            Serialplot_Set_Data(&user_serialplot, 10,
                                &user_vision.tx_packet.q[0], &user_vision.tx_packet.q[1],
                                &user_vision.tx_packet.q[2], &user_vision.tx_packet.q[3],
                                &user_vision.tx_packet.yaw, &user_vision.tx_packet.yaw_vel,
                                &user_vision.tx_packet.pitch, &user_vision.tx_packet.pitch_vel,
                                &user_vision.tx_packet.bullet_speed, &user_vision.tx_count_f);
            break;

        case 8:
            /* ★★ 自瞄接管 / 数据闸门调试（8 通道）：
               ptgt_e 视觉给的 pitch 目标角（已过闸门的绝对角）| pitch 实际俯仰角 |
               ptgt   最终目标角（含延迟外推）| perr pitch 视觉误差 |
               pff    pitch 视觉角速度前馈 deg/s | take 本拍是否由视觉接管(1/0) |
               ok     视觉数据是否有效(1/0) | auto 使能开关 yaw_auto
               ★ 判据：
                 auto=1 且 take=1 且 ok=1 -> 视觉正在控制（ptgt_e 应贴着 pitch 走）
                 ok 一直闪 0 或 auto 自己掉成 0 -> 视觉数据被判无效（看 yaw 侧
                 plot_mode=5 的 vfps/vok；连续 5 帧无效会退出接管，需要重新 yaw_auto=1）*/
            Serialplot_Set_Data(&user_serialplot, 8,
                                &user_gimbal.pitch_target_ext, &user_gimbal.pitch,
                                &user_gimbal.target_pitch, &user_gimbal.pitch_aim_err,
                                &user_gimbal.pitch_aim_rate, &user_gimbal.aim_takeover,
                                &user_gimbal.aim_ok_f, &user_gimbal.yaw_auto);
            break;

        case 9:
            /* ★★ 视觉【接收包】镜像（8 通道 / Frame Size 32）—— 专门用来"把视觉发过来的数据画出来"。
               数据源：user_vision.target_* （包内是【弧度】，这里已 ×VISION_PKG2DEG 换成【度】）
                       + 状态量的 float 镜像（Serialplot_Set_Data 只按 float 读，
                         uint8_t/uint32_t 不能直接传地址，见 user_vision.h 的 float 镜像字段）。
               ★ SerialPlot 里要设：Frame Start AB / #Channels 8 / Frame Size 32
               通道含义与单位：
                 mode    视觉给的控制模式（0=不给控制 1=只给控制 2=控制+请求开火），无量纲
                 vtgt    目标 yaw 角，度
                 vpit    目标 pitch 角，度
                 vvyaw   目标 yaw 角速度（可当速度前馈），度/秒
                 vpitch  目标 pitch 角速度（可当速度前馈），度/秒
                 vok     视觉是否在线（1=在线 0=离线），0-1
                 vfps    累计有效帧数（看帧率，稳定增长说明链路通），计数
                 vcrc    累计 CRC 坏帧数（链路质量），计数
               ★ 判据：vok=1 且 vfps 稳定增长 -> 视觉链路正常；
                 转动/移动目标时 vtgt/vpit 跟着动 -> 视觉确实在跟踪；
                 vfps 不涨而 vcrc 涨 -> 波特率/线材/共地有问题（坏帧被丢，云台不会乱动）。*/
            Serialplot_Set_Data(&user_serialplot, 8,
                                &user_vision.rx_mode_f, &user_vision.target_yaw_deg,
                                &user_vision.target_pitch_deg, &user_vision.target_yaw_vel_deg,
                                &user_vision.target_pitch_vel_deg, &user_vision.online_f,
                                &user_vision.frame_count_f, &user_vision.crc_error_f);
            break;

        case 2:
            /* 云台 yaw 调试（5 通道）：gyaw 相对底盘角 | gtgt 世界目标角 | gimu IMU 世界角 |
               gcmd 目标转速 | gspd 实际转速。gtgt/gimu 贴合 -> 锁世界；gcmd/gspd -> 速度环 */
            Serialplot_Set_Data(&user_serialplot, 5,
                                &user_gimbal.yaw, &user_gimbal.target_yaw,
                                &user_gimbal.imu_yaw, &user_gimbal.target_speed,
                                &user_gimbal.speed);
            break;

        case 1:
            /* 底盘 4 轮 目标 / 反馈转子转速 */
            Serialplot_Set_Data(&user_serialplot, 8,
                                &user_chassis.rpm[0], &user_chassis.rpm_fdb[0],
                                &user_chassis.rpm[1], &user_chassis.rpm_fdb[1],
                                &user_chassis.rpm[2], &user_chassis.rpm_fdb[2],
                                &user_chassis.rpm[3], &user_chassis.rpm_fdb[3]);
            break;

        default:
            /* 0 = 姿态 */
            Serialplot_Set_Data(&user_serialplot, 4,
                                &user_attitude.yaw, &user_attitude.pitch,
                                &user_chassis.omega, &user_chassis.vx);
            break;
    }
}

void ChassisControl_Init(Chassis* chassis, DBUS_DRIVES* dbus) {
    s_chassis = chassis;
    s_dbus = dbus;
    s_mode = CHASSIS_MODE_MANUAL;
    s_offline_ms = 0;
    s_prev_sw1 = 0;
    s_heading_src = ChassisControl_NoHeading;
    s_source_valid = ChassisControl_SourceAlwaysValid;

    Heading_Pid_Reset();
    Chassis_Apply_Wheel_Gain();
    Chassis_Apply_Reverse();
    /* ★ 必须在 Serialplot_Init 之后调用（main.c 里的顺序已经满足）*/
    Chassis_Apply_Plot_Mode();
}

void ChassisControl_Set_HeadingSource(Chassis_HeadingSource source) {
    s_heading_src = (source != NULL) ? source : ChassisControl_NoHeading;
    Heading_Pid_Reset();
}

void ChassisControl_Set_SourceValid(Chassis_SourceValid valid) {
    s_source_valid = (valid != NULL) ? valid : ChassisControl_SourceAlwaysValid;
}

void ChassisControl_Update(void) {
    if (s_chassis == NULL || s_dbus == NULL) {
        return;
    }

    if (s_dbus->is_update) {
        s_dbus->is_update = 0;
        s_offline_ms = 0;
    } else if (s_offline_ms < 255) {
        s_offline_ms++;
    }

    if (s_offline_ms > DBUS_OFFLINE_MS) {
        Chassis_Set_Velocity(s_chassis, 0.0f, 0.0f, 0.0f);
        Chassis_Update(s_chassis);
        return;
    }

    /* 航向源不可信（比如云台掉线）时的保护；可在线上关掉方便单独调底盘 */
    const uint8_t src_ok = (s_src_protect > 0.5f) ? s_source_valid() : (uint8_t)1;

#if CHASSIS_SOURCE_LOST_STOP
    if (!src_ok) {
        Chassis_Set_Velocity(s_chassis, 0.0f, 0.0f, 0.0f);
        Chassis_Update(s_chassis);
        return;
    }
#endif

    if (s_test_wheel != 0.0f) {
        const int idx = (int)s_test_wheel;
        const int wheel = ((idx > 0) ? idx : -idx) - 1;
        const float dir = (idx > 0) ? 1.0f : -1.0f;

        for (int i = 0; i < 4; i++) {
            s_chassis->wheel[i]->Set_Power_Limit(s_chassis->wheel[i], s_chassis->power_limit);
            s_chassis->wheel[i]->Set_Motor_State(s_chassis->wheel[i],
                                                 (i == wheel) ? dir * s_test_rpm : 0.0f);
        }
        return;
    }

    /* 拨杆切模式：上=跟随 中=手动 下=小陀螺 */
    if (s_dbus->sw1 != s_prev_sw1) {
        if (s_dbus->sw1 == 1) {
            s_mode = CHASSIS_MODE_FOLLOW;
        } else if (s_dbus->sw1 == 2) {
            s_mode = CHASSIS_MODE_SPIN;
            /* 进小陀螺时锁存一次转向，避免刚进入时 ch2 抖一下就反着转 */
            s_spin_dir = (RC_Channel(CHASSIS_SPIN_DIR_CH) < -(int16_t)s_dead) ? -1 : 1;
            s_w_cmd = 0.0f;
        } else {
            s_mode = CHASSIS_MODE_MANUAL;
        }
        Heading_Pid_Reset();
        s_prev_sw1 = s_dbus->sw1;
    }

    /* 左摇杆 -> 平移（右推右移） */
    const float vx =  Stick(RC_Channel(CHASSIS_RC_VX_CH)) * CHASSIS_MAX_VX;
    const float vy = -Stick(RC_Channel(CHASSIS_RC_VY_CH)) * CHASSIS_MAX_VY;

    float vx_body = vx;
    float vy_body = vy;

#if CHASSIS_HEADING_COMP
    /* 参考系 -> 车体系：绕 z 转 theta。手动模式取 0（以车头为正方向），跟随/小陀螺用
       航向源（以云台指向为正方向）；航向源不可信时也退回 0（车体系），保证还能开 */
    const uint8_t use_heading = (src_ok && (s_mode != CHASSIS_MODE_MANUAL)) ? 1 : 0;
    const float theta_rad = (use_heading ? Heading() : 0.0f) * DEG2RAD;
    const float cos_t = cosf(theta_rad);
    const float sin_t = sinf(theta_rad);
    vx_body = vx * cos_t - vy * sin_t;
    vy_body = vx * sin_t + vy * cos_t;
#endif

    float omega = 0.0f;

    if (s_mode == CHASSIS_MODE_FOLLOW) {
        /* 比例项：夹角 theta -> 0；D 项固定为 0（theta 在 ±180 处跳变，微分会出尖峰） */
        s_heading_pid.kp = s_head_kp;
        /* ★ theta 跨 ±180° 时禁止积分：否则 |theta|≈180 期间积分被顶到 max_iout，
           翻过 180 之后积分会以【旧符号】顶住新方向的 P 项，表现为"反向时卡住不动"。
           默认 s_head_ki=0 所以原本不发作，但一旦在线把 head_ki 调大就会踩到。 */
        s_heading_pid.ki = (fabsf(Heading()) < 90.0f) ? s_head_ki : 0.0f;
        s_heading_pid.kd = 0.0f;
        PID_Set_Target(&s_heading_pid, Heading());
        omega = PID_Calculate(&s_heading_pid, 0.0f, 0.0f);

        /* ★ 阻尼项：用底盘【实测】角速度抑制超调。不用 d(theta)/dt（theta 有 ±180 跳变，
           微分会出尖峰），实测角速度更真实；只有 P 时会满速冲过 0 再折返形成极限环。
           ⚠️ 架空调试时轮子空转，该反馈不反映车体真实转动。 */
        float vx_fb, vy_fb, omega_fb;
        Chassis_Get_Velocity(s_chassis, &vx_fb, &vy_fb, &omega_fb);
        omega -= s_head_kd * omega_fb;
    } else {
        /* 手动 = 右摇杆比例旋转；小陀螺 = 恒定转速 */
        omega = Chassis_Spin_Omega();
    }

    /* 航向源不可信：方向没有可靠参考，禁止自转 */
    if (!src_ok) {
        omega = 0.0f;
    }

    omega *= s_omega_dir;

    /* CAN 原始收帧数的 float 镜像（排错用）：放在 1kHz 任务里转换，避免在 CAN 中断里做软浮点 */
    user_can_1.rx_count_f = (float)user_can_1.rx_count;
    user_can_2.rx_count_f = (float)user_can_2.rx_count;

    Chassis_Set_Velocity(s_chassis, vx_body, vy_body, Chassis_Slew_Omega(omega));
    Chassis_Update(s_chassis);
}

Chassis_Mode ChassisControl_Get_Mode(void) {
    return s_mode;
}

void ChassisControl_Task(void* arg) {
    (void)arg;
    ChassisControl_Update();
}

void ChassisControl_JScopeCallback(void* arg) {
    (void)arg;
    if (s_chassis == NULL) {
        return;
    }
    jscope_transmit.val_1 = s_chassis->vx;
    jscope_transmit.val_2 = s_chassis->vy;
    jscope_transmit.val_3 = s_chassis->omega;
    JScope_Transmit(100);
}

/* ---- 在线调参变量表 ---- */

static const SERIALPLOT_VAR chassis_tunable[] = {
    /* 串口绘图通道在线切换（发 plot_mode=N#）：0=姿态(4ch) 1=底盘4轮(8ch) 2=云台yaw(5ch)
       3=云台pitch(6ch) 4=射击机构(8ch) 5=自瞄(6ch) 6=四元数(8ch) 7=视觉发送包镜像(10ch)
       8=自瞄接管(8ch) 9=视觉接收包镜像(8ch/Frame Size 32)。⚠️ 上位机通道数也要跟着改
       （serialplot.py COM4 N）*/
    {"plot_mode",  &s_plot_mode,   0.0f,   9.0f,  Chassis_Apply_Plot_Mode},
    {"test_wheel", &s_test_wheel, -4.0f, 4.0f,   NULL},
    {"test_rpm",   &s_test_rpm,    0.0f, 3000.0f, NULL},
    /* ⚠️ 上限必须是 CHASSIS_MAX_W：Chassis_Set_Velocity 里有幅值钳位，
       填得比它大只会让人以为调了有用 */
    {"spin_w",   &s_spin_w,   0.0f,  CHASSIS_MAX_W,  NULL},
    {"omega_dir",&s_omega_dir,-1.0f,   1.0f,  NULL},
    {"dead",     &s_dead,     0.0f, 100.0f,  NULL},
    {"head_kp",  &s_head_kp,  0.0f, 100.0f,  NULL},
    {"head_ki",  &s_head_ki,  0.0f,  10.0f,  NULL},
    {"head_kd",  &s_head_kd,  0.0f,  10.0f,  NULL},
    {"wheel_kp", &s_wheel_kp, 0.0f, 100.0f,  Chassis_Apply_Wheel_Gain},
    {"wheel_ki", &s_wheel_ki, 0.0f,  10.0f,  Chassis_Apply_Wheel_Gain},
    {"wheel_kd", &s_wheel_kd, 0.0f,  10.0f,  Chassis_Apply_Wheel_Gain},
    {"rev_fl",   &s_rev_fl,  -1.0f,   1.0f,  Chassis_Apply_Reverse},
    {"rev_fr",   &s_rev_fr,  -1.0f,   1.0f,  Chassis_Apply_Reverse},
    {"rev_rl",   &s_rev_rl,  -1.0f,   1.0f,  Chassis_Apply_Reverse},
    {"rev_rr",   &s_rev_rr,  -1.0f,   1.0f,  Chassis_Apply_Reverse},
    {"src_protect", &s_src_protect, 0.0f, 1.0f, NULL},
    /* 云台遥控通道：0=右手水平 1=右手垂直 2=左手水平 3=左手垂直 */
    {"gimbal_ch",   &user_gimbal.rc_ch, 0.0f, 3.0f, NULL},
    {"gimbal_max_w", &user_gimbal.max_w, 0.0f, 30.0f, NULL},
    {"gimbal_world_kp", &user_gimbal.world_kp, 0.0f, 50.0f, NULL},
    {"gimbal_world_kd", &user_gimbal.world_kd, 0.0f, 2.0f, NULL},
    {"gimbal_world_dead", &user_gimbal.world_dead, 0.0f, 5.0f, NULL},
    /* ★ 摇杆速率限斜率 deg/s²：治"转快了冲出去再回来"。太小跟手变钝，太大等于没加 */
    {"gimbal_rate_slew", &user_gimbal.rate_slew, 0.0f, 20000.0f, NULL},
    {"gimbal_ff",  &user_gimbal.ff_gain, 0.0f, 1.0f, NULL},
    /* ★★ 摇杆速率前馈增益（跟手主旋钮）：1.0 = 运动学精确值；>1 略微超喂，起步更利落。
       前馈不在反馈回路里 -> 改它不影响稳定性，比加 gimbal_kp 安全。
       代价：走的过程中云台略微"领着"目标，太大（>1.3）松杆会有橡皮筋感。 */
    {"gimbal_kf",  &user_gimbal.stick_kf, 0.0f, 2.0f, NULL},
    /* ★★★ 陀螺零偏在线跟踪系数（治"不动摇杆也隔几秒飘一次"）。填 0 = 关闭，可做 A/B 对比。
       默认 0.0002（1kHz 下 τ=5s）。它只在【摇杆居中 + 编码器没动】时工作。 */
    {"gimbal_bias_k", &user_gimbal.bias_k, 0.0f, 0.01f, NULL},
    /* ---- PID 测试波形（调内环用）：wave_en=1 后波形直接当转速指令，绕过外环
       wave_type: 0=HOLD 1=STEP 2=SQUARE 3=SINE 4=TRIANGLE 5=SAW */
    {"wave_en",   &user_gimbal.wave_en,   0.0f,   1.0f, Gimbal_Wave_Apply},
    {"wave_type", &user_gimbal.wave_type, 0.0f,   5.0f, Gimbal_Wave_Apply},
    {"wave_amp",  &user_gimbal.wave_amp,  0.0f, 150.0f, Gimbal_Wave_Apply},
    {"wave_freq", &user_gimbal.wave_freq, 0.05f, 20.0f, Gimbal_Wave_Apply},
    {"wave_duty", &user_gimbal.wave_duty, 0.0f,   1.0f, Gimbal_Wave_Apply},
    {"gimbal_kp",  &user_gimbal_yaw_pid.kp, 0.0f, 200.0f, NULL},
    {"gimbal_ki",  &user_gimbal_yaw_pid.ki, 0.0f,  50.0f, NULL},
    {"gimbal_kd",  &user_gimbal_yaw_pid.kd, 0.0f,  50.0f, NULL},
    /* ★★★ 电流权限（满量程 16384）：yaw"一直不硬"的第一嫌疑就在这里。
       原来是 3000（只有 18%，0.55A）；V2.2 默认 12000（2.2A）。可以在线试到 16384 看手感/发热。 */
    {"gimbal_max_out", &user_gimbal_yaw_pid.max_out, 0.0f, 16384.0f, NULL},

    /* ---- ★★ 自瞄（视觉接管）----
       顺序建议：先 plot_mode=5# 看 vtgt/vyaw/verr/vok/vfps 确认视觉链路通了，
       再 yaw_auto=1# 让视觉接管；pitch 与 yaw 共用同一份"有效/无效"判定。
       ⚠️ aim_ff_gain / aim_ff_pitch 就是参考工程的 0.5 / 0.6（yaw / pitch 的角速度前馈系数）。
          允许填负值：万一实测前馈方向相反（表现为"抢跑/越追越远"），直接把符号翻过来。
       ⚠️ yaw_auto 会被数据闸门自动清零（连续 5 帧视觉数据无效 -> 退出接管回摇杆），
          看到它自己掉成 0 就说明视觉那一侧在给野值/丢目标，先查视觉，不要硬开。
       ⚠️ 开火链路【没有接】：接法见参考 ita_robot.cpp:424-434，本工程还没确认扳机通道。 */
    {"yaw_auto",      &user_gimbal.yaw_auto,          0.0f,    1.0f, NULL},
    {"aim_ff_gain",   &user_gimbal.aim_ff_gain,      -1.0f,    1.0f, NULL},
    {"aim_ff_pitch",  &user_gimbal.aim_ff_gain_pitch,-1.0f,    1.0f, NULL},
    {"cam_yaw_off",   &user_gimbal.cam_yaw_off,     -10.0f,   10.0f, NULL},
    {"cam_pitch_off", &user_gimbal.cam_pitch_off,   -10.0f,   10.0f, NULL},
    /* 延迟补偿 ms：视觉"看到"的是过去的画面，用云台角速度外推到"现在"。需按实测改 */
    {"aim_delay_ms",  &user_gimbal.aim_delay_ms,      0.0f,  200.0f, NULL},
    {"aim_timeout_ms",&user_gimbal.aim_timeout_ms,    0.0f, 1000.0f, NULL},

    /* ---- 云台 pitch（GM6020 ID4，挂 CAN2，串级 + 增强 PID）----
       外环=角度环(deg->RPM)，内环=速度环(RPM->电流)；⚠️ pitch_spd_ki 不要为 0，顶重力
       靠【内环】积分。_da=低通 _dz=死区 _ia/_ib=变速积分 _is=分离 _mo/_mi/_md=三段限幅 */
    {"pitch_ch",     &user_gimbal.pitch_rc_ch, 0.0f,   3.0f, NULL},
    /* ★ Mahony AHRS 增益（在线可调）：kp 越大越信加速度计；ki>0 才会估陀螺零偏 */
    {"ahrs_kp",   &user_attitude.ahrs.kp,      0.0f, 10.0f,  NULL},
    {"ahrs_ki",   &user_attitude.ahrs.ki,      0.0f,  0.5f,  NULL},
    {"ahrs_alpf", &user_attitude.ahrs.acc_lpf, 0.0f,  1.0f,  NULL},
    {"pitch_dead",   &user_gimbal.pitch_dead, 0.0f, 200.0f, NULL},
    {"pitch_max_w",  &user_gimbal.pitch_max_w, 0.0f, 360.0f, NULL},
    {"pitch_auto",   &user_gimbal.pitch_auto,  0.0f,   1.0f, NULL},
    {"pitch_ang_kp", &user_gimbal.pitch_pid_angle.kp, 0.0f, 100.0f, NULL},
    /* ★★★ pitch"轻微推动响应慢/不跟手"的正解：目标速度前馈 KF。
       理论值 0.167（= 1/6，因为 1RPM = 6deg/s）。0 = 纯 P，要产生转速就必须先攒误差
       （跟上 120deg/s 需要 20RPM 输出 -> kp=12 时要先有 1.67° 滞后）-> 手感慢半拍。
       给到 0.167 滞后归零；给多了会过冲。前馈不受死区影响，所以小推杆也立刻有响应。 */
    {"pitch_ang_kf", &user_gimbal.pitch_pid_angle.kf, 0.0f, 1.0f, NULL},
    {"pitch_ang_ki", &user_gimbal.pitch_pid_angle.ki, 0.0f,  50.0f, NULL},
    {"pitch_ang_kd", &user_gimbal.pitch_pid_angle.kd, 0.0f,  20.0f, NULL},
    {"pitch_ang_kf", &user_gimbal.pitch_pid_angle.kf, 0.0f,   5.0f, NULL},
    {"pitch_ang_dz", &user_gimbal.pitch_pid_angle.dead_zone, 0.0f, 2.0f, NULL},
    {"pitch_ang_da", &user_gimbal.pitch_pid_angle.d_filter_alpha, 0.0f, 1.0f, NULL},
    {"pitch_ang_df", &user_gimbal.pitch_pid_angle.d_first, 0.0f, 1.0f, NULL},
    {"pitch_ang_ia", &user_gimbal.pitch_pid_angle.i_var_speed_a, 0.0f, 60.0f, NULL},
    {"pitch_ang_ib", &user_gimbal.pitch_pid_angle.i_var_speed_b, 0.0f, 120.0f, NULL},
    {"pitch_ang_is", &user_gimbal.pitch_pid_angle.i_separate, 0.0f, 120.0f, NULL},
    {"pitch_ang_mo", &user_gimbal.pitch_pid_angle.max_out, 0.0f, 500.0f, NULL},
    {"pitch_ang_mi", &user_gimbal.pitch_pid_angle.max_iout, 0.0f, 500.0f, NULL},
    {"pitch_ang_md", &user_gimbal.pitch_pid_angle.max_dout, 0.0f, 500.0f, NULL},
    {"pitch_spd_kp", &user_gimbal.pitch_pid_speed.kp, 0.0f, 300.0f, NULL},
    {"pitch_spd_ki", &user_gimbal.pitch_pid_speed.ki, 0.0f, 3000.0f, NULL},
    {"pitch_spd_kd", &user_gimbal.pitch_pid_speed.kd, 0.0f, 30.0f, NULL},
    {"pitch_spd_da", &user_gimbal.pitch_pid_speed.d_filter_alpha, 0.0f, 1.0f, NULL},
    {"pitch_spd_df", &user_gimbal.pitch_pid_speed.d_first, 0.0f, 1.0f, NULL},
    /* ⚠️ pitch_spd_zb（零位积分泄放）pitch 轴必须保持 0：开了之后静止时积分被放掉，枪管会慢慢下沉 */
    {"pitch_spd_zb", &user_gimbal.pitch_pid_speed.zero_bleed, 0.0f, 1.0f, NULL},
    {"pitch_spd_mo", &user_gimbal.pitch_pid_speed.max_out, 0.0f, 16384.0f, NULL},
    {"pitch_spd_mi", &user_gimbal.pitch_pid_speed.max_iout, 0.0f, 16384.0f, NULL},
    {"pitch_spd_md", &user_gimbal.pitch_pid_speed.max_dout, 0.0f, 16384.0f, NULL},

    /* ---- 射击机构（两摩擦轮 + 一拨弹轮，都挂 CAN2）；fire_* 只挂左轮，回调同步右轮 ---- */
    {"fire_rpm",     &user_shooter.fire_rpm,            0.0f, 9000.0f, NULL},
    {"step_deg",     &user_shooter.step_dial_deg,       1.0f,  180.0f, NULL},
    /* ★ 拨盘方向：转反了就翻这个（-1 = 反向）。作用在目标角上，位置环反向是安全的 */
    {"feed_dir",     &user_shooter.feeder_dir,         -1.0f,    1.0f, NULL},
    {"fire_int",     &user_shooter.fire_interval_ms,   30.0f, 1000.0f, NULL},
    {"fire_kp",      &user_shooter.friction_left_pid.kp, 0.0f, 200.0f, Chassis_Apply_Friction_Gain},
    {"fire_ki",      &user_shooter.friction_left_pid.ki, 0.0f,  50.0f, Chassis_Apply_Friction_Gain},
    {"fire_kd",      &user_shooter.friction_left_pid.kd, 0.0f,  50.0f, Chassis_Apply_Friction_Gain},
    {"feed_ang_kp",  &user_shooter.feeder_pid_angle.kp,  0.0f,  50.0f, NULL},
    {"feed_ang_kd",  &user_shooter.feeder_pid_angle.kd,  0.0f,  10.0f, NULL},
    {"feed_spd_kp",  &user_shooter.feeder_pid_speed.kp,  0.0f, 100.0f, NULL},
    {"feed_spd_ki",  &user_shooter.feeder_pid_speed.ki,  0.0f,  20.0f, NULL},
    /* ★★★ 拨盘转速的两个真正瓶颈（不是 KP/KI）：
       feed_ang_mo = 外环(角度)的输出限幅，也就是【拨盘最高转速 RPM】。
         一步 = 45°×36 = 1620° 转子角，一步最快耗时 = 1620/(feed_ang_mo×6) 秒。
         原来 300 -> 一步 0.9s；现在 2500 -> 一步约 0.11s。想更快直接在线加这个。 */
    {"feed_ang_mo",  &user_shooter.feeder_pid_angle.max_out, 0.0f, 10000.0f, NULL},
    {"feed_spd_mo",  &user_shooter.feeder_pid_speed.max_out, 0.0f, 16384.0f, NULL},
    /* 拨盘功率上限 W（45W ≈ 1.9A）。加速不够、拨盘发软就加它；发热不对就减 */
    {"feed_pwr",     &user_feeder.power_limit,               0.0f,   120.0f, NULL},

    /* ---- 弹道解算（牛顿迭代）：dist/height 现在手动给，以后可接视觉/裁判系统；
       v0_k/v0_b 是摩擦轮转速->初速的线性模型，必须按实测标定 */
    {"dist",         &user_shooter.distance_m, 0.3f, 30.0f, NULL},
    {"height",       &user_shooter.height_m,  -3.0f,  3.0f, NULL},
    {"auto_aim",     &user_shooter.auto_aim,   0.0f,  1.0f, NULL},
    {"v0_k",         &user_shooter.ballistics.v0_k, 0.0f, 0.02f, NULL},
    {"v0_b",         &user_shooter.ballistics.v0_b, 0.0f, 30.0f, NULL},
};

const SERIALPLOT_VAR* ChassisControl_Get_Tunable(uint8_t* count) {
    *count = (uint8_t)(sizeof(chassis_tunable) / sizeof(chassis_tunable[0]));
    return chassis_tunable;
}
