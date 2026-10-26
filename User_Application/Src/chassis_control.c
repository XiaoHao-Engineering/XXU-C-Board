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
static uint8_t        s_prev_auto = 0;   /* 上一拍 yaw_auto（0=摇杆 1=自瞄），用来检测"刚退出自瞄" */
static Chassis_HeadingSource s_heading_src = ChassisControl_NoHeading;
static Chassis_SourceValid   s_source_valid = ChassisControl_SourceAlwaysValid;

/* ==================== 串口绘图：只有一个固定模式（12 通道）====================
 *
 * ★ 上电即这个模式，没有模式切换、没有 plot_mode 命令（原来的 case 0~10 已全部删除）。
 *
 * 通道顺序（从上到下，不能改；SerialPlot 里的通道号就是它）：
 *   ch1  target_yaw           视觉目标 YAW 角度        弧度      user_vision.target_yaw
 *   ch2  target_yaw_vel       视觉目标 YAW 角速度      弧度/秒   user_vision.target_yaw_vel
 *   ch3  target_yaw_acc       视觉目标 YAW 角加速度    弧度/秒²  user_vision.target_yaw_acc
 *   ch4  target_pitch         视觉目标 PITCH 角度      弧度      user_vision.target_pitch
 *   ch5  target_pitch_vel     视觉目标 PITCH 角速度    弧度/秒   user_vision.target_pitch_vel
 *   ch6  target_pitch_acc     视觉目标 PITCH 角加速度  弧度/秒²  user_vision.target_pitch_acc
 *   ch7  yaw_fb_rad           位置环实际用的 yaw 反馈  弧度      user_gimbal.yaw_fb_rad
 *                             = imu_yaw − bias_accum（世界系，位置环的同一个 world_yaw）
 *   ch8  yaw_rad              YAW 电机编码器关节角    弧度(±π)  user_gimbal.yaw_rad
 *                             物理量、不漂，用来对比 ch7(IMU) 看漂移
 *   ch9  tx_qw                发给视觉的四元数 w      无量纲    user_gimbal.tx_qw
 *   ch10 tx_qx                发给视觉的四元数 x      无量纲    user_gimbal.tx_qx
 *   ch11 tx_qy                发给视觉的四元数 y      无量纲    user_gimbal.tx_qy
 *   ch12 tx_qz                发给视觉的四元数 z      无量纲    user_gimbal.tx_qz
 *
 * ★ ch1~ch6 就是视觉通过 USART1 发过来的 6 个数，顺序完全一致（yaw 角、yaw 角速度、
 *   yaw 角加速度、pitch 角、pitch 角速度、pitch 角加速度）。这里传的是 user_vision 里
 *   【包内原始单位】的镜像（弧度 / 弧度每秒 / 弧度每秒²）；需要度制时用对应的 _deg 字段
 *   （user_vision.target_yaw_deg 等，收帧时已 ×VISION_PKG2DEG）。
 *
 * ★ ch9~ch12 就是【gimbal.c 每拍算好、经 USART1 发给视觉】的那组四元数
 *   （tx_qw/tx_qx/tx_qy/tx_qz，已补 -bias_accum 旋转，与发给视觉的 yaw 字段同一基准）。
 *   这里引用的是同一份字段、不是另算一份，所以画出来的就是视觉真正收到的数据。
 *
 * ★ ch1(视觉目标 yaw) 与 ch7(世界系 yaw 反馈) 同在世界系，二者之差可当作 yaw 跟随
 *   误差观察（前提是视觉与 IMU 的零偏已对齐）。
 *
 * ★★ SerialPlot（hyOzd 那个 Qt 上位机）里必须这样设，填错就画不出来：
 *      Frame Start = AB
 *      #Channels   = 12
 *      Frame Size  = 48            <-- 12 通道 × 4 字节（float32），不是 50！
 *      Number Type = float（32 位）/ Little Endian
 *      Checksum    = 累加和（Sum，在通道数据之后）
 *    线上实际帧长 = 1(帧头) + 48(数据) + 1(校验和) = 50 字节。
 *    ★ 通道数 / Frame Size 与固件不一致的典型症状：串口"有数据在跑"，
 *      但 SerialPlot 显示 0sps + Checksum failed，曲线完全不动 —— 帧长对不上，
 *      永远同步不到帧头。
 *    ★ 带宽：50 字节 × 100Hz（CHASSIS_SERIALPLOT_PERIOD_MS=10）= 5.0 kB/s ≈ 50kbps
 *      （含起始/停止位），115200 8N1 有 11.52kB/s，占用约 43%，余量充足。
 *
 * ★★★ 这里【只允许传 float 变量的地址】：Serialplot_Set_Data() 内部对每个指针
 *     按 sizeof(float)=4 做 memcpy，传 uint16_t / uint8_t / int32_t 的地址会读到
 *     相邻字段的字节 -> 解出 NaN / 1e38 -> SerialPlot 画不出线。
 *     同理，通道数必须与实参个数完全一致（12 个），否则 data_num 与指针数不符。
 *     （s_yaw_encoder_abs / s_pitch_encoder_abs 两个 float 镜像仍保留并更新，
 *       只是当前不再是绘图通道，以后要用可以直接加回。）
 * ========================================================================== */

/* 单圈绝对角用的编码器分辨率：13 位 = 一圈 8192 个计数（不是 8191）。
   必须与 user_dji_motor.c 里的 ENCODER_CPR 一致，否则每圈多算 0.0122%。 */
#define PLOT_ENCODER_CPR  (8192.0f)

/* GM6020 / C6x0 反馈里的【单圈绝对角】float 镜像（1kHz 同步，见 ChassisControl_Update）*/
static float s_yaw_encoder_abs   = 0.0f;
static float s_pitch_encoder_abs = 0.0f;

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

/* ★ 这里【绝对不能】再调 Serialplot_Set_Data()：
   原来 case 6 里塞了一段 8 通道的绘图登记（当时想借用通道号做快捷切换），
   后果是——只要有谁把通道号设成 6 就会把已登记好的固定通道覆盖成 8 通道，
   帧长跟着变长（当时从 26 变成 34 字节），上位机立刻全部 Checksum failed。已删除。
   绘图通道现在只在 Chassis_Apply_Plot_Mode() 里登记一次。 */
static int16_t RC_Channel(const uint8_t ch) {
    switch (ch) {
        case 0:  return s_dbus->ch0;
        case 1:  return s_dbus->ch1;
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

/* 电机反馈里的 encoder 计数（0 ~ 8191）-> 【单圈绝对机械角】度（0 ~ 360）。
   ★ GM6020 是直驱（转子 = 输出轴），所以转子单圈角就是"枪/云台的机械角"，
     上电位置不同读数就不同，掉电不丢（绝对编码器），过零点会从 360 跳回 0。 */
static float Chassis_Encoder_SingleTurn_Deg(const DJI_MOTOR_DRIVES* motor) {
    return (float)motor->rotor_angle / PLOT_ENCODER_CPR * 360.0f;
}

/* 登记本帧要上报的 12 个通道。
   ★ 单模式：上电就这一套，没有任何模式切换、没有 plot_mode 命令。
   ★ 通道顺序 / 含义 / 单位 / 上位机设置见本文件顶部的说明块。
   ★ 只许传【float 变量】的地址：Serialplot_Set_Data() 按 sizeof(float)
     逐 4 字节 memcpy，传非 float 的地址会读到相邻字段的字节 -> NaN / 1e38。
   ★ 通道数必须与实参个数一致（12）；上位机固定 Frame Start=AB / #Channels=12 /
     Frame Size=48（线上 1 + 12×4 + 1 = 50 字节）。
   ★ ch1~ch6 = 视觉发的 6 个数，ch7 = yaw 控制反馈，ch8 = YAW 编码器关节角，
     ch9~ch12 = 发给视觉的四元数 w/x/y/z（gimbal.c 里每拍更新，见 tx_qw 那一段）。*/
static void Chassis_Apply_Plot_Mode(void) {
    Serialplot_Set_Data(&user_serialplot, 12,
                        &user_vision.target_yaw,                /* ch1 视觉 yaw 角度        弧度     */
                        &user_vision.target_yaw_vel,        /* ch2 视觉 yaw 角速度      弧度/秒  */
                        &user_vision.target_yaw_acc,        /* ch3 视觉 yaw 角加速度    弧度/秒² */
                        &user_vision.target_pitch,            /* ch4 视觉 pitch 角度      弧度     */
                        &user_vision.target_pitch_vel,    /* ch5 视觉 pitch 角速度    弧度/秒  */
                        &user_vision.target_pitch_acc,    /* ch6 视觉 pitch 角加速度  弧度/秒² */
                        &user_gimbal.yaw_fb_rad,           /* ch7 yaw 控制反馈（位置环实际用的世界角）弧度 */
    &user_gimbal.yaw_rad,           /* ch8 YAW 电机编码器关节角 弧度(±π)，物理量、不漂，用来对比 ch7(IMU) 看漂移 */
    &user_gimbal.tx_qw,    /* ch9  发给视觉的四元数 w */
    &user_bmi088.temperature,  /* ch10 ★IMU 温度(°C)，看恒温后漂移 */
    &user_gimbal.pitch,              /* ch11 ★pitch 实际值(度) */
    &user_gimbal.pitch_target_ext);   /* ch12 ★pitch 目标值(度) */
}


void ChassisControl_Init(Chassis* chassis, DBUS_DRIVES* dbus) {
    s_chassis = chassis;
    s_dbus = dbus;
    s_mode = CHASSIS_MODE_MANUAL;
    s_offline_ms = 0;
    s_prev_sw1 = 0;
    s_prev_auto = 0;
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
    /* ---- ★★ 绘图用 float 镜像（1kHz 同步）----
       必须在【所有 early-return 之前】更新：遥控器断线、单轮测试这些分支都会提前
       return，镜像放在后面就会冻住，看起来像"这两个通道没数据"。
       原始量 rotor_angle 是 uint16_t，不能把它的地址传给 Serialplot_Set_Data
       （它按 float 逐 4 字节 memcpy），所以必须先换算成 float 存到镜像里。*/
    s_yaw_encoder_abs   = Chassis_Encoder_SingleTurn_Deg(&user_gimbal_yaw_motor);
    s_pitch_encoder_abs = Chassis_Encoder_SingleTurn_Deg(&user_gimbal_pitch_motor);

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

    /* ★★ sw1（左上角拨杆）是【双语义拨杆】，语义由 yaw_auto 决定：
         yaw_auto = 0 -> sw1 归【底盘】：上=跟随 中=手动 下=小陀螺（就是下面这套映射）
         yaw_auto = 1 -> sw1 归【射击】：上=摩擦轮开（见 shooter.c 的 arm 总闸），
                         底盘模式不再由 sw1 决定
       ⚠️ 所以【跟随模式只能在 yaw_auto=0 时使用】；自瞄期间若已处于跟随，立即退回手动。
       ⚠️ 刚退出自瞄时要强制重新解析一次 sw1（left_auto），否则拨杆没动过、模式会卡在手动。 */
    const uint8_t auto_on   = (user_gimbal.yaw_auto >= 0.5f) ? 1 : 0;
    const uint8_t left_auto = ((s_prev_auto != 0u) && (auto_on == 0u)) ? 1 : 0;
    s_prev_auto = auto_on;

    if (auto_on) {
        /* 自瞄接手：跟随模式不可用 -> 强制回手动；小陀螺仍允许（视觉控 yaw、底盘自转互不冲突）*/
        if (s_mode == CHASSIS_MODE_FOLLOW) {
            s_mode = CHASSIS_MODE_MANUAL;
            Heading_Pid_Reset();
            s_w_cmd = 0.0f;
        }
    } else if ((s_dbus->sw1 != s_prev_sw1) || left_auto) {
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
    /* ★ 串口绘图现在只有一个固定模式（7 通道），原来的 plot_mode 项已删除，
       没有在线切换、也没有 plot_mode 命令。通道定义见 Chassis_Apply_Plot_Mode()。*/
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
    /* ---- ★★ 锁世界位置环（PD）+ 前馈 + 目标平滑：治"调过去晃一下再回来"的五个旋钮 ----
       gimbal_world_kp   位置环 P：大 = 更硬更快、但过冲更大；小 = 稳但迟钝
       gimbal_world_kd   陀螺阻尼 D：大 = 刹车更重、过冲更小；太大会发粘
       gimbal_world_da   阻尼源低通 alpha(0~1)：大 = 微分滞后小、刹车更及时、噪声更大
       gimbal_world_dead 位置死区 deg：压住静止漂移（只清 P 项）
       gimbal_tgt_smooth 目标角一阶平滑 ms（0=关闭）：抹平阶跃目标，抑过冲最有效
       gimbal_ff         底盘角速度前馈 0~1：1.0=运动学精确抵消；阶跃目标给满会冲过头
       gimbal_rate_slew  摇杆速率限斜率 deg/s²：限制"前馈突然掉到 0"的冲击
       ⚠️ 整定顺序：先 gimbal_world_kd / gimbal_tgt_smooth，再动 gimbal_world_kp。
          ⚠️ gimbal_ki 必须保持 0（老 PID 无 dt + 整数转速反馈，开积分必静止震荡）*/
    {"gimbal_world_kp", &user_gimbal.world_kp, 0.0f, 50.0f, NULL},
    {"gimbal_world_kd", &user_gimbal.world_kd, 0.0f, 5.0f, NULL},
    /* ★ 阻尼源一阶低通系数：提高 = 微分滞后更小、阻尼更早起作用（噪声会稍大）。
       默认 0.55。陀螺噪声明显（高频"沙沙"）就降到 0.45，然后把 gimbal_world_kd 补一点 */
    {"gimbal_world_da", &user_gimbal.world_d_alpha, 0.0f, 1.0f, NULL},
    {"gimbal_world_dead", &user_gimbal.world_dead, 0.0f, 5.0f, NULL},
    /* ★★★ 目标角一阶平滑时间常数 ms：治过冲最有效的一条，默认 25。
       调大 = 目标更圆滑、过冲更小，但跟手更"慢半拍"；
       调小（0~10） = 更跟手、过冲回来一些；填 0 = 完全关闭（退回改之前的行为，便于 A/B）。
       ⚠️ 视觉跟快速移动目标觉得"跟不上"就把它调小，不要靠加 kp 去追 */
    {"gimbal_tgt_smooth", &user_gimbal.tgt_smooth_ms, 0.0f, 200.0f, NULL},
    /* ★ 自瞄接管时用的目标角平滑 ms（大 = 稳、不抖）。上面那条现在只管【摇杆/无目标】时；
       两条按 gimbal->aim_takeover 自动二选一（见 gimbal.c），不用手动切换 */
    {"gimbal_aim_smooth", &user_gimbal.tgt_smooth_aim_ms, 0.0f, 200.0f, NULL},
    /* ★ 摇杆速率限斜率 deg/s²：治"转快了冲出去再回来"。太小跟手变钝，太大等于没加 */
    {"gimbal_rate_slew", &user_gimbal.rate_slew, 0.0f, 20000.0f, NULL},
    /* ★ 底盘角速度前馈 0~1：1.0 = 运动学精确抵消底盘自转。
       ⚠️ 摇杆是【阶跃式】目标，满前馈会在起步瞬间给速度环一个转速脉冲 -> 冲过头，
          所以默认降到 0.4；跟踪匀速移动目标（视觉自瞄）时才需要调回 0.8~1.0 */
    {"gimbal_ff",  &user_gimbal.ff_gain, 0.0f, 1.0f, NULL},
    /* ★★ 摇杆速率前馈增益（跟手主旋钮）：1.0 = 运动学精确值；>1 略微超喂，起步更利落。
       前馈不在反馈回路里 -> 改它不影响稳定性，比加 gimbal_kp 安全。
       代价：走的过程中云台略微"领着"目标，太大（>1.3）松杆会有橡皮筋感。 */
    {"gimbal_kf",  &user_gimbal.stick_kf, 0.0f, 2.0f, NULL},
    /* ★★★ 陀螺零偏在线跟踪系数（治"不动摇杆也隔几秒飘一次"）。填 0 = 关闭，可做 A/B 对比。
       默认 0.0002（1kHz 下 τ=5s）。它只在【摇杆居中 + 编码器没动】时工作。 */
    {"gimbal_bias_k", &user_gimbal.bias_k, 0.0f, 0.01f, NULL},
    /* ★★ 上电静态零偏标定（状态机在 gimbal.c 的 Gimbal_Calib_Update，结果看
       user_gimbal.calib_state / calib_gyro_avg / calib_gyro_pp）：
         calib_ms      采样窗口 ms（500~5000，默认 2000）。改它只影响【下一次】标定，
                       本次采样开始时就已锁存到 calib_target_ms。
         calib_restart 写 1 立刻重新标定（状态回 IDLE，重新等加热/静止 200ms），
                       固件内部消费后自动清 0。标定期间别碰云台/底盘。 */
    {"calib_ms",      &user_gimbal.calib_ms_cfg, GIMBAL_CALIB_MS_MIN, GIMBAL_CALIB_MS_MAX, NULL},
    {"calib_restart", &user_gimbal.calib_restart, 0.0f, 1.0f, NULL},
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
       顺序建议：先看串口绘图的 ch5/ch6（遥控目标 YAW / PITCH）确认云台听摇杆，
       再 yaw_auto=1# 让视觉接管；pitch 与 yaw 共用同一份"有效/无效"判定。
       （原来看视觉链路用的是 plot_mode=5 那套通道，多模式已删除；要排查视觉链路
         直接用 tools/vision_rx_test.py 或 tools/vision_echo_test.py。）
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

    /* ---- 射击机构（两摩擦轮 + 一拨弹轮，都挂 CAN2）；fire_* 只挂左轮，回调同步右轮 ----
       ★★ 新发射逻辑（总闸 + 三档 + 停火 FSM）的旋钮都在这一段，全部是 float：
          friction_rpm  保守弹速 20m/s 对应的摩擦轮转速（≈6366 -> 取 6400）。
                        ⚠️ 它是 fire_rpm 的【别名，同一个变量】，两个名字都能改。
          fire_dead     单发【拨轮】触发死区（DBUS roll，±660）：越小越灵敏；
                        回中位判据 = 它 × 0.5（回滞，防一直推着连发）
          stopper_cur   停火 FSM 的电流阈值，单位【mA】：PPT 的 8A 就填 8000
          stopper_ms    停火 FSM 的"激活"超时 ms：PPT 是 1000
       ★ 一发角度用已有的 step_deg（默认 360/7+5 = 56.43° 拨盘角，内部 ×36 减速比）*/
    {"friction_rpm", &user_shooter.fire_rpm,            0.0f, 9000.0f, NULL},
    {"fire_dead",    &user_shooter.fire_dead,           0.0f,  660.0f, NULL},
    {"stopper_cur",  &user_shooter.stopper_cur,         0.0f, 20000.0f, NULL},
    {"stopper_ms",   &user_shooter.stopper_ms,         50.0f, 5000.0f, NULL},
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

    /* ---- ★ IMU 恒温加热（PF6 = TIM10_CH1，100ms 任务）----
       两段式：先满占空比预热到 heater_preheat(默认47℃)，再 PID 恒温 heater_target(默认50℃)。
       heater_target / heater_preheat 范围 30~60（60 已接近过温阈值，别贪高）；
       heater_duty 是【占空比上限 0~1000 千分比】，不是手动强制值：
         0    = 彻底不加热（比 heater_enable=0 更硬，PID 还在算但输出被压成 0）；
         1000 = 允许满占空比（5V/47.8Ω ≈ 0.52W，116mA）。
       heater_enable：1=允许，0=停加热。★ 温度 >65℃ 时固件会强制清 0（过温锁存），
       降温后必须人工重新发 heater_enable=1#，不会自己恢复。
       实际输出/CCR 看 user_heater.output_duty / user_heater.duty_ccr（只读，不进调参表）。*/
    {"heater_enable",  &user_heater.enabled,           0.0f,    1.0f, NULL},
    {"heater_target",  &user_heater.target_temp,      30.0f,   60.0f, NULL},
    {"heater_preheat", &user_heater.preheat_base_temp,30.0f,   60.0f, NULL},
    {"heater_duty",    &user_heater.duty_max,          0.0f, 1000.0f, NULL},
};

const SERIALPLOT_VAR* ChassisControl_Get_Tunable(uint8_t* count) {
    *count = (uint8_t)(sizeof(chassis_tunable) / sizeof(chassis_tunable[0]));
    return chassis_tunable;
}
