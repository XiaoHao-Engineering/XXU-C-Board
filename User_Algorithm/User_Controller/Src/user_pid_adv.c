#include "../user_pid_adv.h"
#include <math.h>
#include <string.h>

void PID_Adv_Init(PID_Adv_Controller* pid) {
    memset(pid, 0, sizeof(PID_Adv_Controller));

    /* 绑定接口（必须在 memset 之后，否则会被清零）*/
    pid->Set_Target  = PID_Adv_Set_Target;
    pid->Calculate   = PID_Adv_Calculate;
    pid->Get_Output  = PID_Adv_Get_Output;
    pid->Set_MaxOut  = PID_Adv_Set_MaxOut;
    pid->Get_MaxOut  = PID_Adv_Get_MaxOut;
    pid->Reset       = PID_Adv_Reset;

    /* 安全默认值：增益全 0（不配就不会输出）、限幅 0=不限、不滤波、无死区 */
    pid->dt = 0.001f;
    pid->direction = PID_ADV_DIRECT;
    pid->i_separate_mode = PID_ADV_I_SEP_DECAY;
    pid->d_first = 0;
    pid->zero_bleed = 0;
    pid->d_filter_alpha = 0.0f;
    pid->dead_zone = 0.0f;
    pid->i_var_speed_a = 0.0f;
    pid->i_var_speed_b = 0.0f;
    pid->i_separate = 0.0f;
    pid->first_flag = 0;
}

void PID_Adv_Set_Target(void* controller, const float target) {
    PID_Adv_Controller* pid = (PID_Adv_Controller*)controller;
    pid->set = target;
}

/* 外部前馈：直接叠加到输出上（量纲 = 输出量纲）。云台 pitch 用它接"视觉角速度前馈"，
   等价于参考工程 Motor_Pitch.Set_Feedforward_Omega()（见 user_pid_adv.h 的说明）。
   ⚠️ 不用时必须显式写回 0，否则会一直带着上一次的残留前馈。 */
void PID_Adv_Set_Feedforward(PID_Adv_Controller* pid, const float ff) {
    pid->ff_ext = ff;
}

/* 清空运行时状态、保留配置参数；first_flag 清零 -> 下次 Calculate 重新走首拍保护 */
void PID_Adv_Reset(void* controller) {
    PID_Adv_Controller* pid = (PID_Adv_Controller*)controller;

    pid->set = 0.0f;
    pid->fdb = 0.0f;
    pid->out = 0.0f;
    pid->pout = 0.0f;
    pid->iout = 0.0f;
    pid->dout = 0.0f;
    pid->fout = 0.0f;
    pid->ff_ext = 0.0f;          /* 外部前馈也清掉：复位后不许带残留前馈再上线 */
    pid->error = 0.0f;
    pid->pre_error = 0.0f;
    pid->pre_now = 0.0f;
    pid->pre_target = 0.0f;
    pid->integral = 0.0f;
    pid->d_filtered = 0.0f;
    pid->i_speed_ratio = 0.0f;
    pid->first_flag = 0;
}

float PID_Adv_Calculate(void* controller, const float main_feedback, const float sub_feedback) {
    PID_Adv_Controller* pid = (PID_Adv_Controller*)controller;

    (void)sub_feedback;      /* 子反馈由 Cascade_Controller 处理，本控制器不用 */

    pid->fdb = main_feedback;

    /* dt 除零保护：不合法就退回 1ms，避免整条链路算出 inf/nan */
    if (pid->dt <= 0.0f) {
        pid->dt = 0.001f;
    }

    /* ★ 首拍保护：第一拍只把历史值初始化成当前值，让微分/前馈首次差分恒为 0。
       不这么做，使能第一拍 D/F 会拿 (Now-0)/dt 算出满量程冲击（可达 −500）。 */
    if (pid->first_flag == 0) {
        pid->first_flag = 1;
        pid->error = pid->set - main_feedback;
        pid->pre_error = pid->error;
        pid->pre_now = main_feedback;
        pid->pre_target = pid->set;
        pid->integral = 0.0f;
        pid->d_filtered = 0.0f;
        pid->pout = 0.0f;
        pid->iout = 0.0f;
        pid->dout = 0.0f;
        pid->fout = 0.0f;
        pid->out = 0.0f;
        pid->i_speed_ratio = 0.0f;
        return 0.0f;
    }

    pid->error = pid->set - main_feedback;

    /* 方向：负相关被控对象在这里统一处理，调用处不用翻符号 */
    if (pid->direction == PID_ADV_REVERSE) {
        pid->error = -pid->error;
    }

    /* 死区：在算 P 之前把误差清零，于是 P 与 D 一起被压掉，零位附近无输出抖动 */
    if (pid->dead_zone > 0.0f && fabsf(pid->error) < pid->dead_zone) {
        pid->error = 0.0f;
    }

    pid->pout = pid->kp * pid->error;

    /* ---- I：变速积分 + 积分分离 + 双域限幅 ---- */
    float speed_ratio = 1.0f;

    if (pid->i_var_speed_b > 0.0f) {
        const float abs_error = fabsf(pid->error);
        if (abs_error <= pid->i_var_speed_a) {
            speed_ratio = 1.0f;
        } else if (abs_error < (pid->i_var_speed_a + pid->i_var_speed_b)) {
            speed_ratio = 1.0f - (abs_error - pid->i_var_speed_a) / pid->i_var_speed_b;
        } else {
            speed_ratio = 0.0f;
        }
    }

    /* ★ 积分分离用"冻结/按 0.9 衰减"而不是硬清零：pitch 靠积分顶重力，
       大阶跃时清零会把顶重力的积分一起抹掉，表现为掉头/下沉。 */
    if (pid->i_separate > 0.0f && fabsf(pid->error) > pid->i_separate) {
        if (pid->i_separate_mode == PID_ADV_I_SEP_DECAY) {
            pid->integral *= 0.9f;
        }
        speed_ratio = 0.0f;
    }

    pid->i_speed_ratio = speed_ratio;

    pid->integral += pid->ki * speed_ratio * pid->error * pid->dt;

    /* 双域抗积分饱和：积分项输出与积分累加器同时限幅。只限输出的话，
       ki 小时累加器会漂到很大，退出饱和极慢。这里两者是同一个量，限一次即可。 */
    if (pid->max_iout > 0.0f) {
        if (pid->integral > pid->max_iout) {
            pid->integral = pid->max_iout;
        } else if (pid->integral < -pid->max_iout) {
            pid->integral = -pid->max_iout;
        }
    }
    pid->iout = pid->integral;

    /* ---- D：微分先行 + 一阶低通 + 独立限幅 ---- */
    float d_raw;

    if (pid->d_first >= 0.5f) {
        /* 微分先行：对测量值微分。目标阶跃时误差会跳变而测量值不会，故无微分冲击 */
        d_raw = -pid->kd * (main_feedback - pid->pre_now) / pid->dt;
        if (pid->direction == PID_ADV_REVERSE) {
            d_raw = -d_raw;
        }
    } else {
        d_raw = pid->kd * (pid->error - pid->pre_error) / pid->dt;
    }

    if (pid->d_filter_alpha > 0.0f) {
        if (pid->d_filter_alpha > 1.0f) {
            pid->d_filter_alpha = 1.0f;
        }
        pid->d_filtered = pid->d_filter_alpha * d_raw
                        + (1.0f - pid->d_filter_alpha) * pid->d_filtered;
        pid->dout = pid->d_filtered;
    } else {
        pid->dout = d_raw;
    }

    /* D 项独立限幅：在求和之前夹住，不会被后级总限幅掩盖 */
    if (pid->max_dout > 0.0f) {
        if (pid->dout > pid->max_dout) {
            pid->dout = pid->max_dout;
        } else if (pid->dout < -pid->max_dout) {
            pid->dout = -pid->max_dout;
        }
    }

    float target_rate = (pid->set - pid->pre_target) / pid->dt;
    if (pid->direction == PID_ADV_REVERSE) {
        target_rate = -target_rate;
    }
    pid->fout = pid->kf * target_rate;

    /* ★ 外部前馈：与 P/I/D/F 一起求和后再统一限幅（前馈在回路外，不会影响稳定性，
       但会被 max_out 一起夹住 —— 正常量级远小于 max_out，只有异常大前馈才会被吃）*/
    pid->out = pid->pout + pid->iout + pid->dout + pid->fout + pid->ff_ext;

    if (pid->max_out > 0.0f) {
        if (pid->out > pid->max_out) {
            pid->out = pid->max_out;
        } else if (pid->out < -pid->max_out) {
            pid->out = -pid->max_out;
        }
    }

    /* 零位积分泄放（静止时缓慢放掉残留积分）：只在目标与误差都进入死区时才
       触发，所以 dead_zone=0 时永不触发。⚠️ pitch 轴禁止开启 —— 静止时也要靠
       积分顶住重力，泄放掉就会往下沉。 */
    if (pid->zero_bleed >= 0.5f
        && pid->dead_zone > 0.0f
        && fabsf(pid->set) < pid->dead_zone
        && fabsf(pid->error) < pid->dead_zone) {
        pid->integral *= 0.95f;
    }

    pid->pre_now = main_feedback;
    pid->pre_target = pid->set;
    pid->pre_error = pid->error;

    return pid->out;
}

float PID_Adv_Get_Output(void* controller) {
    const PID_Adv_Controller* pid = (PID_Adv_Controller*)controller;
    return pid->out;
}

void PID_Adv_Set_MaxOut(void* controller, const float max_out) {
    PID_Adv_Controller* pid = (PID_Adv_Controller*)controller;
    pid->max_out = max_out;
}

float PID_Adv_Get_MaxOut(void* controller) {
    const PID_Adv_Controller* pid = (PID_Adv_Controller*)controller;
    return pid->max_out;
}

/* 参数设置 ---------------------------------------------------------------*/

void PID_Adv_Set_K_P(PID_Adv_Controller* pid, const float kp) { pid->kp = kp; }
void PID_Adv_Set_K_I(PID_Adv_Controller* pid, const float ki) { pid->ki = ki; }
void PID_Adv_Set_K_D(PID_Adv_Controller* pid, const float kd) { pid->kd = kd; }
void PID_Adv_Set_K_F(PID_Adv_Controller* pid, const float kf) { pid->kf = kf; }

void PID_Adv_Set_DT(PID_Adv_Controller* pid, const float dt) {
    /* dt 突变会让 D/F 项产生一个假的巨大差分，所以按首拍保护重新起一次 */
    if (dt > 0.0f && dt != pid->dt) {
        pid->dt = dt;
        pid->first_flag = 0;
    }
}

void PID_Adv_Set_Max_Out(PID_Adv_Controller* pid, const float max_out)   { pid->max_out = max_out; }
void PID_Adv_Set_Max_IOut(PID_Adv_Controller* pid, const float max_iout) { pid->max_iout = max_iout; }
void PID_Adv_Set_Max_DOut(PID_Adv_Controller* pid, const float max_dout) { pid->max_dout = max_dout; }
void PID_Adv_Set_Dead_Zone(PID_Adv_Controller* pid, const float dead_zone) { pid->dead_zone = dead_zone; }

void PID_Adv_Set_D_Filter_Alpha(PID_Adv_Controller* pid, const float alpha) {
    float a = alpha;
    if (a < 0.0f) a = 0.0f;
    if (a > 1.0f) a = 1.0f;
    pid->d_filter_alpha = a;
}

void PID_Adv_Set_I_Variable_Speed(PID_Adv_Controller* pid, const float a, const float b) {
    pid->i_var_speed_a = (a > 0.0f) ? a : 0.0f;
    pid->i_var_speed_b = (b > 0.0f) ? b : 0.0f;
}

void PID_Adv_Set_I_Separate(PID_Adv_Controller* pid, const float threshold,
                            const PID_Adv_ISeparate_Mode mode) {
    pid->i_separate = (threshold > 0.0f) ? threshold : 0.0f;
    pid->i_separate_mode = (uint8_t)mode;
}

void PID_Adv_Set_D_First(PID_Adv_Controller* pid, const float enable) {
    pid->d_first = (enable >= 0.5f) ? 1.0f : 0.0f;
}

void PID_Adv_Set_Direction(PID_Adv_Controller* pid, const PID_Adv_Direction direction) {
    pid->direction = (uint8_t)direction;
}

void PID_Adv_Set_Zero_Bleed(PID_Adv_Controller* pid, const float enable) {
    pid->zero_bleed = (enable >= 0.5f) ? 1.0f : 0.0f;
}

/* 分项观测 ---------------------------------------------------------------*/

float PID_Adv_Get_P_Out(const PID_Adv_Controller* pid)    { return pid->pout; }
float PID_Adv_Get_I_Out(const PID_Adv_Controller* pid)    { return pid->iout; }
float PID_Adv_Get_D_Out(const PID_Adv_Controller* pid)    { return pid->dout; }
float PID_Adv_Get_F_Out(const PID_Adv_Controller* pid)    { return pid->fout; }
float PID_Adv_Get_Error(const PID_Adv_Controller* pid)    { return pid->error; }
float PID_Adv_Get_Integral(const PID_Adv_Controller* pid) { return pid->integral; }
