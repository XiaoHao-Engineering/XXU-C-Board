#ifndef USER_PID_ADV_H
#define USER_PID_ADV_H

#include "main.h"
#include "user_controller.h"

/* 增强型 PID —— 只给需要这些特性的地方用（当前是云台 pitch 双环）。
   不改 user_pid.c 的原因：它的 ki/kd 把 dt 隐含吸收了，加显式 dt 会改变量纲，
   底盘轮速环与云台 yaw 已调好的参数会全部失配。

   特性：显式 dt / 首拍保护 / 微分先行 / D 一阶低通 / 积分·微分·总输出三段限幅 /
        双域抗积分饱和 / 变速积分 / 积分分离 / 死区 / 目标速度前馈 / 方向枚举 /
        零位积分泄放 / 全套 setter + 分项观测 + Clear()。
   两处关键修正（原因见 .c）：① 首拍保护 —— 使能第一拍只建立基准，
   不加微分/前馈，否则第一拍满量程冲击；② 积分分离用衰减或冻结而非硬清零 ——
   pitch 靠积分顶重力。Init() 只给安全默认值，参数用 setter 逐项配。 */

typedef enum {
    PID_ADV_DIRECT = 0,
    PID_ADV_REVERSE,
} PID_Adv_Direction;

typedef enum {
    PID_ADV_I_SEP_DECAY = 0,    /* 超阈值时把积分按 0.9 衰减（默认，推荐）*/
    PID_ADV_I_SEP_FREEZE,       /* 超阈值时冻结积分（保持不动）*/
} PID_Adv_ISeparate_Mode;

typedef struct {
    CONTROLLER_INTERFACE_FUNC

    /* ---- 配置参数（Reset / Clear 都不动；各限幅 0=不限，阈值/死区 0=关闭）---- */
    float kp;
    float ki;                      /* 积分系数  单位：输出/(误差·s) */
    float kd;                      /* 微分系数  单位：输出/(误差/s) */
    float kf;
    float dt;                      /* 控制周期 s（必须是真实调用周期）*/
    float max_out;
    float max_iout;
    float max_dout;
    float dead_zone;
    float d_filter_alpha;
    float i_var_speed_a;           /* 变速积分：a=定速段阈值，b=过渡带宽度 */
    float i_var_speed_b;
    float i_separate;              /* 积分分离阈值 */
    /* ⚠️ 下面两个开关用 float 而不是 uint8_t：在线调参表只支持 float*，
       用 float 才能挂进串口绘图直接在线开关。取值 0 或 1。 */
    float d_first;                 /* 1=D 作用在测量值上（微分先行）*/
    float zero_bleed;              /* 1=零位积分泄放（⚠️ pitch 轴禁止开）*/
    uint8_t direction;
    uint8_t i_separate_mode;

    /* ---- 运行时状态（pout/iout/dout/fout = 分项输出，integral = 积分累加器）---- */
    float set;
    float fdb;
    /* ★★ 外部前馈（量纲 = 输出量纲），直接加在输出上：pout+iout+dout+fout+ff_ext。
       云台 pitch 的"视觉角速度前馈"就灌在这里：本工程 pitch 是【外环角度(deg->RPM) +
       内环速度(RPM->电流)】的串级，外环的输出就是内环（速度环）的目标 —— 所以
       ff_ext = 目标角速度[deg/s] / 6 (RPM) 等价于参考工程 ita_robot.cpp:731 的
       Motor_Pitch.Set_Feedforward_Omega()，即"叠加到速度环的目标上"。
       ⚠️ 它是【指令】不是状态：PID_Adv_Init 的 memset 让它默认 0，
          所以对现有使用者（只有云台 pitch）零影响；不用时必须显式写回 0。 */
    float ff_ext;
    float out;
    float pout;
    float iout;
    float dout;
    float fout;
    float error;                   /* 本次误差（已含方向与死区处理）*/
    float pre_error;
    float pre_now;
    float pre_target;
    float integral;
    float d_filtered;
    float i_speed_ratio;
    uint8_t first_flag;            /* 0=还没跑过第一拍，走首拍保护 */
} PID_Adv_Controller;

/* 函数声明 ---------------------------------------------------------------*/
void PID_Adv_Init(PID_Adv_Controller* pid);

void  PID_Adv_Set_Target(void* controller, float target);
/* 外部前馈直接叠加到输出上（量纲 = 输出量纲）。不用时必须写回 0 */
void  PID_Adv_Set_Feedforward(PID_Adv_Controller* pid, float ff);
float PID_Adv_Calculate(void* controller, float main_feedback, float sub_feedback);
float PID_Adv_Get_Output(void* controller);
void  PID_Adv_Set_MaxOut(void* controller, float max_out);
float PID_Adv_Get_MaxOut(void* controller);
void  PID_Adv_Reset(void* controller);

/* 参数设置（全部可在运行期改）*/
void PID_Adv_Set_K_P(PID_Adv_Controller* pid, float kp);
void PID_Adv_Set_K_I(PID_Adv_Controller* pid, float ki);
void PID_Adv_Set_K_D(PID_Adv_Controller* pid, float kd);
void PID_Adv_Set_K_F(PID_Adv_Controller* pid, float kf);
void PID_Adv_Set_DT(PID_Adv_Controller* pid, float dt);
void PID_Adv_Set_Max_Out(PID_Adv_Controller* pid, float max_out);
void PID_Adv_Set_Max_IOut(PID_Adv_Controller* pid, float max_iout);
void PID_Adv_Set_Max_DOut(PID_Adv_Controller* pid, float max_dout);
void PID_Adv_Set_Dead_Zone(PID_Adv_Controller* pid, float dead_zone);
void PID_Adv_Set_D_Filter_Alpha(PID_Adv_Controller* pid, float alpha);
void PID_Adv_Set_I_Variable_Speed(PID_Adv_Controller* pid, float a, float b);
void PID_Adv_Set_I_Separate(PID_Adv_Controller* pid, float threshold, PID_Adv_ISeparate_Mode mode);
void PID_Adv_Set_D_First(PID_Adv_Controller* pid, float enable);
void PID_Adv_Set_Direction(PID_Adv_Controller* pid, PID_Adv_Direction direction);
void PID_Adv_Set_Zero_Bleed(PID_Adv_Controller* pid, float enable);

/* 分项观测（调参定位用）*/
float PID_Adv_Get_P_Out(const PID_Adv_Controller* pid);
float PID_Adv_Get_I_Out(const PID_Adv_Controller* pid);
float PID_Adv_Get_D_Out(const PID_Adv_Controller* pid);
float PID_Adv_Get_F_Out(const PID_Adv_Controller* pid);
float PID_Adv_Get_Error(const PID_Adv_Controller* pid);
float PID_Adv_Get_Integral(const PID_Adv_Controller* pid);

#endif
