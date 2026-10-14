/* 包含头文件 */
#include "../user_pid_tolerance.h"
#include <math.h>
#include <string.h>

/* 函数体 */

void PID_Tolerance_Init(PID_Tolerance_Controller *pid, const float kp, const float ki, const float kd,
              const float max_out, const float max_iout, const float tolerance) {
    memset(pid, 0, sizeof(PID_Tolerance_Controller));
    
    // 绑定接口函数
    pid->Set_Target = PID_Tolerance_Set_Target;
    pid->Calculate = PID_Tolerance_Calculate;
    pid->Get_Output = PID_Tolerance_Get_Output;
    pid->Set_MaxOut = PID_Tolerance_Set_MaxOut;
    pid->Get_MaxOut = PID_Tolerance_Get_MaxOut;
    pid->Reset = PID_Tolerance_Reset;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->tolerance = tolerance;
}

/* 接口函数实现 */

void PID_Tolerance_Set_Target(void* controller, const float target) {
    PID_Tolerance_Controller* pid = (PID_Tolerance_Controller*)controller;
    pid->set = target;
}

float PID_Tolerance_Calculate(void* controller, const float main_feedback, const float sub_feedback) {
    PID_Tolerance_Controller* pid = (PID_Tolerance_Controller*)controller;
    pid->fdb = main_feedback;

    pid->err[0] = pid->set - pid->fdb;

    pid->Pout = pid->kp *  pid->err[0];

    if (pid->err[0] > pid->tolerance * 50.0f) {
        pid->Pout = pid->kp * ((pid->err[0] - pid->tolerance * 50.0f) * 0.1f + pid->tolerance * 50.0f);
    }

    if (pid->err[0] < -pid->tolerance * 50.0f) {
        pid->Pout = pid->kp * ((pid->err[0] + pid->tolerance * 50.0f) * 0.1f - pid->tolerance * 50.0f);
    }

    pid->Iout += pid->ki * pid->err[0];

    pid->Dout = pid->kd * (pid->err[0] - pid->err[1]);

    if        (fabsf(pid->err[0] - pid->err[1]) < pid->tolerance * 1) {
        pid->Dout = pid->Dout * 0.35f;
    } else if (fabsf(pid->err[0] - pid->err[1]) < pid->tolerance * 2) {
        pid->Dout = pid->Dout * 0.45f;
    } else if (fabsf(pid->err[0] - pid->err[1]) < pid->tolerance * 4) {
        pid->Dout = pid->Dout * 0.60f;
    } else if (fabsf(pid->err[0] - pid->err[1]) < pid->tolerance * 8) {
        pid->Dout = pid->Dout * 0.75f;
    } else if (fabsf(pid->err[0] - pid->err[1]) < pid->tolerance * 16) {
        pid->Dout = pid->Dout * 0.85f;
    } else if (fabsf(pid->err[0] - pid->err[1]) < pid->tolerance * 32) {
        pid->Dout = pid->Dout * 0.95f;
    }

    // 积分限幅
    if (pid->Iout > pid->max_iout) {
        pid->Iout = pid->max_iout;
    } else if (pid->Iout < -pid->max_iout) {
        pid->Iout = -pid->max_iout;
    }

    float out = pid->Pout + pid->Iout + pid->Dout;

    // 输出限幅
    if (out > pid->max_out) {
        out = pid->max_out;
    } else if (out < -pid->max_out) {
        out = -pid->max_out;
    }

    pid->out = out;
    pid->err[1] = pid->err[0];

    return out;
}

float PID_Tolerance_Get_Output(void* controller) {
    const PID_Tolerance_Controller* pid = (PID_Tolerance_Controller*)controller;
    return pid->out;
}

void PID_Tolerance_Set_MaxOut(void* controller, const float max_out) {
    PID_Tolerance_Controller* pid = (PID_Tolerance_Controller*)controller;
    pid->max_out = max_out;
}

float PID_Tolerance_Get_MaxOut(void* controller) {
    const PID_Tolerance_Controller* pid = (PID_Tolerance_Controller*)controller;
    return pid->max_out;
}

void PID_Tolerance_Reset(void* controller) {
    PID_Tolerance_Controller* pid = (PID_Tolerance_Controller*)controller;

    // 清空运行时状态，保留配置参数
    pid->set = 0.0f;
    pid->fdb = 0.0f;
    pid->out = 0.0f;
    pid->Pout = 0.0f;
    pid->Iout = 0.0f;
    pid->Dout = 0.0f;
    pid->err[0] = 0.0f;
    pid->err[1] = 0.0f;
}
