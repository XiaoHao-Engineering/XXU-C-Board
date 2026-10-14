/* 包含头文件 */
#include "../user_inc_pid.h"
#include <string.h>

/* 函数体 */

void Inc_PID_Init(Inc_PID_Controller *pid, const float kp, const float ki,
                          const float kd, const float max_increase, const float max_out) {
    memset(pid, 0, sizeof(Inc_PID_Controller));
    
    // 绑定接口函数
    pid->Set_Target = Inc_PID_Set_Target;
    pid->Calculate = Inc_PID_Calculate;
    pid->Get_Output = Inc_PID_Get_Output;
    pid->Set_MaxOut = Inc_PID_Set_MaxOut;
    pid->Get_MaxOut = Inc_PID_Get_MaxOut;
    pid->Reset = Inc_PID_Reset;
    
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_increase = max_increase;
    pid->max_out = max_out;
}

/* 接口函数实现 */

void Inc_PID_Set_Target(void* controller, const float target) {
    Inc_PID_Controller* pid = (Inc_PID_Controller*)controller;
    pid->set = target;
}

float Inc_PID_Calculate(void* controller, const float main_feedback,
                                 const float sub_feedback) {
    Inc_PID_Controller* pid = (Inc_PID_Controller*)controller;
    pid->fdb = main_feedback;

    // 更新误差
    pid->err[2] = pid->err[1];
    pid->err[1] = pid->err[0];
    pid->err[0] = pid->set - pid->fdb;

    // 增量式PID计算
    float out = pid->kp * (pid->err[0] - pid->err[1]) +
               pid->ki * pid->err[0] +
               pid->kd * (pid->err[0] - 2.0f * pid->err[1] + pid->err[2]);

    // 输出增量限幅
    if (out > pid->max_increase) {
        out = pid->max_increase;
    } else if (out < -pid->max_increase) {
        out = -pid->max_increase;
    }

    out = out + pid->out;

    // 输出限幅
    if (out > pid->max_out) {
        out = pid->max_out;
    } else if (out < -pid->max_out) {
        out = -pid->max_out;
    }

    pid->out = out;

    return out;
}

float Inc_PID_Get_Output(void* controller) {
    const Inc_PID_Controller* pid = (Inc_PID_Controller*)controller;
    return pid->out;
}

void Inc_PID_Set_MaxOut(void* controller, const float max_out) {
    Inc_PID_Controller* pid = (Inc_PID_Controller*)controller;
    pid->max_out = max_out;
}

float Inc_PID_Get_MaxOut(void* controller) {
    const Inc_PID_Controller* pid = (Inc_PID_Controller*)controller;
    return pid->max_out;
}

void Inc_PID_Reset(void* controller) {
    Inc_PID_Controller* pid = (Inc_PID_Controller*)controller;

    // 清空运行时状态，保留配置参数
    pid->set = 0.0f;
    pid->fdb = 0.0f;
    pid->out = 0.0f;
    pid->err[0] = 0.0f;
    pid->err[1] = 0.0f;
    pid->err[2] = 0.0f;
}
