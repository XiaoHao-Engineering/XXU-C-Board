/* 包含头文件 */
#include "../user_ladrc.h"
#include <string.h>


/* 私有函数声明 */
static void LESO_Calculate(LESO_Controller *leso, float feedback, float u, float b0, float dt);
static float LSEF_Calculate(const LSEF_Controller *lsef, float e1, float e2);

/* 函数体 */

void LADRC_Init(LADRC_Controller *ladrc,
                const float wc, const float kp, const float kd,
                const float b0, const float max_out, const float dt) {
    memset(ladrc, 0, sizeof(LADRC_Controller));
    
    // 绑定接口函数
    ladrc->Set_Target = LADRC_Set_Target;
    ladrc->Calculate = LADRC_Calculate;
    ladrc->Get_Output = LADRC_Get_Output;
    ladrc->Set_MaxOut = LADRC_Set_MaxOut;
    ladrc->Get_MaxOut = LADRC_Get_MaxOut;
    ladrc->Reset = LADRC_Reset;
    
    // 初始化线性扩张状态观测器
    ladrc->leso.wc = wc;
    
    // 初始化线性状态误差反馈控制律
    ladrc->lsef.kp = kp;
    ladrc->lsef.kd = kd;
    
    // 初始化控制器参数
    ladrc->b0 = b0;
    ladrc->max_out = max_out;
    ladrc->dt = dt;
}

/* 接口函数实现 */

void LADRC_Set_Target(void* controller, const float target) {
    LADRC_Controller* ladrc = (LADRC_Controller*)controller;
    ladrc->set = target;
}

float LADRC_Calculate(void* controller, const float main_feedback, const float sub_feedback) {
    LADRC_Controller* ladrc = (LADRC_Controller*)controller;
    ladrc->fdb = main_feedback;
    
    // 线性扩张状态观测器
    LESO_Calculate(&ladrc->leso, main_feedback, ladrc->out, ladrc->b0, ladrc->dt);
    
    // 线性状态误差反馈控制律
    const float e1 = ladrc->set - ladrc->leso.z1;
    const float e2 = 0.0f - ladrc->leso.z2;
    ladrc->u0 = LSEF_Calculate(&ladrc->lsef, e1, e2);
    
    // 扰动补偿
    ladrc->out = (ladrc->u0 - ladrc->leso.z3) / ladrc->b0;
    
    // 输出限幅
    if (ladrc->out > ladrc->max_out) {
        ladrc->out = ladrc->max_out;
    } else if (ladrc->out < -ladrc->max_out) {
        ladrc->out = -ladrc->max_out;
    }
    
    return ladrc->out;
}

float LADRC_Get_Output(void* controller) {
    const LADRC_Controller* ladrc = (LADRC_Controller*)controller;
    return ladrc->out;
}

void LADRC_Set_MaxOut(void* controller, const float max_out) {
    LADRC_Controller* ladrc = (LADRC_Controller*)controller;
    ladrc->max_out = max_out;
}

float LADRC_Get_MaxOut(void* controller) {
    const LADRC_Controller* ladrc = (LADRC_Controller*)controller;
    return ladrc->max_out;
}

void LADRC_Reset(void* controller) {
    LADRC_Controller* ladrc = (LADRC_Controller*)controller;

    // 清空运行时状态，保留配置参数
    ladrc->leso.z1 = 0.0f;
    ladrc->leso.z2 = 0.0f;
    ladrc->leso.z3 = 0.0f;
    ladrc->set = 0.0f;
    ladrc->fdb = 0.0f;
    ladrc->out = 0.0f;
    ladrc->u0 = 0.0f;
}

/* 私有函数实现 */

static void LESO_Calculate(LESO_Controller *leso, const float feedback, const float u, const float b0, const float dt) {
    const float e = leso->z1 - feedback;
    
    // 基于带宽参数化的观测器增益
    const float beta1 = 3.0f * leso->wc;
    const float beta2 = 3.0f * leso->wc * leso->wc;
    const float beta3 = leso->wc * leso->wc * leso->wc;
    
    // 状态更新
    leso->z1 += dt * (leso->z2 - beta1 * e);
    leso->z2 += dt * (leso->z3 - beta2 * e + b0 * u);
    leso->z3 += dt * (-beta3 * e);
}

static float LSEF_Calculate(const LSEF_Controller *lsef, const float e1, const float e2) {
    const float u0 = lsef->kp * e1 + lsef->kd * e2;
    return u0;
}
