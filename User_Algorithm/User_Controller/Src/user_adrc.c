/* 包含头文件 */
#include "../user_adrc.h"
#include <math.h>
#include <string.h>

/* 私有函数声明 */
static void TD_Calculate(TD_Controller *td, float target, float dt);
static void ESO_Calculate(ESO_Controller *eso, float feedback, float u, float b0, float dt);
static float NLSEF_Calculate(NLSEF_Controller *nlsef, float e1, float e2);
static float fhan(float x1, float x2, float r, float h);
static float fal(float e, float alpha, float delta);

/* 函数体 */

void ADRC_Init(ADRC_Controller *adrc,
               const float r, const float h,
               const float beta1_eso, const float beta2_eso, const float beta3_eso, const float delta_eso,
               const float beta1_nlsef, const float beta2_nlsef, const float alpha1, const float alpha2, const float delta_nlsef,
               const float b0, const float max_out, const float dt) {
    memset(adrc, 0, sizeof(ADRC_Controller));
    
    // 绑定接口函数
    adrc->Set_Target = ADRC_Set_Target;
    adrc->Calculate = ADRC_Calculate;
    adrc->Get_Output = ADRC_Get_Output;
    adrc->Set_MaxOut = ADRC_Set_MaxOut;
    adrc->Get_MaxOut = ADRC_Get_MaxOut;
    adrc->Reset = ADRC_Reset;
    
    // 初始化跟踪微分器
    adrc->td.r = r;
    adrc->td.h = h;

    // 初始化扩张状态观测器
    adrc->eso.beta1 = beta1_eso;
    adrc->eso.beta2 = beta2_eso;
    adrc->eso.beta3 = beta3_eso;
    adrc->eso.delta = delta_eso;
    
    // 初始化非线性状态误差反馈控制律
    adrc->nlsef.beta1 = beta1_nlsef;
    adrc->nlsef.beta2 = beta2_nlsef;
    adrc->nlsef.alpha1 = alpha1;
    adrc->nlsef.alpha2 = alpha2;
    adrc->nlsef.delta = delta_nlsef;
    
    // 初始化控制器参数
    adrc->b0 = b0;
    adrc->max_out = max_out;
    adrc->dt = dt;
}

/* 接口函数实现 */

void ADRC_Set_Target(void* controller, const float target) {
    ADRC_Controller* adrc = (ADRC_Controller*)controller;
    adrc->set = target;
}

float ADRC_Calculate(void* controller, const float main_feedback, const float sub_feedback) {
    ADRC_Controller* adrc = (ADRC_Controller*)controller;
    adrc->fdb = main_feedback;
    
    // 跟踪微分器
    TD_Calculate(&adrc->td, adrc->set, adrc->dt);
    
    // 扩张状态观测器
    ESO_Calculate(&adrc->eso, main_feedback, adrc->out, adrc->b0, adrc->dt);
    
    // 非线性状态误差反馈控制律
    const float e1 = adrc->td.x1 - adrc->eso.z1;
    const float e2 = adrc->td.x2 - adrc->eso.z2;
    adrc->u0 = NLSEF_Calculate(&adrc->nlsef, e1, e2);
    
    // 扰动补偿
    adrc->out = (adrc->u0 - adrc->eso.z3) / adrc->b0;
    
    // 输出限幅
    if (adrc->out > adrc->max_out) {
        adrc->out = adrc->max_out;
    } else if (adrc->out < -adrc->max_out) {
        adrc->out = -adrc->max_out;
    }
    
    return adrc->out;
}

float ADRC_Get_Output(void* controller) {
    const ADRC_Controller* adrc = (ADRC_Controller*)controller;
    return adrc->out;
}

void ADRC_Set_MaxOut(void* controller, const float max_out) {
    ADRC_Controller* adrc = (ADRC_Controller*)controller;
    adrc->max_out = max_out;
}

float ADRC_Get_MaxOut(void* controller) {
    const ADRC_Controller* adrc = (ADRC_Controller*)controller;
    return adrc->max_out;
}

void ADRC_Reset(void* controller) {
    ADRC_Controller* adrc = (ADRC_Controller*)controller;

    // 清空运行时状态，保留配置参数
    adrc->td.x1 = 0.0f;
    adrc->td.x2 = 0.0f;
    adrc->eso.z1 = 0.0f;
    adrc->eso.z2 = 0.0f;
    adrc->eso.z3 = 0.0f;
    adrc->set = 0.0f;
    adrc->fdb = 0.0f;
    adrc->out = 0.0f;
    adrc->u0 = 0.0f;
}

/* 私有函数实现 */

static void TD_Calculate(TD_Controller *td, const float target, const float dt) {
    const float fh = fhan(td->x1 - target, td->x2, td->r, td->h);
    
    td->x1 += dt * td->x2;
    td->x2 += dt * fh;
}

static void ESO_Calculate(ESO_Controller *eso, const float feedback, const float u, const float b0, const float dt) {
    const float e = eso->z1 - feedback;

    const float fe = fal(e, 0.5f, eso->delta);
    const float fe1 = fal(e, 0.25f, eso->delta);
    
    eso->z1 += dt * (eso->z2 - eso->beta1 * e);
    eso->z2 += dt * (eso->z3 - eso->beta2 * fe + b0 * u);
    eso->z3 += dt * (-eso->beta3 * fe1);
}

static float NLSEF_Calculate(NLSEF_Controller *nlsef, const float e1, const float e2) {
    const float u0 = nlsef->beta1 * fal(e1, nlsef->alpha1, nlsef->delta) +
               nlsef->beta2 * fal(e2, nlsef->alpha2, nlsef->delta);
    return u0;
}

static float fhan(const float x1, const float x2, const float r, const float h) {
    const float d = r * h * h;
    const float a0 = h * x2;
    const float y = x1 + a0;
    const float a1 = sqrtf(d * (d + 8.0f * fabsf(y)));
    const float a2 = a0 + (y >= 0 ? 1.0f : -1.0f) * (a1 - d) / 2.0f;
    const float sy = (fabsf(y + d) >= fabsf(y - d)) ? 1.0f : -1.0f;
    const float a = (fabsf(a2) > d) ? sy : a2 / d;
    const float fhan_value = -r * (a0 / h - a * (fabsf(a) <= d ? 1.0f : sy));
    
    return fhan_value;
}

static float fal(const float e, const float alpha, const float delta) {
    const float abs_e = fabsf(e);
    
    if (abs_e <= delta) {
        return e / powf(delta, 1.0f - alpha);
    } else {
        return powf(abs_e, alpha) * (e >= 0 ? 1.0f : -1.0f);
    }
}
