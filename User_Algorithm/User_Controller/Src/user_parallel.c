/* 包含头文件 */
#include "../user_parallel.h"
#include <string.h>

/* 函数体 */

void Parallel_Controller_Init(Parallel_Controller* parallel,
                              CONTROLLER_INTERFACE* controller_a,
                              CONTROLLER_INTERFACE* controller_b,
                              const float coef_a, const float coef_b,
                              const float max_out) {
    memset(parallel, 0, sizeof(Parallel_Controller));

    // 绑定接口函数
    parallel->Set_Target = Parallel_Controller_Set_Target;
    parallel->Calculate = Parallel_Controller_Calculate;
    parallel->Get_Output = Parallel_Controller_Get_Output;
    parallel->Set_MaxOut = Parallel_Controller_Set_MaxOut;
    parallel->Get_MaxOut = Parallel_Controller_Get_MaxOut;
    parallel->Reset = Parallel_Controller_Reset;

    // 绑定子控制器
    parallel->controller_a = controller_a;
    parallel->controller_b = controller_b;

    // 初始化混合系数
    parallel->coef_a = coef_a;
    parallel->coef_b = coef_b;

    // 初始化控制器参数
    parallel->max_out = max_out;

    // 初始化状态变量
    parallel->target = 0.0f;
    parallel->output = 0.0f;
    parallel->output_a = 0.0f;
    parallel->output_b = 0.0f;
}

/* 接口函数实现 */

// 目标值同时传递给两个子控制器
void Parallel_Controller_Set_Target(void* controller, const float target) {
    Parallel_Controller* parallel = (Parallel_Controller*)controller;
    parallel->target = target;
}

float Parallel_Controller_Calculate(void* controller, const float feedback_a, const float feedback_b) {
    Parallel_Controller* parallel = (Parallel_Controller*)controller;
    const CONTROLLER_INTERFACE* ctrl_a = (CONTROLLER_INTERFACE*)parallel->controller_a;
    const CONTROLLER_INTERFACE* ctrl_b = (CONTROLLER_INTERFACE*)parallel->controller_b;

    // 目标值同时传递给两个子控制器
    ctrl_a->Set_Target(parallel->controller_a, parallel->target);
    ctrl_b->Set_Target(parallel->controller_b, parallel->target);

    // 独立计算两个控制器输出
    parallel->output_a = ctrl_a->Calculate(parallel->controller_a, feedback_a, 0.0f);
    parallel->output_b = ctrl_b->Calculate(parallel->controller_b, feedback_b, 0.0f);

    // 混合输出：加权求和
    parallel->output = parallel->coef_a * parallel->output_a
                     + parallel->coef_b * parallel->output_b;

    // 输出限幅
    if (parallel->output > parallel->max_out) {
        parallel->output = parallel->max_out;
    } else if (parallel->output < -parallel->max_out) {
        parallel->output = -parallel->max_out;
    }

    return parallel->output;
}

float Parallel_Controller_Get_Output(void* controller) {
    const Parallel_Controller* parallel = (Parallel_Controller*)controller;
    return parallel->output;
}

void Parallel_Controller_Set_MaxOut(void* controller, const float max_out) {
    Parallel_Controller* parallel = (Parallel_Controller*)controller;
    parallel->max_out = max_out;
}

float Parallel_Controller_Get_MaxOut(void* controller) {
    const Parallel_Controller* parallel = (Parallel_Controller*)controller;
    return parallel->max_out;
}

void Parallel_Controller_Reset(void* controller) {
    Parallel_Controller* parallel = (Parallel_Controller*)controller;

    // 清空运行时状态，保留配置参数
    parallel->target = 0.0f;
    parallel->output = 0.0f;
    parallel->output_a = 0.0f;
    parallel->output_b = 0.0f;

    /* ★ 并联的两个子控制器也要一起清（原因同串级：掉线时不能留饱和积分）*/
    if (parallel->controller_a != NULL) {
        parallel->controller_a->Reset(parallel->controller_a);
    }
    if (parallel->controller_b != NULL) {
        parallel->controller_b->Reset(parallel->controller_b);
    }
}
