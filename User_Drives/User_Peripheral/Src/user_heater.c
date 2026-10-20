#include "../../../Core/Inc/bsp.h"
#ifdef HAL_TIM_MODULE_ENABLED
/* 包含头文件 */
#include "../user_heater.h"
#include "tim.h"
#include <math.h>
#include <string.h>

/* 私有函数 */
static void Heater_Apply_Duty(HEATER_DRIVES* heater, float duty);
static void Heater_Task(void* arg);

/* 函数体 */

/* 初始化：绑定 PID、把 PWM 通道打开、CCR 清 0，并注册 100ms 周期任务。
   ★ 必须在 MX_TIM10_Init()（以及 JScope_Init，如果它先跑了）之后调用：
     JScope 已经用 HAL_TIM_Base_Start_IT 起了 TIM10 的计数，
     HAL_TIM_PWM_Start 只负责打开 CH1 的 CCxE，两者互不冲突。*/
void Heater_Init(HEATER_DRIVES* heater) {
    memset(heater, 0, sizeof(HEATER_DRIVES));

    /* 在线可调参数默认值（全部 float，方便直接挂进串口调参表）*/
    heater->target_temp = HEATER_TARGET_TEMP_DEF;
    heater->preheat_base_temp = HEATER_PREHEAT_BASE_TEMP_DEF;
    heater->duty_max = HEATER_DUTY_MAX;
    heater->enabled = HEATER_ENABLE_DEF;

    /* 观测量 / 状态清零（显式写一遍，反复 Init 也不会留旧值）*/
    heater->current_temp = 0.0f;
    heater->output_duty = 0.0f;
    heater->duty_ccr = 0U;
    heater->preheat_done = 0U;
    heater->over_temp_fault = 0U;
    heater->sensor_fault = 0U;

    /* PID：dt=0.1s 与 100ms 任务严格对齐（参考工程 dt=0.1s）*/
    PID_Adv_Init(&heater->pid);
    PID_Adv_Set_K_P(&heater->pid, HEATER_PID_KP);
    PID_Adv_Set_K_I(&heater->pid, HEATER_PID_KI);
    PID_Adv_Set_K_D(&heater->pid, HEATER_PID_KD);
    PID_Adv_Set_Max_Out(&heater->pid, HEATER_PID_MAX_OUT);
    PID_Adv_Set_Max_IOut(&heater->pid, HEATER_PID_MAX_IOUT);
    PID_Adv_Set_DT(&heater->pid, HEATER_PID_DT);
    PID_Adv_Reset(&heater->pid);

    /* 启动 PWM 通道并把比较值压到 0：上电默认不加热 */
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, 0U);

    /* 100ms 周期任务（放在 SysTick 中断里，和参考工程 dt=0.1s 一致）。
       ★ 任务里只做一次 PID 运算 + 一次 CCR 写，不做 SPI 读。*/
    SysTick_InitTask(&heater->task, (void*)heater, HEATER_PERIOD_MS, HEATER_PERIOD_MS,
                     Task_REPEAT, Heater_Task);
    SysTick_StartTask(&heater->task);
}

/* 周期控制：照参考工程 Class_Heating_Resistor::TIM_Calculate_PeriodElapsedCallback
   的两段式逻辑，另加过温 / 传感器异常保护。*/
void Heater_Update(HEATER_DRIVES* heater, const float current_temp_degC) {
    heater->current_temp = current_temp_degC;

    /* ---- 保护 1：温度传感器异常（NaN / Inf / 低于 -10℃）----
       读到异常值就彻底停加热：否则反馈失效时 PID 会以为"永远不够热"而满功率烧。*/
    if (!isfinite(current_temp_degC) || current_temp_degC < HEATER_SENSOR_MIN_TEMP) {
        heater->sensor_fault = 1U;
        PID_Adv_Reset(&heater->pid);
        Heater_Apply_Duty(heater, 0.0f);
        return;
    }
    heater->sensor_fault = 0U;

    /* ---- 保护 2：过温（> 65℃）----
       立刻 duty=0 并清 enabled（锁存）：即使温度随后回落，也必须人工把
       heater_enable 重新置 1 才会继续加热，避免在故障附近反复启停。*/
    if (current_temp_degC > HEATER_OVER_TEMP) {
        heater->over_temp_fault = 1U;
        heater->enabled = 0.0f;
        PID_Adv_Reset(&heater->pid);
        Heater_Apply_Duty(heater, 0.0f);
        return;
    }

    /* 回到 60℃ 以下只清"故障发生过"的标志，不自动恢复加热 */
    if (current_temp_degC < (HEATER_OVER_TEMP - HEATER_OVER_TEMP_HYST)) {
        heater->over_temp_fault = 0U;
    }

    /* ---- 人工关断（heater_enable=0）----
       顺带把 PID 复位，避免停加热期间积分/历史值留着，下次使能第一拍就冲。*/
    if (!(heater->enabled >= 0.5f)) {
        PID_Adv_Reset(&heater->pid);
        Heater_Apply_Duty(heater, 0.0f);
        return;
    }

    /* ---- 第一段：预热（从当前温度一路满占空比烧到 47℃）----
       参考工程：Current_Temperature >= HEATER_PREHEAT_BASE_TEMPERATURE 才算预热完成。*/
    float preheat_base = heater->preheat_base_temp;
    if (!isfinite(preheat_base)) {
        preheat_base = HEATER_PREHEAT_BASE_TEMP_DEF;   /* 调参调出 NaN 也不能失去保护 */
    }

    if (heater->preheat_done == 0U) {
        if (current_temp_degC < preheat_base) {
            Heater_Apply_Duty(heater, HEATER_PREHEAT_DUTY);
            return;
        }

        /* 预热完成：切 PID 恒温。复位一次，保证 PID 从干净状态起算 */
        heater->preheat_done = 1U;
        PID_Adv_Reset(&heater->pid);
    }

    /* ---- 第二段：PID 恒温（目标 50℃）----
       目标值做防御性钳位：即使调参表被写超范围，也不许把目标设到过温阈值以上。*/
    float target = heater->target_temp;
    if (!isfinite(target)) {
        target = HEATER_TARGET_TEMP_DEF;
    }
    if (target < HEATER_TARGET_TEMP_MIN) {
        target = HEATER_TARGET_TEMP_MIN;
    }
    if (target > HEATER_TARGET_TEMP_MAX) {
        target = HEATER_TARGET_TEMP_MAX;
    }
    heater->target_temp = target;

    PID_Adv_Set_Target(&heater->pid, target);
    const float duty = PID_Adv_Calculate(&heater->pid, current_temp_degC, 0.0f);

    Heater_Apply_Duty(heater, duty);
}

/* 占空比统一出口：0~1000 千分比 -> 0~16800 CCR。
   ★ 换算是本函数里这一行：ccr = duty × HEATER_PWM_CCR_MAX / HEATER_DUTY_MAX。
   duty_max 作为运行期上限（调参名 heater_duty）在这里生效，
   所以预热满占空比也会被它压住 —— 想彻底停加热把 heater_duty 调 0 即可。*/
static void Heater_Apply_Duty(HEATER_DRIVES* heater, float duty) {
    float max_duty = heater->duty_max;
    if (!isfinite(max_duty)) {
        max_duty = HEATER_DUTY_MAX;
    }
    if (max_duty < 0.0f) {
        max_duty = 0.0f;
    }
    if (max_duty > HEATER_DUTY_MAX) {
        max_duty = HEATER_DUTY_MAX;
    }

    if (!isfinite(duty) || duty < 0.0f) {
        duty = 0.0f;
    }
    if (duty > max_duty) {
        duty = max_duty;
    }

    /* ★ 0~1000 -> 0~16800（四舍五入）；16800 = TIM10 ARR(16800-1)+1，即 100% 占空比 */
    const uint32_t ccr = (uint32_t)(duty * (float)HEATER_PWM_CCR_MAX / HEATER_DUTY_MAX + 0.5f);

    heater->output_duty = duty;
    heater->duty_ccr = ccr;
    __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, ccr);
}

/* 100ms 周期任务：温度取 BMI088 2ms 任务已经读好的值。
   ★ 这里在 SysTick 中断上下文，绝对不能做 SPI 读（1ms 预算会被打爆）。*/
static void Heater_Task(void* arg) {
    HEATER_DRIVES* heater = (HEATER_DRIVES*)arg;
    Heater_Update(heater, user_bmi088.temperature);
}

#endif /* HAL_TIM_MODULE_ENABLED */
