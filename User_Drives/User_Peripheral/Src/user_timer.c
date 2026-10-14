#include "../../Core/Inc/bsp.h"
#ifdef HAL_TIM_MODULE_ENABLED
/* 包含头文件 */
#include "../user_timer.h"
#include <math.h>

/* 私有变量 */
static TIMER_DRIVES* timer_drives[TIMER_NUM];
static uint8_t timer_num = 0;

/* 函数体 */
void TIMER_Init(TIMER_DRIVES* user_timer, TIM_HandleTypeDef* htim, const uint32_t tim_clock) {
    user_timer->htim = htim;
    user_timer->clock = tim_clock;
    user_timer->frequency = user_timer->clock / ((htim->Init.Prescaler + 1) * (__HAL_TIM_GET_AUTORELOAD(htim) + 1));

    timer_drives[timer_num++] = user_timer;
}

void TIMER_RegisterCallback(TIMER_DRIVES* user_timer, const TIMER_Callback callback) {
    user_timer->callbacks[user_timer->callback_num++] = callback;
}

void TIMER_Start(const TIMER_DRIVES* user_timer) {
    HAL_TIM_Base_Start_IT(user_timer->htim);
}

void TIMER_Stop(const TIMER_DRIVES* user_timer) {
    HAL_TIM_Base_Stop_IT(user_timer->htim);
}

void TIMER_Set_Frequency(TIMER_DRIVES* user_timer, const uint32_t frequency) {
    user_timer->frequency = frequency;

    uint32_t prescaler_reg = 0;
    uint32_t reload_reg = 0;

    for (prescaler_reg = 0; prescaler_reg <= 65535; prescaler_reg++) {
        reload_reg = (uint32_t)ceilf((float)user_timer->clock / (float)(frequency * (prescaler_reg + 1))) - 1;
        if (reload_reg <= 65535) {
            break;
        }
    }

    if (reload_reg > 65535) {
        reload_reg = 65535;
    }

    const uint8_t is_running = HAL_TIM_Base_GetState(user_timer->htim) == HAL_TIM_STATE_BUSY;

    if (is_running)
        HAL_TIM_Base_Stop_IT(user_timer->htim);

    __HAL_TIM_SET_PRESCALER(user_timer->htim, prescaler_reg);
    __HAL_TIM_SET_AUTORELOAD(user_timer->htim, reload_reg);
    __HAL_TIM_SET_COUNTER(user_timer->htim, 0);

    if (is_running)
        HAL_TIM_Base_Start_IT(user_timer->htim);

}

/* 覆写中断回调函数 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    for (uint8_t timer_index = 0; timer_index < timer_num; timer_index++) {
        TIMER_DRIVES* timer = timer_drives[timer_index];

        if (timer->htim->Instance == htim->Instance) {
            for (uint8_t i = 0; i < timer->callback_num; i++) {
                if (timer->callbacks[i] != NULL) {
                    timer->callbacks[i](timer);
                }
            }
            break;
        }
    }
}

#endif /* HAL_TIM_MODULE_ENABLED */
