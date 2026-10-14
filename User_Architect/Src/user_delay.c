/* 包含头文件 */
#include "../user_delay.h"
#include "../../Core/Inc/bsp.h"

/* 私有变量 */
static TIM_HandleTypeDef user_tick_timer;

/* 扩展变量 */
uint32_t TimeFlag = 0;

/* 私有函数 */

static inline void get_stable_time(uint32_t *start_flag, uint32_t *start_time) {
    *start_flag = TimeFlag;
    *start_time = __HAL_TIM_GET_COUNTER(&user_tick_timer);
    while (*start_flag != TimeFlag) {
        *start_flag = TimeFlag;
        *start_time = __HAL_TIM_GET_COUNTER(&user_tick_timer);
    }
}

static inline uint32_t calculate_elapsed_time(const uint32_t start_flag, const uint32_t start_time, const uint32_t current_flag, const uint32_t current_time) {
    return (current_flag - start_flag) * 10000 + (current_time - start_time) * 5;
}

/* 函数体 */

// 定时器配置要求：
void Delay_Init(const TIM_HandleTypeDef *htim) {
    user_tick_timer = *htim;
}

// 此函数应在定时器中断回调函数中调用
void Delay_Update_TimeFlag(void) {
    TimeFlag++;
}

// 延时期间会调用循环事件处理函数
void Delay_us(const uint32_t us) {
    uint32_t start_flag, start_time;
    get_stable_time(&start_flag, &start_time);

    uint32_t current_flag, current_time;

    do {
        get_stable_time(&current_flag, &current_time);
        LOOP_EVENT_Handle();
    } while (calculate_elapsed_time(start_flag, start_time, current_flag, current_time) * 100 < us);
}

// 延时期间会多次调用循环事件处理函数
void Delay_ms(const uint32_t ms) {
    uint32_t start_flag, start_time;
    get_stable_time(&start_flag, &start_time);

    uint32_t current_flag, current_time;

    do {
        get_stable_time(&current_flag, &current_time);

        for (uint16_t i = 0; i < 10; i++)
            LOOP_EVENT_Handle();

    } while (calculate_elapsed_time(start_flag, start_time, current_flag, current_time) < ms * 10);
}

// 延时期间会大量调用循环事件处理函数
void Delay_s(const uint32_t s) {
    uint32_t start_flag, start_time;
    get_stable_time(&start_flag, &start_time);

    uint32_t current_flag, current_time;

    do {
        get_stable_time(&current_flag, &current_time);

        for (uint16_t i = 0; i < 10000; i++)
            LOOP_EVENT_Handle();

    } while (calculate_elapsed_time(start_flag, start_time, current_flag, current_time) < s * 10000);
}
