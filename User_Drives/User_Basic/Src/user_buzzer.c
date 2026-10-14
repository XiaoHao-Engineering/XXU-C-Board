#include "../../../Core/Inc/bsp.h"
#ifdef HAL_TIM_MODULE_ENABLED
/* 包含头文件 */
#include "../user_buzzer.h"

/* 函数体 */

void Buzzer_Init(BUZZER_DRIVES *user_buzzer, TIM_HandleTypeDef *htim, const uint32_t channel, const uint32_t tim_clock) {
    memset(user_buzzer, 0, sizeof(BUZZER_DRIVES));
    
    PWM_Init(&user_buzzer->pwm, htim, channel, PWM_16BIT, tim_clock);
    user_buzzer->state = BUZZER_OFF;
    user_buzzer->frequency = 0;

    /* 先把 PWM 通道打开，否则改占空比不会有波形 */
    PWM_Start(&user_buzzer->pwm);

    /* 初始化时关闭蜂鸣器 */
    Buzzer_Off(user_buzzer);
}

void Buzzer_On(BUZZER_DRIVES *user_buzzer, const uint32_t frequency) {
    user_buzzer->state = BUZZER_ON;
    user_buzzer->frequency = frequency;
    
    PWM_Set_Frequency(&user_buzzer->pwm, frequency);
    PWM_Set_Duty(&user_buzzer->pwm, 0.5f);
}

void Buzzer_Off(BUZZER_DRIVES *user_buzzer) {
    user_buzzer->state = BUZZER_OFF;
    user_buzzer->frequency = 0;
    
    PWM_Set_Duty(&user_buzzer->pwm, 0.0f);
}

#endif /* HAL_TIM_MODULE_ENABLED */
