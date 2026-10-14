#include "../../../Core/Inc/bsp.h"
#ifdef HAL_GPIO_MODULE_ENABLED
/* 包含头文件 */
#include "../user_led.h"
#include <string.h>

/* 函数体 */

void LED_Init(LED_DRIVES* user_led, GPIO_TypeDef* GPIO, const uint16_t pin, const uint8_t is_reversal){
    memset(user_led, 0, sizeof(LED_DRIVES));
    
    user_led->GPIO = GPIO;
    user_led->pin = pin;
    user_led->is_reversal = is_reversal;
}

void LED_Set_State(const LED_DRIVES* user_led, const LED_State state){
    if (user_led->is_reversal) {
        switch (state) {
            case LED_ON:
                HAL_GPIO_WritePin(user_led->GPIO, user_led->pin, GPIO_PIN_RESET);  /*LED 点亮*/
                break;
            case LED_OFF:
                HAL_GPIO_WritePin(user_led->GPIO, user_led->pin, GPIO_PIN_SET);    /*LED 熄灭*/
                break;
        }
    } else {
        switch (state) {
            case LED_ON:
                HAL_GPIO_WritePin(user_led->GPIO, user_led->pin, GPIO_PIN_SET);    /*LED 点亮*/
                break;
            case LED_OFF:
                HAL_GPIO_WritePin(user_led->GPIO, user_led->pin, GPIO_PIN_RESET);  /*LED 熄灭*/
                break;
        }
    }
}

void LED_On(const LED_DRIVES* user_led){
    if (user_led->is_reversal)
        HAL_GPIO_WritePin(user_led->GPIO, user_led->pin, GPIO_PIN_RESET);  /*LED 点亮*/
    else
        HAL_GPIO_WritePin(user_led->GPIO, user_led->pin, GPIO_PIN_SET);    /*LED 点亮*/
}

void LED_Off(const LED_DRIVES* user_led){
    if (user_led->is_reversal)
        HAL_GPIO_WritePin(user_led->GPIO, user_led->pin, GPIO_PIN_SET);    /*LED 熄灭*/
    else
        HAL_GPIO_WritePin(user_led->GPIO, user_led->pin, GPIO_PIN_RESET);  /*LED 熄灭*/
}

void LED_Toggle(const LED_DRIVES* user_led){
    HAL_GPIO_TogglePin(user_led->GPIO, user_led->pin);
}

LED_State LED_Get_State(const LED_DRIVES* user_led){
    const GPIO_PinState pinState = HAL_GPIO_ReadPin(user_led->GPIO, user_led->pin);

    if (user_led->is_reversal)
        switch (pinState) {
            case GPIO_PIN_RESET:
                return LED_ON;
            case GPIO_PIN_SET:
                return LED_OFF;
        }
    else
        switch (pinState) {
            case GPIO_PIN_SET:
                return LED_ON;
            case GPIO_PIN_RESET:
                return LED_OFF;
        }

    return LED_OFF;
}

#endif /* HAL_GPIO_MODULE_ENABLED */