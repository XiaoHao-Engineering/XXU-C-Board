#include "../../../Core/Inc/bsp.h"
#ifdef HAL_ADC_MODULE_ENABLED
/* 包含头文件 */
#include "../user_adc.h"

/* 私有变量 */
static ADC_DRIVES *adc_drives[ADC_NUM];
static uint8_t adc_num = 0;

/* 函数体 */

// ADC 使能扫描模式，使能连续转换模式，使能连续请求，配置规则组，开启 DMA 并开启循环模式
void ADC_Init(ADC_DRIVES* user_adc, ADC_HandleTypeDef* hadc, const uint8_t channel_num, const uint8_t vref_rank, const uint8_t resolution) {

    user_adc->hadc = hadc;
    user_adc->channel_num = channel_num;
    user_adc->vref_rank = vref_rank;
    user_adc->resolution = resolution;

    HAL_ADC_Start_DMA(user_adc->hadc, (uint32_t*)user_adc->adc_value, user_adc->channel_num);

    adc_drives[adc_num] = user_adc;
    adc_num++;
}

void ADC_RegisterCallback(ADC_DRIVES* user_adc, const ADC_Callback callback) {
    user_adc->callbacks[user_adc->callback_num] = callback;
    user_adc->callback_num++;
}

float ADC_GetVoltage(const ADC_DRIVES* user_adc, const uint8_t channel_rank) {
    if (user_adc->vref_rank)
        return (float)user_adc->adc_value[channel_rank - 1] * 1.2f / (float)user_adc->adc_value[user_adc->vref_rank - 1];
    else
        return (float)user_adc->adc_value[channel_rank - 1] * 3.3f / (float)(1 << user_adc->resolution);
}

/* 覆写中断回调函数 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    for (uint8_t adc_index = 0; adc_index < adc_num; adc_index++) {
        ADC_DRIVES* adc_drive = adc_drives[adc_index];

        if (hadc != adc_drive->hadc)
            continue;

        for (uint8_t i = 0; i < adc_drive->callback_num; i++) {
            adc_drive->callbacks[i](adc_drive);
        }
    }
}


#endif /* HAL_ADC_MODULE_ENABLED */
