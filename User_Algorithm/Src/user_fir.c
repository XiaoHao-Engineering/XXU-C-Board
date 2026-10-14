/* 包含头文件 */
#include "../user_fir.h"
#include <stdlib.h>
#include <string.h>

/* 函数体 */

void FIR_Init(FIR_Filter *fir, const float *coeffs, const uint16_t order) {
    memset(fir, 0, sizeof(FIR_Filter));
    
    fir->order = order;

    fir->buffer = malloc((order + 1) * sizeof(float));
    fir->coeffs = malloc((order + 1) * sizeof(float));
    fir->state_buffer = malloc((order + 1) * sizeof(float));

    memcpy(fir->coeffs, coeffs, (order + 1) * sizeof(float));

    arm_fir_init_f32(&fir->instance, fir->order + 1, fir->coeffs, fir->state_buffer, 1);
}

float FIR_Update(FIR_Filter *fir, const float input) {
    float output;

    for (uint16_t i = 0; i < fir->order; i++)
        fir->buffer[i+1] = fir->buffer[i];
    fir->buffer[0] = input;

    arm_fir_f32(&fir->instance, fir->buffer, &output, 1);
    
    return output;
}