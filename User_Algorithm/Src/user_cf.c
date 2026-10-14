/* 包含头文件 */
#include "../user_cf.h"
#include <string.h>
#include <math.h>

/* 函数体 */

// signal1 是绝对量、signal2 是增量；时间常数越大越信 signal2
void CF_Init(CF_Filter *filter, const float time_constant, const float dt) {
    memset(filter, 0, sizeof(CF_Filter));

    filter->alpha = time_constant / (time_constant + dt);
    filter->dt = dt;
    filter->initialized = 0;
}

float CF_Update(CF_Filter *filter, const float signal1, const float signal2) {
    if (!filter->initialized) {
        filter->output = signal1;
        filter->prev_signal1 = signal1;
        filter->prev_signal2 = signal2;
        filter->initialized = 1;
        return filter->output;
    }
    
    filter->output = filter->alpha
                   * (filter->output + signal2 - filter->prev_signal2)
                   + (1.0f - filter->alpha) * signal1;
    
    filter->prev_signal1 = signal1;
    filter->prev_signal2 = signal2;
    
    return filter->output;
}
