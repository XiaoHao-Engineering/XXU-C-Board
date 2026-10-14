#include "../../Core/Inc/bsp.h"
#include "../user_waveform.h"
#include <math.h>

#define WAVE_PI     (3.14159265f)
#define WAVE_2PI    (6.28318531f)

/* 私有函数声明 */
static float Wave_Clamp(float x, float min, float max);
static float Wave_Wrap01(float p);

/* 约束到 [min, max] */
static float Wave_Clamp(float x, float min, float max) {
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

/* 把相位折到 [0, 1) */
static float Wave_Wrap01(float p) {
    if (p >= 1.0f) p -= (float)((int32_t)p);
    if (p <  0.0f) p += (float)((int32_t)(-p) + 1);
    if (p >= 1.0f) p = 0.0f;
    return p;
}

/* 函数体 */

void Waveform_Init(WAVEFORM_DRIVES* w, const float dt_s) {
    w->type = WAVE_HOLD;

    w->dt_s = (dt_s > 0.0f) ? dt_s : 0.001f;
    w->time_s = 0.0f;
    w->phase = 0.0f;
    w->output = 0.0f;

    w->out_min = -1e9f;
    w->out_max =  1e9f;

    w->amplitude = 0.0f;
    w->offset = 0.0f;
    w->freq_hz = 1.0f;
    w->duty = 0.5f;

    w->low = 0.0f;
    w->high = 0.0f;
    w->step_t_s = 0.0f;

    w->period_s = 1.0f;
    w->width_s = 0.1f;
}

void Waveform_Reset(WAVEFORM_DRIVES* w) {
    w->time_s = 0.0f;
    w->phase = 0.0f;
}

float Waveform_Get(const WAVEFORM_DRIVES* w) {
    return w->output;
}

/* ---- 配置 --------------------------------------------------------------*/

void Waveform_Set_Hold(WAVEFORM_DRIVES* w, const float value) {
    w->type = WAVE_HOLD;
    w->output = value;
}

void Waveform_Set_Step(WAVEFORM_DRIVES* w, const float low, const float high,
                       const float t_s) {
    w->type = WAVE_STEP;
    w->low = low;
    w->high = high;
    w->step_t_s = (t_s >= 0.0f) ? t_s : 0.0f;
}

void Waveform_Set_Square(WAVEFORM_DRIVES* w, const float amplitude, const float freq_hz,
                         const float offset, const float duty) {
    w->type = WAVE_SQUARE;
    w->amplitude = (amplitude >= 0.0f) ? amplitude : -amplitude;
    w->offset = offset;
    w->freq_hz = (freq_hz > 0.0f) ? freq_hz : 0.0f;
    w->duty = Wave_Clamp(duty, 0.0f, 1.0f);
}

void Waveform_Set_Sine(WAVEFORM_DRIVES* w, const float amplitude,
                       const float freq_hz, const float offset) {
    w->type = WAVE_SINE;
    w->amplitude = (amplitude >= 0.0f) ? amplitude : -amplitude;
    w->offset = offset;
    w->freq_hz = (freq_hz > 0.0f) ? freq_hz : 0.0f;
}

void Waveform_Set_Triangle(WAVEFORM_DRIVES* w, const float amplitude,
                           const float freq_hz, const float offset) {
    w->type = WAVE_TRIANGLE;
    w->amplitude = (amplitude >= 0.0f) ? amplitude : -amplitude;
    w->offset = offset;
    w->freq_hz = (freq_hz > 0.0f) ? freq_hz : 0.0f;
}

void Waveform_Set_Saw(WAVEFORM_DRIVES* w, const float amplitude,
                      const float freq_hz, const float offset) {
    w->type = WAVE_SAW;
    w->amplitude = (amplitude >= 0.0f) ? amplitude : -amplitude;
    w->offset = offset;
    w->freq_hz = (freq_hz > 0.0f) ? freq_hz : 0.0f;
}

void Waveform_Set_Pulse(WAVEFORM_DRIVES* w, const float low, const float high,
                        const float period_s, const float width_s) {
    w->type = WAVE_PULSE;
    w->low = low;
    w->high = high;
    w->period_s = (period_s > 0.0f) ? period_s : 1.0f;
    w->width_s = Wave_Clamp(width_s, 0.0f, w->period_s);
}

void Waveform_Set_Limit(WAVEFORM_DRIVES* w, const float min, const float max) {
    w->out_min = min;
    w->out_max = max;
}

/* ---- 每个控制周期调一次 -------------------------------------------------*/

float Waveform_Update(WAVEFORM_DRIVES* w) {
    w->time_s += w->dt_s;

    float y = w->output;

    switch (w->type) {
        case WAVE_STEP: {
            y = (w->time_s < w->step_t_s) ? w->low : w->high;
            break;
        }

        case WAVE_SQUARE: {
            /* 调 PID 最常用：正负交替的阶跃，能一次看到上升/下降的响应 */
            w->phase = Wave_Wrap01(w->phase + w->freq_hz * w->dt_s);
            const float s = (w->phase < w->duty) ? 1.0f : -1.0f;
            y = w->offset + w->amplitude * s;
            break;
        }

        case WAVE_SINE: {
            w->phase = Wave_Wrap01(w->phase + w->freq_hz * w->dt_s);
            y = w->offset + w->amplitude * sinf(WAVE_2PI * w->phase);
            break;
        }

        case WAVE_TRIANGLE: {
            w->phase = Wave_Wrap01(w->phase + w->freq_hz * w->dt_s);
            const float p = w->phase;
            const float tri = (p < 0.5f) ? (4.0f * p - 1.0f) : (3.0f - 4.0f * p);
            y = w->offset + w->amplitude * tri;
            break;
        }

        case WAVE_SAW: {
            w->phase = Wave_Wrap01(w->phase + w->freq_hz * w->dt_s);
            y = w->offset + w->amplitude * (2.0f * w->phase - 1.0f);
            break;
        }

        case WAVE_PULSE: {
            float t = w->time_s;
            while (t >= w->period_s) t -= w->period_s;
            y = (t < w->width_s) ? w->high : w->low;
            break;
        }

        case WAVE_HOLD:
        default: {
            y = w->output;
            break;
        }
    }

    w->output = Wave_Clamp(y, w->out_min, w->out_max);
    return w->output;
}
