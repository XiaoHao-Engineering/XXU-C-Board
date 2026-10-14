#ifndef USER_WAVEFORM_H
#define USER_WAVEFORM_H

/* 包含头文件 */
#include "main.h"

/* PID 测试用波形发生器（C 版，参照 alg_waveform 精简）
   用法：
       Waveform_Init(&user_wave, 0.001f);              // 1kHz 任务
       Waveform_Set_Square(&user_wave, 30, 1.0f, 0, 0.5f);  // ±30，1Hz，50%
       每个周期: y = Waveform_Update(&user_wave);
*/

/* 波形类型 */
typedef enum {
    WAVE_HOLD = 0,      /* 保持一个固定值 */
    WAVE_STEP,          /* 单次阶跃：t < T 为 Low，之后 High */
    WAVE_SQUARE,        /* 方波 —— 调 PID 最常用 */
    WAVE_SINE,          /* 正弦 */
    WAVE_TRIANGLE,      /* 三角 */
    WAVE_SAW,           /* 锯齿 */
    WAVE_PULSE,         /* 窄脉冲 */
    WAVE_TYPE_NUM
} WAVE_TYPE;

/* 类型定义 */
typedef struct {
    /* ---- 配置 ---- */
    uint8_t type;
    float amplitude;                /* 幅值（方波输出 ±amplitude）*/
    float offset;                   /* 偏置 */
    float freq_hz;                  /* 频率 Hz，0 = 不推进相位 */
    float duty;                     /* 占空比 0~1 */
    float low, high;                /* 阶跃/脉冲的高低电平 */
    float step_t_s;                 /* 阶跃切换时刻 s */
    float period_s, width_s;        /* 脉冲周期/宽度 s */
    float out_min, out_max;         /* 输出限幅 */

    /* ---- 运行状态 ---- */
    float dt_s;                     /* 采样周期 s */
    float time_s;                   /* 已运行时间 s */
    float phase;                    /* 相位 0~1 */
    float output;                   /* 当前输出 */
} WAVEFORM_DRIVES;

/* 函数声明 ----------------------------------------------------------------*/
void  Waveform_Init(WAVEFORM_DRIVES* w, float dt_s);
void  Waveform_Reset(WAVEFORM_DRIVES* w);

/* 每个控制周期调用一次，返回当前波形值 */
float Waveform_Update(WAVEFORM_DRIVES* w);
float Waveform_Get(const WAVEFORM_DRIVES* w);

/* 配置各种波形 ------------------------------------------------------------*/
void  Waveform_Set_Hold(WAVEFORM_DRIVES* w, float value);
void  Waveform_Set_Step(WAVEFORM_DRIVES* w, float low, float high, float t_s);
void  Waveform_Set_Square(WAVEFORM_DRIVES* w, float amplitude, float freq_hz,
                          float offset, float duty);
void  Waveform_Set_Sine(WAVEFORM_DRIVES* w, float amplitude, float freq_hz, float offset);
void  Waveform_Set_Triangle(WAVEFORM_DRIVES* w, float amplitude, float freq_hz, float offset);
void  Waveform_Set_Saw(WAVEFORM_DRIVES* w, float amplitude, float freq_hz, float offset);
void  Waveform_Set_Pulse(WAVEFORM_DRIVES* w, float low, float high,
                         float period_s, float width_s);
void  Waveform_Set_Limit(WAVEFORM_DRIVES* w, float min, float max);

#endif // USER_WAVEFORM_H
