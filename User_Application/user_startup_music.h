#ifndef USER_STARTUP_MUSIC_H
#define USER_STARTUP_MUSIC_H
#include "main.h"
#ifdef HAL_TIM_MODULE_ENABLED

#include "../User_Drives/User_Basic/user_buzzer.h"
#include "../User_Architect/user_systick.h"

/* 开机音乐总时长上限（ms），0 = 不限制。bad_apple 有 885 个音符、约 177 秒，不封顶
   上电后会一直响；闸门放在"开始播下一拍之前"判断，超限就停在上一拍，不切音符。 */
#define STARTUP_MUSIC_MAX_MS   (1500)

typedef enum {
    MUSIC_IDLE,
    MUSIC_PLAYING,
    MUSIC_FINISHED,
} MUSIC_State;

typedef struct {
    BUZZER_DRIVES *buzzer;
    const uint16_t (*music)[2];    /* 音乐数据 [频率, 时长] */
    SysTick_Task *task;
    uint16_t note_count;
    uint16_t current_note;
    uint16_t time_counter;
    uint32_t elapsed_ms;           /* 本次已播放时长 ms */
    uint32_t max_ms;
    MUSIC_State state;
} STARTUP_MUSIC_DRIVES;

void StartupMusic_Init(STARTUP_MUSIC_DRIVES *music_player, BUZZER_DRIVES *buzzer,SysTick_Task *task,
                       const uint16_t (*music_data)[2], uint16_t note_count);

void StartupMusic_Start(STARTUP_MUSIC_DRIVES *music_player);

#endif /* HAL_TIM_MODULE_ENABLED */
#endif
