#include "../../Core/Inc/bsp.h"
#ifdef HAL_TIM_MODULE_ENABLED
#include "../user_startup_music.h"
#include "../../User_Architect/user_systick.h"

static void StartupMusic_Update(void *arg);

/* 本函数挂在 1ms 周期任务上，所以 time_counter / elapsed_ms 的单位就是 ms */
static void StartupMusic_Update(void *arg) {
    STARTUP_MUSIC_DRIVES *music_player = (STARTUP_MUSIC_DRIVES *)arg;

    if (music_player->state != MUSIC_PLAYING) {
        return;
    }

    /* 每个 1ms 任务拍 +1，所以是真实毫秒数，不受"每段结尾多一拍"的影响 */
    music_player->elapsed_ms++;

    if (music_player->current_note >= music_player->note_count && music_player->time_counter == 0) {
        Buzzer_Off(music_player->buzzer);
        music_player->state = MUSIC_FINISHED;
        SysTick_StopTask(music_player->task);
        return;
    }

    if (music_player->time_counter > 0) {
        music_player->time_counter--;
        return;
    }

    if (music_player->current_note < music_player->note_count) {
        const uint16_t frequency = music_player->music[music_player->current_note][0];
        const uint16_t duration = music_player->music[music_player->current_note][1];

        /* ★ 总时长闸门：放在"开始播这一拍之前"判断，所以不会出现音符被切一半
           的爆音；设 max_ms = 0 表示不限制。 */
        if (music_player->max_ms > 0
            && music_player->elapsed_ms + duration > music_player->max_ms) {
            Buzzer_Off(music_player->buzzer);
            music_player->state = MUSIC_FINISHED;
            SysTick_StopTask(music_player->task);
            return;
        }

        if (frequency == 0) {
            Buzzer_Off(music_player->buzzer);
        } else {
            Buzzer_On(music_player->buzzer, frequency);
        }

        music_player->time_counter = duration;
        music_player->current_note++;
    }
}

void StartupMusic_Init(STARTUP_MUSIC_DRIVES *music_player, BUZZER_DRIVES *buzzer, SysTick_Task *task,
                       const uint16_t (*music_data)[2], const uint16_t note_count) {

    music_player->buzzer = buzzer;
    music_player->music = music_data;
    music_player->task = task;
    music_player->note_count = note_count;
    music_player->current_note = 0;
    music_player->time_counter = 0;
    music_player->elapsed_ms = 0;
    music_player->max_ms = STARTUP_MUSIC_MAX_MS;
    music_player->state = MUSIC_IDLE;

    SysTick_InitTask(task, (void *)music_player, 1, 1, Task_REPEAT, StartupMusic_Update);
    SysTick_StartTask(task);
}

void StartupMusic_Start(STARTUP_MUSIC_DRIVES *music_player) {
    music_player->current_note = 0;
    music_player->time_counter = 0;
    music_player->elapsed_ms = 0;
    music_player->state = MUSIC_PLAYING;
}

#endif /* HAL_TIM_MODULE_ENABLED */
