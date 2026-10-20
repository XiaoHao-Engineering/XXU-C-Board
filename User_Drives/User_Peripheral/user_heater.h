#ifndef USER_HEATER_H
#define USER_HEATER_H
#include "main.h"
#ifdef HAL_TIM_MODULE_ENABLED

/* 包含头文件 */
#include "../../User_Algorithm/User_Controller/user_pid_adv.h"
#include "../../User_Architect/user_systick.h"

/* ---------------------------------------------------------------------------
 * IMU 恒温加热驱动（驱动层）
 *
 * 硬件事实（已核实，勿改）：
 *   PF6 = TIM10_CH1，PWM，高电平加热；
 *   9 个 430Ω 电阻并联 = 47.8Ω，5V -> 0.52W，最大约 116mA。
 *   TIM10 由 CubeMX 配成 Prescaler=0、Period=16800-1，
 *   168MHz / 16800 = 10kHz PWM。
 *
 * ★★ 所以 CCR 的有效范围是 0~16800，而不是参考工程的 0~1000：
 *    参考工程 ARR=1000，它的 duty 直接写 CCR；本工程 ARR=16799，
 *    必须换算。本驱动【对外统一用 0~1000 千分比】，
 *    只在 Heater_Apply_Duty() 里换算成 CCR = duty × 16800 / 1000。
 *
 * 温控逻辑照着参考工程 Class_Heating_Resistor 移植（两段式）：
 *   1) 未预热：温度 < 47℃ 时满占空比加热；到 47℃ 标记预热完成；
 *   2) 预热完成：PID 恒温 50℃，参数 Kp=100 / Ki=10 / Kd=0，
 *      max_out=1000、max_iout=1000、dt=0.1s（与 100ms 任务对齐）。
 *
 * ★ 安全保护（本工程新增，参考工程没有）：
 *   - 温度 > 65℃：立刻 duty=0 并把 enabled 清 0（过温锁存，需人工重新置 1）；
 *   - 温度 < -10℃ 或非有限值（NaN/Inf，传感器读飞）：duty=0 并置 sensor_fault，
 *     防止温度反馈失效时一直满功率加热。
 *
 * ★ 任务放在 SysTick 中断里（100ms 一次），里面【绝不做 SPI 读】：
 *   温度直接用 BMI088 的 2ms 任务已经更新好的 user_bmi088.temperature。
 * ------------------------------------------------------------------------- */

/* 宏定义 ------------------------------------------------------------------*/

/* ★ 折算基准：TIM10 ARR = 16800-1 -> 一个 PWM 周期 16800 个计数（10kHz@168MHz）。
   这个值必须和 CubeMX 生成的 tim.c 一致；改 .ioc 的 Period 就得同步改这里。*/
#define HEATER_PWM_CCR_MAX              (16800U)

#define HEATER_DUTY_MAX                 (1000.0f)  /* 对外占空比满量程（千分比）*/
#define HEATER_PREHEAT_DUTY             (1000.0f)  /* 预热阶段满占空比（仍受 duty_max 上限约束）*/

#define HEATER_TARGET_TEMP_DEF          (50.0f)    /* 恒温目标 ℃ */
#define HEATER_PREHEAT_BASE_TEMP_DEF    (47.0f)    /* 预热结束阈值 ℃ */
#define HEATER_ENABLE_DEF               (1.0f)     /* 上电默认允许加热（float：调参表只收 float*）*/

#define HEATER_OVER_TEMP                (65.0f)    /* ★ 过温保护阈值 ℃ */
#define HEATER_OVER_TEMP_HYST           (5.0f)     /* 回落到 60℃ 以下才清 over_temp_fault 标志 */
#define HEATER_SENSOR_MIN_TEMP          (-10.0f)   /* ★ 低于此值视为温度传感器异常 */
#define HEATER_TARGET_TEMP_MIN          (0.0f)     /* 目标值下限（防御性钳位）*/
#define HEATER_TARGET_TEMP_MAX          (64.0f)    /* 目标值上限：必须低于过温阈值 */

#define HEATER_PID_KP                   (100.0f)
#define HEATER_PID_KI                   (10.0f)
#define HEATER_PID_KD                   (0.0f)
#define HEATER_PID_MAX_OUT              (HEATER_DUTY_MAX)  /* PID 输出量纲 = 千分比 */
#define HEATER_PID_MAX_IOUT             (HEATER_DUTY_MAX)
#define HEATER_PID_DT                   (0.1f)     /* ★ 必须与任务周期 100ms 一致 */

#define HEATER_PERIOD_MS                (100U)     /* 控制周期 100ms，对应 dt=0.1s */

/* 类型定义 ----------------------------------------------------------------*/

typedef struct {
    /* ---- 在线可调参数：全部 float（串口调参表只接受 float*）---- */
    float    target_temp;        /* 恒温目标 ℃（调参名 heater_target，30~60）*/
    float    preheat_base_temp;  /* 预热结束阈值 ℃（调参名 heater_preheat，30~60）*/
    float    duty_max;           /* 占空比上限 0~1000（调参名 heater_duty；0=不加热）*/
    float    enabled;            /* 0=停加热 1=允许（调参名 heater_enable；过温会自动清 0）*/

    /* ---- 观测量 ---- */
    float    current_temp;       /* 最近一次送进来的 IMU 温度 ℃ */
    float    output_duty;        /* 实际输出占空比 0~1000（已受 duty_max 限制）*/
    uint32_t duty_ccr;           /* 实际写入 TIM10->CCR1 的值 0~16800 */

    /* ---- 运行时状态 ---- */
    uint8_t  preheat_done;       /* 0=预热阶段（满占空比）1=已到 47℃，切 PID */
    uint8_t  over_temp_fault;    /* 1=曾触发过温保护（enabled 已被清 0）*/
    uint8_t  sensor_fault;       /* 1=温度值异常（NaN/Inf 或 < -10℃），已强制 duty=0 */

    PID_Adv_Controller pid;      /* 恒温 PID，输出量纲就是 0~1000 千分比 */
    SysTick_Task       task;     /* 100ms 周期任务 */
} HEATER_DRIVES;

/* 函数声明 ----------------------------------------------------------------*/

/* 启动 PWM、CCR 清 0、配置 PID 并注册 100ms SysTick 任务。
   调用位置必须在 MX_TIM10_Init() 之后；建议紧跟 BMI088_Init() 之后。*/
void Heater_Init(HEATER_DRIVES* heater);

/* 周期控制入口（只在 SysTick 100ms 任务里调用）。
   current_temp_degC：直接传 user_bmi088.temperature，本函数内部不做任何 SPI 读。*/
void Heater_Update(HEATER_DRIVES* heater, float current_temp_degC);

#endif /* HAL_TIM_MODULE_ENABLED */
#endif /* USER_HEATER_H */
