#ifndef USER_DJI_MOTOR_H
#define USER_DJI_MOTOR_H
#include "main.h"
#ifdef HAL_CAN_MODULE_ENABLED

#include "user_motor.h"
#include "../../Core/Inc/bsp_config.h"
#include "../User_Peripheral/user_can.h"
#include "../../User_Algorithm/User_Controller/user_controller.h"

#define MOTOR_EXECUTE_PERIOD          (  2  )

/* 电机掉线判定：连续这么久收不到反馈就判掉线（ms） */
#define DJI_MOTOR_OFFLINE_MS          (100)

/* 过温保护：从 START 开始线性降额，到 CUTOFF 完全断输出 */
#define DJI_MOTOR_TEMP_LIMIT_ENABLE   (1)
#define DJI_MOTOR_TEMP_DERATE_START   (70.0f)   /* ℃ 开始降额 */
#define DJI_MOTOR_TEMP_CUTOFF         (85.0f)   /* ℃ 完全断输出 */

/* 各电调的电流满量程：对应电流帧的 ±16384 */
/* ⚠️ 这三个值决定"功率上限->电流限幅"折算和 Get_Current 的 mA 换算，
   必须按电调型号区分，不能统一用 20A（否则 C610/M2006 的功率限幅会宽一倍）*/
#define GM6020_MAX_CURRENT            ( 3.0f)   /* GM6020 电流帧满量程 ±3A */
#define C620_MAX_CURRENT              (20.0f)   /* C620 满量程 ±20A */
#define C610_MAX_CURRENT              (10.0f)   /* C610 满量程 ±10A */

/* 母线电压估计值，仅用于把"功率上限 W"折算成电流限幅 */
#define MOTOR_BUS_VOLTAGE             (24.0f)

#define GM6020_CURRENT_CONTROL_ID_1   (0x1FE)
#define GM6020_CURRENT_CONTROL_ID_2   (0x2FE)
#define GM6020_FEEDBACK_BASE_ID       (0x204)

#define C6x0_CURRENT_CONTROL_ID_1     (0x200)
#define C6x0_CURRENT_CONTROL_ID_2     (0x1FF)
#define C6x0_FEEDBACK_BASE_ID         (0x200)

// 大疆不同型号电调的 ID 可能存在冲突：C610 与 C620 的 ID 全部一致，故统一定义为 C6x0 电调。

typedef enum {
    GM6020,
    M3508_gear,
    M3508_direct,
    M2006,
} Dji_Motor_Type;

typedef enum {
    GM6020_Controller,
    C620_Controller,
    C610_Controller,
} Dji_Controller_Type;

typedef enum {
    Servo_angle,      /* 单圈角度控制模式 单位：度 */
    Rotor_angle,      /* 多圈角度控制模式 单位：度 */
    Rotor_speed,      /* 转子速度控制模式 单位：RPM */
    Rotor_current,    /* 转矩电流控制模式 目标值范围 (-3000 ~ 3000) 单位：mA */
    OpenLoop_current, /* 开环电流控制模式 目标值范围 (-16384 ~ 16384) */
} Dji_Control_Mode;

typedef struct {
    MOTOR_DRIVES_INTERFACE_FUNC
    CAN_DRIVES *can;
    Dji_Motor_Type motor_type;
    Dji_Controller_Type controller_type;
    Dji_Control_Mode control_mode;
    float target;
    float power_limit;                    /* 功率限制上限 单位：W */
    uint8_t id;                           /* 电机 ID (1 ~ 7) */
    uint16_t ctrl_id;
    uint16_t fdb_id;
    float max_current;                    /* 该电调的电流满量程 A，用于功率折算与 mA 换算 */
    uint16_t rotor_angle;                 /* 转子角度 单位：编码器的脉冲数 (0 ~ 8191) */
    int16_t rotor_angle_offset;           /* 角度零点偏移量 单位：编码器的脉冲数 */
    int32_t total_angle;                  /* 多圈角度 单位：编码器的脉冲数 */
    int16_t rotor_speed;                  /* 转子速度 单位: rpm */
    int16_t torque_current;
    uint8_t temperate;                    /* 温度 单位: 摄氏度 */
    uint32_t rx_count;
    uint32_t last_rx_count;
    uint16_t offline_ms;                  /* 连续未收到反馈的时间 ms */
    uint8_t is_online;                    /* 1=反馈正常 */
    uint8_t has_ever_online;              /* 1=本次上电曾经收到过反馈 */
    uint8_t encoder_init_flag;            /* 1=已用第一帧建立编码器基准（见 .c 里的说明）*/
    CONTROLLER_INTERFACE* controller;
} DJI_MOTOR_DRIVES;

void DJI_Motor_Init(DJI_MOTOR_DRIVES *user_motor, CAN_DRIVES* user_can, uint8_t id, float rotor_angle_offset,
                    Dji_Motor_Type motor_type, Dji_Control_Mode mode, CONTROLLER_INTERFACE* controller);

/* 把【电机当前位置】定义成指定角度（单位：度）：等价于"就地重新标定零点"。
   用于 Rotor_angle / Servo_angle 这类绝对角度闭环在启动时对齐真实机械角度（如 pitch 俯仰角）。*/
void DJI_Motor_Set_Angle_Here(DJI_MOTOR_DRIVES *user_motor, float angle_deg);

void DJI_Motor_Set_State(void* motor, float value);
void DJI_Motor_Set_Power_Limit(void* motor, float power_limit);
void DJI_Motor_Set_Rotor_Angle_Offset(void* motor, float angle);

float DJI_Motor_Get_Power_Limit(void* motor);
float DJI_Motor_Get_Speed(void* motor);
float DJI_Motor_Get_Angle(void* motor);
float DJI_Motor_Get_Current(void* motor);

/* 反馈是否正常：掉线返回 0。掉线时输出会被强制清零，见 .c 里的 "掉线保护" */
uint8_t DJI_Motor_Is_Online(const DJI_MOTOR_DRIVES* user_motor);

/* 本次上电是否曾经收到过反馈（区分"没接线"和"接上又掉线"）*/
uint8_t DJI_Motor_Has_Ever_Online(const DJI_MOTOR_DRIVES* user_motor);

/* 电机温度 ℃（来自反馈帧第 7 字节）*/
float DJI_Motor_Get_Temperature(const DJI_MOTOR_DRIVES* user_motor);

#endif /* HAL_CAN_MODULE_ENABLED */
#endif // USER_DJI_MOTOR_H
