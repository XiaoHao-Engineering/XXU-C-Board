#include "../../Core/Inc/bsp.h"
#ifdef HAL_CAN_MODULE_ENABLED
#include <math.h>
#include "../user_dji_motor.h"

/* 13 位绝对编码器：一圈 8192 个计数（不是 8191），角度换算必须用它。
   用 8191 每圈多算 0.0122%，转 200 圈会累积偏 8.8°，表现为"直行走偏"。 */
#define ENCODER_CPR  (8192.0f)
static DJI_MOTOR_DRIVES* motor_drives[DJI_MOTOR_NUM];
static uint8_t motor_num = 0;

static void DJI_Motor_Execute(void* user_can);
static void DJI_Motor_Handle(void* user_can);
static void DJI_Motor_Check_Alive(DJI_MOTOR_DRIVES* motor);

// @param id                  电机 ID (1 ~ 8) 注意：GM6020 仅支持 1 ~ 7
void DJI_Motor_Init(DJI_MOTOR_DRIVES *user_motor, CAN_DRIVES* user_can, const uint8_t id, const float rotor_angle_offset,
                    const Dji_Motor_Type motor_type, const Dji_Control_Mode mode, CONTROLLER_INTERFACE* controller) {

    user_motor->Set_Motor_State        = DJI_Motor_Set_State;
    user_motor->Set_Power_Limit        = DJI_Motor_Set_Power_Limit;
    user_motor->Set_Rotor_Angle_Offset = DJI_Motor_Set_Rotor_Angle_Offset;
    user_motor->Get_Power_Limit        = DJI_Motor_Get_Power_Limit;
    user_motor->Get_Motor_Speed        = DJI_Motor_Get_Speed;
    user_motor->Get_Motor_Angle        = DJI_Motor_Get_Angle;
    user_motor->Get_Motor_Current      = DJI_Motor_Get_Current;
    
    user_motor->can = user_can;
    user_motor->id = id;
    user_motor->motor_type = motor_type;
    user_motor->control_mode = mode;
    user_motor->power_limit = 72.0f;
    user_motor->controller = controller;

    user_motor->last_rx_count = 0;
    user_motor->offline_ms = 0;
    user_motor->is_online = 0;
    user_motor->has_ever_online = 0;
    user_motor->encoder_init_flag = 0;

    DJI_Motor_Set_Rotor_Angle_Offset(user_motor, rotor_angle_offset);

    switch (motor_type) {
        case GM6020:
            user_motor->controller_type = GM6020_Controller;
            break;
        case M3508_direct:
        case M3508_gear:
            user_motor->controller_type = C620_Controller;
            break;
        case M2006:
            user_motor->controller_type = C610_Controller;
            break;
    }

    switch (user_motor->controller_type) {
        case GM6020_Controller:
            user_motor->max_current = GM6020_MAX_CURRENT;
            user_motor->fdb_id  = GM6020_FEEDBACK_BASE_ID + id;
            if (1 <= id && id <= 4) user_motor->ctrl_id = GM6020_CURRENT_CONTROL_ID_1;
            if (5 <= id && id <= 7) user_motor->ctrl_id = GM6020_CURRENT_CONTROL_ID_2;
            break;
        case C620_Controller:
            user_motor->max_current = C620_MAX_CURRENT;
            user_motor->fdb_id = C6x0_FEEDBACK_BASE_ID + id;
            if (1 <= id && id <= 4) user_motor->ctrl_id = C6x0_CURRENT_CONTROL_ID_1;
            if (5 <= id && id <= 8) user_motor->ctrl_id = C6x0_CURRENT_CONTROL_ID_2;
            break;
        case C610_Controller:
            user_motor->max_current = C610_MAX_CURRENT;
            user_motor->fdb_id = C6x0_FEEDBACK_BASE_ID + id;
            if (1 <= id && id <= 4) user_motor->ctrl_id = C6x0_CURRENT_CONTROL_ID_1;
            if (5 <= id && id <= 8) user_motor->ctrl_id = C6x0_CURRENT_CONTROL_ID_2;
            break;
    }

    uint8_t is_callback_register = 0;
    for (uint8_t motor_index = 0; motor_index < motor_num; motor_index++) {
        const DJI_MOTOR_DRIVES *motor = motor_drives[motor_index];
        if (motor->can->hcan == user_can->hcan)
            is_callback_register = 1;
    }

    motor_drives[motor_num] = user_motor;
    motor_num++;

    if (is_callback_register == 0) {
        CAN_RegisterCallback(user_can, DJI_Motor_Handle);
        SysTick_Task* DJI_Motor_Execute_Task = (SysTick_Task*)malloc(sizeof(SysTick_Task));
        SysTick_InitTask(DJI_Motor_Execute_Task, user_can, 100,
            MOTOR_EXECUTE_PERIOD,Task_REPEAT, DJI_Motor_Execute);
        SysTick_StartTask(DJI_Motor_Execute_Task);
    }
}

// 会在初始化时自动注册到 CAN 总线接收回调函数表
static void DJI_Motor_Handle(void* user_can) {
    const CAN_DRIVES* can = (CAN_DRIVES*)user_can;
    for (uint8_t motor_index = 0; motor_index < motor_num; motor_index++) {
        DJI_MOTOR_DRIVES *motor = motor_drives[motor_index];

        if (motor->can->hcan != can->hcan || motor->fdb_id != can->rx_msg.StdId)
            continue;

        motor->rx_count++;

        const uint16_t last_rotor_angle = motor->rotor_angle;

        const uint8_t *data = can->rx_msg.Data;
        motor->rotor_angle    = (uint16_t)(data[0] << 8 | data[1]);
        motor->rotor_speed    = (int16_t) (data[2] << 8 | data[3]);
        motor->torque_current = (int16_t) (data[4] << 8 | data[5]);
        motor->temperate      = data[6];

        /* ★ 首帧编码器基准保护：电机对象是全局 {0} 初始化的，第一帧 last_rotor_angle 恒为 0，
           若上电时电机恰好停在编码器 > 4096 的位置，delta 会被下面的"跨半圈"判据误判成反向
           跨了半圈，total_angle 立刻注入一个 ±4096 计数（±180°）的【恒定】偏差且永不自愈
           （表现为"上电时停的位置不同，行为就不一样"）。所以第一帧只建立基准，不判跨圈。 */
        if (motor->encoder_init_flag == 0) {
            motor->encoder_init_flag = 1;
            motor->total_angle += (int32_t)motor->rotor_angle;
        } else {
            int32_t delta_angle = motor->rotor_angle - last_rotor_angle;

            /* 编码器 0~8191，周期是 8192 不是 8191，否则每次回绕丢 1 个脉冲 */
            if (delta_angle >= 4096) {
                delta_angle -= 8192;
            } else if (delta_angle <= -4096) {
                delta_angle += 8192;
            }

            motor->total_angle += delta_angle;
        }

        if (motor->control_mode == OpenLoop_current)
            continue;

        switch (motor->control_mode) {
            case Rotor_speed: {
                motor->controller->Calculate(motor->controller,
                    DJI_Motor_Get_Speed(motor),
                     DJI_Motor_Get_Current(motor));
                break;
            }
            case Rotor_angle: {
                motor->controller->Calculate(motor->controller,
                    DJI_Motor_Get_Angle(motor),
                     DJI_Motor_Get_Speed(motor));
                break;
            }
            case Rotor_current: {
                motor->controller->Calculate(motor->controller,
                    DJI_Motor_Get_Current(motor),
                     0.0f);
                break;
            }
            case Servo_angle: {
                float error =  motor->target - (float)motor->rotor_angle / ENCODER_CPR * 360.0f;

                if (error > 180.0f)
                    error -= 360.0f;
                if (error < -180.0f)
                    error += 360.0f;

                motor->controller->Calculate(motor->controller,
                    -error,
                     DJI_Motor_Get_Speed(motor));
                break;
            }
            default:
                break;
        }
    }
}

/* 掉线保护：反馈计数不涨就累计超时，判掉线时把控制器状态清干净 */
/* 为什么必须清：反馈一断，DJI_Motor_Handle 不再被调用 -> controller->Calculate 不再执行
   -> Get_Output() 冻结在最后一次的值；不清的话，电调重新上线时会带着一个饱和的积分和历史误差踢一脚。
   本函数由 DJI_Motor_Execute 每 MOTOR_EXECUTE_PERIOD ms 调一次。 */
static void DJI_Motor_Check_Alive(DJI_MOTOR_DRIVES* motor) {
    if (motor->rx_count != motor->last_rx_count) {
        motor->last_rx_count = motor->rx_count;
        motor->offline_ms = 0;
        motor->is_online = 1;
        motor->has_ever_online = 1;
        return;
    }

    if (motor->offline_ms < DJI_MOTOR_OFFLINE_MS) {
        motor->offline_ms += MOTOR_EXECUTE_PERIOD;
    }

    /* 判掉线的那一刻清一次控制器状态（边沿触发，不是每个周期都清）*/
    if (motor->offline_ms >= DJI_MOTOR_OFFLINE_MS && motor->is_online) {
        motor->is_online = 0;

        if (motor->controller != NULL) {
            motor->controller->Reset(motor->controller);
        }
    }
}

// 将所有电机控制指令一并打包发送
static void DJI_Motor_Execute(void* user_can) {
    CAN_DRIVES* can = (CAN_DRIVES*)user_can;

    uint8_t GM6020_control_id_1_frame[8] = {0};
    uint8_t GM6020_control_id_2_frame[8] = {0};
    uint8_t C6x0_control_id_1_frame[8] = {0};
    uint8_t C6x0_control_id_2_frame[8] = {0};
    uint8_t GM6020_control_id_1_sign = 0;
    uint8_t GM6020_control_id_2_sign = 0;
    uint8_t C6x0_control_id_1_sign = 0;
    uint8_t C6x0_control_id_2_sign = 0;

    for (uint8_t motor_index = 0; motor_index < motor_num; motor_index++) {
        DJI_MOTOR_DRIVES *motor = motor_drives[motor_index];
        if (motor->can != can)
            continue;

        /* 掉线检测（本函数每 MOTOR_EXECUTE_PERIOD ms 跑一次）*/
        DJI_Motor_Check_Alive(motor);

        int16_t current_target = 0;

        /* 开环模式不依赖反馈，不参与掉线保护（与 Handle 里 OpenLoop_current 直接 continue 一致）*/
        const uint8_t is_output_forbidden =
            (motor->control_mode != OpenLoop_current) && (motor->is_online == 0);

        if (is_output_forbidden == 0) {
            if (motor->control_mode == OpenLoop_current) {
                current_target = (int16_t)motor->target;
            } else {
                current_target = (int16_t)motor->controller->Get_Output(motor->controller);
            }

            /* 过温保护：START 以上线性降额，CUTOFF 以上彻底断输出 */
#if DJI_MOTOR_TEMP_LIMIT_ENABLE
            const float temperature = (float)motor->temperate;
            if (temperature >= DJI_MOTOR_TEMP_CUTOFF) {
                current_target = 0;
            } else if (temperature > DJI_MOTOR_TEMP_DERATE_START) {
                const float temp_scale = (DJI_MOTOR_TEMP_CUTOFF - temperature)
                                       / (DJI_MOTOR_TEMP_CUTOFF - DJI_MOTOR_TEMP_DERATE_START);
                current_target = (int16_t)((float)current_target * temp_scale);
            }
#endif

            /* 功率上限 -> 电流限幅：按【该电调】电流满量程折算，不能统一用 20A（否则 C610/M2006 限幅宽一倍）*/
            float current_limit = motor->power_limit / MOTOR_BUS_VOLTAGE
                                / motor->max_current * 16384.0f;
            /* 限幅本身要先夹进 int16 范围，否则 (int16_t) 转换会溢出成负数 */
            if (current_limit > 16384.0f) current_limit = 16384.0f;
            if (current_limit <     0.0f) current_limit = 0.0f;

            const int16_t limit_reg = (int16_t)current_limit;
            if (current_target >  limit_reg) current_target =  limit_reg;
            if (current_target < -limit_reg) current_target = -limit_reg;
        }
        /* ⚠️ 无论在线与否都要写进帧：掉线/禁止输出时这里写的是 0，并且照样置 sign 把这一帧发出去。
           若因"没数据可发"就整帧不发，电调会保留上一次收到的电流指令继续出力 —— 反馈一断电机反而持续跑。 */

        switch (motor->ctrl_id) {
            case GM6020_CURRENT_CONTROL_ID_1:
                GM6020_control_id_1_frame[2 * motor->id - 2] = (uint8_t)(current_target >> 8);
                GM6020_control_id_1_frame[2 * motor->id - 1] = (uint8_t)(current_target >> 0);
                GM6020_control_id_1_sign = 1;
                break;
            case GM6020_CURRENT_CONTROL_ID_2:
                GM6020_control_id_2_frame[2 * motor->id - 10] = (uint8_t)(current_target >> 8);
                GM6020_control_id_2_frame[2 * motor->id - 9]  = (uint8_t)(current_target >> 0);
                GM6020_control_id_2_sign = 1;
                break;
            case C6x0_CURRENT_CONTROL_ID_1:
                C6x0_control_id_1_frame[2 * motor->id - 2] = (uint8_t)(current_target >> 8);
                C6x0_control_id_1_frame[2 * motor->id - 1] = (uint8_t)(current_target >> 0);
                C6x0_control_id_1_sign = 1;
                break;
            case C6x0_CURRENT_CONTROL_ID_2:
                C6x0_control_id_2_frame[2 * motor->id - 10] = (uint8_t)(current_target >> 8);
                C6x0_control_id_2_frame[2 * motor->id - 9]  = (uint8_t)(current_target >> 0);
                C6x0_control_id_2_sign = 1;
                break;
            default:
                break;
        }
    }

    if (GM6020_control_id_1_sign)
        CAN_Send(can, GM6020_CURRENT_CONTROL_ID_1, GM6020_control_id_1_frame, 8);
    if (GM6020_control_id_2_sign)
        CAN_Send(can, GM6020_CURRENT_CONTROL_ID_2, GM6020_control_id_2_frame, 8);
    if (C6x0_control_id_1_sign)
        CAN_Send(can, C6x0_CURRENT_CONTROL_ID_1,   C6x0_control_id_1_frame,   8);
    if (C6x0_control_id_2_sign)
        CAN_Send(can, C6x0_CURRENT_CONTROL_ID_2,   C6x0_control_id_2_frame,   8);

}

void DJI_Motor_Set_State(void* motor, const float value) {
    DJI_MOTOR_DRIVES* dji_motor = (DJI_MOTOR_DRIVES*)motor;

    dji_motor->target = value;

    switch (dji_motor->control_mode) {
        case Rotor_angle:
        case Rotor_speed:
        case Rotor_current:
        // case Servo_angle:
        // case OpenLoop_current:
            /* ★ 非显然原因：电机对象是全局 {0} 初始化的，在 DJI_Motor_Init 之前 controller
               还是 NULL，而有些模块（云台 pitch / 射击机构）的周期任务比 Init 更早开始跑，
               所以这里必须显式判空。 */
            if (dji_motor->controller != NULL) {
                dji_motor->controller->Set_Target(dji_motor->controller, value);
            }
            break;
        default:
            break;
    }

}

void DJI_Motor_Set_Power_Limit(void* motor, const float power_limit) {
    DJI_MOTOR_DRIVES* dji_motor = (DJI_MOTOR_DRIVES*)motor;
    dji_motor->power_limit = power_limit;
}

void DJI_Motor_Set_Rotor_Angle_Offset(void* motor, const float angle) {
    DJI_MOTOR_DRIVES* dji_motor = (DJI_MOTOR_DRIVES*)motor;
    const int16_t rotor_angle_offset = (int16_t)(angle / 360.0f * ENCODER_CPR);
    dji_motor->total_angle += rotor_angle_offset - dji_motor->rotor_angle_offset;
    dji_motor->rotor_angle_offset = rotor_angle_offset;
}

/* Set_Rotor_Angle_Offset 是 total_angle += (新偏移 - 旧偏移)，反解出令结果等于 angle_deg 的新偏移 */
void DJI_Motor_Set_Angle_Here(DJI_MOTOR_DRIVES* user_motor, const float angle_deg) {
    const float offset_old_deg = (float)user_motor->rotor_angle_offset / ENCODER_CPR * 360.0f;
    const float offset_new_deg = offset_old_deg - DJI_Motor_Get_Angle(user_motor) + angle_deg;
    DJI_Motor_Set_Rotor_Angle_Offset(user_motor, offset_new_deg);
}

float DJI_Motor_Get_Power_Limit(void* motor) {
    const DJI_MOTOR_DRIVES* dji_motor = (DJI_MOTOR_DRIVES*)motor;
    return dji_motor->power_limit;
}

float DJI_Motor_Get_Speed(void* motor) {
    const DJI_MOTOR_DRIVES* dji_motor = (DJI_MOTOR_DRIVES*)motor;
    return (float)dji_motor->rotor_speed;
}

float DJI_Motor_Get_Angle(void* motor) {
    const DJI_MOTOR_DRIVES* dji_motor = (DJI_MOTOR_DRIVES*)motor;
    return (float)dji_motor->total_angle / ENCODER_CPR * 360.0f;
}

float DJI_Motor_Get_Current(void* motor) {
    const DJI_MOTOR_DRIVES* dji_motor = (DJI_MOTOR_DRIVES*)motor;
    /* raw ±16384 对应该电调的电流满量程（C620 ±20A / C610 ±10A / GM6020 ±3A），换算成 mA */
    return (float)dji_motor->torque_current / 16384.0f * dji_motor->max_current * 1000.0f;
}

uint8_t DJI_Motor_Is_Online(const DJI_MOTOR_DRIVES* user_motor) {
    return (user_motor != NULL) ? user_motor->is_online : 0;
}

uint8_t DJI_Motor_Has_Ever_Online(const DJI_MOTOR_DRIVES* user_motor) {
    return (user_motor != NULL) ? user_motor->has_ever_online : 0;
}

float DJI_Motor_Get_Temperature(const DJI_MOTOR_DRIVES* user_motor) {
    return (user_motor != NULL) ? (float)user_motor->temperate : 0.0f;
}

#endif /* HAL_CAN_MODULE_ENABLED */
