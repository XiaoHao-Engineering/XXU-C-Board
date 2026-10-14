#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include "main.h"
#include "chassis.h"
#include "chassis_config.h"
#include "../User_Drives/user_dji_bus.h"
#include "../User_Drives/User_Peripheral/user_serialplot.h"

/* 底盘模式 */
typedef enum {
    CHASSIS_MODE_MANUAL = 0,   /* 手动：ch2 直接给自转 */
    CHASSIS_MODE_SPIN,         /* 小陀螺：恒定自转，行进方向按航向补偿 */
    CHASSIS_MODE_FOLLOW,       /* 跟随：把航向夹角控到 0 */
} Chassis_Mode;

/* 航向数据源：返回「参考系相对底盘」的角度 deg，逆时针为正
   A 板在云台上，IMU 的 yaw 是云台世界姿态，不能当底盘航向；
   要小陀螺走直线就接云台 yaw 编码器，返回云台相对底盘的角度 */
typedef float (*Chassis_HeadingSource)(void);

/* 航向源是否可信：返回 0 表示角度当前不可用（比如云台电机掉线） */
typedef uint8_t (*Chassis_SourceValid)(void);

/* 默认值：返回 0，功能退化为车体系直控 */
float ChassisControl_NoHeading(void);

/* 默认值：始终可信 */
uint8_t ChassisControl_SourceAlwaysValid(void);

void ChassisControl_Init(Chassis* chassis, DBUS_DRIVES* dbus);
void ChassisControl_Set_HeadingSource(Chassis_HeadingSource source);
void ChassisControl_Set_SourceValid(Chassis_SourceValid valid);

/* 遥控指令 -> 底盘速度 -> 下发电机，按 CHASSIS_PERIOD_MS 周期调用 */
void ChassisControl_Update(void);

/* 挂到 SysTick 任务表的包装 */
void ChassisControl_Task(void* arg);

/* 挂到 TIM10 的 JScope 数据源回调 */
void ChassisControl_JScopeCallback(void* arg);

Chassis_Mode ChassisControl_Get_Mode(void);

/* 在线调参变量表：交给串口绘图挂载 */
const SERIALPLOT_VAR* ChassisControl_Get_Tunable(uint8_t* count);

#endif /* CHASSIS_CONTROL_H */
