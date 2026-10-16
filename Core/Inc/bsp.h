#ifndef USER_BSP_H
#define USER_BSP_H

#include "main.h"
#include "bsp_config.h"
#include "../../SEGGER_RTT/SEGGER_RTT.h"

/* 主循环事件表 */
#define MAX_LOOP_EVENT 32
typedef void (*LOOP_Event)(void);
extern LOOP_Event loop_event[MAX_LOOP_EVENT];
extern uint8_t loop_event_num;
void LOOP_EVENT_Handle(void);

/* 滴答任务 */
#include "../../User_Architect/user_systick.h"
extern SysTick_Task LED_Blink_Task;
void LED_Blink_Callback(void *arg);

/* JScope */
#include "../../SEGGER_RTT/user_JScope_Transmit.h"
extern JScope_Transmit_t jscope_transmit;
extern uint8_t JScope_RTT_UpBuffer[BUFFER_SIZE_UP];

/* 串口 */
#include "../../User_Drives/User_Peripheral/user_uart.h"
extern UART_DRIVES user_imu_uart;

/* 状态灯 */
#include "../../User_Drives/User_Basic/user_led.h"
extern LED_DRIVES user_red_led;
extern LED_DRIVES user_green_led;

/* CAN 总线 */
#include "../../User_Drives/User_Peripheral/user_can.h"
extern CAN_DRIVES user_can_1;
extern CAN_DRIVES user_can_2;

/* 定时器 */
#include "../../User_Drives/User_Peripheral/user_timer.h"
extern TIMER_DRIVES user_timer_10;

/* 蜂鸣器 */
#include "../../User_Drives/User_Basic/user_buzzer.h"
extern BUZZER_DRIVES user_buzzer_1;

/* 启动音乐 */
#include "../../User_Application/user_startup_music.h"
extern STARTUP_MUSIC_DRIVES user_startup_music;
extern SysTick_Task user_startup_music_task;

/* 遥控器 */
#include "../../User_Drives/user_dji_bus.h"
extern DBUS_DRIVES user_dbus;

/* 姿态传感器 */
#include "../../User_Drives/User_Sensor/user_hwt906.h"
extern HWT906_DRIVES user_imu;

/* 底盘驱动电机 */
#include "../../User_Drives/User_Motor/user_dji_motor.h"
extern DJI_MOTOR_DRIVES user_wheel_fl;
extern DJI_MOTOR_DRIVES user_wheel_fr;
extern DJI_MOTOR_DRIVES user_wheel_rl;
extern DJI_MOTOR_DRIVES user_wheel_rr;

/* 轮速 PID */
#include "../../User_Algorithm/User_Controller/user_pid.h"

/* PID 测试用波形发生器 */
#include "../../User_Algorithm/user_waveform.h"
extern WAVEFORM_DRIVES user_wave;
extern PID_Controller user_wheel_pid_fl;
extern PID_Controller user_wheel_pid_fr;
extern PID_Controller user_wheel_pid_rl;
extern PID_Controller user_wheel_pid_rr;

/* 云台 yaw / pitch */
#include "../../User_Application/gimbal.h"
extern DJI_MOTOR_DRIVES user_gimbal_yaw_motor;    /* GM6020 (ID 1)  挂 CAN1 */
extern DJI_MOTOR_DRIVES user_gimbal_pitch_motor;  /* GM6020 (ID 4)  挂 CAN2 */
extern PID_Controller   user_gimbal_yaw_pid;      /* yaw 速度环（注入给电机）*/
extern GIMBAL_DRIVES    user_gimbal;              /* 含 pitch 的两个 PID 与串级 */

/* 射击机构：两个摩擦轮 + 一个拨弹轮（都挂 CAN2）*/
#include "../../User_Application/shooter.h"
extern DJI_MOTOR_DRIVES user_friction_left;       /* M3508 (ID 1) */
extern DJI_MOTOR_DRIVES user_friction_right;      /* M3508 (ID 2) */
extern DJI_MOTOR_DRIVES user_feeder;              /* M2006 (ID 3) */
extern SHOOTER_DRIVES   user_shooter;

/* 底盘 */
#include "../../User_Application/chassis.h"
#include "../../User_Application/chassis_control.h"
extern Chassis user_chassis;
extern SysTick_Task chassis_task;

/* 串口绘图 / 在线调参 */
#include "../../User_Drives/User_Peripheral/user_serialplot.h"
#include "../../User_Drives/User_Peripheral/user_vision.h"
extern UART_DRIVES       user_serialplot_uart;
extern UART_DRIVES  user_vision_uart;
extern VISION_DRIVES user_vision;
extern SERIALPLOT_DRIVES user_serialplot;

/* 板载 IMU 与姿态解算 */
#include "../../User_Drives/User_Peripheral/user_spi.h"
#include "../../User_Drives/User_Sensor/user_bmi088.h"
#include "../../User_Algorithm/User_Attitude/attitude.h"
extern SPI_DRIVES      user_spi_accel;
extern SPI_DRIVES      user_spi_gyro;
extern BMI088_DRIVES   user_bmi088;
extern ATTITUDE_DRIVES user_attitude;

#endif /* USER_BSP_H */
