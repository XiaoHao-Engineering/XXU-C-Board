#include "bsp.h"

/* 主循环事件表 */
LOOP_Event loop_event[MAX_LOOP_EVENT] = {0};
uint8_t loop_event_num = 0;

void LOOP_EVENT_Handle(void) {
    for (uint8_t i = 0; i < loop_event_num; i++) {
        loop_event[i]();
    }
}

JScope_Transmit_t jscope_transmit = {0};
uint8_t JScope_RTT_UpBuffer[BUFFER_SIZE_UP] = {0};

UART_DRIVES user_imu_uart = {0};

LED_DRIVES user_red_led = {0};
LED_DRIVES user_green_led = {0};

CAN_DRIVES user_can_1 = {0};
CAN_DRIVES user_can_2 = {0};

TIMER_DRIVES user_timer_10 = {0};

BUZZER_DRIVES user_buzzer_1 = {0};

STARTUP_MUSIC_DRIVES user_startup_music = {0};
SysTick_Task user_startup_music_task = {0};

DBUS_DRIVES user_dbus = {0};

HWT906_DRIVES user_imu = {0};

DJI_MOTOR_DRIVES user_wheel_fl = {0};
DJI_MOTOR_DRIVES user_wheel_fr = {0};
DJI_MOTOR_DRIVES user_wheel_rl = {0};
DJI_MOTOR_DRIVES user_wheel_rr = {0};

PID_Controller user_wheel_pid_fl = {0};
PID_Controller user_wheel_pid_fr = {0};
PID_Controller user_wheel_pid_rl = {0};
PID_Controller user_wheel_pid_rr = {0};

Chassis user_chassis = {0};
SysTick_Task chassis_task = {0};

WAVEFORM_DRIVES user_wave = {0};

DJI_MOTOR_DRIVES user_gimbal_yaw_motor = {0};
DJI_MOTOR_DRIVES user_gimbal_pitch_motor = {0};
PID_Controller   user_gimbal_yaw_pid = {0};
GIMBAL_DRIVES    user_gimbal = {0};

/* 射击机构：两个摩擦轮 + 一个拨弹轮 */
DJI_MOTOR_DRIVES user_friction_left = {0};
DJI_MOTOR_DRIVES user_friction_right = {0};
DJI_MOTOR_DRIVES user_feeder = {0};
SHOOTER_DRIVES   user_shooter = {0};

SysTick_Task LED_Blink_Task = {0};

/* 串口绘图 / 在线调参 */
UART_DRIVES       user_serialplot_uart = {0};
UART_DRIVES   user_vision_uart = {0};
VISION_DRIVES user_vision = {0};
SERIALPLOT_DRIVES user_serialplot = {0};

/* 板载 IMU 与姿态解算 */
SPI_DRIVES      user_spi_accel = {0};
SPI_DRIVES      user_spi_gyro = {0};
BMI088_DRIVES   user_bmi088 = {0};
ATTITUDE_DRIVES user_attitude = {0};

/* IMU 恒温加热（PF6 = TIM10_CH1）*/
HEATER_DRIVES   user_heater = {0};

void LED_Blink_Callback(void *arg) {
    LED_Toggle((const LED_DRIVES*)arg);
}
