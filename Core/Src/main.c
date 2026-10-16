/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp.h"
#include <stdio.h>
#include "../../User_Lib/user_music.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static float Chassis_Heading_Get(void) {
    return Gimbal_Get_Yaw(&user_gimbal);
}

/* 航向源可信度：从没接过=可信（裸车调试，角度恒 0）；接过又掉线（角度冻结）或未对齐=不可信 */
static uint8_t Chassis_Source_Valid(void) {
    return Gimbal_Is_Reliable(&user_gimbal);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
/* ★★ 启动进度播报宏：用来定位"卡在哪一步"。
   每过一关就往【绘图口 USART6】和【视觉口 USART1】各发一行 ASCII。
   排查完可以整段删掉，不影响功能。 */
#define BOOT_TRACE(tag) do {                                        \
    static const char _bt[] = "\r\n[BOOT] " tag "\r\n";              \
    HAL_UART_Transmit(&huart6, (uint8_t*)_bt, sizeof(_bt) - 1, 200); \
    HAL_UART_Transmit(&huart1, (uint8_t*)_bt, sizeof(_bt) - 1, 200); \
} while (0)

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    /* ★★ 硬件 FPU 必须在使用任何浮点指令之前打开，否则第一条 VFP 指令就是 NoCP 异常 -> 上电即跑飞。
       正常路径是 SystemInit() 里根据 __FPU_USED 自动开；这里再显式开一次做保险
       （与 CMakeLists 里的 -mfpu=fpv4-sp-d16 -mfloat-abi=hard 配套）。幂等，可重复执行。*/
#if (__FPU_PRESENT == 1U)
    SCB->CPACR |= ((3UL << (10U * 2U)) | (3UL << (11U * 2U)));
    __DSB();
    __ISB();
#endif
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
    /* ★★ 性能关键：CubeMX 生成的 spi.c 里 BaudRatePrescaler = 256
       -> APB2 84MHz/256 = 328 kHz，读一次 IMU(19字节) 要 ≈463us，占满 1ms 预算的 46%！
       BMI088 的 SPI 支持到 10MHz，这里提到 84/16 = 5.25MHz -> 同一次读取降到 ≈29us。
       放在这里而不是改 spi.c，是为了不被下次 CubeMX 重新生成覆盖掉。
       （建议同时在 CubeMX 里把 SPI1 的 Prescaler 也改成 16，保持两边一致）*/
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
  MX_CAN2_Init();
  MX_TIM10_Init();
  MX_USART6_UART_Init();
    BOOT_TRACE("M1 gpio/dma/uart inits OK");
  /* USER CODE BEGIN 2 */
  /* JScope 采样节拍：TIM10 跑 10kHz（C 板蜂鸣器占 TIM4_CH3，PWM_Set_Frequency 会改
     TIM4 的 ARR，节拍不能挂 TIM4；PF6 上的加热通道不启动）*/
  JScope_Init(&htim10);
  TIMER_Init(&user_timer_10, &htim10, TIM10_CLK);
  TIMER_RegisterCallback(&user_timer_10, ChassisControl_JScopeCallback);

    /* ★★ HWT906（串口姿态传感器）本车【不使用】，已整段停用。
       原因一：没接、没用到。
       原因二（更关键）：它和【遥控接收机 DBUS】共用 USART3 ——
              两个驱动在同一 UART 上各自跑 DMA 收包会互相破坏，
               可能在中断里跑飞 -> 表现就是"所有串口都没有输出"。
       将来真要用 HWT906，必须给它单独分配一个空闲 UART，不能再与 DBUS 共用。
    // UART_Init(&user_imu_uart, &huart3);
  /* 状态灯：C 板是 RGB 三色灯，红 = PH12，绿 = PH11，低电平点亮 */
  LED_Init(&user_red_led, LED_RED_GPIO_Port, LED_RED_Pin, 1);
  LED_Init(&user_green_led, LED_GREEN_GPIO_Port, LED_GREEN_Pin, 1);
  LED_On(&user_green_led);

  Buzzer_Init(&user_buzzer_1, &htim4, TIM_CHANNEL_3, TIM4_CLK);
  StartupMusic_Init(&user_startup_music, &user_buzzer_1, &user_startup_music_task,
                    dji_starting_music, sizeof(dji_starting_music) / sizeof(dji_starting_music[0]));
  StartupMusic_Start(&user_startup_music);

  CAN_Init(&user_can_1, &hcan1);
  CAN_Init(&user_can_2, &hcan2);

  /* 底盘电机：4 个 M3508 共用 0x200 帧，ID 按实际接线：左前=2 右前=3 左后=1 右后=4 */
  PID_Init(&user_wheel_pid_fl, CHASSIS_SPEED_KP, CHASSIS_SPEED_KI, CHASSIS_SPEED_KD,
           CHASSIS_SPEED_MAX_OUT, CHASSIS_SPEED_MAX_IOUT);
  PID_Init(&user_wheel_pid_fr, CHASSIS_SPEED_KP, CHASSIS_SPEED_KI, CHASSIS_SPEED_KD,
           CHASSIS_SPEED_MAX_OUT, CHASSIS_SPEED_MAX_IOUT);
  PID_Init(&user_wheel_pid_rl, CHASSIS_SPEED_KP, CHASSIS_SPEED_KI, CHASSIS_SPEED_KD,
           CHASSIS_SPEED_MAX_OUT, CHASSIS_SPEED_MAX_IOUT);
  PID_Init(&user_wheel_pid_rr, CHASSIS_SPEED_KP, CHASSIS_SPEED_KI, CHASSIS_SPEED_KD,
           CHASSIS_SPEED_MAX_OUT, CHASSIS_SPEED_MAX_IOUT);

  DJI_Motor_Init(&user_wheel_fl, &user_can_1, 2, 0.0f, M3508_gear, Rotor_speed,
                 (CONTROLLER_INTERFACE*)&user_wheel_pid_fl);
  DJI_Motor_Init(&user_wheel_fr, &user_can_1, 3, 0.0f, M3508_gear, Rotor_speed,
                 (CONTROLLER_INTERFACE*)&user_wheel_pid_fr);
  DJI_Motor_Init(&user_wheel_rl, &user_can_1, 1, 0.0f, M3508_gear, Rotor_speed,
                 (CONTROLLER_INTERFACE*)&user_wheel_pid_rl);
  DJI_Motor_Init(&user_wheel_rr, &user_can_1, 4, 0.0f, M3508_gear, Rotor_speed,
                 (CONTROLLER_INTERFACE*)&user_wheel_pid_rr);

  Chassis_Init(&user_chassis,
               (MOTOR_INTERFACE*)&user_wheel_fl, (MOTOR_INTERFACE*)&user_wheel_fr,
               (MOTOR_INTERFACE*)&user_wheel_rl, (MOTOR_INTERFACE*)&user_wheel_rr);

  /* 遥控器（C 板 DBUS 走 USART3：PC10/PC11，100000 + 8E1）与姿态传感器 */
  DBUS_Init(&user_dbus, &huart3);
    // HWT906_Init(&user_imu, &user_imu_uart);   /* 同上：未使用，且会与 DBUS 抢 USART3 */
  /* 板载 IMU：SPI1 + 姿态解算（BMI088 双片选：加速度 PA4 / 陀螺 PB0） */
  SPI_Init(&user_spi_accel, &hspi1, IMU_CS_ACCEL_GPIO_Port, IMU_CS_ACCEL_Pin);
  SPI_Init(&user_spi_gyro, &hspi1, IMU_CS_GYRO_GPIO_Port, IMU_CS_GYRO_Pin);
  BMI088_Init(&user_bmi088, &user_spi_accel, &user_spi_gyro, 1);
  Attitude_Init(&user_attitude, &user_bmi088, 0.001f);
    BOOT_TRACE("M2 IMU + attitude OK");

  UART_Init(&user_serialplot_uart, &huart6);

  /* ---- 串口自检探针 ----------------------------
     阻塞发送，绕过 TX 队列和 DMA，用来定位故障：
     文本模式看到 SERIALPLOT OK -> 引脚/接线/波特率 OK，问题在 TX 队列或 DMA；
     看不到则是接线问题（TX/RX 反了？GND 没接？）。
      ⚠ ASCII 用文本模式看，正常绘图帧是二进制要用 HEX 模式；USART1 与上位机波特率都是 115200 */
  {
    static const char selftest[] = "SERIALPLOT OK\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)selftest, sizeof(selftest) - 1, 100);
  }
  uint8_t tunable_num = 0;
  Serialplot_Init(&user_serialplot, &user_serialplot_uart,
                  ChassisControl_Get_Tunable(&tunable_num), tunable_num,
                  CHASSIS_SERIALPLOT_PERIOD_MS);

    /* 视觉链路（与算法组的自瞄对接）：UART1 */
    UART_Init(&user_vision_uart, &huart1);
    Vision_Init(&user_vision, &user_vision_uart);

    /* ★★ 开机把标识发到三个串口。
       之后用【普通串口助手】逐个物理口连一遍，看到哪行字就知道那个口是哪个 USART，
       再也不用猜接线（本次反复出错就是因为猜接线）。
       确认完这段可以整段删掉，不影响功能。 */
    {
        static const char id_uart1[] = "\r\n=== this port is USART1 ===\r\n";
        static const char id_uart3[] = "\r\n=== this port is USART3 ===\r\n";
        static const char id_uart6[] = "\r\n=== this port is USART6 ===\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)id_uart1, sizeof(id_uart1) - 1, 100);
        HAL_UART_Transmit(&huart3, (uint8_t*)id_uart3, sizeof(id_uart3) - 1, 100);
        HAL_UART_Transmit(&huart6, (uint8_t*)id_uart6, sizeof(id_uart6) - 1, 100);
    }
  /* 绘图通道在 Chassis_Apply_Plot_Mode() 里按 CHASSIS_SERIALPLOT_MODE 登记，可在线切 */

  /* 云台 yaw：GM6020，绝对编码器提供云台相对底盘的角度 */
  PID_Init(&user_gimbal_yaw_pid, GIMBAL_YAW_KP, GIMBAL_YAW_KI, GIMBAL_YAW_KD,
           GIMBAL_YAW_MAX_OUT, GIMBAL_YAW_MAX_IOUT);
  /* ★ Rotor_speed：云台 yaw 用转速模式，锁世界的外环在 gimbal.c 里 */
  DJI_Motor_Init(&user_gimbal_yaw_motor, &user_can_1, GIMBAL_YAW_MOTOR_ID, 0.0f,
                 GM6020, Rotor_speed, (CONTROLLER_INTERFACE*)&user_gimbal_yaw_pid);
  DJI_Motor_Set_Power_Limit(&user_gimbal_yaw_motor, GIMBAL_YAW_POWER_LIMIT);
  Gimbal_Init(&user_gimbal, &user_gimbal_yaw_motor);

  /* ---- 云台 pitch：GM6020 (ID 4)，★ 挂 CAN2 不挂 CAN1 ----
     CAN1 已有 4 轮 + yaw ≈ 6000 帧/s（78% 负载），再加 pitch 会到 ~91%，超出 CAN
     可靠区间；挂 CAN2 后 CAN1 保持 78%，CAN2（3 个射击电机 + pitch）约 65%。
     ★ 必须先 Gimbal_Pitch_Init（两个 PID 与串级）再 DJI_Motor_Init（注入
       pitch_cascade），否则 CAN 回调可能在内环指针还是 NULL 时调用 Calculate */
#if GIMBAL_PITCH_ENABLE
  Gimbal_Pitch_Init(&user_gimbal, &user_gimbal_pitch_motor);
#if GIMBAL_PITCH_CASCADE
  DJI_Motor_Init(&user_gimbal_pitch_motor, &user_can_2, GIMBAL_PITCH_MOTOR_ID, 0.0f,
                 GM6020, Rotor_angle, (CONTROLLER_INTERFACE*)&user_gimbal.pitch_cascade);
#else
  DJI_Motor_Init(&user_gimbal_pitch_motor, &user_can_2, GIMBAL_PITCH_MOTOR_ID, 0.0f,
                 GM6020, Rotor_speed, (CONTROLLER_INTERFACE*)&user_gimbal.pitch_pid_speed);
#endif
  DJI_Motor_Set_Power_Limit(&user_gimbal_pitch_motor, GIMBAL_PITCH_POWER_LIMIT);
#endif /* GIMBAL_PITCH_ENABLE */

  /* ---- 射击机构：两摩擦轮(M3508) + 一拨弹轮(M2006)，都挂 CAN2 ----
     反馈 ID 0x201/0x202/0x203 共用 0x200 帧（各占 2 字节：ID1->0,1 ID2->2,3 ID3->4,5）；同样先 Shooter_Init 再挂电机 */
#if SHOOTER_ENABLE
  Shooter_Init(&user_shooter, &user_friction_left, &user_friction_right, &user_feeder);

  DJI_Motor_Init(&user_friction_left, &user_can_2, SHOOTER_FRICTION_LEFT_ID, 0.0f,
                 M3508_gear, Rotor_speed, (CONTROLLER_INTERFACE*)&user_shooter.friction_left_pid);
  DJI_Motor_Init(&user_friction_right, &user_can_2, SHOOTER_FRICTION_RIGHT_ID, 0.0f,
                 M3508_gear, Rotor_speed, (CONTROLLER_INTERFACE*)&user_shooter.friction_right_pid);
  DJI_Motor_Init(&user_feeder, &user_can_2, SHOOTER_FEEDER_ID, 0.0f,
                 M2006, Rotor_angle, (CONTROLLER_INTERFACE*)&user_shooter.feeder_cascade);

  DJI_Motor_Set_Power_Limit(&user_friction_left, SHOOTER_FRICTION_POWER_MAX);
  DJI_Motor_Set_Power_Limit(&user_friction_right, SHOOTER_FRICTION_POWER_MAX);
  DJI_Motor_Set_Power_Limit(&user_feeder, SHOOTER_FEEDER_POWER_MAX);
#endif /* SHOOTER_ENABLE */

  ChassisControl_Init(&user_chassis, &user_dbus);
    BOOT_TRACE("M4 chassis_control OK - plot should start");
    /* ★ 启动进度标记：走到这里说明绘图通道已登记，串口上应该开始出数据了。
       若三个口都收不到这行字，说明卡在更早的初始化（那就是初始化问题，不是绘图问题）。 */
    {
        static const char boot_ok[] = "\r\n[BOOT] chassis_control OK - plot should start\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)boot_ok, sizeof(boot_ok) - 1, 200);
        HAL_UART_Transmit(&huart3, (uint8_t*)boot_ok, sizeof(boot_ok) - 1, 200);
        HAL_UART_Transmit(&huart6, (uint8_t*)boot_ok, sizeof(boot_ok) - 1, 200);
    }
  ChassisControl_Set_HeadingSource(Chassis_Heading_Get);
  ChassisControl_Set_SourceValid(Chassis_Source_Valid);
  SysTick_InitTask(&chassis_task, NULL, CHASSIS_PERIOD_MS, CHASSIS_PERIOD_MS,
                   Task_REPEAT, ChassisControl_Task);
  SysTick_StartTask(&chassis_task);

  SysTick_InitTask(&LED_Blink_Task, &user_red_led, 10, 800, Task_REPEAT, LED_Blink_Callback);
  SysTick_StartTask(&LED_Blink_Task);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    LOOP_EVENT_Handle();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
