#ifndef USER_BSP_CONFIG_H
#define USER_BSP_CONFIG_H

/* 时钟参数 ------------------------------------------------------------------*/

#define APB1_CLK         (     HAL_RCC_GetPCLK1Freq()    )
#define APB1_TIM_CLK     ( 2 * HAL_RCC_GetPCLK1Freq()    )
#define APB2_CLK         (     HAL_RCC_GetPCLK2Freq()    )
#define APB2_TIM_CLK     ( 2 * HAL_RCC_GetPCLK2Freq()    )
#define HCLK_CLK         (     HAL_RCC_GetHCLKFreq()     )
#define SYS_CLK          (     HAL_RCC_GetSysClockFreq() )

#define TIM1_CLK         APB2_TIM_CLK
#define TIM8_CLK         APB2_TIM_CLK
#define TIM9_CLK         APB2_TIM_CLK
#define TIM10_CLK        APB2_TIM_CLK
#define TIM11_CLK        APB2_TIM_CLK

#define TIM2_CLK         APB1_TIM_CLK
#define TIM3_CLK         APB1_TIM_CLK
#define TIM4_CLK         APB1_TIM_CLK
#define TIM5_CLK         APB1_TIM_CLK
#define TIM6_CLK         APB1_TIM_CLK
#define TIM7_CLK         APB1_TIM_CLK
#define TIM12_CLK        APB1_TIM_CLK
#define TIM13_CLK        APB1_TIM_CLK
#define TIM14_CLK        APB1_TIM_CLK

/* 接口定义 ------------------------------------------------------------------*/
#define MAX_SYSTICK_TASK      (32)        /* 滴答定时器最大任务数量 */

#define TIMER_NUM             (14)        /* 开发板的定时器总数 */
#define TIMER_CALLBACK_NUM    (8)         /* 通用定时器回调函数数量 */

#define UART_NUM              (5)         /* 开发板的 UART 总数 */
#define VISION_NUM              (1)     /* 视觉串口对接设备数 */
#define UART_CALLBACK_NUM     (8)         /* 最大回调函数数量 */

#define CAN_NUM               (2)         /* 开发板的 CAN 总数 */
#define CAN_CALLBACK_NUM      (8)         /* 最大回调函数数量 */

#define ADC_NUM               (3)         /* 开发板的 ADC 总数 */
#define ADC_CHANNEL_MAX       (16)        /* 单个 ADC 的最大通道数 */
#define ADC_CALLBACK_NUM      (8)         /* 最大回调函数数量 */

#define DJI_MOTOR_NUM         (16)        /* 大疆电机最大挂载数量 */

#define STP23_NUM             (5)         /* 开发板最多可挂载的 STP23 单点激光测距传感器数量 该传感器需要独占串口 */
#define STP23_CALLBACK_NUM    (8)         /* STP23 单点激光测距模块每个实例的最大回调函数数量 */

#define HWT906_NUM            (2)         /* 开发板最多可挂载的 HWT906 姿态传感器数量 该传感器需要独占串口*/
#define HWT906_CALLBACK_NUM   (8)         /* HWT906 每个实例的最大回调函数数量 */


/* 板载 IMU：C 板是 BMI088（六轴）+ IST8310（磁力计，暂未接）
   A 板的 MPU6500 驱动保留在 User_Sensor 里备查，不再参与构建 */
#define MPU6500_NUM           (1)         /* 板载 MPU6500 数量 */
#define MPU6500_CALLBACK_NUM  (8)         /* MPU6500 每个实例的最大回调函数数量 */

#define BMI088_NUM            (1)         /* 板载 BMI088 数量 */
#define BMI088_CALLBACK_NUM   (8)         /* BMI088 每个实例的最大回调函数数量 */

#define SERIALPLOT_NUM        (2)         /* 串口绘图实例数量 */

#define ATTITUDE_NUM          (2)         /* 姿态解算实例数量 */

/* 全局结构体 ----------------------------------------------------------------*/


#endif // USER_BSP_CONFIG_H
