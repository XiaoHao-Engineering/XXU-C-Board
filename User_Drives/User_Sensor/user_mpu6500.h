#ifndef USER_MPU6500_H
#define USER_MPU6500_H
#include "main.h"
#ifdef HAL_SPI_MODULE_ENABLED

/* 包含头文件 */
#include "../User_Peripheral/user_spi.h"
#include "../../Core/Inc/bsp_config.h"
#include "../../User_Architect/user_systick.h"

/* 寄存器 ----------------------------------------------------------------*/
#define MPU6500_SMPLRT_DIV       (0x19)
#define MPU6500_CONFIG           (0x1A)
#define MPU6500_GYRO_CONFIG      (0x1B)
#define MPU6500_ACCEL_CONFIG     (0x1C)
#define MPU6500_ACCEL_XOUT_H     (0x3B)
#define MPU6500_SIGNAL_PATH_RST  (0x68)
#define MPU6500_USER_CTRL        (0x6A)
#define MPU6500_PWR_MGMT_1       (0x6B)
#define MPU6500_PWR_MGMT_2       (0x6C)
#define MPU6500_WHO_AM_I         (0x75)

/* 读标志位与固定值 ------------------------------------------------------*/
#define MPU6500_READ_FLAG        (0x80)
#define MPU6500_CHIP_ID          (0x70)   /* WHO_AM_I 应答值 */

/* 量程：陀螺 ±2000dps，加速度 ±8g --------------------------------------*/
#define MPU6500_GYRO_FS_2000     (0x18)
#define MPU6500_ACCEL_FS_8G      (0x10)

#define MPU6500_GYRO_LSB_PER_DPS  (16.4f)     /* ±2000dps */
#define MPU6500_ACCEL_LSB_PER_G   (4096.0f)   /* ±8g */
#define MPU6500_GRAVITY           (9.80665f)

/* 零偏标定采样次数（1kHz 下约 1 秒） ------------------------------------*/
#define MPU6500_CALIB_SAMPLES    (1000)

/* 类型定义 --------------------------------------------------------------*/

typedef void (*MPU6500_Callback)(void* user_mpu6500);

typedef struct {
    SPI_DRIVES* user_spi;                          /* SPI 驱动 */
    float gyro[3];                                 /* 角速度 rad/s，已减零偏 */
    float accel[3];                                /* 加速度 m/s^2 */
    float temperature;                             /* 温度 ℃ */
    float gyro_bias[3];                            /* 零偏 rad/s */
    uint32_t calib_count;                          /* 标定累计采样次数 */
    float calib_sum[3];                            /* 标定累加和 */
    uint8_t calibrated;                            /* 零偏标定完成标志 */
    uint8_t is_online;                             /* 通信正常标志 */
    MPU6500_Callback callbacks[MPU6500_CALLBACK_NUM];
    uint8_t callback_num;
    SysTick_Task task;
} MPU6500_DRIVES;

/* 函数声明 --------------------------------------------------------------*/
void    MPU6500_Init(MPU6500_DRIVES* user_mpu6500, SPI_DRIVES* user_spi, uint32_t period_ms);
void    MPU6500_RegisterCallback(MPU6500_DRIVES* user_mpu6500, MPU6500_Callback callback);
void    MPU6500_Calibrate_Start(MPU6500_DRIVES* user_mpu6500);
uint8_t MPU6500_Is_Calibrated(const MPU6500_DRIVES* user_mpu6500);

#endif /* HAL_SPI_MODULE_ENABLED */
#endif // USER_MPU6500_H
