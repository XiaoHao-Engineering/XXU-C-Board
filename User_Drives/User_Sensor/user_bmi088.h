#ifndef USER_BMI088_H
#define USER_BMI088_H
#include "main.h"
#ifdef HAL_SPI_MODULE_ENABLED

/* 包含头文件 */
#include "../User_Peripheral/user_spi.h"
#include "../../Core/Inc/bsp_config.h"
#include "../../User_Architect/user_systick.h"

/* 寄存器：加速度计（CS = PA4） ------------------------------------------*/
#define BMI088_ACC_CHIP_ID       (0x00)
#define BMI088_ACC_DATA          (0x12)   /* 0x12..0x17 加速度 6 字节，小端 */
#define BMI088_ACC_TEMP_MSB      (0x22)   /* 0x22..0x23 温度 2 字节，高字节在前 */
#define BMI088_ACC_CONF          (0x40)
#define BMI088_ACC_RANGE         (0x41)
#define BMI088_ACC_PWR_CONF      (0x7C)
#define BMI088_ACC_PWR_CTRL      (0x7D)
#define BMI088_ACC_SOFTRESET     (0x7E)

/* 寄存器：陀螺仪（CS = PB0） --------------------------------------------*/
#define BMI088_GYRO_CHIP_ID      (0x00)
#define BMI088_GYRO_RATE         (0x02)   /* 0x02..0x07 角速度 6 字节，小端 */
#define BMI088_GYRO_RANGE        (0x0F)
#define BMI088_GYRO_BANDWIDTH    (0x10)
#define BMI088_GYRO_LPM1         (0x11)
#define BMI088_GYRO_SOFTRESET    (0x14)

/* 读标志位与固定值 ------------------------------------------------------*/
#define BMI088_READ_FLAG         (0x80)
#define BMI088_SOFTRESET_CMD     (0xB6)
#define BMI088_ACC_CHIP_ID_VALUE  (0x1E)   /* 加速度计 ID 应答值 */
#define BMI088_GYRO_CHIP_ID_VALUE (0x0F)   /* 陀螺仪 ID 应答值 */

/* 量程与带宽：陀螺 ±2000dps / 1kHz，加速度 ±6g / 800Hz ------------------*/
#define BMI088_ACC_PWR_CTRL_ON   (0x04)   /* 加速度计上电 */
#define BMI088_ACC_PWR_CONF_ACTIVE (0x00) /* 正常工作模式（0x03 = 挂起） */
#define BMI088_ACC_CONF_BWP_NORMAL (0x02 << 4)  /* bits6:4 正常带宽 */
#define BMI088_ACC_CONF_ODR_800HZ  (0x0B)       /* bits3:0 800Hz */
#define BMI088_ACC_RANGE_6G      (0x01)
#define BMI088_GYRO_RANGE_2000   (0x00)
#define BMI088_GYRO_BW_1000HZ_116HZ (0x02)      /* ODR 1kHz / 滤波 116Hz */
#define BMI088_GYRO_LPM1_NORMAL  (0x00)

/* 加速度计 SPI 读时序：命令字节后固定跟 1 个 dummy 字节，数据从第 3 字节起 */
#define BMI088_ACC_DUMMY_LEN     (1)

#define BMI088_GYRO_LSB_PER_DPS  (16.384f)   /* ±2000dps */
#define BMI088_ACCEL_LSB_PER_G   (5461.333f) /* ±6g */
#define BMI088_GRAVITY           (9.80665f)

/* 零偏标定采样次数（1kHz 下约 1 秒） ------------------------------------*/
#define BMI088_CALIB_SAMPLES     (1000)

/* 类型定义 --------------------------------------------------------------*/

typedef void (*BMI088_Callback)(void* user_bmi088);

typedef struct {
    SPI_DRIVES* spi_accel;                         /* 加速度计 SPI（片选 PA4） */
    SPI_DRIVES* spi_gyro;                          /* 陀螺仪 SPI（片选 PB0） */
    float gyro[3];                                 /* 角速度 rad/s，已减零偏 */
    float accel[3];                                /* 加速度 m/s^2 */
    float temperature;                             /* 温度 ℃ */
    float gyro_bias[3];                            /* 零偏 rad/s */
    uint32_t calib_count;                          /* 标定累计采样次数 */
    float calib_sum[3];                            /* 标定累加和 */
    uint8_t calibrated;                            /* 零偏标定完成标志 */
    uint8_t is_online;                             /* 通信正常标志 */
    uint8_t accel_id;                              /* 回读的加速度计 ID，调试看 */
    uint8_t gyro_id;                               /* 回读的陀螺仪 ID，调试看 */
    BMI088_Callback callbacks[BMI088_CALLBACK_NUM];
    uint8_t callback_num;
    SysTick_Task task;
} BMI088_DRIVES;

/* 函数声明 --------------------------------------------------------------*/
void    BMI088_Init(BMI088_DRIVES* user_bmi088, SPI_DRIVES* spi_accel, SPI_DRIVES* spi_gyro,
                    uint32_t period_ms);
void    BMI088_RegisterCallback(BMI088_DRIVES* user_bmi088, BMI088_Callback callback);
void    BMI088_Calibrate_Start(BMI088_DRIVES* user_bmi088);
uint8_t BMI088_Is_Calibrated(const BMI088_DRIVES* user_bmi088);

#endif /* HAL_SPI_MODULE_ENABLED */
#endif // USER_BMI088_H
