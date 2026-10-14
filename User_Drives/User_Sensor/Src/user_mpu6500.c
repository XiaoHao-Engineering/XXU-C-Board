#include "../../../Core/Inc/bsp.h"
#ifdef HAL_SPI_MODULE_ENABLED

/* 包含头文件 */
#include "../user_mpu6500.h"

/* 私有变量 */
static MPU6500_DRIVES* mpu6500_drives[MPU6500_NUM];
static uint8_t mpu6500_num = 0;

/* 私有函数声明 */
static void    MPU6500_Handle(void* arg);
static uint8_t MPU6500_Write_Reg(MPU6500_DRIVES* imu, uint8_t reg, uint8_t value);
static uint8_t MPU6500_Read_Regs(MPU6500_DRIVES* imu, uint8_t reg, uint8_t* buf, uint8_t len);

/* 函数体 */

/* 单个寄存器写 */
static uint8_t MPU6500_Write_Reg(MPU6500_DRIVES* imu, const uint8_t reg, const uint8_t value) {
    uint8_t tx[2] = {reg, value};
    uint8_t rx[2] = {0};
    return SPI_Transfer(imu->user_spi, tx, rx, 2);
}

/* 连续读，buf 收到 len 个数据字节（不含命令字节） */
static uint8_t MPU6500_Read_Regs(MPU6500_DRIVES* imu, const uint8_t reg, uint8_t* buf, const uint8_t len) {
    uint8_t tx[16] = {0};
    uint8_t rx[16] = {0};

    tx[0] = reg | MPU6500_READ_FLAG;
    if (!SPI_Transfer(imu->user_spi, tx, rx, len + 1)) {
        return 0;
    }
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = rx[i + 1];
    }
    return 1;
}

void MPU6500_Init(MPU6500_DRIVES* user_mpu6500, SPI_DRIVES* user_spi, const uint32_t period_ms) {
    user_mpu6500->user_spi = user_spi;
    user_mpu6500->callback_num = 0;
    user_mpu6500->calibrated = 0;
    user_mpu6500->calib_count = 0;
    user_mpu6500->is_online = 0;
    for (uint8_t i = 0; i < 3; i++) {
        user_mpu6500->gyro_bias[i] = 0.0f;
        user_mpu6500->calib_sum[i] = 0.0f;
    }

    /* 复位，等内部稳定 */
    MPU6500_Write_Reg(user_mpu6500, MPU6500_PWR_MGMT_1, 0x80);
    HAL_Delay(100);

    /* 唤醒，时钟选 X 轴陀螺 PLL */
    MPU6500_Write_Reg(user_mpu6500, MPU6500_PWR_MGMT_1, 0x01);
    /* 关闭主 I2C，走 SPI */
    MPU6500_Write_Reg(user_mpu6500, MPU6500_USER_CTRL, 0x10);
    /* 复位信号通路 */
    MPU6500_Write_Reg(user_mpu6500, MPU6500_SIGNAL_PATH_RST, 0x07);
    HAL_Delay(10);

    MPU6500_Write_Reg(user_mpu6500, MPU6500_SMPLRT_DIV, 0x00);       /* 1kHz */
    MPU6500_Write_Reg(user_mpu6500, MPU6500_CONFIG, 0x03);          /* DLPF 41Hz */
    MPU6500_Write_Reg(user_mpu6500, MPU6500_GYRO_CONFIG, MPU6500_GYRO_FS_2000);
    MPU6500_Write_Reg(user_mpu6500, MPU6500_ACCEL_CONFIG, MPU6500_ACCEL_FS_8G);

    /* 读 WHO_AM_I 确认通信 */
    uint8_t id = 0;
    if (MPU6500_Read_Regs(user_mpu6500, MPU6500_WHO_AM_I, &id, 1) && id == MPU6500_CHIP_ID) {
        user_mpu6500->is_online = 1;
    }

    mpu6500_drives[mpu6500_num] = user_mpu6500;
    mpu6500_num++;

    SysTick_InitTask(&user_mpu6500->task, (void*)user_mpu6500, 50, period_ms,
                     Task_REPEAT, MPU6500_Handle);
    SysTick_StartTask(&user_mpu6500->task);
}

void MPU6500_RegisterCallback(MPU6500_DRIVES* user_mpu6500, const MPU6500_Callback callback) {
    user_mpu6500->callbacks[user_mpu6500->callback_num] = callback;
    user_mpu6500->callback_num++;
}

/* 重新开始零偏标定，标定期间请保持静止 */
void MPU6500_Calibrate_Start(MPU6500_DRIVES* user_mpu6500) {
    user_mpu6500->calibrated = 0;
    user_mpu6500->calib_count = 0;
    for (uint8_t i = 0; i < 3; i++) {
        user_mpu6500->calib_sum[i] = 0.0f;
    }
}

uint8_t MPU6500_Is_Calibrated(const MPU6500_DRIVES* user_mpu6500) {
    return user_mpu6500->calibrated;
}

/* 私有函数 */

/* 周期读取 14 字节：加速度 6 + 温度 2 + 角速度 6 */
static void MPU6500_Handle(void* arg) {
    MPU6500_DRIVES* imu = (MPU6500_DRIVES*)arg;

    uint8_t buf[14] = {0};
    if (!MPU6500_Read_Regs(imu, MPU6500_ACCEL_XOUT_H, buf, 14)) {
        imu->is_online = 0;
        return;
    }
    imu->is_online = 1;

    const int16_t raw[7] = {
        (int16_t)((buf[0]  << 8) | buf[1]),    /* ax */
        (int16_t)((buf[2]  << 8) | buf[3]),    /* ay */
        (int16_t)((buf[4]  << 8) | buf[5]),    /* az */
        (int16_t)((buf[6]  << 8) | buf[7]),    /* temp */
        (int16_t)((buf[8]  << 8) | buf[9]),    /* gx */
        (int16_t)((buf[10] << 8) | buf[11]),   /* gy */
        (int16_t)((buf[12] << 8) | buf[13]),   /* gz */
    };

    const float to_ms2  = MPU6500_GRAVITY / MPU6500_ACCEL_LSB_PER_G;
    const float to_rads = 0.0174532925f / MPU6500_GYRO_LSB_PER_DPS;

    imu->accel[0] = (float)raw[0] * to_ms2;
    imu->accel[1] = (float)raw[1] * to_ms2;
    imu->accel[2] = (float)raw[2] * to_ms2;
    imu->temperature = (float)raw[3] / 333.87f + 21.0f;

    const float gyro[3] = {
        (float)raw[4] * to_rads,
        (float)raw[5] * to_rads,
        (float)raw[6] * to_rads,
    };

    if (!imu->calibrated) {
        /* 标定中：累加求平均 */
        for (uint8_t i = 0; i < 3; i++) {
            imu->calib_sum[i] += gyro[i];
        }
        imu->calib_count++;

        if (imu->calib_count >= MPU6500_CALIB_SAMPLES) {
            for (uint8_t i = 0; i < 3; i++) {
                imu->gyro_bias[i] = imu->calib_sum[i] / (float)imu->calib_count;
                imu->gyro[i] = 0.0f;
            }
            imu->calibrated = 1;
        }
    } else {
        for (uint8_t i = 0; i < 3; i++) {
            imu->gyro[i] = gyro[i] - imu->gyro_bias[i];
        }
    }

    for (uint8_t i = 0; i < imu->callback_num; i++) {
        imu->callbacks[i](imu);
    }
}

#endif /* HAL_SPI_MODULE_ENABLED */
