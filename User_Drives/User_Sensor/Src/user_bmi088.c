#include "../../../Core/Inc/bsp.h"
#ifdef HAL_SPI_MODULE_ENABLED

/* 包含头文件 */
#include "../user_bmi088.h"

/* 私有变量 */
static BMI088_DRIVES* bmi088_drives[BMI088_NUM];
static uint8_t bmi088_num = 0;

/* 私有函数声明 */
static void    BMI088_Handle(void* arg);
static uint8_t BMI088_Accel_Write_Reg(BMI088_DRIVES* imu, uint8_t reg, uint8_t value);
static uint8_t BMI088_Accel_Read_Regs(BMI088_DRIVES* imu, uint8_t reg, uint8_t* buf, uint8_t len);
static uint8_t BMI088_Gyro_Write_Reg(BMI088_DRIVES* imu, uint8_t reg, uint8_t value);
static uint8_t BMI088_Gyro_Read_Regs(BMI088_DRIVES* imu, uint8_t reg, uint8_t* buf, uint8_t len);

/* 函数体 */

/* 加速度计：写寄存器（2 字节，读写都不带 dummy） */
static uint8_t BMI088_Accel_Write_Reg(BMI088_DRIVES* imu, const uint8_t reg, const uint8_t value) {
    uint8_t tx[2] = {reg, value};
    uint8_t rx[2] = {0};
    return SPI_Transfer(imu->spi_accel, tx, rx, 2);
}

/* 加速度计：连续读，buf 收到 len 个数据字节（命令字节后跟 1 个 dummy 字节） */
static uint8_t BMI088_Accel_Read_Regs(BMI088_DRIVES* imu, const uint8_t reg, uint8_t* buf, const uint8_t len) {
    uint8_t tx[16] = {0};
    uint8_t rx[16] = {0};

    tx[0] = reg | BMI088_READ_FLAG;
    if (!SPI_Transfer(imu->spi_accel, tx, rx, len + 1 + BMI088_ACC_DUMMY_LEN)) {
        return 0;
    }
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = rx[i + 1 + BMI088_ACC_DUMMY_LEN];
    }
    return 1;
}

/* 陀螺仪：写寄存器 */
static uint8_t BMI088_Gyro_Write_Reg(BMI088_DRIVES* imu, const uint8_t reg, const uint8_t value) {
    uint8_t tx[2] = {reg, value};
    uint8_t rx[2] = {0};
    return SPI_Transfer(imu->spi_gyro, tx, rx, 2);
}

/* 陀螺仪：连续读，buf 收到 len 个数据字节（无 dummy 字节） */
static uint8_t BMI088_Gyro_Read_Regs(BMI088_DRIVES* imu, const uint8_t reg, uint8_t* buf, const uint8_t len) {
    uint8_t tx[16] = {0};
    uint8_t rx[16] = {0};

    tx[0] = reg | BMI088_READ_FLAG;
    if (!SPI_Transfer(imu->spi_gyro, tx, rx, len + 1)) {
        return 0;
    }
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = rx[i + 1];
    }
    return 1;
}

void BMI088_Init(BMI088_DRIVES* user_bmi088, SPI_DRIVES* spi_accel, SPI_DRIVES* spi_gyro,
                 const uint32_t period_ms) {
    user_bmi088->spi_accel = spi_accel;
    user_bmi088->spi_gyro = spi_gyro;
    user_bmi088->callback_num = 0;
    user_bmi088->calibrated = 0;
    user_bmi088->calib_count = 0;
    user_bmi088->is_online = 0;
    user_bmi088->accel_id = 0;
    user_bmi088->gyro_id = 0;
    for (uint8_t i = 0; i < 3; i++) {
        user_bmi088->gyro_bias[i] = 0.0f;
        user_bmi088->calib_sum[i] = 0.0f;
    }

    /* 上电后加速度计默认挂在 I2C 上，先空读一次芯片 ID，
       靠这次 CS 的上升沿把它的 SPI 接口激活，读完的结果直接丢掉 */
    uint8_t id = 0;
    BMI088_Accel_Read_Regs(user_bmi088, BMI088_ACC_CHIP_ID, &id, 1);
    HAL_Delay(1);

    /* ---- 加速度计 ---- */
    BMI088_Accel_Write_Reg(user_bmi088, BMI088_ACC_SOFTRESET, BMI088_SOFTRESET_CMD);
    HAL_Delay(50);                                   /* 手册要求复位后 >= 1ms */

    /* 软复位后要再读一次芯片 ID 重新激活 SPI，否则后面的配置写不进去 */
    BMI088_Accel_Read_Regs(user_bmi088, BMI088_ACC_CHIP_ID, &id, 1);

    BMI088_Accel_Write_Reg(user_bmi088, BMI088_ACC_PWR_CTRL, BMI088_ACC_PWR_CTRL_ON);
    HAL_Delay(50);                                   /* 手册要求使能后 >= 5ms */
    BMI088_Accel_Write_Reg(user_bmi088, BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_CONF_ACTIVE);

    /* ACC_CONF 的 bit7 是保留位（读回来是 1），照抄回去，只改带宽与 ODR */
    uint8_t acc_conf = 0;
    BMI088_Accel_Read_Regs(user_bmi088, BMI088_ACC_CONF, &acc_conf, 1);
    BMI088_Accel_Write_Reg(user_bmi088, BMI088_ACC_CONF,
                           (uint8_t)((acc_conf & 0x80)
                                     | BMI088_ACC_CONF_BWP_NORMAL
                                     | BMI088_ACC_CONF_ODR_800HZ));
    BMI088_Accel_Write_Reg(user_bmi088, BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G);
    HAL_Delay(1);

    /* ---- 陀螺仪 ---- */
    BMI088_Gyro_Write_Reg(user_bmi088, BMI088_GYRO_SOFTRESET, BMI088_SOFTRESET_CMD);
    HAL_Delay(30);                                   /* 手册要求复位后 >= 30ms */
    BMI088_Gyro_Write_Reg(user_bmi088, BMI088_GYRO_RANGE, BMI088_GYRO_RANGE_2000);
    BMI088_Gyro_Write_Reg(user_bmi088, BMI088_GYRO_BANDWIDTH, BMI088_GYRO_BW_1000HZ_116HZ);
    BMI088_Gyro_Write_Reg(user_bmi088, BMI088_GYRO_LPM1, BMI088_GYRO_LPM1_NORMAL);
    HAL_Delay(1);

    /* 读芯片 ID 确认通信：加速度计 0x1E，陀螺仪 0x0F */
    BMI088_Accel_Read_Regs(user_bmi088, BMI088_ACC_CHIP_ID, &user_bmi088->accel_id, 1);
    BMI088_Gyro_Read_Regs(user_bmi088, BMI088_GYRO_CHIP_ID, &user_bmi088->gyro_id, 1);
    if (user_bmi088->accel_id == BMI088_ACC_CHIP_ID_VALUE &&
        user_bmi088->gyro_id == BMI088_GYRO_CHIP_ID_VALUE) {
        user_bmi088->is_online = 1;
    }

    bmi088_drives[bmi088_num] = user_bmi088;
    bmi088_num++;

    SysTick_InitTask(&user_bmi088->task, (void*)user_bmi088, 50, period_ms,
                     Task_REPEAT, BMI088_Handle);
    SysTick_StartTask(&user_bmi088->task);
}

void BMI088_RegisterCallback(BMI088_DRIVES* user_bmi088, const BMI088_Callback callback) {
    user_bmi088->callbacks[user_bmi088->callback_num] = callback;
    user_bmi088->callback_num++;
}

/* 重新开始零偏标定，标定期间请保持静止 */
void BMI088_Calibrate_Start(BMI088_DRIVES* user_bmi088) {
    user_bmi088->calibrated = 0;
    user_bmi088->calib_count = 0;
    for (uint8_t i = 0; i < 3; i++) {
        user_bmi088->calib_sum[i] = 0.0f;
    }
}

uint8_t BMI088_Is_Calibrated(const BMI088_DRIVES* user_bmi088) {
    return user_bmi088->calibrated;
}

/* 私有函数 */

/* 周期读取：加速度 6 字节 + 温度 2 字节 + 角速度 6 字节 */
static void BMI088_Handle(void* arg) {
    BMI088_DRIVES* imu = (BMI088_DRIVES*)arg;

    uint8_t acc_buf[6] = {0};
    uint8_t gyro_buf[6] = {0};
    uint8_t temp_buf[2] = {0};

    if (!BMI088_Accel_Read_Regs(imu, BMI088_ACC_DATA, acc_buf, 6) ||
        !BMI088_Gyro_Read_Regs(imu, BMI088_GYRO_RATE, gyro_buf, 6)) {
        imu->is_online = 0;
        return;
    }
    imu->is_online = 1;

    /* 温度：11 位，0.125℃/LSB，23℃ 偏移 */
    if (BMI088_Accel_Read_Regs(imu, BMI088_ACC_TEMP_MSB, temp_buf, 2)) {
        const int16_t raw_temp = (int16_t)((temp_buf[0] << 3) | (temp_buf[1] >> 5));
        imu->temperature = (float)raw_temp * 0.125f + 23.0f;
    }

    const int16_t raw_accel[3] = {
        (int16_t)((acc_buf[1] << 8) | acc_buf[0]),    /* ax */
        (int16_t)((acc_buf[3] << 8) | acc_buf[2]),    /* ay */
        (int16_t)((acc_buf[5] << 8) | acc_buf[4]),    /* az */
    };
    const int16_t raw_gyro[3] = {
        (int16_t)((gyro_buf[1] << 8) | gyro_buf[0]),  /* gx */
        (int16_t)((gyro_buf[3] << 8) | gyro_buf[2]),  /* gy */
        (int16_t)((gyro_buf[5] << 8) | gyro_buf[4]),  /* gz */
    };

    const float to_ms2  = BMI088_GRAVITY / BMI088_ACCEL_LSB_PER_G;
    const float to_rads = 0.0174532925f / BMI088_GYRO_LSB_PER_DPS;

    imu->accel[0] = (float)raw_accel[0] * to_ms2;
    imu->accel[1] = (float)raw_accel[1] * to_ms2;
    imu->accel[2] = (float)raw_accel[2] * to_ms2;

    const float gyro[3] = {
        (float)raw_gyro[0] * to_rads,
        (float)raw_gyro[1] * to_rads,
        (float)raw_gyro[2] * to_rads,
    };

    if (!imu->calibrated) {
        /* 标定中：累加求平均 */
        for (uint8_t i = 0; i < 3; i++) {
            imu->calib_sum[i] += gyro[i];
        }
        imu->calib_count++;

        if (imu->calib_count >= BMI088_CALIB_SAMPLES) {
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
