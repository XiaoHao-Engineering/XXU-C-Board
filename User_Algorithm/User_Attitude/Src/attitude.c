#include "../../../Core/Inc/bsp.h"
#ifdef HAL_SPI_MODULE_ENABLED

/* 包含头文件 */
#include <math.h>
#include "../attitude.h"

/* 私有变量 */
static ATTITUDE_DRIVES* attitude_drives[ATTITUDE_NUM];
static uint8_t attitude_num = 0;

/* 私有函数声明 */
static void  Attitude_Update(void* arg);
static float Attitude_Wrap180(float deg);

/* 函数体 */

void Attitude_Init(ATTITUDE_DRIVES* attitude, BMI088_DRIVES* user_bmi088, const float dt) {
    attitude->user_bmi088 = user_bmi088;
    attitude->dt = dt;

    attitude->pitch_acc = 0.0f;
    attitude->pitch_gyro = 0.0f;
    attitude->yaw_gyro = 0.0f;
    attitude->pitch = 0.0f;
    attitude->yaw = 0.0f;
    attitude->yaw_rate = 0.0f;
    attitude->yaw_offset = 0.0f;
    attitude->update_cnt = 0;

    CF_Init(&attitude->cf_pitch, ATTITUDE_CF_TC, dt);

    BMI088_RegisterCallback(user_bmi088, Attitude_Update);

    attitude_drives[attitude_num] = attitude;
    attitude_num++;
}

void Attitude_Reset_Yaw(ATTITUDE_DRIVES* attitude) {
    attitude->yaw_offset = attitude->yaw_gyro;
    attitude->yaw = 0.0f;
}

float Attitude_Get_Pitch(const ATTITUDE_DRIVES* attitude) {
    return attitude->pitch;
}

float Attitude_Get_Yaw(const ATTITUDE_DRIVES* attitude) {
    return attitude->yaw;
}

float Attitude_Get_Yaw_Rate(const ATTITUDE_DRIVES* attitude) {
    return attitude->yaw_rate;
}

/* 私有函数 */

/* 收到一帧 IMU 数据后更新姿态：加速度修长漂，陀螺修短抖 */
static void Attitude_Update(void* arg) {
    for (uint8_t index = 0; index < attitude_num; index++) {
        ATTITUDE_DRIVES* attitude = attitude_drives[index];

        if (attitude->user_bmi088 != (BMI088_DRIVES*)arg) {
            continue;
        }

        const BMI088_DRIVES* imu = attitude->user_bmi088;
        const float dt = attitude->dt;

        const float ax = imu->accel[0] * ATTITUDE_SIGN_X;
        const float ay = imu->accel[1] * ATTITUDE_SIGN_Y;
        const float az = imu->accel[2] * ATTITUDE_SIGN_Z;

        const float gx = imu->gyro[0] * ATTITUDE_SIGN_X * ATTITUDE_RAD2DEG;
        const float gy = imu->gyro[1] * ATTITUDE_SIGN_Y * ATTITUDE_RAD2DEG;
        const float gz = imu->gyro[2] * ATTITUDE_SIGN_Z * ATTITUDE_RAD2DEG;

        /* 俯仰：加速度给绝对角，陀螺积分给动态，互补滤波融合 */
        attitude->pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * ATTITUDE_RAD2DEG;
        attitude->pitch_gyro += gy * dt;
        attitude->pitch = CF_Update(&attitude->cf_pitch, attitude->pitch_acc, attitude->pitch_gyro);

        /* 偏航：先把机体 z 轴转速按俯仰投影回世界垂直轴，再积分 */
        const float pitch_rad = attitude->pitch * ATTITUDE_DEG2RAD;
        const float yaw_rate = gz * cosf(pitch_rad) - gx * sinf(pitch_rad);
        attitude->yaw_rate = yaw_rate;      /* 暴露给云台做阻尼（原来算完就丢了）*/
        attitude->yaw_gyro += yaw_rate * dt;

        if (attitude->yaw_gyro >= 360.0f || attitude->yaw_gyro <= -360.0f) {
            attitude->yaw_gyro = Attitude_Wrap180(attitude->yaw_gyro);
        }
        attitude->yaw = Attitude_Wrap180(attitude->yaw_gyro - attitude->yaw_offset);

        attitude->update_cnt++;
    }
}

static float Attitude_Wrap180(float deg) {
    while (deg >= 180.0f) {
        deg -= 360.0f;
    }
    while (deg < -180.0f) {
        deg += 360.0f;
    }
    return deg;
}

#endif /* HAL_SPI_MODULE_ENABLED */
