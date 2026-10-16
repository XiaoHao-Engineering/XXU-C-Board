#include "../../../Core/Inc/bsp.h"
#ifdef HAL_SPI_MODULE_ENABLED

/* 包含头文件 */
#include <math.h>
#include "../attitude.h"
#if ATTITUDE_EKF_ENABLE
#include "../user_quaternion_ekf.h"
#endif

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
    /* ★ 四元数初值 = 单位四元数。EKF 收敛前（或 EKF 关闭时）就发这个，
       绝不能发 (0,0,0,0) —— 那不是合法四元数，视觉解出来会是 NaN。 */
    attitude->qe_q[0] = 1.0f; attitude->qe_q[1] = 0.0f;
    attitude->qe_q[2] = 0.0f; attitude->qe_q[3] = 0.0f;

    /* ★ 四元数 EKF 与互补滤波【并行】运行，不替换：先对比再决定用哪个。
       参数 (Q1 四元数过程噪声, Q2 零偏过程噪声, R 加速度计量测噪声, lambda 渐消因子, lpf)
       ⚠️ R 是按加速度计单位 m/s^2 整定的；若你的 accel 是 g，需乘 9.8^2 或把 accel 乘 9.8 */
    #if ATTITUDE_EKF_ENABLE
    IMU_QuaternionEKF_Init(10.0f, 0.001f, 10000000.0f, 1.0f, 0.0f);
    attitude->qe_q[0] = 1.0f; attitude->qe_q[1] = 0.0f; attitude->qe_q[2] = 0.0f; attitude->qe_q[3] = 0.0f;
    attitude->qe_roll = 0.0f; attitude->qe_pitch = 0.0f; attitude->qe_yaw = 0.0f;
    #endif

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
        /* 四元数 EKF：输入陀螺(rad/s，与 imu->gyro 同单位) + 加速度计(m/s^2) + dt */
        #if ATTITUDE_EKF_ENABLE
        IMU_QuaternionEKF_Update(gx * ATTITUDE_DEG2RAD, gy * ATTITUDE_DEG2RAD, gz * ATTITUDE_DEG2RAD,
                                 ax, ay, az, dt);
        attitude->qe_roll = QEKF_INS.Roll;
        attitude->qe_pitch = QEKF_INS.Pitch;
        attitude->qe_yaw = QEKF_INS.Yaw;
        attitude->qe_q[0] = QEKF_INS.q[0]; attitude->qe_q[1] = QEKF_INS.q[1];
        attitude->qe_q[2] = QEKF_INS.q[2]; attitude->qe_q[3] = QEKF_INS.q[3];
        #endif

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
