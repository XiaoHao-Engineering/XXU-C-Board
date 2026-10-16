#include "../../../Core/Inc/bsp.h"

/* 包含头文件 */
#include <math.h>
#include "../user_ahrs.h"

/* 宏定义 */
#define AHRS_RAD2DEG    (57.29578f)

/* 函数体 */

void AHRS_Init(AHRS_DRIVES* ahrs, const float kp, const float ki, const float acc_lpf) {
    /* 初始姿态 = 单位四元数（机体与世界重合）*/
    ahrs->q[0] = 1.0f;
    ahrs->q[1] = 0.0f;
    ahrs->q[2] = 0.0f;
    ahrs->q[3] = 0.0f;

    ahrs->roll = 0.0f;
    ahrs->pitch = 0.0f;
    ahrs->yaw = 0.0f;

    ahrs->gyro_bias[0] = 0.0f;
    ahrs->gyro_bias[1] = 0.0f;
    ahrs->gyro_bias[2] = 0.0f;

    ahrs->kp = kp;
    ahrs->ki = ki;
    ahrs->acc_lpf = acc_lpf;
    ahrs->acc_f[0] = 0.0f;
    ahrs->acc_f[1] = 0.0f;
    ahrs->acc_f[2] = 0.0f;

    ahrs->is_init = 1;
}

void AHRS_Update(AHRS_DRIVES* ahrs, float gx, float gy, float gz,
                 float ax, float ay, float az, const float dt) {
    if (ahrs->is_init == 0 || dt <= 0.0f) {
        return;
    }

    /* ---- 0. 加速度计可选一阶低通：机体振动大时能明显改善 ---------------- */
    if (ahrs->acc_lpf > 0.0f && ahrs->acc_lpf < 1.0f) {
        ahrs->acc_f[0] += ahrs->acc_lpf * (ax - ahrs->acc_f[0]);
        ahrs->acc_f[1] += ahrs->acc_lpf * (ay - ahrs->acc_f[1]);
        ahrs->acc_f[2] += ahrs->acc_lpf * (az - ahrs->acc_f[2]);
        ax = ahrs->acc_f[0];
        ay = ahrs->acc_f[1];
        az = ahrs->acc_f[2];
    }

    /* ---- 1. 归一化加速度计：只取它的【方向】（大小是重力，不含姿态信息）---- */
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 1e-6f) {
        return;                     /* 数据异常（全零）时直接跳过这一拍，避免除零 */
    }
    const float recip_norm = 1.0f / norm;
    ax *= recip_norm;
    ay *= recip_norm;
    az *= recip_norm;

    /* ---- 2. 用当前四元数把世界系重力方向转到机体系，得到【估计】的重力方向 --
       标准 Mahony 形式：v = [2(qxqz - qwqy), 2(qwqx + qyqz), qw^2-qx^2-qy^2+qz^2] */
    const float qw = ahrs->q[0];
    const float qx = ahrs->q[1];
    const float qy = ahrs->q[2];
    const float qz = ahrs->q[3];

    const float vx = 2.0f * (qx * qz - qw * qy);
    const float vy = 2.0f * (qw * qx + qy * qz);
    const float vz = qw * qw - qx * qx - qy * qy + qz * qz;

    /* ---- 3. 实际重力方向 × 估计重力方向 = 误差（方向一致时叉积为零）-------- */
    const float ex = ay * vz - az * vy;
    const float ey = az * vx - ax * vz;
    const float ez = ax * vy - ay * vx;

    /* ---- 4. 比例-积分校正：PI 输出直接修正陀螺 ----------------------------
       比例项 kp 修姿态；积分项 ki 慢慢把【零偏】学出来（学出来之后稳态误差为 0）*/
    if (ahrs->ki > 0.0f) {
        ahrs->gyro_bias[0] += ahrs->ki * ex * dt;
        ahrs->gyro_bias[1] += ahrs->ki * ey * dt;
        ahrs->gyro_bias[2] += ahrs->ki * ez * dt;
    }

    gx += ahrs->kp * ex + ahrs->gyro_bias[0];
    gy += ahrs->kp * ey + ahrs->gyro_bias[1];
    gz += ahrs->kp * ez + ahrs->gyro_bias[2];

    /* ---- 5. 一阶龙格库塔积分四元数微分方程 q' = 0.5 * q ⊗ ω ------------- */
    const float half_dt = 0.5f * dt;
    ahrs->q[0] += half_dt * (-qx * gx - qy * gy - qz * gz);
    ahrs->q[1] += half_dt * ( qw * gx + qy * gz - qz * gy);
    ahrs->q[2] += half_dt * ( qw * gy - qx * gz + qz * gx);
    ahrs->q[3] += half_dt * ( qw * gz + qx * gy - qy * gx);

    /* ---- 6. 归一化：四元数必须保持单位长度，否则姿态会慢慢"胀掉" --------- */
    norm = sqrtf(ahrs->q[0] * ahrs->q[0] + ahrs->q[1] * ahrs->q[1]
               + ahrs->q[2] * ahrs->q[2] + ahrs->q[3] * ahrs->q[3]);
    if (norm < 1e-6f) {
        AHRS_Init(ahrs, ahrs->kp, ahrs->ki, ahrs->acc_lpf);   /* 数值崩了就复位 */
        return;
    }
    const float recip_q = 1.0f / norm;
    ahrs->q[0] *= recip_q;
    ahrs->q[1] *= recip_q;
    ahrs->q[2] *= recip_q;
    ahrs->q[3] *= recip_q;

    /* ---- 7. 输出欧拉角（ZYX 内旋）供绘图/对比 ---------------------------- */
    const float q0 = ahrs->q[0], q1 = ahrs->q[1], q2 = ahrs->q[2], q3 = ahrs->q[3];

    ahrs->roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                        1.0f - 2.0f * (q1 * q1 + q2 * q2)) * AHRS_RAD2DEG;

    /* asinf 的定义域必须夹住：浮点误差可能让它略微超过 ±1 -> 返回 NaN */
    float sin_pitch = 2.0f * (q0 * q2 - q3 * q1);
    if (sin_pitch > 1.0f) {
        sin_pitch = 1.0f;
    } else if (sin_pitch < -1.0f) {
        sin_pitch = -1.0f;
    }
    ahrs->pitch = asinf(sin_pitch) * AHRS_RAD2DEG;

    ahrs->yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                       1.0f - 2.0f * (q2 * q2 + q3 * q3)) * AHRS_RAD2DEG;
}
