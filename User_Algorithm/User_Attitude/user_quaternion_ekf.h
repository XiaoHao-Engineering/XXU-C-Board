/**
 * @file    QuaternionEKF.h
 * @author  Wang Hongxi
 * @version V1.2.0
 * @date    2022/3/8
 * @brief   attitude update with gyro bias estimate and chi-square test
 */

#ifndef USER_QUATERNION_EKF_H
#define USER_QUATERNION_EKF_H

/* Includes ------------------------------------------------------------------*/

#include "user_kalman_filter.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported macros -----------------------------------------------------------*/

/* boolean type definitions */
#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif

#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 四元数扩展卡尔曼滤波器结构体
 * @note  姿态欧拉角存储在结构体中，单位为度。
 *        输出的欧拉角遵循 ZYX (Yall, Roll, Pitch) 内旋顺序.
 *        注意：本库内部计算的欧拉角命名与实际旋转轴对应关系如下：
 *        - Yaw   : 绕 Z 轴的旋转
 *        - Roll  : 绕 Y 轴的旋转
 *        - Pitch : 绕 X 轴的旋转
 *        若需要标准 ZYX 内旋顺序（Yaw 绕 Z,Roll 绕 X, Pitch 绕 Y, ），则使用时需交换 Roll 和 Pitch 的值。
 */
typedef struct
{
    uint8_t Initialized;
    KalmanFilter_t IMU_QuaternionEKF;
    uint8_t ConvergeFlag;
    uint8_t StableFlag;
    uint64_t ErrorCount;
    uint64_t UpdateCount;

    float q[4];        // 四元数估计值
    float GyroBias[3]; // 陀螺仪零偏估计值

    float Gyro[3];
    float Accel[3];

    float OrientationCosine[3];

    float accLPFcoef;
    float gyro_norm;
    float accl_norm;
    float AdaptiveGainScale;

    float Roll;
    float Pitch;
    float Yaw;

    float YawTotalAngle;

    float Q1; // 四元数更新过程噪声
    float Q2; // 陀螺仪零偏过程噪声
    float R;  // 加速度计量测噪声

    float dt; // 姿态更新周期
    mat ChiSquare;
    float ChiSquare_Data[1];      // 卡方检验检测函数
    float ChiSquareTestThreshold; // 卡方检验阈值
    float lambda;                 // 渐消因子

    int16_t YawRoundCount;

    float YawAngleLast;
} QEKF_INS_t;

/* Exported variables --------------------------------------------------------*/

extern QEKF_INS_t QEKF_INS;
extern float chiSquare;
extern float ChiSquareTestThreshold;

/* Exported function declarations --------------------------------------------*/

void IMU_QuaternionEKF_Init(float process_noise1, float process_noise2, float measure_noise, float lambda, float lpf);
void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

#ifdef __cplusplus
}
#endif

#endif /* USER_QUATERNION_EKF_H */
