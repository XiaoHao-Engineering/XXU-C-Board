/**
 ******************************************************************************
 * @file    kalman filter.c
 * @author  Wang Hongxi
 * @version V1.2.2
 * @date    2022/1/8
 * @brief   C implementation of kalman filter
 ******************************************************************************
 * @attention
 * 该卡尔曼滤波器可以在传感器采样频率不同的情况下，动态调整矩阵H R和K的维数与数值。
 * This implementation of kalman filter can dynamically adjust dimension and
 * value of matrix H R and K according to the measurement validity under any
 * circumstance that the sampling rate of component sensors are different.
 *
 * 因此矩阵H和R的初始化会与矩阵P A和Q有所不同。另外的，在初始化量测向量z时需要额外写
 * 入传感器量测所对应的状态与这个量测的方式，详情请见例程
 * Therefore, the initialization of matrix P, F, and Q is sometimes different
 * from that of matrices H R. when initialization. Additionally, the corresponding
 * state and the method of the measurement should be provided when initializing
 * measurement vector z. For more details, please see the example.
 *
 * 若不需要动态调整量测向量z，可简单将结构体中的Use_Auto_Adjustment初始化为0，并像初
 * 始化矩阵P那样用常规方式初始化z H R即可。
 * If automatic adjustment is not required, assign zero to the UseAutoAdjustment
 * and initialize z H R in the normal way as matrix P.
 *
 * 要求量测向量z与控制向量u在传感器回调函数中更新。整数0意味着量测无效，即自上次卡尔曼
 * 滤波更新后无传感器数据更新。因此量测向量z与控制向量u会在卡尔曼滤波更新过程中被清零
 * MeasuredVector and ControlVector are required to be updated in the sensor
 * callback function. Integer 0 in measurement vector z indicates the invalidity
 * of current measurement, so MeasuredVector and ControlVector will be reset
 * (to 0) during each update.
 *
 * 此外，矩阵P过度收敛后滤波器将难以再适应状态的缓慢变化，从而产生滤波估计偏差。该算法
 * 通过限制矩阵P最小值的方法，可有效抑制滤波器的过度收敛，详情请见例程。
 * Additionally, the excessive convergence of matrix P will make filter incapable
 * of adopting the slowly changing state. This implementation can effectively
 * suppress filter excessive convergence through boundary limiting for matrix P.
 * For more details, please see the example.
 *
 * @example:
 * x =
 *   |   height   |
 *   |  velocity  |
 *   |acceleration|
 *
 * KalmanFilter_t Height_KF;
 *
 * void INS_Task_Init(void)
 * {
 *     static float P_Init[9] =
 *     {
 *         10, 0, 0,
 *         0, 30, 0,
 *         0, 0, 10,
 *     };
 *     static float F_Init[9] =
 *     {
 *         1, dt, 0.5*dt*dt,
 *         0, 1, dt,
 *         0, 0, 1,
 *     };
 *     static float Q_Init[9] =
 *     {
 *         0.25*dt*dt*dt*dt, 0.5*dt*dt*dt, 0.5*dt*dt,
 *         0.5*dt*dt*dt,        dt*dt,         dt,
 *         0.5*dt*dt,              dt,         1,
 *     };
 *
 *     // 设置最小方差
 *     static float state_min_variance[3] = {0.03, 0.005, 0.1};
 *
 *     // 开启自动调整
 *     Height_KF.UseAutoAdjustment = 1;
 *
 *     // 气压测得高度 GPS测得高度 加速度计测得z轴运动加速度
 *     static uint8_t measurement_reference[3] = {1, 1, 3}
 *
 *     static float measurement_degree[3] = {1, 1, 1}
 *     // 根据measurement_reference与measurement_degree生成H矩阵如下（在当前周期全部测量数据有效情况下）
 *       |1   0   0|
 *       |1   0   0|
 *       |0   0   1|
 *
 *     static float mat_R_diagonal_elements = {30, 25, 35}
 *     //根据mat_R_diagonal_elements生成R矩阵如下（在当前周期全部测量数据有效情况下）
 *       |30   0   0|
 *       | 0  25   0|
 *       | 0   0  35|
 *
 *     Kalman_Filter_Init(&Height_KF, 3, 0, 3);
 *
 *     // 设置矩阵值
 *     memcpy(Height_KF.P_data, P_Init, sizeof(P_Init));
 *     memcpy(Height_KF.F_data, F_Init, sizeof(F_Init));
 *     memcpy(Height_KF.Q_data, Q_Init, sizeof(Q_Init));
 *     memcpy(Height_KF.MeasurementMap, measurement_reference, sizeof(measurement_reference));
 *     memcpy(Height_KF.MeasurementDegree, measurement_degree, sizeof(measurement_degree));
 *     memcpy(Height_KF.MatR_DiagonalElements, mat_R_diagonal_elements, sizeof(mat_R_diagonal_elements));
 *     memcpy(Height_KF.StateMinVariance, state_min_variance, sizeof(state_min_variance));
 * }
 *
 * void INS_Task(void const *pvParameters)
 * {
 *     // 循环更新
 *     Kalman_Filter_Update(&Height_KF);
 *     vTaskDelay(ts);
 * }
 *
 * // 测量数据更新应按照以下形式 即向MeasuredVector赋值
 * void Barometer_Read_Over(void)
 * {
 *     ......
 *     INS_KF.MeasuredVector[0] = baro_height;
 * }
 * void GPS_Read_Over(void)
 * {
 *     ......
 *     INS_KF.MeasuredVector[1] = GPS_height;
 * }
 * void Acc_Data_Process(void)
 * {
 *     ......
 *     INS_KF.MeasuredVector[2] = acc.z;
 * }
 ******************************************************************************
 */

#include "../user_kalman_filter.h"

//float 和 double的字节数
uint16_t sizeof_float, sizeof_double;

static void H_K_R_Adjustment(KalmanFilter_t *kf);

/**
 * @brief 初始化矩阵维度信息并为矩阵分配空间
 *
 * @param kf kf类型定义
 * @param xhatSize 状态变量维度
 * @param uSize 控制变量维度
 * @param zSize 观测量维度
 */
void Kalman_Filter_Init(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize)
{
    sizeof_float = sizeof(float);
    sizeof_double = sizeof(double);

    kf->xhatSize = xhatSize;
    kf->uSize = uSize;
    kf->zSize = zSize;

    kf->MeasurementValidNum = 0;

    // measurement flags
    kf->MeasurementMap = (uint8_t *)user_malloc(sizeof(uint8_t) * zSize);
    memset(kf->MeasurementMap, 0, sizeof(uint8_t) * zSize);
    kf->MeasurementDegree = (float *)user_malloc(sizeof_float * zSize);
    memset(kf->MeasurementDegree, 0, sizeof_float * zSize);
    kf->MatR_DiagonalElements = (float *)user_malloc(sizeof_float * zSize);
    memset(kf->MatR_DiagonalElements, 0, sizeof_float * zSize);
    kf->StateMinVariance = (float *)user_malloc(sizeof_float * xhatSize);
    memset(kf->StateMinVariance, 0, sizeof_float * xhatSize);
    kf->temp = (uint8_t *)user_malloc(sizeof(uint8_t) * zSize);
    memset(kf->temp, 0, sizeof(uint8_t) * zSize);

    // filter data
    kf->FilteredValue = (float *)user_malloc(sizeof_float * xhatSize);
    memset(kf->FilteredValue, 0, sizeof_float * xhatSize);
    kf->MeasuredVector = (float *)user_malloc(sizeof_float * zSize);
    memset(kf->MeasuredVector, 0, sizeof_float * zSize);
    kf->ControlVector = (float *)user_malloc(sizeof_float * uSize);
    memset(kf->ControlVector, 0, sizeof_float * uSize);

    // xhat x(k|k)
    kf->xhat_data = (float *)user_malloc(sizeof_float * xhatSize);
    memset(kf->xhat_data, 0, sizeof_float * xhatSize);
    Matrix_Init(&kf->xhat, kf->xhatSize, 1, (float *)kf->xhat_data);

    // xhatminus x(k|k-1)
    kf->xhatminus_data = (float *)user_malloc(sizeof_float * xhatSize);
    memset(kf->xhatminus_data, 0, sizeof_float * xhatSize);
    Matrix_Init(&kf->xhatminus, kf->xhatSize, 1, (float *)kf->xhatminus_data);

    if (uSize != 0)
    {
        // control vector u
        kf->u_data = (float *)user_malloc(sizeof_float * uSize);
        memset(kf->u_data, 0, sizeof_float * uSize);
        Matrix_Init(&kf->u, kf->uSize, 1, (float *)kf->u_data);
    }

    // measurement vector z
    kf->z_data = (float *)user_malloc(sizeof_float * zSize);
    memset(kf->z_data, 0, sizeof_float * zSize);
    Matrix_Init(&kf->z, kf->zSize, 1, (float *)kf->z_data);

    // covariance matrix P(k|k)
    kf->P_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->P_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->P, kf->xhatSize, kf->xhatSize, (float *)kf->P_data);

    // create covariance matrix P(k|k-1)
    kf->Pminus_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->Pminus_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->Pminus, kf->xhatSize, kf->xhatSize, (float *)kf->Pminus_data);

    // state transition matrix F FT
    kf->F_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    kf->FT_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->F_data, 0, sizeof_float * xhatSize * xhatSize);
    memset(kf->FT_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->F, kf->xhatSize, kf->xhatSize, (float *)kf->F_data);
    Matrix_Init(&kf->FT, kf->xhatSize, kf->xhatSize, (float *)kf->FT_data);

    if (uSize != 0)
    {
        // control matrix B
        kf->B_data = (float *)user_malloc(sizeof_float * xhatSize * uSize);
        memset(kf->B_data, 0, sizeof_float * xhatSize * uSize);
        Matrix_Init(&kf->B, kf->xhatSize, kf->uSize, (float *)kf->B_data);
    }

    // measurement matrix H
    kf->H_data = (float *)user_malloc(sizeof_float * zSize * xhatSize);
    kf->HT_data = (float *)user_malloc(sizeof_float * xhatSize * zSize);
    memset(kf->H_data, 0, sizeof_float * zSize * xhatSize);
    memset(kf->HT_data, 0, sizeof_float * xhatSize * zSize);
    Matrix_Init(&kf->H, kf->zSize, kf->xhatSize, (float *)kf->H_data);
    Matrix_Init(&kf->HT, kf->xhatSize, kf->zSize, (float *)kf->HT_data);

    // process noise covariance matrix Q
    kf->Q_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->Q_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->Q, kf->xhatSize, kf->xhatSize, (float *)kf->Q_data);

    // measurement noise covariance matrix R
    kf->R_data = (float *)user_malloc(sizeof_float * zSize * zSize);
    memset(kf->R_data, 0, sizeof_float * zSize * zSize);
    Matrix_Init(&kf->R, kf->zSize, kf->zSize, (float *)kf->R_data);

    // kalman gain K
    kf->K_data = (float *)user_malloc(sizeof_float * xhatSize * zSize);
    memset(kf->K_data, 0, sizeof_float * xhatSize * zSize);
    Matrix_Init(&kf->K, kf->xhatSize, kf->zSize, (float *)kf->K_data);

    kf->S_data = (float *)user_malloc(sizeof_float * kf->xhatSize * kf->xhatSize);
    kf->temp_matrix_data = (float *)user_malloc(sizeof_float * kf->xhatSize * kf->xhatSize);
    kf->temp_matrix_data1 = (float *)user_malloc(sizeof_float * kf->xhatSize * kf->xhatSize);
    kf->temp_vector_data = (float *)user_malloc(sizeof_float * kf->xhatSize);
    kf->temp_vector_data1 = (float *)user_malloc(sizeof_float * kf->xhatSize);
    Matrix_Init(&kf->S, kf->xhatSize, kf->xhatSize, (float *)kf->S_data);
    Matrix_Init(&kf->temp_matrix, kf->xhatSize, kf->xhatSize, (float *)kf->temp_matrix_data);
    Matrix_Init(&kf->temp_matrix1, kf->xhatSize, kf->xhatSize, (float *)kf->temp_matrix_data1);
    Matrix_Init(&kf->temp_vector, kf->xhatSize, 1, (float *)kf->temp_vector_data);
    Matrix_Init(&kf->temp_vector1, kf->xhatSize, 1, (float *)kf->temp_vector_data1);

    kf->SkipEq1 = 0;
    kf->SkipEq2 = 0;
    kf->SkipEq3 = 0;
    kf->SkipEq4 = 0;
    kf->SkipEq5 = 0;
}

/**
 * @brief QuaternionEKF 的静态内存初始化（状态维数6，控制维数0，测量维数3）
 * @param kf 卡尔曼滤波器结构体指针
 * @note 此函数使用内部静态数组，不依赖堆内存，适用于禁用动态内存分配的项目。
 *       由于使用静态缓冲区，该函数只能初始化一个实例；如需多个实例，需修改代码。
 */
void Kalman_Filter_Init_Static_6_0_3(KalmanFilter_t *kf)
{
    // IMPORTANT: keep global sizeof_* consistent with heap init
    sizeof_float = sizeof(float);
    sizeof_double = sizeof(double);

    // Fixed sizes
    const uint8_t xhatSize = 6;
    const uint8_t uSize = 0;
    const uint8_t zSize = 3;

    // Static buffers (single instance)
    static float FilteredValue_s[6];
    static float MeasuredVector_s[3];
    static float ControlVector_s[1];

    static uint8_t MeasurementMap_s[3];
    static float MeasurementDegree_s[3];
    static float MatR_DiagonalElements_s[3];
    static float StateMinVariance_s[6];
    static uint8_t temp_s[3];

    static float xhat_data_s[6];
    static float xhatminus_data_s[6];
    static float u_data_s[1];
    static float z_data_s[3];

    static float P_data_s[36];
    static float Pminus_data_s[36];
    static float F_data_s[36];
    static float FT_data_s[36];
    static float B_data_s[6];
    static float H_data_s[18];
    static float HT_data_s[18];
    static float Q_data_s[36];
    static float R_data_s[9];
    static float K_data_s[18];

    static float S_data_s[36];
    static float temp_matrix_data_s[36];
    static float temp_matrix_data1_s[36];
    static float temp_vector_data_s[6];
    static float temp_vector_data1_s[6];

    // Clear buffers
    memset(FilteredValue_s, 0, sizeof(FilteredValue_s));
    memset(MeasuredVector_s, 0, sizeof(MeasuredVector_s));
    memset(ControlVector_s, 0, sizeof(ControlVector_s));
    memset(MeasurementMap_s, 0, sizeof(MeasurementMap_s));
    memset(MeasurementDegree_s, 0, sizeof(MeasurementDegree_s));
    memset(MatR_DiagonalElements_s, 0, sizeof(MatR_DiagonalElements_s));
    memset(StateMinVariance_s, 0, sizeof(StateMinVariance_s));
    memset(temp_s, 0, sizeof(temp_s));

    memset(xhat_data_s, 0, sizeof(xhat_data_s));
    memset(xhatminus_data_s, 0, sizeof(xhatminus_data_s));
    memset(u_data_s, 0, sizeof(u_data_s));
    memset(z_data_s, 0, sizeof(z_data_s));
    memset(P_data_s, 0, sizeof(P_data_s));
    memset(Pminus_data_s, 0, sizeof(Pminus_data_s));
    memset(F_data_s, 0, sizeof(F_data_s));
    memset(FT_data_s, 0, sizeof(FT_data_s));
    memset(B_data_s, 0, sizeof(B_data_s));
    memset(H_data_s, 0, sizeof(H_data_s));
    memset(HT_data_s, 0, sizeof(HT_data_s));
    memset(Q_data_s, 0, sizeof(Q_data_s));
    memset(R_data_s, 0, sizeof(R_data_s));
    memset(K_data_s, 0, sizeof(K_data_s));
    memset(S_data_s, 0, sizeof(S_data_s));
    memset(temp_matrix_data_s, 0, sizeof(temp_matrix_data_s));
    memset(temp_matrix_data1_s, 0, sizeof(temp_matrix_data1_s));
    memset(temp_vector_data_s, 0, sizeof(temp_vector_data_s));
    memset(temp_vector_data1_s, 0, sizeof(temp_vector_data1_s));

    // Bind pointers
    kf->FilteredValue = FilteredValue_s;
    kf->MeasuredVector = MeasuredVector_s;
    kf->ControlVector = ControlVector_s;

    kf->xhatSize = xhatSize;
    kf->uSize = uSize;
    kf->zSize = zSize;

    kf->UseAutoAdjustment = 0;
    kf->MeasurementValidNum = 0;

    kf->MeasurementMap = MeasurementMap_s;
    kf->MeasurementDegree = MeasurementDegree_s;
    kf->MatR_DiagonalElements = MatR_DiagonalElements_s;
    kf->StateMinVariance = StateMinVariance_s;
    kf->temp = temp_s;

    kf->xhat_data = xhat_data_s;
    kf->xhatminus_data = xhatminus_data_s;
    kf->u_data = u_data_s;
    kf->z_data = z_data_s;
    kf->P_data = P_data_s;
    kf->Pminus_data = Pminus_data_s;
    kf->F_data = F_data_s;
    kf->FT_data = FT_data_s;
    kf->B_data = B_data_s;
    kf->H_data = H_data_s;
    kf->HT_data = HT_data_s;
    kf->Q_data = Q_data_s;
    kf->R_data = R_data_s;
    kf->K_data = K_data_s;
    kf->S_data = S_data_s;
    kf->temp_matrix_data = temp_matrix_data_s;
    kf->temp_matrix_data1 = temp_matrix_data1_s;
    kf->temp_vector_data = temp_vector_data_s;
    kf->temp_vector_data1 = temp_vector_data1_s;

    // Init matrices
    Matrix_Init(&kf->xhat, xhatSize, 1, (float *)kf->xhat_data);
    Matrix_Init(&kf->xhatminus, xhatSize, 1, (float *)kf->xhatminus_data);
    Matrix_Init(&kf->u, (uSize > 0) ? uSize : 1, 1, (float *)kf->u_data);
    Matrix_Init(&kf->z, zSize, 1, (float *)kf->z_data);

    Matrix_Init(&kf->P, xhatSize, xhatSize, (float *)kf->P_data);
    Matrix_Init(&kf->Pminus, xhatSize, xhatSize, (float *)kf->Pminus_data);
    Matrix_Init(&kf->F, xhatSize, xhatSize, (float *)kf->F_data);
    Matrix_Init(&kf->FT, xhatSize, xhatSize, (float *)kf->FT_data);
    Matrix_Init(&kf->B, xhatSize, (uSize > 0) ? uSize : 1, (float *)kf->B_data);
    Matrix_Init(&kf->H, zSize, xhatSize, (float *)kf->H_data);
    Matrix_Init(&kf->HT, xhatSize, zSize, (float *)kf->HT_data);
    Matrix_Init(&kf->Q, xhatSize, xhatSize, (float *)kf->Q_data);
    Matrix_Init(&kf->R, zSize, zSize, (float *)kf->R_data);
    Matrix_Init(&kf->K, xhatSize, zSize, (float *)kf->K_data);

    Matrix_Init(&kf->S, xhatSize, xhatSize, (float *)kf->S_data);
    Matrix_Init(&kf->temp_matrix, xhatSize, xhatSize, (float *)kf->temp_matrix_data);
    Matrix_Init(&kf->temp_matrix1, xhatSize, xhatSize, (float *)kf->temp_matrix_data1);
    Matrix_Init(&kf->temp_vector, xhatSize, 1, (float *)kf->temp_vector_data);
    Matrix_Init(&kf->temp_vector1, xhatSize, 1, (float *)kf->temp_vector_data1);

    kf->SkipEq1 = 0;
    kf->SkipEq2 = 0;
    kf->SkipEq3 = 0;
    kf->SkipEq4 = 0;
    kf->SkipEq5 = 0;
}

/**
 * @brief 卡尔曼滤波测量值预处理函数
 * @param kf 卡尔曼滤波器结构体指针
 * @note 根据是否启用自动调整，处理测量向量和控制向量：
 *       - 若 kf->UseAutoAdjustment 非零，调用 H_K_R_Adjustment 动态调整矩阵 H、R、K 的维数，
 *         同时根据测量有效性重构 z、H、R 矩阵。
 *       - 否则直接复制 kf->MeasuredVector 到内部 z_data，并将 MeasuredVector 清零。
 *       最后复制 kf->ControlVector 到内部 u_data。
 *       该函数在每次滤波更新前被调用，用于准备最新测量和控制输入。
 */
void Kalman_Filter_Measure(KalmanFilter_t *kf)
{
    // 矩阵H K R根据量测情况自动调整
    // matrix H K R auto adjustment
    if (kf->UseAutoAdjustment != 0)
        H_K_R_Adjustment(kf);
    else
    {
        memcpy(kf->z_data, kf->MeasuredVector, sizeof_float * kf->zSize);
        memset(kf->MeasuredVector, 0, sizeof_float * kf->zSize);
    }

    memcpy(kf->u_data, kf->ControlVector, sizeof_float * kf->uSize);
}

/**
 * @brief 状态先验估计更新（卡尔曼滤波第一式）
 * @param kf 卡尔曼滤波器结构体指针
 * @note 根据 SkipEq1 标志决定是否执行：
 *       - 若不跳过，计算 xhatminus = F * xhat + B * u
 *         - 当 uSize > 0 时，分别计算 F*xhat 和 B*u，然后相加
 *         - 否则直接计算 xhatminus = F * xhat
 *       运算结果存储在 kf->xhatminus 中，并更新 MatStatus 反映矩阵运算状态。
 */
void Kalman_Filter_xhatMinusUpdate(KalmanFilter_t *kf)
{
    if (!kf->SkipEq1)
    {
        if (kf->uSize > 0)
        {
            kf->temp_vector.numRows = kf->xhatSize;
            kf->temp_vector.numCols = 1;
            kf->MatStatus = Matrix_Multiply(&kf->F, &kf->xhat, &kf->temp_vector);
            kf->temp_vector1.numRows = kf->xhatSize;
            kf->temp_vector1.numCols = 1;
            kf->MatStatus = Matrix_Multiply(&kf->B, &kf->u, &kf->temp_vector1);
            kf->MatStatus = Matrix_Add(&kf->temp_vector, &kf->temp_vector1, &kf->xhatminus);
        }
        else
        {
            kf->MatStatus = Matrix_Multiply(&kf->F, &kf->xhat, &kf->xhatminus);
        }
    }
}

/**
 * @brief 先验协方差矩阵更新（卡尔曼滤波第二式）
 * @param kf 卡尔曼滤波器结构体指针
 * @note 根据 SkipEq2 标志决定是否执行：
 *       若不跳过，计算 Pminus = F * P * F^T + Q
 *       其中 F 为状态转移矩阵，P 为后验协方差，Q 为过程噪声协方差。
 *       计算过程：
 *         1. 求 F 的转置 FT
 *         2. 计算 F * P，结果暂存于 Pminus
 *         3. 将上一步结果乘以 FT，得到 F * P * F^T，存入 temp_matrix
 *         4. 将 temp_matrix 加上 Q，结果存入 Pminus 完成更新
 *       运算结果存储在 kf->Pminus 中，并更新 MatStatus。
 */
void Kalman_Filter_PminusUpdate(KalmanFilter_t *kf)
{
    if (!kf->SkipEq2)
    {
        kf->MatStatus = Matrix_Transpose(&kf->F, &kf->FT);
        kf->MatStatus = Matrix_Multiply(&kf->F, &kf->P, &kf->Pminus);
        kf->temp_matrix.numRows = kf->Pminus.numRows;
        kf->temp_matrix.numCols = kf->FT.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->FT, &kf->temp_matrix); // temp_matrix = F P(k-1) FT
        kf->MatStatus = Matrix_Add(&kf->temp_matrix, &kf->Q, &kf->Pminus);
    }
}

/**
 * @brief 卡尔曼增益计算（卡尔曼滤波第三式）
 * @param kf 卡尔曼滤波器结构体指针
 * @note 根据 SkipEq3 标志决定是否执行：
 *       若不跳过，计算卡尔曼增益 K = P⁻·Hᵀ·(H·P⁻·Hᵀ + R)⁻¹
 *       计算步骤：
 *         1. 求测量矩阵 H 的转置 HT
 *         2. 计算 temp_matrix = H · P⁻
 *         3. 计算 temp_matrix1 = temp_matrix · HT = H · P⁻ · Hᵀ
 *         4. 计算 S = temp_matrix1 + R
 *         5. 对 S 求逆，结果存入 temp_matrix1 (即 inv(H·P⁻·Hᵀ + R))
 *         6. 计算 temp_matrix = P⁻ · HT
 *         7. 计算 K = temp_matrix · temp_matrix1 = P⁻·Hᵀ·inv(H·P⁻·Hᵀ + R)
 *       最终结果存储在 kf->K 中，并更新 MatStatus。
 */
void Kalman_Filter_SetK(KalmanFilter_t *kf)
{
    if (!kf->SkipEq3)
    {
        kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT); // z|x => x|z
        kf->temp_matrix.numRows = kf->H.numRows;
        kf->temp_matrix.numCols = kf->Pminus.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix); // temp_matrix = H·P'(k)
        kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
        kf->temp_matrix1.numCols = kf->HT.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1); // temp_matrix1 = H·P'(k)·HT
        kf->S.numRows = kf->R.numRows;
        kf->S.numCols = kf->R.numCols;
        kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S); // S = H P'(k) HT + R
        kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1);     // temp_matrix1 = inv(H·P'(k)·HT + R)
        kf->temp_matrix.numRows = kf->Pminus.numRows;
        kf->temp_matrix.numCols = kf->HT.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix); // temp_matrix = P'(k)·HT
        kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);
    }
}

/**
 * @brief 后验状态估计更新（卡尔曼滤波第四式）
 * @param kf 卡尔曼滤波器结构体指针
 * @note 根据 SkipEq4 标志决定是否执行：
 *       若不跳过，计算 xhat = xhat⁻ + K·(z - H·xhat⁻)
 *       计算步骤：
 *         1. 计算预测测量值 temp_vector = H · xhat⁻
 *         2. 计算测量残差 temp_vector1 = z - temp_vector
 *         3. 计算修正量 temp_vector = K · temp_vector1
 *         4. 将修正量加到先验状态得到后验状态 xhat = xhat⁻ + temp_vector
 *       最终结果存储在 kf->xhat 中，并更新 MatStatus。
 */
void Kalman_Filter_xhatUpdate(KalmanFilter_t *kf)
{
    if (!kf->SkipEq4)
    {
        kf->temp_vector.numRows = kf->H.numRows;
        kf->temp_vector.numCols = 1;
        kf->MatStatus = Matrix_Multiply(&kf->H, &kf->xhatminus, &kf->temp_vector); // temp_vector = H xhat'(k)
        kf->temp_vector1.numRows = kf->z.numRows;
        kf->temp_vector1.numCols = 1;
        kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1); // temp_vector1 = z(k) - H·xhat'(k)
        kf->temp_vector.numRows = kf->K.numRows;
        kf->temp_vector.numCols = 1;
        kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector); // temp_vector = K(k)·(z(k) - H·xhat'(k))
        kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
    }
}

/**
 * @brief 后验协方差矩阵更新（卡尔曼滤波第五式）
 * @param kf 卡尔曼滤波器结构体指针
 * @note 根据 SkipEq5 标志决定是否执行：
 *       若不跳过，计算 P = P⁻ - K·H·P⁻
 *       计算步骤：
 *         1. 计算 temp_matrix = K · H
 *         2. 计算 temp_matrix1 = temp_matrix · P⁻ = K·H·P⁻
 *         3. 计算 P = P⁻ - temp_matrix1
 *       最终结果存储在 kf->P 中，并更新 MatStatus。
 */
void Kalman_Filter_P_Update(KalmanFilter_t *kf)
{
    if (!kf->SkipEq5)
    {
        kf->temp_matrix.numRows = kf->K.numRows;
        kf->temp_matrix.numCols = kf->H.numCols;
        kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
        kf->temp_matrix1.numCols = kf->Pminus.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->K, &kf->H, &kf->temp_matrix);                 // temp_matrix = K(k)·H
        kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->Pminus, &kf->temp_matrix1); // temp_matrix1 = K(k)·H·P'(k)
        kf->MatStatus = Matrix_Subtract(&kf->Pminus, &kf->temp_matrix1, &kf->P);
    }
}

/**
 * @brief 执行卡尔曼滤波黄金五式,提供了用户定义函数,可以替代五个中的任意一个环节,方便自行扩展为EKF/UKF/ESKF/AUKF等
 * 
 * @param kf kf类型定义
 * @return float* 返回滤波值
 */
float *Kalman_Filter_Update(KalmanFilter_t *kf)
{
    // 0. 获取量测信息
    Kalman_Filter_Measure(kf);
    if (kf->User_Func0_f != NULL)
        kf->User_Func0_f(kf);

    // 先验估计
    // 1. xhat'(k)= A·xhat(k-1) + B·u
    Kalman_Filter_xhatMinusUpdate(kf);
    if (kf->User_Func1_f != NULL)
        kf->User_Func1_f(kf);

    // 预测更新
    // 2. P'(k) = A·P(k-1)·AT + Q
    Kalman_Filter_PminusUpdate(kf);
    if (kf->User_Func2_f != NULL)
        kf->User_Func2_f(kf);

    if (kf->MeasurementValidNum != 0 || kf->UseAutoAdjustment == 0)
    {
        // 量测更新
        // 3. K(k) = P'(k)·HT / (H·P'(k)·HT + R)
        Kalman_Filter_SetK(kf);

        if (kf->User_Func3_f != NULL)
            kf->User_Func3_f(kf);

        // 融合
        // 4. xhat(k) = xhat'(k) + K(k)·(z(k) - H·xhat'(k))
        Kalman_Filter_xhatUpdate(kf);

        if (kf->User_Func4_f != NULL)
            kf->User_Func4_f(kf);

        // 修正方差
        // 5. P(k) = (1-K(k)·H)·P'(k) ==> P(k) = P'(k)-K(k)·H·P'(k)
        Kalman_Filter_P_Update(kf);
    }
    else
    {
        // 无有效量测,仅预测
        // xhat(k) = xhat'(k)
        // P(k) = P'(k)
        memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);
        memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);
    }

    // 自定义函数,可以提供后处理等
    if (kf->User_Func5_f != NULL)
        kf->User_Func5_f(kf);

    // 避免滤波器过度收敛
    // suppress filter excessive convergence
    for (uint8_t i = 0; i < kf->xhatSize; i++)
    {
        if (kf->P_data[i * kf->xhatSize + i] < kf->StateMinVariance[i])
            kf->P_data[i * kf->xhatSize + i] = kf->StateMinVariance[i];
    }

    memcpy(kf->FilteredValue, kf->xhat_data, sizeof_float * kf->xhatSize);

    if (kf->User_Func6_f != NULL)
        kf->User_Func6_f(kf);

    return kf->FilteredValue;
}

/**
 * @brief 根据测量值有效性动态调整卡尔曼滤波器的 H、R、K 矩阵及 z 向量
 * @param kf 卡尔曼滤波器结构体指针
 * @note 该函数在启用自动调整（kf->UseAutoAdjustment != 0）时被调用。
 *       它根据 MeasuredVector 中非零项的数量，重构 z、H、R 矩阵，
 *       并相应调整矩阵的行/列数，实现变维度卡尔曼滤波。
 *       具体步骤：
 *         1. 将 MeasuredVector 复制到内部 z_data 并清零外部向量
 *         2. 遍历所有可能测量，收集有效（非零）测量值到 z_data 前端
 *         3. 根据 MeasurementMap 和 MeasurementDegree 重构 H 矩阵
 *         4. 根据 MatR_DiagonalElements 重构 R 矩阵（对角阵）
 *         5. 调整 H、HT、R、K、z 的矩阵维度为有效测量数
 */
static void H_K_R_Adjustment(KalmanFilter_t *kf)
{
    kf->MeasurementValidNum = 0;

    memcpy(kf->z_data, kf->MeasuredVector, sizeof_float * kf->zSize);
    memset(kf->MeasuredVector, 0, sizeof_float * kf->zSize);

    // 识别量测数据有效性并调整矩阵H R K
    // recognize measurement validity and adjust matrices H R K
    memset(kf->R_data, 0, sizeof_float * kf->zSize * kf->zSize);
    memset(kf->H_data, 0, sizeof_float * kf->xhatSize * kf->zSize);
    for (uint8_t i = 0; i < kf->zSize; i++)
    {
        if (kf->z_data[i] != 0)
        {
            // 重构向量z
            // rebuild vector z
            kf->z_data[kf->MeasurementValidNum] = kf->z_data[i];
            kf->temp[kf->MeasurementValidNum] = i;
            // 重构矩阵H
            // rebuild matrix H
            kf->H_data[kf->xhatSize * kf->MeasurementValidNum + kf->MeasurementMap[i] - 1] = kf->MeasurementDegree[i];
            kf->MeasurementValidNum++;
        }
    }
    for (uint8_t i = 0; i < kf->MeasurementValidNum; i++)
    {
        // 重构矩阵R
        // rebuild matrix R
        kf->R_data[i * kf->MeasurementValidNum + i] = kf->MatR_DiagonalElements[kf->temp[i]];
    }

    // 调整矩阵维数
    // adjust the dimensions of system matrices
    kf->H.numRows = kf->MeasurementValidNum;
    kf->H.numCols = kf->xhatSize;
    kf->HT.numRows = kf->xhatSize;
    kf->HT.numCols = kf->MeasurementValidNum;
    kf->R.numRows = kf->MeasurementValidNum;
    kf->R.numCols = kf->MeasurementValidNum;
    kf->K.numRows = kf->xhatSize;
    kf->K.numCols = kf->MeasurementValidNum;
    kf->z.numRows = kf->MeasurementValidNum;
}
