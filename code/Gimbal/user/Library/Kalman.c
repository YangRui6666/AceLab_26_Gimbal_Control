#include <math.h>
#include "main.h"
#include "Kalman.h"


Kalman_t KalmanX = {
    .Q_angle = 0.001f, // 角度过程噪声协方差
    .Q_bias = 0.003f,  // 偏置过程噪声协方差
    .R_measure = 0.03f // 测量噪声协方差
};

Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f};

Kalman_t KalmanZ = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f};

BMI088_t bmi088Data; // 用于储存卡尔曼滤波后的得到的欧拉角数据

uint32_t timer;

// 定义角度归一化函数
static double normalizeAngle(double angle)
{
    while (angle > 180.0)
    {
        angle -= 360.0;
    }
    while (angle < -180.0)
    {
        angle += 360.0;
    }
    return angle;
}

/**
 * @brief 初始化 BMI088 传感器数据结构体、卡尔曼滤波器结构体以及计时器。
 *        此函数用于对 BMI088 传感器相关的数据存储结构体、卡尔曼滤波器的参数结构体
 *        以及记录时间的计时器进行初始化操作，为后续的数据读取和滤波处理做好准备。
 * @param kalmanX 指向 Kalman_t 结构体的指针，用于对滚转角进行卡尔曼滤波。函数会为其设置
 *                初始的过程噪声协方差、偏置过程噪声协方差、测量噪声协方差等参数，并将
 *                协方差矩阵初始化为 0。
 * @param kalmanY 指向 Kalman_t 结构体的指针，用于对俯仰角进行卡尔曼滤波。同样会为其设置
 *                初始参数，并将协方差矩阵初始化为 0。
 * @param kalmanZ 指向 Kalman_t 结构体的指针，用于对偏航角进行卡尔曼滤波。同样会为其设置
 *                初始参数，并将协方差矩阵初始化为 0。
 * @retval 无
 */

void initialize_BMI088_and_kalman(Kalman_t *kalmanX, Kalman_t *kalmanY, Kalman_t *kalmanZ)
{
    // 初始化 BMI088 数据结构体
    bmi088Data.Roll = 0;
    bmi088Data.Pitch = 0;
    bmi088Data.Yaw = 0;

    // 初始化卡尔曼滤波器结构体
    kalmanX->Q_angle = 0.001f;
    kalmanX->Q_bias = 0.003f;
    kalmanX->R_measure = 0.03f;
    kalmanX->angle = 0;
    kalmanX->bias = 0;
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            kalmanX->P[i][j] = 0;
        }
    }

    kalmanY->Q_angle = 0.001f;
    kalmanY->Q_bias = 0.003f;
    kalmanY->R_measure = 0.03f;
    kalmanY->angle = 0;
    kalmanY->bias = 0;
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            kalmanY->P[i][j] = 0;
        }
    }

    kalmanZ->Q_angle = 0.001f;
    kalmanZ->Q_bias = 0.003f;
    kalmanZ->R_measure = 0.03f;
    kalmanZ->angle = 0;
    kalmanZ->bias = 0;
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            kalmanZ->P[i][j] = 0;
        }
    }

    // 初始化计时器
    timer = HAL_GetTick();
}

/**
 * @brief 使用卡尔曼滤波，融合角度预测值与角度测量值，得到最优角度值。
 *        卡尔曼滤波通过结合系统的预测值和测量值，来修正角度和偏置的估计，减少噪声对结果的影响。
 * @param Kalman 指向 Kalman_t 结构体的指针，包含预测角度、预测偏置、角度协方差、偏置协方差、噪声协方差及协方差矩阵。
 * @param newAngle 角度测量值（读取加速度计的三轴加速度分量，再计算反正切得到角度测量值）。
 * @param newRate 角速度“实际“值（可看作角速度测量值，因陀螺仪精度问题存在过程噪声）。
 * @param dt 时间间隔，两个传感器数据采样之间的时间差（秒）。
 * @retval double 滤波后的最优角度值。
 */
double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
    /*---------------------预测阶段--------------------------*/
    // 1. 预测角度
    double rate = newRate - Kalman->bias; // 角速度 = 陀螺仪角速度 - 陀螺仪偏置值 (得到无偏角速度)
    Kalman->angle += dt * rate;           // 预测角度 = 前一时刻角速 + 时间间隔*角速度

    // 2. 预测协方差矩阵
    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle); // 预测角度协方差
    Kalman->P[0][1] -= dt * Kalman->P[1][1];                                                              // 预测角度和偏置的协方差
    Kalman->P[1][0] -= dt * Kalman->P[1][1];                                                              // 预测偏置和角度的协方差
    Kalman->P[1][1] += Kalman->Q_bias * dt;                                                               // 预测偏置协方差

    /*---------------------更新阶段--------------------------*/
    // 3. 更新卡尔曼增益
    double S = Kalman->P[0][0] + Kalman->R_measure; // 总误差协方差 = 预测协方差 + 测量噪声协方差
    double K[2];
    K[0] = Kalman->P[0][0] / S; // 角度的卡尔曼增益
    K[1] = Kalman->P[1][0] / S; // 偏置的卡尔曼增益

    // 4. 更新角度和偏置
    double y = newAngle - Kalman->angle; // 测量残差 = 测量值 - 预测值
    Kalman->angle += K[0] * y;           // 更新角度估计。
    Kalman->bias += K[1] * y;            // 更新偏置估计

    // 5. 更新协方差矩阵 P
    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];
    Kalman->P[0][0] -= K[0] * P00_temp; // 更新角度协方差
    Kalman->P[0][1] -= K[0] * P01_temp; // 更新角度和偏置的协方差
    Kalman->P[1][0] -= K[1] * P00_temp; // 更新偏置和角度的协方差
    Kalman->P[1][1] -= K[1] * P01_temp; // 更新偏置协方差

    // 6. 返回滤波后的最优角度值
    return Kalman->angle;
}

/**
 * @brief 对 BMI088 传感器数据进行卡尔曼滤波处理，计算并更新滚转角和俯仰角。
 *        此函数利用传入的 BMI088 传感器原始数据，结合卡尔曼滤波算法，
 *        对加速度计和陀螺仪数据进行融合处理，以减少噪声干扰，
 *        从而得到更准确的滚转角和俯仰角估计值。
 * @param kalmanX 指向 Kalman_t 结构体的指针，用于对滚转角进行卡尔曼滤波。
 *                该结构体中包含了卡尔曼滤波所需的各种参数，如过程噪声协方差、测量噪声协方差等。
 * @param kalmanY 指向 Kalman_t 结构体的指针，用于对俯仰角进行卡尔曼滤波。
 *                同样包含了卡尔曼滤波所需的相关参数。
 * @param kalmanZ 指向 Kalman_t 结构体的指针，用于对偏航角进行卡尔曼滤波。
 *                同样包含了卡尔曼滤波所需的相关参数。
 * @param imu_data 指向 BMI088_ReceiveDataTypedef 结构体的指针，包含 IMU 原始数据。
 * @retval 无
 */
void BMI088_Kalman_Filter(Kalman_t *kalmanX, Kalman_t *kalmanY, Kalman_t *kalmanZ, BMI088_ReceiveDataTypedef *imu_data)
{
    // 计算时间增量 dt，单位为秒
    double dt = (double)(HAL_GetTick() - timer) / 1000;
    timer = HAL_GetTick();
	
	imu_data->gyro_x -= imu_data->x_error;
	imu_data->gyro_y -= imu_data->y_error;
	imu_data->gyro_z -= imu_data->z_error;
	

    // 计算滚转角 roll
    double roll;
    double roll_sqrt = sqrt(imu_data->acc_x * imu_data->acc_x + imu_data->acc_z * imu_data->acc_z);
    if (roll_sqrt != 0.0)
    {
        roll = atan(imu_data->acc_y / roll_sqrt) * RAD_TO_DEG;
    }
    else
    {
        roll = 0.0;
    }

    // 计算俯仰角 pitch
    double pitch = atan2(-imu_data->acc_x, imu_data->acc_z) * RAD_TO_DEG;

    // 如果俯仰角度变化过快 (超过 90 度)，防止角度跳变
    if ((pitch < -90 && bmi088Data.Pitch > 90) || (pitch > 90 && bmi088Data.Pitch < -90))
    {
        kalmanY->angle = pitch;
        bmi088Data.Pitch = pitch;
    }
    else
    {
        // 卡尔曼滤波器更新俯仰角度 Y
        bmi088Data.Pitch = Kalman_getAngle(kalmanY, pitch, imu_data->gyro_y, dt);
    }

    // 如果俯仰角绝对值超过 90 度，则反转 X 轴的陀螺仪角速度，防止符号错误
    if (fabs(bmi088Data.Pitch) > 90)
        imu_data->gyro_x = -imu_data->gyro_x;

    // 卡尔曼滤波器更新滚转角度 X
    bmi088Data.Roll = Kalman_getAngle(kalmanX, roll, imu_data->gyro_x, dt);

    // 卡尔曼滤波器更新偏航角度 Z
    bmi088Data.Yaw = Kalman_getAngle(kalmanZ, bmi088Data.Yaw, imu_data->gyro_z, dt);
    
    bmi088Data.Yaw = normalizeAngle(bmi088Data.Yaw);

    imu_data->pitch = bmi088Data.Pitch;
    imu_data->yaw = bmi088Data.Yaw;
    imu_data->roll = -bmi088Data.Roll;

}
