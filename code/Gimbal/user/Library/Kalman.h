#ifndef __KALMAN_H
#define __KALMAN_H



#define RAD_TO_DEG 57.295779513082320876798154814105

// 新增 BMI088 数据结构体（用于卡尔曼滤波）
typedef struct
{
  float acc_x;          // 加速度计 X 轴 (m/s^2)
  float acc_y;          // 加速度计 Y 轴
  float acc_z;          // 加速度计 Z 轴
  float gyro_x;         // 陀螺仪 X 轴 (deg/s)
  float gyro_y;         // 陀螺仪 Y 轴
  float gyro_z;         // 陀螺仪 Z 轴
  float x_error;        // X 轴零偏补偿
  float y_error;        // Y 轴零偏补偿
  float z_error;        // Z 轴零偏补偿
  float roll;           // 输出的横滚角 (度)
  float pitch;          // 输出的俯仰角 (度)
  float yaw;            // 输出的偏航角 (度)
} BMI088_ReceiveDataTypedef;


typedef struct
{
  double Roll;  // X 轴的卡尔曼滤波计算角度
  double Pitch; // Y 轴的卡尔曼滤波计算角度
  double Yaw;   // Z 轴的卡尔曼滤波计算角度
  float yaw_error;
  float yaw_sumerror;
  float yaw_last; 
} BMI088_t;

typedef struct
{
  double Q_angle;   // 角度过程噪声协方差
  double Q_bias;    // 偏置过程噪声协方差
  double R_measure; // 测量噪声协方差
  double angle;     // 估计角度
  double bias;      // 估计偏置
  double P[2][2];   // 协方差矩阵
} Kalman_t;

extern Kalman_t KalmanX;
extern Kalman_t KalmanY;
extern Kalman_t KalmanZ;

void initialize_BMI088_and_kalman(Kalman_t *KalmanX, Kalman_t *KalmanY, Kalman_t *KalmanZ);
double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);
void BMI088_Kalman_Filter(Kalman_t *KalmanX, Kalman_t *KalmanY, Kalman_t *KalmanZ, BMI088_ReceiveDataTypedef *BMI088);

#endif
