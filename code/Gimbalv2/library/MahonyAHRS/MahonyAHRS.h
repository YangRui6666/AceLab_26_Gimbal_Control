//=====================================================================================================
// MahonyAHRS.h
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// 马德威克的梅奥尼姿态航向参考系统算法实现
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
// 参见：http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 日期			作者			备注
// 29/09/2011	SOH Madgwick    Initial release
// 初始版本
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 优化以降低 CPU 负载
//
//=====================================================================================================
#ifndef MahonyAHRS_h
#define MahonyAHRS_h

//----------------------------------------------------------------------------------------------------
// Variable declaration
// 变量声明

extern volatile float twoKp;			// 2 * proportional gain (Kp)
										// 2 * 比例增益 (Kp)
extern volatile float twoKi;			// 2 * integral gain (Ki)
										// 2 * 积分增益 (Ki)
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
										// 传感器坐标系相对于辅助坐标系的四元数
extern volatile float sampleFreq;		// sample frequency in Hz
										// 采样频率（赫兹）

//---------------------------------------------------------------------------------------------------
// Function declarations
// 函数声明

void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void MahonyAHRSsetSampleFreq(float frequency_hz);
void MahonyAHRSreset(void);

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
