#include <stdbool.h>
#include <stdint.h>

typedef struct 
{
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temp;
} IMURawData;




/**
 * @brief       初始化bmi088
 * 
 * @date        2026-04-09
 * @author      Rui.
 * 
 * @return true 
 * @return false 
 */
bool bsp_imu_init(void)
{
    //TODO:
    //这里写imu的初始化函数
}

/**
 * @brief       更新六轴原始数据
 * 
 * @date        2026-04-09
 * @author      Rui.
 * 
 * @param raw   传入一个指针,向其写入六轴陀螺仪数值
 * @return true 
 * @return false 
 */
bool bsp_imu_update(IMURawData *raw)
{

}

/**
 * @brief       检查陀螺仪是否掉线
 * 
 * @date        2026-04-09
 * @author      Rui.
 * 
 * @return true 
 * @return false 
 */
bool imu_check(void)
{

}
