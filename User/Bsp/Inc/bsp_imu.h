#ifndef BSP_IMU_H
#define BSP_IMU_H

#ifdef __cplusplus
extern "C" {
#endif

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

bool bsp_imu_init(void);
bool bsp_imu_update(IMURawData *raw);
bool imu_check(void);

#ifdef __cplusplus
}
#endif

#endif
