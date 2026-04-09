//
// Created by CORE on 2026/4/9.
//
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "MotorManage.h"
#include "imu_fusion.h"

void StartCtrlTask(void *argument)
{
    /* USER CODE BEGIN StartCtrlTask */
    /* Infinite loop */
    
    MotorManage motor_manage;
    motor_manage.init();

    TickType_t last_wake_time = xTaskGetTickCount();
        for(;;)
        {
            /*code*/
            imu_update();
            imu_data_t imu_data_fusion;
            imu_get_data(&imu_data_fusion);
            
            //这里要做一些处理变换

            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1));
        }
    /* USER CODE END StartCtrlTask */
}

