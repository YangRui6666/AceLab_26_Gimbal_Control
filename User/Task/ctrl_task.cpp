//
// Created by CORE on 2026/4/9.
//
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "MotorManage.h"
#include "imu_fusion.h"




static int text_wave()
{
    static int i = 0;
    if (i == 0)
    {
        i = 1;
        return 30;
    }
    else
    {
        i = 0;
        return 0;
    }
}

extern "C" void StartCtrlTask(void *argument)
{
    /* USER CODE BEGIN StartCtrlTask */
    /* Infinite loop */
    
    MotorManage motor_manage;

    TickType_t last_wake_time = xTaskGetTickCount();
    imu_data_t imu_data_fusion;
    imu_data_fusion.pitch = -1;
    imu_data_fusion.roll = -1;
    imu_data_fusion.yaw = -1;

    GM6020::Target targ_yaw = motor_manage.get_yaw_target();
    GM6020::Target targ_pitch = motor_manage.get_pitch_target();
    targ_pitch.target_current = -1;
    targ_pitch.target_speed_cdps = -1;
    targ_yaw.target_current = -1;
    targ_yaw.target_speed_cdps = -1;

        for(;;)
        {
            /*code*/
            imu_update();
            imu_get_data(&imu_data_fusion);

            targ_pitch = motor_manage.get_pitch_target();
            targ_yaw = motor_manage.get_yaw_target();

            motor_manage.update_feedback();
            motor_manage.set(text_wave(),0);
            motor_manage.send_can_cmd();




            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1));
        }
    /* USER CODE END StartCtrlTask */
}