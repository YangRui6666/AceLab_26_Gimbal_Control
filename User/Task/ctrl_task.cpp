//
// Created by CORE on 2026/4/9.
//
#include "bsp_usb.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "MotorManage.h"
#include "imu_fusion.h"




static float text_wave()
{
    static TickType_t last_switch_tick = 0;
    static bool high = false;

    const TickType_t now = xTaskGetTickCount();
    if ((now - last_switch_tick) >= pdMS_TO_TICKS(1000))
    {
        last_switch_tick = now;
        high = !high;
    }

    return high ? 30.0f : 0.0f;
}

extern "C" void StartCtrlTask(void *argument)
{
    /* USER CODE BEGIN StartCtrlTask */
    /* Infinite loop */
    (void)argument;

    MotorManage motor_manage;

    TickType_t last_wake_time = xTaskGetTickCount();
    imu_data_t imu_data_fusion = {0.0f, 0.0f, 0.0f};

    GM6020::Target targ_yaw = motor_manage.get_yaw_target();
    GM6020::Target targ_pitch = motor_manage.get_pitch_target();


    bsp_usb_init();
    bsp_can_init();
    imu_init();
    osDelay(100);
    last_wake_time = xTaskGetTickCount();
    for(;;)
    {
        imu_update();
        imu_get_data(&imu_data_fusion);

        targ_pitch = motor_manage.get_pitch_target();
        targ_yaw = motor_manage.get_yaw_target();

        motor_manage.update_feedback();
        motor_manage.set(text_wave(), 0.0f);
        motor_manage.send_can_cmd();

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1));
    }
    /* USER CODE END StartCtrlTask */
}
