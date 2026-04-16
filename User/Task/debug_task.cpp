//
// Created by CORE on 2026/4/14.
//

//
// Created by CORE on 2026/4/9.
//
#include <cmath>

#include "bsp_usb.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "MotorManage.h"
#include "imu_fusion.h"


float gen_step_target(float step_deg)
{
    // tick 假设 1ms
    float t = osKernelGetTickCount() * 0.001f;

    // 每 4 秒一个状态，共 5 个状态，20 秒一循环
    int phase = static_cast<int>(t / 4.0f) % 5;

    switch (phase)
    {
        case 0: return 0.0f;
        case 1: return step_deg;
        case 2: return 0.0f;
        case 3: return -step_deg;
        case 4: return 0.0f;
        default: return 0.0f;
    }
}

float gen_sin_target(float amp_deg, float freq_hz)
{
    const float two_pi = 6.2831853f;

    // 时间（假设 tick = 1ms）
    float t = osKernelGetTickCount() * 0.001f;

    return amp_deg * sinf(two_pi * freq_hz * t);
}

extern "C" void StartdddebugTask(void *argument)
{
    /* USER CODE BEGIN StartCtrlTask */
    /* Infinite loop */
    osDelay(osWaitForever);
    
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
        (void)imu_check();
        motor_manage.update_feedback();
        (void)can_check(0x206);
        // motor_manage.set(0,gen_sin_target(10.0f, 0.5f));
        // motor_manage.set(0, gen_step_target(20.0f));
        // motor_manage.set(30.f, 0.0f);
        motor_manage.set(gen_sin_target(10.0f, 0.5f),-10.0f);
        // motor_manage.set(gen_step_target(60.0f), 0);
        motor_manage.send_can_cmd();

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1));
    }
    /* USER CODE END StartCtrlTask */
}
