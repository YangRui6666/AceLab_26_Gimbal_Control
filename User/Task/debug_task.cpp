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
#include "SEGGER_RTT.h"

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

float gen_ramp_hold_target(float amp_deg,
                           float ramp_time_s,
                           float hold_time_s,
                           float start_tick_s = 0.0f)
{
    const float t = osKernelGetTickCount() * 0.001f - start_tick_s;
    const float step_s = ramp_time_s + hold_time_s;
    const float cycle_s = step_s * 4.0f;

    if (cycle_s <= 0.0f)
    {
        return 0.0f;
    }

    float phase_t = fmodf(t, cycle_s);
    if (phase_t < 0.0f)
    {
        phase_t += cycle_s;
    }

    if (phase_t < ramp_time_s)
    {
        return amp_deg * (phase_t / ramp_time_s);
    }

    if (phase_t < step_s)
    {
        return amp_deg;
    }

    phase_t -= step_s;

    if (phase_t < ramp_time_s)
    {
        return amp_deg * (1.0f - (phase_t / ramp_time_s));
    }

    if (phase_t < step_s)
    {
        return 0.0f;
    }

    phase_t -= step_s;

    if (phase_t < ramp_time_s)
    {
        return -amp_deg * (phase_t / ramp_time_s);
    }

    if (phase_t < step_s)
    {
        return -amp_deg;
    }

    phase_t -= step_s;

    if (phase_t < ramp_time_s)
    {
        return -amp_deg * (1.0f - (phase_t / ramp_time_s));
    }

    return 0.0f;
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
    imu_update();
    imu_get_data(&imu_data_fusion);
    osDelay(100);
    last_wake_time = xTaskGetTickCount();
    while (!imu_attitude_ready())
    {
        imu_update();
        motor_manage.update_feedback();
        motor_manage.lock();
        osDelay(2);
        static int abc = 0;
        SEGGER_RTT_printf(0,"Hello,SEGGER RTT %d\n\r",abc);
        abc += 1;
    }
    SEGGER_RTT_Init();
    osDelay(100);
    SEGGER_RTT_printf(0,"Hello,SEGGER RTT\n\r");
    for(;;)
    {
        imu_update();
        imu_get_data(&imu_data_fusion);

        targ_pitch = motor_manage.get_pitch_target();
        targ_yaw = motor_manage.get_yaw_target();
        (void)imu_attitude_ready();
        motor_manage.update_feedback();
        (void)can_check(0x206);
        float yaw_test = gen_sin_target(40.0f,1.0f);
        // motor_manage.set_world_target(0, gen_sin_target(10.0f, 0.5f), imu_data_fusion.yaw, imu_data_fusion.pitch);
        // motor_manage.set_world_target(0, gen_step_target(20.0f), imu_data_fusion.yaw, imu_data_fusion.pitch);
        // motor_manage.set_world_target(30.f, 0.0f, imu_data_fusion.yaw, imu_data_fusion.pitch);
        motor_manage.set_world_target(yaw_test,
                                      0.0f,
                                      imu_data_fusion.yaw,
                                      imu_data_fusion.pitch,
                                      true);
        // motor_manage.set_world_target(gen_sin_target(20.0f,0.5f),
        //                               0.0f,
        //                               imu_data_fusion.yaw,
        //                               imu_data_fusion.pitch);
        // motor_manage.set_world_target(gen_step_target(60.0f), 0, imu_data_fusion.yaw, imu_data_fusion.pitch);
        motor_manage.send_can_cmd();

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1));
    }
    /* USER CODE END StartCtrlTask */
}
