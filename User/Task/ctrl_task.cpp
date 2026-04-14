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


extern "C" void StartCtrlTask(void *argument)
{
    /* USER CODE BEGIN StartCtrlTask */
    /* Infinite loop */
    (void)argument;


    TickType_t last_wake_time = xTaskGetTickCount();
    //@warning:
    //TODO:
    //  不要删掉这行！！！
    //  osDelay函数的设计目的是阻塞当前程序
    osDelay(osWaitForever);
    //  不要删掉这行！现在在跑debug任务！

    for(;;)
    {


        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1));
    }
    /* USER CODE END StartCtrlTask */
}
