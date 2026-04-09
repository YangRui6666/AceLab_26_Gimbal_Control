//
// Created by CORE on 2026/4/9.
//
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

void StartCtrlTask(void *argument)
{
    /* USER CODE BEGIN StartCtrlTask */
    /* Infinite loop */
    
    TickType_t last_wake_time = xTaskGetTickCount();
        for(;;)
        {
            /*code*/



            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1));
        }
    /* USER CODE END StartCtrlTask */
}

