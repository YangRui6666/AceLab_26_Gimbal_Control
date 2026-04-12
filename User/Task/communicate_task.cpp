//
// Created by CORE on 2026/4/9.
//

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usbd_def.h"


void StartCommunicateTask(void *argument)
{

    for(;;)
    {
        /*code*/
        osDelay(osWaitForever);
    }
}

