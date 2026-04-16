//
// Created by CORE on 2026/4/9.
//

#include "bsp_usb.h"
#include "cmsis_os2.h"
#include "ctrl_msg_queue.h"

extern "C" osSemaphoreId_t comm_semHandle;

extern "C" void StartCommunicateTask(void *argument)
{
    static CtrlMsg_t ctrl_msg;

    (void)argument;
    bsp_usb_init();
    ctrl_msg_queue_init();

    for(;;)
    {
        osSemaphoreAcquire(comm_semHandle, osWaitForever);

        while (bsp_usb_parse_next_ctrl_msg(&ctrl_msg))
        {
            ctrl_msg_queue_push(&ctrl_msg);
        }
    }
}
