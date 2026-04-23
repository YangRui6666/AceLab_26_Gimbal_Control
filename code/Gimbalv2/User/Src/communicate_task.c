#include "communicate_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include "gimbal_protocol.h"
#include "gimbal_rtos.h"

void Start_communicate_Task(void *argument)
{
    (void)argument;
    gimbal_protocol_rx_reset();

    for (;;) {
        CtrlMsg_t msg = {0};
        uint8_t produced_msg = 0U;

        if (g_comm_sem == NULL) {
            vTaskDelay(pdMS_TO_TICKS(1U));
            continue;
        }

        (void)xSemaphoreTake(g_comm_sem, portMAX_DELAY);

        while (gimbal_protocol_process_next(&msg, &produced_msg)) {
            if ((produced_msg != 0U) && (g_ctrl_msg_queue != NULL)) {
                (void)xQueueSend(g_ctrl_msg_queue, &msg, 0U);
            }
        }
    }
}
