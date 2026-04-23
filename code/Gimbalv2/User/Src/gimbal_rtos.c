#include "gimbal_rtos.h"

#include "gimbal_config.h"
#include "gimbal_types.h"

QueueHandle_t g_ctrl_msg_queue = NULL;
SemaphoreHandle_t g_comm_sem = NULL;

bool gimbal_rtos_init(void)
{
    if (g_ctrl_msg_queue == NULL) {
        g_ctrl_msg_queue = xQueueCreate(CTRL_MSG_QUEUE_LENGTH, sizeof(CtrlMsg_t));
    }

    if (g_comm_sem == NULL) {
        g_comm_sem = xSemaphoreCreateCounting(COMM_SEMAPHORE_MAX_COUNT, 0U);
    }

    return (g_ctrl_msg_queue != NULL) && (g_comm_sem != NULL);
}

void gimbal_comm_signal_from_isr(void)
{
    BaseType_t x_higher_priority_task_woken = pdFALSE;

    if (g_comm_sem != NULL) {
        xSemaphoreGiveFromISR(g_comm_sem, &x_higher_priority_task_woken);
        portYIELD_FROM_ISR(x_higher_priority_task_woken);
    }
}
