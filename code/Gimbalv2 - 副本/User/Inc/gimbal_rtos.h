#ifndef GIMBAL_RTOS_H
#define GIMBAL_RTOS_H

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

extern QueueHandle_t g_ctrl_msg_queue;
extern SemaphoreHandle_t g_comm_sem;

bool gimbal_rtos_init(void);
void gimbal_comm_signal_from_isr(void);

#ifdef __cplusplus
}
#endif

#endif
