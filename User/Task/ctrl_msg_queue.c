#include "ctrl_msg_queue.h"

#include "cmsis_os2.h"

#define CTRL_MSG_QUEUE_LENGTH 32U

static osMessageQueueId_t ctrl_msg_queue_handle = NULL;

bool ctrl_msg_queue_init(void)
{
    if (ctrl_msg_queue_handle != NULL)
    {
        return true;
    }

    ctrl_msg_queue_handle = osMessageQueueNew(
        CTRL_MSG_QUEUE_LENGTH,
        sizeof(CtrlMsg_t),
        NULL);

    return ctrl_msg_queue_handle != NULL;
}

bool ctrl_msg_queue_push(const CtrlMsg_t *msg)
{
    if ((msg == NULL) || (ctrl_msg_queue_handle == NULL))
    {
        return false;
    }

    return osMessageQueuePut(ctrl_msg_queue_handle, msg, 0U, 0U) == osOK;
}

bool ctrl_msg_queue_pop(CtrlMsg_t *msg, uint32_t timeout_ms)
{
    if ((msg == NULL) || (ctrl_msg_queue_handle == NULL))
    {
        return false;
    }

    return osMessageQueueGet(ctrl_msg_queue_handle, msg, NULL, timeout_ms) == osOK;
}
