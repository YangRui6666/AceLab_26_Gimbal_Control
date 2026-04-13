#ifndef CTRL_MSG_QUEUE_H
#define CTRL_MSG_QUEUE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    CTRL_MSG_NONE = 0,
    CTRL_MSG_ENTER_SEARCH,
    CTRL_MSG_AUTO_AIM_DELTA,
    CTRL_MSG_ENTER_LOCK,
    CTRL_MSG_EXIT_LOCK
} CtrlMsgType_e;

typedef struct
{
    CtrlMsgType_e type;
    uint32_t time_stamp;
    float delta_yaw;
    float delta_pitch;
} CtrlMsg_t;

bool ctrl_msg_queue_init(void);
bool ctrl_msg_queue_push(const CtrlMsg_t *msg);
bool ctrl_msg_queue_pop(CtrlMsg_t *msg, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif
