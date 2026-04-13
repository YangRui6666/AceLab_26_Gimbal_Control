#ifndef BSP_USB_H
#define BSP_USB_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "ctrl_msg_queue.h"

typedef struct
{
    int16_t yaw_target;
    int16_t pitch_target;
    int16_t roll_target;
    uint32_t time_stamp;
    uint8_t mode;
    uint8_t reserved;
} usb_status_feedback_t;

bool bsp_usb_init(void);
bool bsp_usb_rx_push(const uint8_t *data, uint16_t len);
bool bsp_usb_parse_next_ctrl_msg(CtrlMsg_t *msg_out);
bool usb_protocol_send_status(const usb_status_feedback_t *status);
bool usb_protocol_send_lock_feedback(uint32_t time_stamp, uint8_t reason);
bool usb_send_raw(const char *str);
bool usb_send_rawf(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif
