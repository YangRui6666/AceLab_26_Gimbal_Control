#ifndef GIMBAL_PROTOCOL_H
#define GIMBAL_PROTOCOL_H

#include "gimbal_types.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void gimbal_protocol_rx_reset(void);
void gimbal_protocol_rx_push_bytes(const uint8_t *data, uint16_t len);
bool gimbal_protocol_process_next(CtrlMsg_t *out_msg, uint8_t *produced_msg);
bool gimbal_protocol_send_status(const AttitudeAngle *attitude,
                                 uint32_t time_stamp_ms,
                                 GimbalMode_e mode,
                                 bool telemetry_enabled);
bool gimbal_protocol_send_lock_feedback(LockReason_e reason, uint32_t time_stamp_ms);

#ifdef __cplusplus
}
#endif

#endif
