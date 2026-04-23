#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "gimbal_types.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool bsp_can_init(void);
bool bsp_can_get_latest(uint16_t can_id, CanRxFrame *frame, uint32_t *rx_time_ms);
bool bsp_can_send_gimbal_currents(int16_t yaw_current, int16_t pitch_current);

#ifdef __cplusplus
}
#endif

#endif
