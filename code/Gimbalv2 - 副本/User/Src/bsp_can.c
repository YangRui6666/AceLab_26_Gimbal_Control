

#include "bsp_can.h"

#include "can.h"
#include "gimbal_config.h"

#include <string.h>

typedef struct {
    CanRxFrame frame;
    uint32_t rx_time_ms;
    bool valid;
} CanSlot_t;

static CanSlot_t s_can_slots[2];
static bool s_can_ready = false;

static int32_t can_id_to_slot(uint16_t can_id)
{
    if (can_id == YAW_MOTOR_CAN_ID) {
        return 0;
    }

    if (can_id == PITCH_MOTOR_CAN_ID) {
        return 1;
    }

    return -1;
}

bool bsp_can_init(void)
{
    CAN_FilterTypeDef filter = {0};

    if (s_can_ready) {
        return true;
    }

    filter.FilterActivation = ENABLE;
    filter.FilterBank = 0U;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterIdHigh = 0U;
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = 0U;
    filter.FilterMaskIdLow = 0U;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.SlaveStartFilterBank = 14U;

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) {
        return false;
    }

    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        return false;
    }

    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        return false;
    }

    memset(s_can_slots, 0, sizeof(s_can_slots));
    s_can_ready = true;
    return true;
}

bool bsp_can_get_latest(uint16_t can_id, CanRxFrame *frame, uint32_t *rx_time_ms)
{
    int32_t slot_index = can_id_to_slot(can_id);
    bool result = false;

    if ((slot_index < 0) || (frame == NULL) || (rx_time_ms == NULL)) {
        return false;
    }

    __disable_irq();
    if (s_can_slots[slot_index].valid) {
        memcpy(frame, &s_can_slots[slot_index].frame, sizeof(*frame));
        *rx_time_ms = s_can_slots[slot_index].rx_time_ms;
        result = true;
    }
    __enable_irq();

    return result;
}

bool bsp_can_send_gimbal_currents(int16_t yaw_current, int16_t pitch_current)
{
    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t mailbox = 0U;
    uint8_t tx_data[8] = {0};
    int16_t currents[4] = {0, 0, 0, 0};

    currents[YAW_MOTOR_CTRL_SLOT] = yaw_current;
    currents[PITCH_MOTOR_CTRL_SLOT] = pitch_current;

    tx_header.StdId = GIMBAL_CURRENT_TX_CAN_ID;
    tx_header.ExtId = 0U;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8U;
    tx_header.TransmitGlobalTime = DISABLE;

    tx_data[0] = (uint8_t)((currents[0] >> 8U) & 0xFFU);
    tx_data[1] = (uint8_t)(currents[0] & 0xFFU);
    tx_data[2] = (uint8_t)((currents[1] >> 8U) & 0xFFU);
    tx_data[3] = (uint8_t)(currents[1] & 0xFFU);
    tx_data[4] = (uint8_t)((currents[2] >> 8U) & 0xFFU);
    tx_data[5] = (uint8_t)(currents[2] & 0xFFU);
    tx_data[6] = (uint8_t)((currents[3] >> 8U) & 0xFFU);
    tx_data[7] = (uint8_t)(currents[3] & 0xFFU);

    if (!s_can_ready) {
        return false;
    }

    return HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &mailbox) == HAL_OK;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header = {0};
    uint8_t rx_data[8] = {0};

    if ((hcan == NULL) || (hcan->Instance != CAN1)) {
        return;
    }

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U) {
        int32_t slot_index = -1;

        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
            return;
        }

        slot_index = can_id_to_slot((uint16_t)rx_header.StdId);
        if (slot_index < 0) {
            continue;
        }

        s_can_slots[slot_index].frame.can_id = (uint16_t)rx_header.StdId;
        s_can_slots[slot_index].frame.dlc = rx_header.DLC;
        memcpy(s_can_slots[slot_index].frame.data, rx_data, sizeof(rx_data));
        s_can_slots[slot_index].rx_time_ms = HAL_GetTick();
        s_can_slots[slot_index].valid = true;
    }
}
