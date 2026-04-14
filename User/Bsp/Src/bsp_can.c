//include...

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "main.h"
#include "bsp_can.h"

extern CAN_HandleTypeDef hcan2;

#define BSP_CAN_RX_SLOT_COUNT      16U
#define BSP_CAN_MAX_DLC            8U
#define BSP_CAN_COMM_TIMEOUT_MS    1000U

typedef struct {
    uint16_t can_id;
    CanRxFrame frame;
    uint32_t last_seen_ms;
    bool used;
    bool unread;
} CanCacheSlot;

static CanCacheSlot s_can_cache[BSP_CAN_RX_SLOT_COUNT];

static CanCacheSlot *find_slot(uint16_t can_id);
static CanCacheSlot *find_or_alloc_slot(uint16_t can_id, uint32_t now_ms);
static void can_ISR(void);


/**
 * @brief       初始化can接受相关配置
 * 
 * @date        2026-04-09
 * @author      Rui.
 * 
 * @return true 初始化成功
 * @return false 失败
 */
bool bsp_can_init(void)
{
    CAN_FilterTypeDef filter = {0};

    filter.FilterBank = 14;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0;
    filter.FilterIdLow = 0;
    filter.FilterMaskIdHigh = 0;
    filter.FilterMaskIdLow = 0;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan2, &filter) != HAL_OK) {
        return false;
    }

    if (HAL_CAN_Start(&hcan2) != HAL_OK) {
        return false;
    }

    if (HAL_CAN_ActivateNotification(&hcan2,
                                     CAN_IT_RX_FIFO0_MSG_PENDING |
                                     CAN_IT_RX_FIFO0_FULL |
                                     CAN_IT_RX_FIFO0_OVERRUN |
                                     CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK) {
        return false;
    }

    return true;
}

/**
 * @brief       can发送函数，一个原始的封装，目前只支持标准帧
 * 
 * @date        2026-04-09
 * @author      Rui.
 * 
 * @param can_id 要发送的canid
 * @param data  需要发送的数据
 * @param dlc   dlc长度
 * @return true  发送成功
 * @return false 发送失败
 */
bool bsp_tx(uint16_t can_id, const uint8_t *data, uint8_t dlc)
{
    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t tx_mailbox = 0;

    if ((data == NULL) || (dlc > BSP_CAN_MAX_DLC) || (can_id > 0x7FFU)) {
        return false;
    }

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0U) {
        return false;
    }

    tx_header.StdId = can_id;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = dlc;
    tx_header.TransmitGlobalTime = DISABLE;

    return HAL_CAN_AddTxMessage(&hcan2, &tx_header, data, &tx_mailbox) == HAL_OK;
}

/**
 * @brief       对外的can读取函数
 * 
 * @date        2026-04-09
 * @author      Rui.
 * 
 * @param can_id 
 * @param frame 
 * @return true 
 * @return false 
 */
bool bsp_can_rx(uint16_t can_id, CanRxFrame *frame)
{
    CanCacheSlot *slot;

    if (frame == NULL) {
        return false;
    }

    slot = find_slot(can_id);
    if ((slot == NULL) || (!slot->unread)) {
        return false;
    }

    *frame = slot->frame;
    slot->unread = false;
    return true;
}
/**
 * @brief       CAN中断函数，发送中断后会调用该函数
 * 
 * @date        2026-04-09
 * @author      Rui.
 * 
 */
static void can_ISR(void)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[BSP_CAN_MAX_DLC];
    uint32_t now_ms = HAL_GetTick();

    while (HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO0) > 0U) {
        CanCacheSlot *slot;

        if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
            break;
        }

        if ((rx_header.IDE != CAN_ID_STD) || (rx_header.RTR != CAN_RTR_DATA)) {
            continue;
        }

        slot = find_or_alloc_slot((uint16_t)rx_header.StdId, now_ms);
        if (slot == NULL) {
            continue;
        }

        memset(slot->frame.data, 0, sizeof(slot->frame.data));
        if (rx_header.DLC > BSP_CAN_MAX_DLC) {
            continue;
        }
        memcpy(slot->frame.data, rx_data, rx_header.DLC);
        slot->frame.dlc = (uint8_t)rx_header.DLC;
        slot->frame.timestamp_ms = now_ms;
        slot->last_seen_ms = now_ms;
        slot->unread = true;
    }
}

/**
 * @brief       检查指定id通讯是否正常，超时1000ms返回false
 * 
 * @date        2026-04-09
 * @author      Rui.
 * 
 * @param can_id 
 * @return true 
 * @return false 
 */
bool can_check(uint16_t can_id)
{
    CanCacheSlot *slot = find_slot(can_id);

    if ((slot == NULL) || (!slot->used)) {
        return false;
    }

    return (HAL_GetTick() - slot->last_seen_ms) < BSP_CAN_COMM_TIMEOUT_MS;
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan2) {
        can_ISR();
    }
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan2) {
        can_ISR();
    }
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan2) {
        can_ISR();
    }
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan2) {
        can_ISR();
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan2) {
        can_ISR();
    }
}

static CanCacheSlot *find_slot(uint16_t can_id)
{
    uint32_t i;

    for (i = 0; i < BSP_CAN_RX_SLOT_COUNT; ++i) {
        if (s_can_cache[i].used && (s_can_cache[i].can_id == can_id)) {
            return &s_can_cache[i];
        }
    }

    return NULL;
}

static CanCacheSlot *find_or_alloc_slot(uint16_t can_id, uint32_t now_ms)
{
    uint32_t i;
    uint32_t oldest_index = 0U;
    uint32_t oldest_time = UINT32_MAX;

    for (i = 0; i < BSP_CAN_RX_SLOT_COUNT; ++i) {
        if (s_can_cache[i].used && (s_can_cache[i].can_id == can_id)) {
            return &s_can_cache[i];
        }

        if (!s_can_cache[i].used) {
            s_can_cache[i].used = true;
            s_can_cache[i].can_id = can_id;
            s_can_cache[i].last_seen_ms = now_ms;
            s_can_cache[i].unread = false;
            memset(&s_can_cache[i].frame, 0, sizeof(s_can_cache[i].frame));
            return &s_can_cache[i];
        }

        if (s_can_cache[i].last_seen_ms < oldest_time) {
            oldest_time = s_can_cache[i].last_seen_ms;
            oldest_index = i;
        }
    }

    s_can_cache[oldest_index].used = true;
    s_can_cache[oldest_index].can_id = can_id;
    s_can_cache[oldest_index].last_seen_ms = now_ms;
    s_can_cache[oldest_index].unread = false;
    memset(&s_can_cache[oldest_index].frame, 0, sizeof(s_can_cache[oldest_index].frame));
    return &s_can_cache[oldest_index];
}
