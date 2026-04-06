/**
 * @file VisionTask.c
 * @brief 视觉通信任务实现
 * @author CORE
 * @date 2026-04-06
 */

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "../Bsp/Inc/bsp_usb.h"
#include "../Config/vision_config.h"

// 外部信号量句柄
extern osSemaphoreId_t VisionBinarySemHandle;

typedef enum
{
    VISION_RX_WAIT_SOF_0 = 0,
    VISION_RX_WAIT_SOF_1,
    VISION_RX_WAIT_LEN,
    VISION_RX_WAIT_CMD,
    VISION_RX_WAIT_DATA,
    VISION_RX_WAIT_CRC_0,
    VISION_RX_WAIT_CRC_1,
    VISION_RX_WAIT_EOF_0,
    VISION_RX_WAIT_EOF_1
} vision_rx_state_t;

typedef enum
{
    VISION_TX_PRIORITY_NONE = 0,
    VISION_TX_PRIORITY_STATUS = 1,
    VISION_TX_PRIORITY_LOCK = 2
} vision_tx_priority_t;

typedef struct
{
    vision_rx_state_t state;
    uint8_t frame_buffer[VISION_PROTOCOL_MAX_FRAME_LEN];
    uint8_t payload_length;
    uint8_t payload_index;
    uint16_t received_crc;
} vision_rx_parser_t;

typedef struct
{
    uint8_t cmd;
    uint8_t data[VISION_PROTOCOL_MAX_DATA_LEN];
    uint8_t data_length;
} vision_packet_t;

typedef struct
{
    gimbal_mode_t requested_mode;
    bool feedback_enabled;
    bool disable_latched;
    float yaw_error_deg;
    float pitch_error_deg;
    uint32_t command_timestamp;
    uint32_t last_valid_packet_tick;
    uint32_t last_search_packet_tick;
    uint32_t last_auto_aim_packet_tick;
    uint32_t mode_sequence;
    uint32_t auto_aim_sequence;
    uint32_t last_status_send_tick;
    bool lock_report_pending;
    vision_lock_reason_t pending_lock_reason;
} vision_protocol_state_t;

static vision_command_mailbox_t g_vision_command_mailbox = {
    .requested_mode = GIMBAL_MODE_STABLE
};

static gimbal_feedback_snapshot_t g_gimbal_feedback_snapshot = {
    .current_mode = GIMBAL_MODE_STABLE
};

static uint8_t g_pending_tx_frame[VISION_PROTOCOL_MAX_FRAME_LEN];
static uint16_t g_pending_tx_length = 0U;
static vision_tx_priority_t g_pending_tx_priority = VISION_TX_PRIORITY_NONE;

static uint16_t vision_crc16_modbus(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;

    if (data == NULL)
    {
        return crc;
    }

    for (uint16_t i = 0U; i < length; ++i)
    {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return crc;
}

static uint16_t vision_read_u16_le(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static uint32_t vision_read_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) |
           ((uint32_t)data[3] << 24U);
}

static void vision_write_u16_le(uint8_t *buffer, uint16_t value)
{
    buffer[0] = (uint8_t)(value & 0xFFU);
    buffer[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

static void vision_write_u32_le(uint8_t *buffer, uint32_t value)
{
    buffer[0] = (uint8_t)(value & 0xFFU);
    buffer[1] = (uint8_t)((value >> 8U) & 0xFFU);
    buffer[2] = (uint8_t)((value >> 16U) & 0xFFU);
    buffer[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

static int16_t vision_encode_angle_fixed100(float angle_deg)
{
    float scaled = angle_deg * VISION_ANGLE_SCALE;

    if (scaled > 32767.0f)
    {
        scaled = 32767.0f;
    }
    else if (scaled < -32768.0f)
    {
        scaled = -32768.0f;
    }

    return (int16_t)scaled;
}

static float vision_decode_angle_fixed100(int16_t angle_fixed)
{
    return ((float)angle_fixed) / VISION_ANGLE_SCALE;
}

static void vision_publish_command_mailbox_internal(const vision_protocol_state_t *state)
{
    vision_command_mailbox_t snapshot = {0};

    if (state == NULL)
    {
        return;
    }

    snapshot.requested_mode = state->requested_mode;
    snapshot.feedback_enabled = state->feedback_enabled;
    snapshot.yaw_error_deg = state->yaw_error_deg;
    snapshot.pitch_error_deg = state->pitch_error_deg;
    snapshot.command_timestamp = state->command_timestamp;
    snapshot.last_valid_packet_tick = state->last_valid_packet_tick;
    snapshot.mode_sequence = state->mode_sequence;
    snapshot.auto_aim_sequence = state->auto_aim_sequence;

    taskENTER_CRITICAL();
    g_vision_command_mailbox = snapshot;
    taskEXIT_CRITICAL();
}

bool vision_read_command_mailbox(vision_command_mailbox_t *out)
{
    if (out == NULL)
    {
        return false;
    }

    taskENTER_CRITICAL();
    *out = g_vision_command_mailbox;
    taskEXIT_CRITICAL();
    return true;
}

void vision_publish_feedback_snapshot(const gimbal_feedback_snapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    g_gimbal_feedback_snapshot = *snapshot;
    taskEXIT_CRITICAL();
}

static void vision_read_feedback_snapshot(gimbal_feedback_snapshot_t *out)
{
    if (out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *out = g_gimbal_feedback_snapshot;
    taskEXIT_CRITICAL();
}

static void vision_parser_reset(vision_rx_parser_t *parser)
{
    if (parser == NULL)
    {
        return;
    }

    memset(parser, 0, sizeof(*parser));
    parser->state = VISION_RX_WAIT_SOF_0;
}

static void vision_parser_resync(vision_rx_parser_t *parser, uint8_t current_byte)
{
    vision_parser_reset(parser);

    if (current_byte == VISION_FRAME_SOF_0)
    {
        parser->frame_buffer[0] = current_byte;
        parser->state = VISION_RX_WAIT_SOF_1;
    }
}

static bool vision_parser_push_byte(vision_rx_parser_t *parser, uint8_t byte, vision_packet_t *packet)
{
    if (parser == NULL || packet == NULL)
    {
        return false;
    }

    switch (parser->state)
    {
        case VISION_RX_WAIT_SOF_0:
            if (byte == VISION_FRAME_SOF_0)
            {
                parser->frame_buffer[0] = byte;
                parser->state = VISION_RX_WAIT_SOF_1;
            }
            break;

        case VISION_RX_WAIT_SOF_1:
            if (byte == VISION_FRAME_SOF_1)
            {
                parser->frame_buffer[1] = byte;
                parser->state = VISION_RX_WAIT_LEN;
            }
            else
            {
                vision_parser_resync(parser, byte);
            }
            break;

        case VISION_RX_WAIT_LEN:
            if (byte <= VISION_PROTOCOL_MAX_DATA_LEN)
            {
                parser->payload_length = byte;
                parser->payload_index = 0U;
                parser->frame_buffer[2] = byte;
                parser->state = VISION_RX_WAIT_CMD;
            }
            else
            {
                vision_parser_resync(parser, byte);
            }
            break;

        case VISION_RX_WAIT_CMD:
            parser->frame_buffer[3] = byte;
            if (parser->payload_length == 0U)
            {
                parser->state = VISION_RX_WAIT_CRC_0;
            }
            else
            {
                parser->state = VISION_RX_WAIT_DATA;
            }
            break;

        case VISION_RX_WAIT_DATA:
            parser->frame_buffer[4U + parser->payload_index] = byte;
            ++parser->payload_index;
            if (parser->payload_index >= parser->payload_length)
            {
                parser->state = VISION_RX_WAIT_CRC_0;
            }
            break;

        case VISION_RX_WAIT_CRC_0:
            parser->received_crc = byte;
            parser->state = VISION_RX_WAIT_CRC_1;
            break;

        case VISION_RX_WAIT_CRC_1:
            parser->received_crc |= (uint16_t)((uint16_t)byte << 8U);
            parser->state = VISION_RX_WAIT_EOF_0;
            break;

        case VISION_RX_WAIT_EOF_0:
            if (byte == VISION_FRAME_EOF_0)
            {
                parser->state = VISION_RX_WAIT_EOF_1;
            }
            else
            {
                vision_parser_resync(parser, byte);
            }
            break;

        case VISION_RX_WAIT_EOF_1:
        {
            const uint16_t crc_length = (uint16_t)(4U + parser->payload_length);
            const uint16_t expected_crc = vision_crc16_modbus(parser->frame_buffer, crc_length);

            if (byte != VISION_FRAME_EOF_1 || expected_crc != parser->received_crc)
            {
                vision_parser_resync(parser, byte);
                return false;
            }

            packet->cmd = parser->frame_buffer[3];
            packet->data_length = parser->payload_length;
            if (packet->data_length > 0U)
            {
                memcpy(packet->data, &parser->frame_buffer[4], packet->data_length);
            }

            vision_parser_reset(parser);
            return true;
        }

        default:
            vision_parser_reset(parser);
            break;
    }

    return false;
}

static bool vision_set_requested_mode(vision_protocol_state_t *state, gimbal_mode_t mode)
{
    if (state == NULL)
    {
        return false;
    }

    if (state->requested_mode == mode)
    {
        return false;
    }

    state->requested_mode = mode;
    ++state->mode_sequence;
    return true;
}

static void vision_queue_frame(const uint8_t *frame, uint16_t length, vision_tx_priority_t priority)
{
    if (frame == NULL || length == 0U || length > VISION_PROTOCOL_MAX_FRAME_LEN)
    {
        return;
    }

    if (g_pending_tx_length != 0U && priority < g_pending_tx_priority)
    {
        return;
    }

    memcpy(g_pending_tx_frame, frame, length);
    g_pending_tx_length = length;
    g_pending_tx_priority = priority;
}

static void vision_try_send_pending_frame(void)
{
    if (g_pending_tx_length == 0U)
    {
        return;
    }

    if (bsp_usb_transmit(g_pending_tx_frame, g_pending_tx_length))
    {
        g_pending_tx_length = 0U;
        g_pending_tx_priority = VISION_TX_PRIORITY_NONE;
    }
}

static uint16_t vision_build_frame(uint8_t cmd, const uint8_t *payload, uint8_t payload_length, uint8_t *out_frame)
{
    const uint16_t crc_input_length = (uint16_t)(4U + payload_length);
    const uint16_t frame_length = (uint16_t)(crc_input_length + 4U);
    uint16_t crc = 0U;

    if (out_frame == NULL || payload_length > VISION_PROTOCOL_MAX_DATA_LEN)
    {
        return 0U;
    }

    out_frame[0] = VISION_FRAME_SOF_0;
    out_frame[1] = VISION_FRAME_SOF_1;
    out_frame[2] = payload_length;
    out_frame[3] = cmd;

    if (payload_length > 0U && payload != NULL)
    {
        memcpy(&out_frame[4], payload, payload_length);
    }

    crc = vision_crc16_modbus(out_frame, crc_input_length);
    vision_write_u16_le(&out_frame[4U + payload_length], crc);
    out_frame[6U + payload_length] = VISION_FRAME_EOF_0;
    out_frame[7U + payload_length] = VISION_FRAME_EOF_1;
    return frame_length;
}

static uint16_t vision_build_status_frame(const gimbal_feedback_snapshot_t *feedback, uint8_t *out_frame)
{
    uint8_t payload[VISION_STATUS_PAYLOAD_LEN] = {0};

    if (feedback == NULL || out_frame == NULL)
    {
        return 0U;
    }

    vision_write_u16_le(&payload[0], (uint16_t)vision_encode_angle_fixed100(feedback->yaw_deg));
    vision_write_u16_le(&payload[2], (uint16_t)vision_encode_angle_fixed100(feedback->pitch_deg));
    vision_write_u16_le(&payload[4], (uint16_t)vision_encode_angle_fixed100(feedback->roll_deg));
    vision_write_u32_le(&payload[6], feedback->timestamp);
    payload[10] = (uint8_t)feedback->current_mode;
    payload[11] = 0U;
    return vision_build_frame(VISION_PACKET_STATUS, payload, VISION_STATUS_PAYLOAD_LEN, out_frame);
}

static uint16_t vision_build_lock_frame(uint32_t timestamp, vision_lock_reason_t reason, uint8_t *out_frame)
{
    uint8_t payload[VISION_LOCK_PAYLOAD_LEN] = {0};

    if (out_frame == NULL)
    {
        return 0U;
    }

    vision_write_u32_le(&payload[0], timestamp);
    payload[4] = (uint8_t)reason;
    return vision_build_frame(VISION_PACKET_LOCKED, payload, VISION_LOCK_PAYLOAD_LEN, out_frame);
}

static void vision_enter_lock_mode(vision_protocol_state_t *state, vision_lock_reason_t reason)
{
    if (state == NULL || state->disable_latched)
    {
        return;
    }

    if (vision_set_requested_mode(state, GIMBAL_MODE_LOCK_PROTECT))
    {
        state->lock_report_pending = true;
        state->pending_lock_reason = reason;
    }
}

static void vision_latch_disable_mode(vision_protocol_state_t *state)
{
    if (state == NULL || state->disable_latched)
    {
        return;
    }

    state->disable_latched = true;
    (void)vision_set_requested_mode(state, GIMBAL_MODE_DISABLE);
}

static void vision_handle_packet(vision_protocol_state_t *state, const vision_packet_t *packet, uint32_t now_tick)
{
    bool handled = false;

    if (state == NULL || packet == NULL)
    {
        return;
    }

    switch ((vision_packet_cmd_t)packet->cmd)
    {
        case VISION_PACKET_ENABLE_STREAM:
            if (packet->data_length == 0U)
            {
                state->feedback_enabled = true;
                handled = true;
            }
            break;

        case VISION_PACKET_ENTER_SEARCH:
            if (packet->data_length == VISION_SEARCH_PAYLOAD_LEN && !state->disable_latched)
            {
                state->command_timestamp = vision_read_u32_le(packet->data);
                state->last_search_packet_tick = now_tick;
                (void)vision_set_requested_mode(state, GIMBAL_MODE_SEARCH);
                handled = true;
            }
            break;

        case VISION_PACKET_AUTO_AIM:
            if (packet->data_length == VISION_AUTO_AIM_PAYLOAD_LEN && !state->disable_latched)
            {
                state->yaw_error_deg = vision_decode_angle_fixed100((int16_t)vision_read_u16_le(&packet->data[0]));
                state->pitch_error_deg = vision_decode_angle_fixed100((int16_t)vision_read_u16_le(&packet->data[2]));
                state->command_timestamp = vision_read_u32_le(&packet->data[4]);
                state->last_auto_aim_packet_tick = now_tick;
                ++state->auto_aim_sequence;
                (void)vision_set_requested_mode(state, GIMBAL_MODE_AUTO_AIM);
                handled = true;
            }
            break;

        case VISION_PACKET_LOCK:
            if (packet->data_length == 0U && !state->disable_latched)
            {
                vision_enter_lock_mode(state, VISION_LOCK_REASON_MANUAL);
                handled = true;
            }
            break;

        case VISION_PACKET_UNLOCK:
            if (packet->data_length == 0U && !state->disable_latched && state->requested_mode == GIMBAL_MODE_LOCK_PROTECT)
            {
                (void)vision_set_requested_mode(state, GIMBAL_MODE_STABLE);
                handled = true;
            }
            break;

        default:
            break;
    }

    if (handled)
    {
        state->last_valid_packet_tick = now_tick;
    }
}

static void vision_handle_timeouts(vision_protocol_state_t *state, const gimbal_feedback_snapshot_t *feedback, uint32_t now_tick)
{
    if (state == NULL)
    {
        return;
    }

    if (feedback != NULL && feedback->disable_active)
    {
        vision_latch_disable_mode(state);
        return;
    }

    if (state->disable_latched)
    {
        return;
    }

    if (state->requested_mode != GIMBAL_MODE_LOCK_PROTECT &&
        (now_tick - state->last_valid_packet_tick) >= pdMS_TO_TICKS(VISION_COMMUNICATION_TIMEOUT_MS))
    {
        vision_enter_lock_mode(state, VISION_LOCK_REASON_COMM_TIMEOUT);
        return;
    }

    if (state->requested_mode == GIMBAL_MODE_AUTO_AIM &&
        (now_tick - state->last_auto_aim_packet_tick) >= pdMS_TO_TICKS(VISION_AUTO_AIM_TIMEOUT_MS))
    {
        if (vision_set_requested_mode(state, GIMBAL_MODE_SEARCH))
        {
            state->last_search_packet_tick = now_tick;
        }
    }

    if (state->requested_mode == GIMBAL_MODE_SEARCH &&
        (now_tick - state->last_search_packet_tick) >= pdMS_TO_TICKS(VISION_SEARCH_TIMEOUT_MS))
    {
        (void)vision_set_requested_mode(state, GIMBAL_MODE_STABLE);
    }
}

void StartVisionTask03(void *argument)
{
    vision_protocol_state_t protocol_state = {
        .requested_mode = GIMBAL_MODE_STABLE
    };
    vision_rx_parser_t parser = {0};
    gimbal_feedback_snapshot_t feedback_snapshot = {0};
    uint8_t rx_buffer[VISION_PROTOCOL_RX_CHUNK_SIZE] = {0};
    uint8_t tx_frame[VISION_PROTOCOL_MAX_FRAME_LEN] = {0};

    (void)argument;

    bsp_usb_init();
    bsp_usb_clear_rx_buffer();
    vision_parser_reset(&parser);

    protocol_state.last_valid_packet_tick = xTaskGetTickCount();
    protocol_state.last_status_send_tick = protocol_state.last_valid_packet_tick;
    vision_publish_command_mailbox_internal(&protocol_state);

    for (;;)
    {
        const uint32_t now_tick = xTaskGetTickCount();
        (void)osSemaphoreAcquire(VisionBinarySemHandle, pdMS_TO_TICKS(VISION_TASK_POLL_PERIOD_MS));

        while (bsp_usb_has_data())
        {
            const uint16_t bytes_read = bsp_usb_get_rx_data(rx_buffer, sizeof(rx_buffer));

            for (uint16_t i = 0U; i < bytes_read; ++i)
            {
                vision_packet_t packet = {0};
                if (vision_parser_push_byte(&parser, rx_buffer[i], &packet))
                {
                    vision_handle_packet(&protocol_state, &packet, xTaskGetTickCount());
                }
            }
        }

        vision_read_feedback_snapshot(&feedback_snapshot);
        vision_handle_timeouts(&protocol_state, &feedback_snapshot, xTaskGetTickCount());
        vision_publish_command_mailbox_internal(&protocol_state);

        if (protocol_state.lock_report_pending)
        {
            const uint16_t frame_length = vision_build_lock_frame(xTaskGetTickCount(),
                                                                  protocol_state.pending_lock_reason,
                                                                  tx_frame);
            if (frame_length > 0U)
            {
                vision_queue_frame(tx_frame, frame_length, VISION_TX_PRIORITY_LOCK);
            }
            protocol_state.lock_report_pending = false;
        }

        if (protocol_state.feedback_enabled &&
            (now_tick - protocol_state.last_status_send_tick) >= pdMS_TO_TICKS(VISION_STATUS_SEND_PERIOD_MS))
        {
            const uint16_t frame_length = vision_build_status_frame(&feedback_snapshot, tx_frame);
            if (frame_length > 0U)
            {
                vision_queue_frame(tx_frame, frame_length, VISION_TX_PRIORITY_STATUS);
            }
            protocol_state.last_status_send_tick = now_tick;
        }

        vision_try_send_pending_frame();
    }
}
