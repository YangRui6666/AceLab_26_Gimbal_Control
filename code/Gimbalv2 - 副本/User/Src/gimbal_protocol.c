#include "gimbal_protocol.h"

#include "gimbal_config.h"
#include "gimbal_crc.h"
#include "usbd_cdc_if.h"

#include <string.h>

static uint8_t s_rx_ring[USB_RX_RING_BUFFER_SIZE];
static uint16_t s_rx_head = 0U;
static uint16_t s_rx_tail = 0U;
static uint16_t s_rx_count = 0U;

static uint16_t ring_next(uint16_t index)
{
    return (uint16_t)((index + 1U) % USB_RX_RING_BUFFER_SIZE);
}

static void ring_pop_one(void)
{
    if (s_rx_count > 0U) {
        s_rx_tail = ring_next(s_rx_tail);
        --s_rx_count;
    }
}

static bool ring_peek(uint16_t offset, uint8_t *value)
{
    uint16_t index = 0U;

    if ((value == NULL) || (offset >= s_rx_count)) {
        return false;
    }

    index = (uint16_t)((s_rx_tail + offset) % USB_RX_RING_BUFFER_SIZE);
    *value = s_rx_ring[index];
    return true;
}

static void ring_copy_out(uint8_t *dst, uint16_t len)
{
    uint16_t i = 0U;

    for (i = 0U; i < len; ++i) {
        (void)ring_peek(i, &dst[i]);
    }
}

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static uint32_t read_u32_le(const uint8_t *data)
{
    return ((uint32_t)data[0]) |
           ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) |
           ((uint32_t)data[3] << 24U);
}

static int16_t angle_to_centideg(float angle_deg)
{
    float scaled = angle_deg * 100.0f;

    if (scaled > 32767.0f) {
        scaled = 32767.0f;
    } else if (scaled < -32768.0f) {
        scaled = -32768.0f;
    }

    return (int16_t)scaled;
}

static bool extract_next_frame(uint8_t *frame, uint16_t *frame_len)
{
    uint8_t len_byte = 0U;
    uint16_t total_len = 0U;
    uint16_t expected_crc = 0U;
    uint16_t calculated_crc = 0U;

    while (s_rx_count >= 2U) {
        uint8_t sof0 = 0U;
        uint8_t sof1 = 0U;

        (void)ring_peek(0U, &sof0);
        (void)ring_peek(1U, &sof1);

        if (sof0 != PROTOCOL_SOF0) {
            ring_pop_one();
            continue;
        }

        if (sof1 != PROTOCOL_SOF1) {
            ring_pop_one();
            continue;
        }

        if (s_rx_count < 3U) {
            return false;
        }

        (void)ring_peek(2U, &len_byte);
        total_len = (uint16_t)len_byte + 8U;
        if ((len_byte > PROTOCOL_MAX_DATA_LEN) || (total_len > (PROTOCOL_MAX_DATA_LEN + 8U))) {
            ring_pop_one();
            continue;
        }

        if (s_rx_count < total_len) {
            return false;
        }

        ring_copy_out(frame, total_len);
        if ((frame[total_len - 2U] != PROTOCOL_EOF0) ||
            (frame[total_len - 1U] != PROTOCOL_EOF1)) {
            ring_pop_one();
            continue;
        }

        expected_crc = read_u16_le(&frame[4U + len_byte]);
        calculated_crc = gimbal_crc16_modbus(frame, (size_t)(4U + len_byte));
        if (expected_crc != calculated_crc) {
            ring_pop_one();
            continue;
        }

        while (total_len > 0U) {
            ring_pop_one();
            --total_len;
        }

        *frame_len = (uint16_t)len_byte + 8U;
        return true;
    }

    return false;
}

static bool send_frame(uint8_t cmd_id, const uint8_t *data, uint8_t len)
{
    uint8_t frame[PROTOCOL_MAX_DATA_LEN + 8U];
    uint16_t crc = 0U;
    uint16_t total_len = (uint16_t)len + 8U;

    if (len > PROTOCOL_MAX_DATA_LEN) {
        return false;
    }

    frame[0] = PROTOCOL_SOF0;
    frame[1] = PROTOCOL_SOF1;
    frame[2] = len;
    frame[3] = cmd_id;
    if ((data != NULL) && (len > 0U)) {
        memcpy(&frame[4], data, len);
    }

    crc = gimbal_crc16_modbus(frame, (size_t)(4U + len));
    frame[4U + len] = (uint8_t)(crc & 0xFFU);
    frame[5U + len] = (uint8_t)((crc >> 8U) & 0xFFU);
    frame[6U + len] = PROTOCOL_EOF0;
    frame[7U + len] = PROTOCOL_EOF1;

    return CDC_Transmit_FS(frame, total_len) == USBD_OK;
}

void gimbal_protocol_rx_reset(void)
{
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_rx_count = 0U;
}

void gimbal_protocol_rx_push_bytes(const uint8_t *data, uint16_t len)
{
    uint16_t i = 0U;

    if (data == NULL) {
        return;
    }

    for (i = 0U; i < len; ++i) {
        if (s_rx_count >= USB_RX_RING_BUFFER_SIZE) {
            ring_pop_one();
        }

        s_rx_ring[s_rx_head] = data[i];
        s_rx_head = ring_next(s_rx_head);
        ++s_rx_count;
    }
}

bool gimbal_protocol_process_next(CtrlMsg_t *out_msg, uint8_t *produced_msg)
{
    uint8_t frame[PROTOCOL_MAX_DATA_LEN + 8U];
    uint16_t frame_len = 0U;
    uint8_t data_len = 0U;
    uint8_t cmd_id = 0U;
    const uint8_t *data = NULL;

    if ((out_msg == NULL) || (produced_msg == NULL)) {
        return false;
    }

    *produced_msg = 0U;
    if (!extract_next_frame(frame, &frame_len)) {
        return false;
    }

    (void)frame_len;
    memset(out_msg, 0, sizeof(*out_msg));
    data_len = frame[2];
    cmd_id = frame[3];
    data = &frame[4];

    switch (cmd_id) {
    case CMD_ID_ENABLE_TELEMETRY:
        out_msg->type = CTRL_MSG_ENABLE_TELEMETRY;
        *produced_msg = 1U;
        break;

    case CMD_ID_ENTER_SEARCH:
        if (data_len >= 4U) {
            out_msg->type = CTRL_MSG_ENTER_SEARCH;
            out_msg->time_stamp_ms = read_u32_le(data);
            *produced_msg = 1U;
        }
        break;

    case CMD_ID_AUTO_AIM:
        if (data_len >= 10U) {
            out_msg->type = CTRL_MSG_AUTO_AIM_DELTA;
            out_msg->delta_yaw_deg = ((float)(int16_t)read_u16_le(&data[0])) / 100.0f;
            out_msg->delta_pitch_deg = ((float)(int16_t)read_u16_le(&data[2])) / 100.0f;
            out_msg->time_stamp_ms = read_u32_le(&data[4]);
            *produced_msg = 1U;
        }
        break;

    case CMD_ID_ENTER_LOCK:
        out_msg->type = CTRL_MSG_ENTER_LOCK;
        *produced_msg = 1U;
        break;

    case CMD_ID_EXIT_LOCK:
        out_msg->type = CTRL_MSG_EXIT_LOCK;
        *produced_msg = 1U;
        break;

    default:
        break;
    }

    return true;
}

bool gimbal_protocol_send_status(const AttitudeAngle *attitude,
                                 uint32_t time_stamp_ms,
                                 GimbalMode_e mode,
                                 bool telemetry_enabled)
{
    uint8_t payload[12U];
    int16_t yaw_centideg = 0;
    int16_t pitch_centideg = 0;
    int16_t roll_centideg = 0;

    if ((!telemetry_enabled) || (attitude == NULL)) {
        return false;
    }

    yaw_centideg = angle_to_centideg(attitude->yaw_deg);
    pitch_centideg = angle_to_centideg(attitude->pitch_deg);
    roll_centideg = angle_to_centideg(attitude->roll_deg);

    payload[0] = (uint8_t)(yaw_centideg & 0xFFU);
    payload[1] = (uint8_t)((yaw_centideg >> 8U) & 0xFFU);
    payload[2] = (uint8_t)(pitch_centideg & 0xFFU);
    payload[3] = (uint8_t)((pitch_centideg >> 8U) & 0xFFU);
    payload[4] = (uint8_t)(roll_centideg & 0xFFU);
    payload[5] = (uint8_t)((roll_centideg >> 8U) & 0xFFU);
    payload[6] = (uint8_t)(time_stamp_ms & 0xFFU);
    payload[7] = (uint8_t)((time_stamp_ms >> 8U) & 0xFFU);
    payload[8] = (uint8_t)((time_stamp_ms >> 16U) & 0xFFU);
    payload[9] = (uint8_t)((time_stamp_ms >> 24U) & 0xFFU);
    payload[10] = (uint8_t)mode;
    payload[11] = 0U;

    return send_frame(CMD_ID_STATUS_FEEDBACK, payload, (uint8_t)sizeof(payload));
}

bool gimbal_protocol_send_lock_feedback(LockReason_e reason, uint32_t time_stamp_ms)
{
    uint8_t payload[5U];

    payload[0] = (uint8_t)(time_stamp_ms & 0xFFU);
    payload[1] = (uint8_t)((time_stamp_ms >> 8U) & 0xFFU);
    payload[2] = (uint8_t)((time_stamp_ms >> 16U) & 0xFFU);
    payload[3] = (uint8_t)((time_stamp_ms >> 24U) & 0xFFU);
    payload[4] = (uint8_t)reason;

    return send_frame(CMD_ID_LOCK_FEEDBACK, payload, (uint8_t)sizeof(payload));
}
