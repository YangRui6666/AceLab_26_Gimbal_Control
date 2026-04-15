#include "bsp_usb.h"

#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "stm32f4xx.h"
#include "usbd_cdc_if.h"
#define USB_RX_BUFFER_SIZE 1024U
#define USB_TX_BUFFER_SIZE APP_TX_DATA_SIZE
#define USB_SOF_BYTE_0 0xAAU
#define USB_SOF_BYTE_1 0x55U
#define USB_EOF_BYTE_0 0x5AU
#define USB_EOF_BYTE_1 0xA5U
#define USB_FRAME_OVERHEAD 8U
#define USB_FRAME_MAX_SIZE (255U + USB_FRAME_OVERHEAD)
#define USB_MAX_DELTA_YAW_DEG 60.0f
#define USB_MAX_DELTA_PITCH_DEG 40.0f

typedef struct
{
    uint8_t data[USB_RX_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} usb_ring_buffer_t;

static usb_ring_buffer_t usb_rx_buffer;
static volatile bool usb_tx_enabled = false;
static uint8_t usb_tx_buffer[USB_TX_BUFFER_SIZE];

static uint32_t usb_lock(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void usb_unlock(uint32_t primask)
{
    if ((primask & 1U) == 0U)
    {
        __enable_irq();
    }
}

static uint16_t usb_ring_count_locked(void)
{
    if (usb_rx_buffer.head >= usb_rx_buffer.tail)
    {
        return usb_rx_buffer.head - usb_rx_buffer.tail;
    }

    return (USB_RX_BUFFER_SIZE - usb_rx_buffer.tail) + usb_rx_buffer.head;
}

static uint16_t usb_ring_count(void)
{
    uint16_t count;
    uint32_t primask = usb_lock();

    count = usb_ring_count_locked();

    usb_unlock(primask);
    return count;
}

static bool usb_ring_peek(uint16_t offset, uint8_t *value)
{
    bool ok = false;
    uint32_t primask = usb_lock();

    if ((value != NULL) && (offset < usb_ring_count_locked()))
    {
        uint16_t index = usb_rx_buffer.tail + offset;

        if (index >= USB_RX_BUFFER_SIZE)
        {
            index -= USB_RX_BUFFER_SIZE;
        }

        *value = usb_rx_buffer.data[index];
        ok = true;
    }

    usb_unlock(primask);
    return ok;
}

static bool usb_ring_copy(uint16_t offset, uint8_t *dst, uint16_t len)
{
    bool ok = false;
    uint32_t primask = usb_lock();

    if ((dst != NULL) && ((uint32_t)offset + len <= usb_ring_count_locked()))
    {
        uint16_t index = usb_rx_buffer.tail + offset;
        uint16_t i;

        if (index >= USB_RX_BUFFER_SIZE)
        {
            index -= USB_RX_BUFFER_SIZE;
        }

        for (i = 0; i < len; i++)
        {
            dst[i] = usb_rx_buffer.data[index];
            index++;

            if (index >= USB_RX_BUFFER_SIZE)
            {
                index = 0U;
            }
        }

        ok = true;
    }

    usb_unlock(primask);
    return ok;
}

static void usb_ring_drop(uint16_t len)
{
    uint32_t primask = usb_lock();
    uint16_t count = usb_ring_count_locked();

    if (len > count)
    {
        len = count;
    }

    usb_rx_buffer.tail = (usb_rx_buffer.tail + len) % USB_RX_BUFFER_SIZE;

    usb_unlock(primask);
}

static uint16_t usb_crc16_modbus(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;

    for (i = 0; i < len; i++)
    {
        uint8_t bit;

        crc ^= data[i];

        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc = (crc >> 1U) ^ 0xA001U;
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return crc;
}

static uint16_t usb_read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static uint32_t usb_read_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0]
         | ((uint32_t)data[1] << 8U)
         | ((uint32_t)data[2] << 16U)
         | ((uint32_t)data[3] << 24U);
}

static void usb_write_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

static void usb_write_u32_le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
    data[2] = (uint8_t)((value >> 16U) & 0xFFU);
    data[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

static bool usb_build_frame(uint8_t cmd, const uint8_t *payload, uint8_t payload_len, uint8_t *frame, uint16_t *frame_len)
{
    uint16_t crc;
    uint16_t total_len;

    if ((frame == NULL) || (frame_len == NULL))
    {
        return false;
    }

    total_len = USB_FRAME_OVERHEAD + payload_len;
    if (total_len > USB_TX_BUFFER_SIZE)
    {
        return false;
    }

    frame[0] = USB_SOF_BYTE_0;
    frame[1] = USB_SOF_BYTE_1;
    frame[2] = payload_len;
    frame[3] = cmd;

    if ((payload_len > 0U) && (payload != NULL))
    {
        memcpy(&frame[4], payload, payload_len);
    }

    crc = usb_crc16_modbus(frame, (uint16_t)(4U + payload_len));
    usb_write_u16_le(&frame[4U + payload_len], crc);
    frame[6U + payload_len] = USB_EOF_BYTE_0;
    frame[7U + payload_len] = USB_EOF_BYTE_1;

    *frame_len = total_len;
    return true;
}

static bool usb_transmit_bytes(const uint8_t *data, uint16_t len)
{
    uint8_t status;
    uint32_t primask;

    if ((data == NULL) || (len == 0U) || (len > USB_TX_BUFFER_SIZE))
    {
        return false;
    }

    primask = usb_lock();
    memcpy(usb_tx_buffer, data, len);
    status = CDC_Transmit_FS(usb_tx_buffer, len);
    usb_unlock(primask);

    return status == USBD_OK;
}

static bool usb_decode_ctrl_msg(uint8_t cmd, const uint8_t *payload, uint8_t payload_len, CtrlMsg_t *msg)
{
    if (msg == NULL)
    {
        return false;
    }

    msg->type = CTRL_MSG_NONE;
    msg->time_stamp = 0U;
    msg->delta_yaw = 0.0f;
    msg->delta_pitch = 0.0f;

    switch (cmd)
    {
        case 0x82:
            if (payload_len != 0U)
            {
                return false;
            }

            usb_tx_enabled = true;
            return false;

        case 0x83:
            if (payload_len != 4U)
            {
                return false;
            }

            msg->type = CTRL_MSG_ENTER_SEARCH;
            msg->time_stamp = usb_read_u32_le(payload);
            return true;

        case 0x84:
            if (payload_len != 10U)
            {
                return false;
            }

            msg->type = CTRL_MSG_AUTO_AIM_DELTA;
            msg->delta_yaw = (float)(int16_t)usb_read_u16_le(&payload[0]) / 100.0f;
            msg->delta_pitch = (float)(int16_t)usb_read_u16_le(&payload[2]) / 100.0f;
            msg->time_stamp = usb_read_u32_le(&payload[4]);

            if ((fabsf(msg->delta_yaw) > USB_MAX_DELTA_YAW_DEG) ||
                (fabsf(msg->delta_pitch) > USB_MAX_DELTA_PITCH_DEG))
            {
                return false;
            }

            return true;

        case 0x87:
            if (payload_len != 0U)
            {
                return false;
            }

            msg->type = CTRL_MSG_ENTER_LOCK;
            return true;

        case 0x88:
            if (payload_len != 0U)
            {
                return false;
            }

            msg->type = CTRL_MSG_EXIT_LOCK;
            return true;

        default:
            return false;
    }
}

bool bsp_usb_init(void)
{
    uint32_t primask = usb_lock();

    memset((void *)&usb_rx_buffer, 0, sizeof(usb_rx_buffer));
    usb_tx_enabled = false;

    usb_unlock(primask);
    return true;
}

bool bsp_usb_rx_push(const uint8_t *data, uint16_t len)
{
    uint16_t i;
    uint32_t primask;

    if ((data == NULL) || (len == 0U))
    {
        return false;
    }

    primask = usb_lock();

    for (i = 0; i < len; i++)
    {
        uint16_t next_head = usb_rx_buffer.head + 1U;

        if (next_head >= USB_RX_BUFFER_SIZE)
        {
            next_head = 0U;
        }

        if (next_head == usb_rx_buffer.tail)
        {
            usb_rx_buffer.tail++;
            if (usb_rx_buffer.tail >= USB_RX_BUFFER_SIZE)
            {
                usb_rx_buffer.tail = 0U;
            }
        }

        usb_rx_buffer.data[usb_rx_buffer.head] = data[i];
        usb_rx_buffer.head = next_head;
    }

    usb_unlock(primask);
    return true;
}

bool bsp_usb_parse_next_ctrl_msg(CtrlMsg_t *msg)
{
    uint8_t frame[USB_FRAME_MAX_SIZE];

    for (;;)
    {
        uint16_t available = usb_ring_count();
        uint16_t sof_offset = 0U;
        bool found_sof = false;

        if (available < USB_FRAME_OVERHEAD)
        {
            return false;
        }

        while ((uint16_t)(sof_offset + 1U) < available)
        {
            uint8_t b0;
            uint8_t b1;

            if (!usb_ring_peek(sof_offset, &b0) || !usb_ring_peek((uint16_t)(sof_offset + 1U), &b1))
            {
                return false;
            }

            if ((b0 == USB_SOF_BYTE_0) && (b1 == USB_SOF_BYTE_1))
            {
                found_sof = true;
                break;
            }

            sof_offset++;
        }

        if (!found_sof)
        {
            usb_ring_drop((uint16_t)(available - 1U));
            return false;
        }

        if (sof_offset > 0U)
        {
            usb_ring_drop(sof_offset);
            continue;
        }

        if (!usb_ring_copy(0U, frame, 4U))
        {
            return false;
        }

        {
            uint8_t payload_len = frame[2];
            uint16_t total_len = (uint16_t)payload_len + USB_FRAME_OVERHEAD;
            uint16_t frame_crc;
            uint16_t calc_crc;

            if (total_len > USB_FRAME_MAX_SIZE)
            {
                usb_ring_drop(1U);
                continue;
            }

            if (usb_ring_count() < total_len)
            {
                return false;
            }

            if (!usb_ring_copy(0U, frame, total_len))
            {
                return false;
            }

            if ((frame[total_len - 2U] != USB_EOF_BYTE_0) || (frame[total_len - 1U] != USB_EOF_BYTE_1))
            {
                usb_ring_drop(1U);
                continue;
            }

            frame_crc = usb_read_u16_le(&frame[4U + payload_len]);
            calc_crc = usb_crc16_modbus(frame, (uint16_t)(4U + payload_len));
            if (frame_crc != calc_crc)
            {
                usb_ring_drop(1U);
                continue;
            }

            usb_ring_drop(total_len);

            if (usb_decode_ctrl_msg(frame[3], &frame[4], payload_len, msg))
            {
                return true;
            }
        }
    }
}

bool usb_protocol_send_status(const usb_status_feedback_t *status)
{
    uint8_t payload[12];
    uint8_t frame[USB_FRAME_MAX_SIZE];
    uint16_t frame_len;

    if ((status == NULL) || !usb_tx_enabled)
    {
        return false;
    }

    usb_write_u16_le(&payload[0], (uint16_t)status->yaw_target);
    usb_write_u16_le(&payload[2], (uint16_t)status->pitch_target);
    usb_write_u16_le(&payload[4], (uint16_t)status->roll_target);
    usb_write_u32_le(&payload[6], status->time_stamp);
    payload[10] = status->mode;
    payload[11] = status->reserved;

    if (!usb_build_frame(0x03U, payload, sizeof(payload), frame, &frame_len))
    {
        return false;
    }

    return usb_transmit_bytes(frame, frame_len);
}

bool usb_protocol_send_lock_feedback(uint32_t time_stamp, uint8_t reason)
{
    uint8_t payload[5];
    uint8_t frame[USB_FRAME_MAX_SIZE];
    uint16_t frame_len;

    if (!usb_tx_enabled)
    {
        return false;
    }

    usb_write_u32_le(&payload[0], time_stamp);
    payload[4] = reason;

    if (!usb_build_frame(0x08U, payload, sizeof(payload), frame, &frame_len))
    {
        return false;
    }

    return usb_transmit_bytes(frame, frame_len);
}

bool usb_send_raw(const char *str)
{
    size_t len;

    if (str == NULL)
    {
        return false;
    }

    len = strlen(str);
    if ((len == 0U) || (len > USB_TX_BUFFER_SIZE))
    {
        return false;
    }

    return usb_transmit_bytes((const uint8_t *)str, (uint16_t)len);
}

bool usb_send_rawf(const char *fmt, ...)
{
    va_list args;
    int written;
    char text[USB_TX_BUFFER_SIZE];

    if (fmt == NULL)
    {
        return false;
    }

    va_start(args, fmt);
    written = vsnprintf(text, sizeof(text), fmt, args);
    va_end(args);

    if ((written < 0) || ((size_t)written >= sizeof(text)))
    {
        return false;
    }

    return usb_send_raw(text);
}
