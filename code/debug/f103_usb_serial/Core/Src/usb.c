#include "usb.h"

#include <stddef.h>
#include <string.h>

#include "LED.h"

#define USB_APP_SOF_0               0xAAU
#define USB_APP_SOF_1               0x55U
#define USB_APP_EOF_0               0x5AU
#define USB_APP_EOF_1               0xA5U
#define USB_APP_FRAME_OVERHEAD      8U
#define USB_APP_MAX_DATA_LEN        16U
#define USB_APP_RX_BUFFER_SIZE      256U
#define USB_APP_LED_ON_TIME_MS      500U
#define USB_APP_SEARCH_LEN          4U
#define USB_APP_AUTO_AIM_LEN        10U

typedef struct
{
    uint8_t rx_buffer[USB_APP_RX_BUFFER_SIZE];
    uint16_t rx_length;
} UsbAppContext_t;

static UsbAppContext_t g_usb;

static uint16_t USB_Crc16(const uint8_t *data, uint16_t length);
static uint16_t USB_ReadU16Le(const uint8_t *src);
static uint8_t USB_IsSupportedCommand(uint8_t cmd, uint8_t length);
static void USB_HandleFrame(uint8_t cmd, uint8_t length, uint32_t now_ms);
static void USB_ProcessRxBuffer(uint32_t now_ms);

void USB_AppInit(void)
{
    memset(&g_usb, 0, sizeof(g_usb));
}

void USB_AppTask(uint32_t now_ms)
{
    (void)now_ms;
}

void USB_AppOnRx(const uint8_t *buf, uint16_t len, uint32_t now_ms)
{
    if ((buf == NULL) || (len == 0U))
    {
        return;
    }

    if (len >= USB_APP_RX_BUFFER_SIZE)
    {
        uint16_t tail_len = USB_APP_RX_BUFFER_SIZE - 1U;
        memcpy(g_usb.rx_buffer, &buf[len - tail_len], tail_len);
        g_usb.rx_length = tail_len;
    }
    else
    {
        if ((g_usb.rx_length + len) > USB_APP_RX_BUFFER_SIZE)
        {
            g_usb.rx_length = 0U;
        }

        memcpy(&g_usb.rx_buffer[g_usb.rx_length], buf, len);
        g_usb.rx_length = (uint16_t)(g_usb.rx_length + len);
    }

    USB_ProcessRxBuffer(now_ms);
}

static uint16_t USB_Crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t bit;

    for (i = 0U; i < length; ++i)
    {
        crc ^= data[i];
        for (bit = 0U; bit < 8U; ++bit)
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

static uint16_t USB_ReadU16Le(const uint8_t *src)
{
    return (uint16_t)((uint16_t)src[0] | ((uint16_t)src[1] << 8U));
}

static uint8_t USB_IsSupportedCommand(uint8_t cmd, uint8_t length)
{
    switch (cmd)
    {
    case USB_CMD_VISION_ENABLE_STREAM:
    case USB_CMD_VISION_LOCK:
    case USB_CMD_VISION_UNLOCK:
        return (length == 0U) ? 1U : 0U;

    case USB_CMD_VISION_SEARCH:
        return (length == USB_APP_SEARCH_LEN) ? 1U : 0U;

    case USB_CMD_VISION_AUTO_AIM:
        return (length == USB_APP_AUTO_AIM_LEN) ? 1U : 0U;

    default:
        return 0U;
    }
}

static void USB_HandleFrame(uint8_t cmd, uint8_t length, uint32_t now_ms)
{
    if (USB_IsSupportedCommand(cmd, length) != 0U)
    {
        LED_TriggerPulse(now_ms, USB_APP_LED_ON_TIME_MS);
    }
}

static void USB_ProcessRxBuffer(uint32_t now_ms)
{
    uint16_t index = 0U;

    while ((g_usb.rx_length - index) >= USB_APP_FRAME_OVERHEAD)
    {
        uint16_t remaining;
        uint8_t data_len;
        uint16_t frame_len;
        uint16_t expected_crc;
        uint16_t actual_crc;

        if ((g_usb.rx_buffer[index] != USB_APP_SOF_0) ||
            (g_usb.rx_buffer[index + 1U] != USB_APP_SOF_1))
        {
            ++index;
            continue;
        }

        data_len = g_usb.rx_buffer[index + 2U];
        frame_len = (uint16_t)(USB_APP_FRAME_OVERHEAD + data_len);
        remaining = (uint16_t)(g_usb.rx_length - index);

        if ((data_len > USB_APP_MAX_DATA_LEN) || (frame_len > USB_APP_RX_BUFFER_SIZE))
        {
            ++index;
            continue;
        }

        if (remaining < frame_len)
        {
            break;
        }

        if ((g_usb.rx_buffer[index + frame_len - 2U] != USB_APP_EOF_0) ||
            (g_usb.rx_buffer[index + frame_len - 1U] != USB_APP_EOF_1))
        {
            ++index;
            continue;
        }

        expected_crc = USB_ReadU16Le(&g_usb.rx_buffer[index + 4U + data_len]);
        actual_crc = USB_Crc16(&g_usb.rx_buffer[index], (uint16_t)(4U + data_len));
        if (expected_crc != actual_crc)
        {
            ++index;
            continue;
        }

        USB_HandleFrame(g_usb.rx_buffer[index + 3U], data_len, now_ms);
        index = (uint16_t)(index + frame_len);
    }

    if (index > 0U)
    {
        if (index < g_usb.rx_length)
        {
            memmove(g_usb.rx_buffer, &g_usb.rx_buffer[index], g_usb.rx_length - index);
        }
        g_usb.rx_length = (uint16_t)(g_usb.rx_length - index);
    }
}
