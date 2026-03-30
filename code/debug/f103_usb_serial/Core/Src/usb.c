#include "usb.h"

#include <stddef.h>
#include <string.h>

#include "LED.h"
#include "arm_math.h"
#include "main.h"
#include "usbd_cdc_if.h"

#define USB_APP_SOF_0                       0xAAU
#define USB_APP_SOF_1                       0x55U
#define USB_APP_EOF_0                       0x5AU
#define USB_APP_EOF_1                       0xA5U
#define USB_APP_FRAME_OVERHEAD              8U
#define USB_APP_MAX_DATA_LEN                16U
#define USB_APP_MAX_FRAME_LEN               (USB_APP_FRAME_OVERHEAD + USB_APP_MAX_DATA_LEN)
#define USB_APP_RX_BUFFER_SIZE              256U
#define USB_APP_BOOT_TIMEOUT_MS             15000U
#define USB_APP_SEARCH_TIMEOUT_MS           1000U
#define USB_APP_AUTO_AIM_TIMEOUT_MS         200U
#define USB_APP_FEEDBACK_PERIOD_MS          10U
#define USB_APP_SEARCH_KEEPALIVE_LEN        0U
#define USB_APP_DISABLE_KEEPALIVE_LEN       0U
#define USB_APP_UNLOCK_KEEPALIVE_LEN        0U
#define USB_APP_HANDSHAKE_REQ_LEN           0U
#define USB_APP_HANDSHAKE_ACK_LEN           4U
#define USB_APP_AUTO_AIM_LEN                10U
#define USB_APP_FEEDBACK_LEN                12U
#define USB_APP_WAVEFORM_PERIOD_MS          4000.0f
#define USB_APP_YAW_AMPLITUDE_DEG_X1000     30000.0f
#define USB_APP_PITCH_AMPLITUDE_DEG_X1000   15000.0f
#define USB_APP_PI                          3.14159265358979323846f

typedef struct
{
    uint8_t rx_buffer[USB_APP_RX_BUFFER_SIZE];
    uint8_t tx_buffer[USB_APP_MAX_FRAME_LEN];
    uint16_t rx_length;
    uint32_t start_ms;
    uint32_t last_host_rx_ms;
    uint32_t last_feedback_ms;
    uint32_t waveform_start_ms;
    CloudMode_t mode;
    UsbAimControl_t last_aim_command;
    uint8_t boot_message_sent;
    uint8_t handshake_reply_pending;
    uint8_t disable_notification_pending;
    uint8_t stream_enabled;
} UsbAppContext_t;

static UsbAppContext_t g_usb;

static void USB_SetLedPattern(void);
static void USB_SetMode(CloudMode_t mode);
static void USB_EnterDisabled(void);
static uint16_t USB_Crc16(const uint8_t *data, uint16_t length);
static void USB_WriteU16Le(uint8_t *dst, uint16_t value);
static void USB_WriteU32Le(uint8_t *dst, uint32_t value);
static int16_t USB_ReadI16Le(const uint8_t *src);
static uint16_t USB_ReadU16Le(const uint8_t *src);
static uint32_t USB_ReadU32Le(const uint8_t *src);
static uint8_t USB_SendFrame(uint8_t cmd, const uint8_t *data, uint8_t length);
static uint8_t USB_SendBootMessage(void);
static uint8_t USB_SendHandshakeAck(uint32_t now_ms);
static uint8_t USB_SendFeedback(uint32_t now_ms);
static uint8_t USB_SendDisableNotification(void);
static void USB_HandleFrame(uint8_t cmd, const uint8_t *data, uint8_t length, uint32_t now_ms);
static void USB_ProcessRxBuffer(uint32_t now_ms);

void USB_AppInit(void)
{
    memset(&g_usb, 0, sizeof(g_usb));
    g_usb.start_ms = HAL_GetTick();
    g_usb.last_feedback_ms = g_usb.start_ms;
    g_usb.waveform_start_ms = g_usb.start_ms;
    g_usb.mode = MODE_STANDBY;
    USB_SetLedPattern();
}

void USB_AppTask(uint32_t now_ms)
{
    if (g_usb.disable_notification_pending != 0U)
    {
        if (USB_SendDisableNotification() != 0U)
        {
            g_usb.disable_notification_pending = 0U;
        }
        return;
    }

    if (g_usb.handshake_reply_pending != 0U)
    {
        if (USB_SendHandshakeAck(now_ms) != 0U)
        {
            g_usb.handshake_reply_pending = 0U;
            g_usb.stream_enabled = 1U;
            g_usb.last_host_rx_ms = now_ms;
            USB_SetMode(MODE_SEARCH);
            g_usb.last_feedback_ms = now_ms - USB_APP_FEEDBACK_PERIOD_MS;
        }
        return;
    }

    if ((g_usb.boot_message_sent == 0U) && (g_usb.stream_enabled == 0U))
    {
        if (USB_SendBootMessage() != 0U)
        {
            g_usb.boot_message_sent = 1U;
        }
    }

    if ((g_usb.stream_enabled == 0U) && ((now_ms - g_usb.start_ms) >= USB_APP_BOOT_TIMEOUT_MS))
    {
        USB_EnterDisabled();
    }

    if (g_usb.mode == MODE_SEARCH)
    {
        if ((now_ms - g_usb.last_host_rx_ms) > USB_APP_SEARCH_TIMEOUT_MS)
        {
            USB_EnterDisabled();
        }
    }
    else if (g_usb.mode == MODE_AUTO_AIM)
    {
        if ((now_ms - g_usb.last_host_rx_ms) > USB_APP_AUTO_AIM_TIMEOUT_MS)
        {
            USB_EnterDisabled();
        }
    }

    if ((g_usb.stream_enabled != 0U) &&
        (g_usb.mode != MODE_DISABLED) &&
        ((now_ms - g_usb.last_feedback_ms) >= USB_APP_FEEDBACK_PERIOD_MS))
    {
        if (USB_SendFeedback(now_ms) != 0U)
        {
            g_usb.last_feedback_ms = now_ms;
        }
    }
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

static void USB_SetLedPattern(void)
{
    LedPattern_t pattern = LED_PATTERN_OFF;

    if (g_usb.mode == MODE_DISABLED)
    {
        pattern = LED_PATTERN_DISABLED;
    }
    else if (g_usb.stream_enabled != 0U)
    {
        pattern = LED_PATTERN_USB_ACTIVE;
    }
    else
    {
        pattern = LED_PATTERN_WAIT_CONNECT;
    }

    LED_SetPattern(pattern);
}

static void USB_SetMode(CloudMode_t mode)
{
    g_usb.mode = mode;
    USB_SetLedPattern();
}

static void USB_EnterDisabled(void)
{
    if (g_usb.mode == MODE_DISABLED)
    {
        return;
    }

    USB_SetMode(MODE_DISABLED);
    g_usb.disable_notification_pending = 1U;
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

static void USB_WriteU16Le(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

static void USB_WriteU32Le(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8U) & 0xFFU);
    dst[2] = (uint8_t)((value >> 16U) & 0xFFU);
    dst[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

static int16_t USB_ReadI16Le(const uint8_t *src)
{
    return (int16_t)USB_ReadU16Le(src);
}

static uint16_t USB_ReadU16Le(const uint8_t *src)
{
    return (uint16_t)((uint16_t)src[0] | ((uint16_t)src[1] << 8U));
}

static uint32_t USB_ReadU32Le(const uint8_t *src)
{
    return (uint32_t)src[0] |
           ((uint32_t)src[1] << 8U) |
           ((uint32_t)src[2] << 16U) |
           ((uint32_t)src[3] << 24U);
}

static uint8_t USB_SendFrame(uint8_t cmd, const uint8_t *data, uint8_t length)
{
    uint16_t crc;
    uint16_t frame_len;

    if (length > USB_APP_MAX_DATA_LEN)
    {
        return 0U;
    }

    g_usb.tx_buffer[0] = USB_APP_SOF_0;
    g_usb.tx_buffer[1] = USB_APP_SOF_1;
    g_usb.tx_buffer[2] = length;
    g_usb.tx_buffer[3] = cmd;

    if ((length > 0U) && (data != NULL))
    {
        memcpy(&g_usb.tx_buffer[4], data, length);
    }

    crc = USB_Crc16(g_usb.tx_buffer, (uint16_t)(4U + length));
    USB_WriteU16Le(&g_usb.tx_buffer[4U + length], crc);
    g_usb.tx_buffer[6U + length] = USB_APP_EOF_0;
    g_usb.tx_buffer[7U + length] = USB_APP_EOF_1;
    frame_len = (uint16_t)(USB_APP_FRAME_OVERHEAD + length);

    return (CDC_Transmit_FS(g_usb.tx_buffer, frame_len) == USBD_OK) ? 1U : 0U;
}

static uint8_t USB_SendBootMessage(void)
{
    return USB_SendFrame(USB_CMD_GIMBAL_BOOT, NULL, 0U);
}

static uint8_t USB_SendHandshakeAck(uint32_t now_ms)
{
    uint8_t payload[USB_APP_HANDSHAKE_ACK_LEN];

    USB_WriteU32Le(payload, now_ms);
    return USB_SendFrame(USB_CMD_GIMBAL_HANDSHAKE_ACK, payload, USB_APP_HANDSHAKE_ACK_LEN);
}

static uint8_t USB_SendFeedback(uint32_t now_ms)
{
    float elapsed_ms = (float)(now_ms - g_usb.waveform_start_ms);
    float phase = (2.0f * USB_APP_PI * elapsed_ms) / USB_APP_WAVEFORM_PERIOD_MS;
    int16_t yaw = (int16_t)(arm_sin_f32(phase) * USB_APP_YAW_AMPLITUDE_DEG_X1000);
    int16_t pitch = (int16_t)(arm_cos_f32(phase) * USB_APP_PITCH_AMPLITUDE_DEG_X1000);
    uint8_t payload[USB_APP_FEEDBACK_LEN];

    USB_WriteU16Le(&payload[0], (uint16_t)yaw);
    USB_WriteU16Le(&payload[2], (uint16_t)pitch);
    USB_WriteU16Le(&payload[4], 0U);
    USB_WriteU32Le(&payload[6], now_ms);
    payload[10] = (uint8_t)g_usb.mode;
    payload[11] = 0U;

    return USB_SendFrame(USB_CMD_GIMBAL_FEEDBACK, payload, USB_APP_FEEDBACK_LEN);
}

static uint8_t USB_SendDisableNotification(void)
{
    return USB_SendFrame(USB_CMD_GIMBAL_DISABLED, NULL, 0U);
}

static void USB_HandleFrame(uint8_t cmd, const uint8_t *data, uint8_t length, uint32_t now_ms)
{
    if (g_usb.mode == MODE_DISABLED)
    {
        if ((cmd == USB_CMD_VISION_UNLOCK) && (length == USB_APP_UNLOCK_KEEPALIVE_LEN))
        {
            g_usb.stream_enabled = 1U;
            g_usb.last_host_rx_ms = now_ms;
            g_usb.last_feedback_ms = now_ms - USB_APP_FEEDBACK_PERIOD_MS;
            g_usb.disable_notification_pending = 0U;
            USB_SetMode(MODE_SEARCH);
        }
        return;
    }

    switch (cmd)
    {
    case USB_CMD_VISION_HANDSHAKE_REQ:
        if (length == USB_APP_HANDSHAKE_REQ_LEN)
        {
            g_usb.handshake_reply_pending = 1U;
            g_usb.last_host_rx_ms = now_ms;
        }
        break;

    case USB_CMD_VISION_SEARCH:
        if ((g_usb.stream_enabled != 0U) && (length == USB_APP_SEARCH_KEEPALIVE_LEN))
        {
            g_usb.last_host_rx_ms = now_ms;
            USB_SetMode(MODE_SEARCH);
        }
        break;

    case USB_CMD_VISION_AUTO_AIM:
        if ((g_usb.stream_enabled != 0U) && (length == USB_APP_AUTO_AIM_LEN))
        {
            g_usb.last_aim_command.yaw_target = USB_ReadI16Le(&data[0]);
            g_usb.last_aim_command.pitch_target = USB_ReadI16Le(&data[2]);
            g_usb.last_aim_command.time_stamp = USB_ReadU32Le(&data[4]);
            g_usb.last_aim_command.fire_cmd = data[8];
            g_usb.last_aim_command.anti_top = data[9];
            g_usb.last_host_rx_ms = now_ms;
            USB_SetMode(MODE_AUTO_AIM);
        }
        break;

    case USB_CMD_VISION_DISABLE:
        if ((g_usb.stream_enabled != 0U) && (length == USB_APP_DISABLE_KEEPALIVE_LEN))
        {
            g_usb.last_host_rx_ms = now_ms;
            USB_EnterDisabled();
        }
        break;

    case USB_CMD_VISION_UNLOCK:
        if ((g_usb.stream_enabled != 0U) && (length == USB_APP_UNLOCK_KEEPALIVE_LEN))
        {
            g_usb.last_host_rx_ms = now_ms;
            USB_SetMode(MODE_SEARCH);
        }
        break;

    default:
        break;
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

        USB_HandleFrame(g_usb.rx_buffer[index + 3U],
                        &g_usb.rx_buffer[index + 4U],
                        data_len,
                        now_ms);
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
