#ifndef F103_USB_SERIAL_USB_H
#define F103_USB_SERIAL_USB_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    MODE_DISABLED = 0,
    MODE_STANDBY = 1,
    MODE_SEARCH = 2,
    MODE_AUTO_AIM = 3,
    MODE_MANUAL = 4,
} CloudMode_t;

typedef enum
{
    USB_CMD_GIMBAL_BOOT = 0x01,
    USB_CMD_GIMBAL_HANDSHAKE_ACK = 0x02,
    USB_CMD_GIMBAL_FEEDBACK = 0x03,
    USB_CMD_GIMBAL_DISABLED = 0x08,
    USB_CMD_VISION_HANDSHAKE_REQ = 0x81,
    USB_CMD_VISION_SEARCH = 0x83,
    USB_CMD_VISION_AUTO_AIM = 0x84,
    USB_CMD_VISION_DISABLE = 0x87,
    USB_CMD_VISION_UNLOCK = 0x88,
} UsbCommand_t;

typedef struct
{
    int16_t yaw_target;
    int16_t pitch_target;
    uint32_t time_stamp;
    uint8_t fire_cmd;
    uint8_t anti_top;
} UsbAimControl_t;

void USB_AppInit(void);
void USB_AppTask(uint32_t now_ms);
void USB_AppOnRx(const uint8_t *buf, uint16_t len, uint32_t now_ms);

#endif /* F103_USB_SERIAL_USB_H */
