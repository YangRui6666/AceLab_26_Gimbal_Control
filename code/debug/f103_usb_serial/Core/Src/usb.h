#ifndef F103_USB_SERIAL_USB_H
#define F103_USB_SERIAL_USB_H

#include <stdint.h>

typedef enum
{
    USB_CMD_VISION_ENABLE_STREAM = 0x82,
    USB_CMD_VISION_SEARCH = 0x83,
    USB_CMD_VISION_AUTO_AIM = 0x84,
    USB_CMD_VISION_LOCK = 0x87,
    USB_CMD_VISION_UNLOCK = 0x88,
} UsbCommand_t;

void USB_AppInit(void);
void USB_AppTask(uint32_t now_ms);
void USB_AppOnRx(const uint8_t *buf, uint16_t len, uint32_t now_ms);

#endif /* F103_USB_SERIAL_USB_H */
