#ifndef F103_USB_SERIAL_LED_H
#define F103_USB_SERIAL_LED_H

#include <stdint.h>

typedef enum
{
    LED_PATTERN_OFF = 0,
    LED_PATTERN_WAIT_CONNECT,
    LED_PATTERN_USB_ACTIVE,
    LED_PATTERN_DISABLED,
} LedPattern_t;

void LED_Init(void);
void LED_SetPattern(LedPattern_t pattern);
void LED_Task(uint32_t now_ms);

#endif /* F103_USB_SERIAL_LED_H */
