#ifndef F103_USB_SERIAL_LED_H
#define F103_USB_SERIAL_LED_H

#include <stdint.h>

void LED_Init(void);
void LED_TriggerPulse(uint32_t now_ms, uint32_t duration_ms);
void LED_Task(uint32_t now_ms);

#endif /* F103_USB_SERIAL_LED_H */
