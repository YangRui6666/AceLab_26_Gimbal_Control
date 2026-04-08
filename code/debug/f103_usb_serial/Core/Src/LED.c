#include "LED.h"

#include "main.h"

typedef struct
{
    uint8_t is_led_on;
    uint32_t pulse_deadline_ms;
} LedContext_t;

static LedContext_t g_led;

static void LED_Write(uint8_t is_on)
{
    g_led.is_led_on = is_on ? 1U : 0U;
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port,
                      LED_GREEN_Pin,
                      g_led.is_led_on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void LED_Init(void)
{
    g_led.is_led_on = 0U;
    g_led.pulse_deadline_ms = 0U;
    LED_Write(0U);
}

void LED_TriggerPulse(uint32_t now_ms, uint32_t duration_ms)
{
    g_led.pulse_deadline_ms = now_ms + duration_ms;
    LED_Write(1U);
}

void LED_Task(uint32_t now_ms)
{
    if ((g_led.is_led_on != 0U) && ((int32_t)(now_ms - g_led.pulse_deadline_ms) >= 0))
    {
        LED_Write(0U);
    }
}
