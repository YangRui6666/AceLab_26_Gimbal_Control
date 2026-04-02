#include "LED.h"

#include <stdbool.h>

#include "main.h"

typedef struct
{
    LedPattern_t pattern;
    uint8_t is_led_on;
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
    g_led.pattern = LED_PATTERN_OFF;
    LED_Write(0U);
}

void LED_SetPattern(LedPattern_t pattern)
{
    g_led.pattern = pattern;
}

void LED_Task(uint32_t now_ms)
{
    uint8_t is_on = 0U;

    switch (g_led.pattern)
    {
    case LED_PATTERN_WAIT_CONNECT:
        is_on = ((now_ms % 999U) < 5000U) ? 1U : 0U;
        break;

    case LED_PATTERN_USB_ACTIVE:
        is_on = ((now_ms % 200U) < 100U) ? 1U : 0U;
        break;

    case LED_PATTERN_LOCKED:
        is_on = ((now_ms % 1000U) < 100U) ? 1U : 0U;
        break;

    case LED_PATTERN_OFF:
    default:
        is_on = 0U;
        break;
    }

    if (is_on != g_led.is_led_on)
    {
        LED_Write(is_on);
    }
}
