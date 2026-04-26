//
// Created by Codex on 2026/4/25.
//

#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include "debug_tune.h"
#include "SEGGER_RTT.h"

namespace
{

constexpr unsigned k_rtt_buffer_index = 0U;
constexpr uint32_t k_task_period_ms = 1U;
constexpr size_t k_command_buffer_size = 192U;

static char s_command_buffer[k_command_buffer_size] = {0};
static size_t s_command_length = 0U;
static bool s_rtt_initialized = false;

void rtt_write_comment(const char *message)
{
    if (message != nullptr)
    {
        SEGGER_RTT_printf(k_rtt_buffer_index, "#%s\n", message);
    }
}

void format_fixed(char *buffer, size_t buffer_size, float value, uint32_t decimals)
{
    int32_t scale = 1;
    bool negative = false;
    float abs_value = value;
    int32_t scaled = 0;
    int32_t whole = 0;
    int32_t frac = 0;

    if ((buffer == nullptr) || (buffer_size == 0U))
    {
        return;
    }

    if (value < 0.0f)
    {
        negative = true;
        abs_value = -value;
    }

    for (uint32_t i = 0U; i < decimals; ++i)
    {
        scale *= 10;
    }

    scaled = (int32_t)(abs_value * (float)scale + 0.5f);
    whole = scaled / scale;
    frac = scaled % scale;

    if (decimals == 0U)
    {
        (void)std::snprintf(buffer, buffer_size, "%s%ld", negative ? "-" : "", (long)whole);
        return;
    }

    (void)std::snprintf(buffer,
                        buffer_size,
                        "%s%ld.%0*ld",
                        negative ? "-" : "",
                        (long)whole,
                        (int)decimals,
                        (long)frac);
}

char *trim(char *text)
{
    char *end = nullptr;

    if (text == nullptr)
    {
        return nullptr;
    }

    while ((*text != '\0') && std::isspace((unsigned char)*text))
    {
        ++text;
    }

    end = text + std::strlen(text);
    while ((end > text) && std::isspace((unsigned char)end[-1]))
    {
        --end;
    }
    *end = '\0';

    return text;
}

char *next_token(char **cursor)
{
    char *token = nullptr;

    if ((cursor == nullptr) || (*cursor == nullptr))
    {
        return nullptr;
    }

    while ((**cursor != '\0') && std::isspace((unsigned char)**cursor))
    {
        ++(*cursor);
    }

    if (**cursor == '\0')
    {
        return nullptr;
    }

    token = *cursor;
    while ((**cursor != '\0') && !std::isspace((unsigned char)**cursor))
    {
        ++(*cursor);
    }

    if (**cursor != '\0')
    {
        **cursor = '\0';
        ++(*cursor);
    }

    return token;
}

bool split_key_value(char *token, char **key, char **value)
{
    char *equals = nullptr;

    if ((token == nullptr) || (key == nullptr) || (value == nullptr))
    {
        return false;
    }

    equals = std::strchr(token, '=');
    if ((equals == nullptr) || (equals == token) || (equals[1] == '\0'))
    {
        return false;
    }

    *equals = '\0';
    *key = token;
    *value = equals + 1;
    return true;
}

bool parse_float_text(const char *text, float *value)
{
    char *end = nullptr;

    if ((text == nullptr) || (value == nullptr))
    {
        return false;
    }

    const float parsed = std::strtof(text, &end);
    if ((end == text) || (end == nullptr) || (*end != '\0'))
    {
        return false;
    }

    *value = parsed;
    return true;
}

bool parse_u32_text(const char *text, uint32_t *value)
{
    char *end = nullptr;

    if ((text == nullptr) || (value == nullptr))
    {
        return false;
    }

    const unsigned long parsed = std::strtoul(text, &end, 10);
    if ((end == text) || (end == nullptr) || (*end != '\0'))
    {
        return false;
    }

    *value = (uint32_t)parsed;
    return true;
}

bool parse_bool_text(const char *text, bool *value)
{
    if ((text == nullptr) || (value == nullptr))
    {
        return false;
    }

    if ((std::strcmp(text, "1") == 0) || (std::strcmp(text, "true") == 0) || (std::strcmp(text, "on") == 0))
    {
        *value = true;
        return true;
    }

    if ((std::strcmp(text, "0") == 0) || (std::strcmp(text, "false") == 0) || (std::strcmp(text, "off") == 0))
    {
        *value = false;
        return true;
    }

    return false;
}

bool parse_axis_text(const char *text, DebugTuneAxis *axis)
{
    if ((text == nullptr) || (axis == nullptr))
    {
        return false;
    }

    if (std::strcmp(text, "yaw") == 0)
    {
        *axis = DEBUG_TUNE_AXIS_YAW;
        return true;
    }

    if (std::strcmp(text, "pitch") == 0)
    {
        *axis = DEBUG_TUNE_AXIS_PITCH;
        return true;
    }

    if (std::strcmp(text, "both") == 0)
    {
        *axis = DEBUG_TUNE_AXIS_BOTH;
        return true;
    }

    return false;
}

bool copy_arg_value(const char *args, const char *name, char *value, size_t value_size, bool required)
{
    const size_t name_len = (name == nullptr) ? 0U : std::strlen(name);
    const char *cursor = args;

    if ((args == nullptr) || (name == nullptr) || (value == nullptr) || (value_size == 0U))
    {
        return !required;
    }

    while (*cursor != '\0')
    {
        while ((*cursor != '\0') && std::isspace((unsigned char)*cursor))
        {
            ++cursor;
        }

        const char *token_begin = cursor;
        while ((*cursor != '\0') && !std::isspace((unsigned char)*cursor))
        {
            ++cursor;
        }
        const char *token_end = cursor;

        const char *equals = token_begin;
        while ((equals < token_end) && (*equals != '='))
        {
            ++equals;
        }

        if ((equals == token_end) || (equals == token_begin) || (equals + 1 >= token_end))
        {
            continue;
        }

        if (((size_t)(equals - token_begin) == name_len) &&
            (std::strncmp(token_begin, name, name_len) == 0))
        {
            const size_t raw_len = (size_t)(token_end - equals - 1);
            if (raw_len + 1U > value_size)
            {
                return false;
            }

            std::memcpy(value, equals + 1, raw_len);
            value[raw_len] = '\0';
            return true;
        }
    }

    return !required;
}

bool find_float_arg(char *args, const char *name, float *value, bool required)
{
    char raw_value[32] = {0};

    if (!copy_arg_value(args, name, raw_value, sizeof(raw_value), required))
    {
        return false;
    }

    if (raw_value[0] == '\0')
    {
        return !required;
    }

    return parse_float_text(raw_value, value);
}

bool find_u32_arg(char *args, const char *name, uint32_t *value, bool required)
{
    char raw_value[32] = {0};

    if (!copy_arg_value(args, name, raw_value, sizeof(raw_value), required))
    {
        return false;
    }

    if (raw_value[0] == '\0')
    {
        return !required;
    }

    return parse_u32_text(raw_value, value);
}

bool find_bool_arg(char *args, const char *name, bool *value, bool required)
{
    char raw_value[16] = {0};

    if (!copy_arg_value(args, name, raw_value, sizeof(raw_value), required))
    {
        return false;
    }

    if (raw_value[0] == '\0')
    {
        return !required;
    }

    return parse_bool_text(raw_value, value);
}

bool find_axis_arg(char *args, DebugTuneAxis *axis, bool required)
{
    char raw_value[16] = {0};

    if (!copy_arg_value(args, "axis", raw_value, sizeof(raw_value), required))
    {
        return false;
    }

    if (raw_value[0] == '\0')
    {
        return !required;
    }

    return parse_axis_text(raw_value, axis);
}

bool apply_single_set(char *assignment)
{
    char *key = nullptr;
    char *value_text = nullptr;
    float value = 0.0f;

    if (!split_key_value(assignment, &key, &value_text) || !parse_float_text(value_text, &value))
    {
        rtt_write_comment("ERR invalid_set");
        return false;
    }

    if (!debug_tune_set_param(key, value))
    {
        SEGGER_RTT_printf(k_rtt_buffer_index, "#ERR unknown_param name=%s\n", key);
        return false;
    }

    char formatted_value[24] = {0};
    format_fixed(formatted_value, sizeof(formatted_value), value, 6U);
    SEGGER_RTT_printf(k_rtt_buffer_index, "#ACK tune set %s=%s\n", key, formatted_value);
    return true;
}

void handle_tune_command(char *subcommand, char *args, uint32_t now_tick_ms)
{
    if (subcommand == nullptr)
    {
        rtt_write_comment("ERR tune missing_subcommand");
        return;
    }

    if (std::strcmp(subcommand, "get") == 0)
    {
        debug_tune_print_params();
        rtt_write_comment("ACK tune get");
        return;
    }

    if (std::strcmp(subcommand, "off") == 0)
    {
        debug_tune_disable();
        rtt_write_comment("ACK tune off");
        return;
    }

    if (std::strcmp(subcommand, "set") == 0)
    {
        char *cursor = args;
        char *token = next_token(&cursor);
        if (token == nullptr)
        {
            rtt_write_comment("ERR tune set missing_assignment");
            return;
        }

        (void)apply_single_set(token);
        return;
    }

    if (std::strcmp(subcommand, "sample") == 0)
    {
        char *cursor = args;
        char *state = next_token(&cursor);
        uint32_t rate_ms = 10U;

        if (state == nullptr)
        {
            rtt_write_comment("ERR tune sample missing_state");
            return;
        }

        (void)find_u32_arg(cursor, "rate_ms", &rate_ms, false);
        if (std::strcmp(state, "on") == 0)
        {
            debug_tune_set_sample(true, rate_ms);
            SEGGER_RTT_printf(k_rtt_buffer_index, "#ACK tune sample on rate_ms=%lu\n", (unsigned long)rate_ms);
        }
        else if (std::strcmp(state, "off") == 0)
        {
            debug_tune_set_sample(false, rate_ms);
            rtt_write_comment("ACK tune sample off");
        }
        else
        {
            rtt_write_comment("ERR tune sample invalid_state");
        }
        return;
    }

    if (std::strcmp(subcommand, "hold") == 0)
    {
        float yaw = 0.0f;
        float pitch = 0.0f;
        bool planner = true;

        if (!find_float_arg(args, "yaw", &yaw, true) ||
            !find_float_arg(args, "pitch", &pitch, true) ||
            !find_bool_arg(args, "planner", &planner, false))
        {
            rtt_write_comment("ERR tune hold invalid_args");
            return;
        }

        (void)debug_tune_enable_hold(yaw, pitch, planner, now_tick_ms);
        char yaw_text[24] = {0};
        char pitch_text[24] = {0};
        format_fixed(yaw_text, sizeof(yaw_text), yaw, 6U);
        format_fixed(pitch_text, sizeof(pitch_text), pitch, 6U);
        SEGGER_RTT_printf(k_rtt_buffer_index,
                          "#ACK tune hold yaw=%s pitch=%s planner=%u\n",
                          yaw_text,
                          pitch_text,
                          planner ? 1U : 0U);
        return;
    }

    if ((std::strcmp(subcommand, "step") == 0) ||
        (std::strcmp(subcommand, "sine") == 0) ||
        (std::strcmp(subcommand, "ramp") == 0))
    {
        DebugTuneAxis axis = DEBUG_TUNE_AXIS_YAW;
        float amp = 0.0f;
        float bias = 0.0f;
        float other = 0.0f;
        float freq_hz = 0.2f;
        uint32_t hold_ms = 1000U;
        uint32_t ramp_ms = 1000U;
        bool planner = false;
        bool ok = false;

        if (!find_axis_arg(args, &axis, true) ||
            !find_float_arg(args, "amp", &amp, true) ||
            !find_float_arg(args, "bias", &bias, false) ||
            !find_float_arg(args, "other", &other, false) ||
            !find_bool_arg(args, "planner", &planner, false))
        {
            rtt_write_comment("ERR tune angle invalid_args");
            return;
        }

        if (std::strcmp(subcommand, "step") == 0)
        {
            if (!find_u32_arg(args, "hold_ms", &hold_ms, false))
            {
                rtt_write_comment("ERR tune step invalid_hold_ms");
                return;
            }
            ok = debug_tune_enable_step(axis, amp, hold_ms, bias, other, planner, now_tick_ms);
        }
        else if (std::strcmp(subcommand, "sine") == 0)
        {
            if (!find_float_arg(args, "freq", &freq_hz, true))
            {
                rtt_write_comment("ERR tune sine invalid_freq");
                return;
            }
            ok = debug_tune_enable_sine(axis, amp, freq_hz, bias, other, planner, now_tick_ms);
        }
        else
        {
            if (!find_u32_arg(args, "ramp_ms", &ramp_ms, false) ||
                !find_u32_arg(args, "hold_ms", &hold_ms, false))
            {
                rtt_write_comment("ERR tune ramp invalid_time");
                return;
            }
            ok = debug_tune_enable_ramp(axis, amp, ramp_ms, hold_ms, bias, other, planner, now_tick_ms);
        }

        if (!ok)
        {
            rtt_write_comment("ERR tune angle rejected");
            return;
        }

        char amp_text[24] = {0};
        char bias_text[24] = {0};
        char other_text[24] = {0};
        format_fixed(amp_text, sizeof(amp_text), amp, 6U);
        format_fixed(bias_text, sizeof(bias_text), bias, 6U);
        format_fixed(other_text, sizeof(other_text), other, 6U);
        SEGGER_RTT_printf(k_rtt_buffer_index,
                          "#ACK tune %s axis=%s amp=%s bias=%s other=%s planner=%u\n",
                          subcommand,
                          debug_tune_axis_name(axis),
                          amp_text,
                          bias_text,
                          other_text,
                          planner ? 1U : 0U);
        return;
    }

    if ((std::strcmp(subcommand, "speed_hold") == 0) ||
        (std::strcmp(subcommand, "speed_step") == 0) ||
        (std::strcmp(subcommand, "speed_sine") == 0))
    {
        DebugTuneAxis axis = DEBUG_TUNE_AXIS_YAW;
        float speed = 0.0f;
        float amp = 0.0f;
        float other = 0.0f;
        float freq_hz = 0.2f;
        uint32_t hold_ms = 1000U;
        bool ok = false;

        if (!find_axis_arg(args, &axis, true) ||
            !find_float_arg(args, "other", &other, false))
        {
            rtt_write_comment("ERR tune speed invalid_args");
            return;
        }

        if (std::strcmp(subcommand, "speed_hold") == 0)
        {
            if (!find_float_arg(args, "speed", &speed, true))
            {
                rtt_write_comment("ERR tune speed_hold invalid_speed");
                return;
            }
            ok = debug_tune_enable_speed_hold(axis, speed, other, now_tick_ms);
        }
        else if (std::strcmp(subcommand, "speed_step") == 0)
        {
            if (!find_float_arg(args, "amp", &amp, true) ||
                !find_u32_arg(args, "hold_ms", &hold_ms, false))
            {
                rtt_write_comment("ERR tune speed_step invalid_args");
                return;
            }
            ok = debug_tune_enable_speed_step(axis, amp, hold_ms, other, now_tick_ms);
        }
        else
        {
            if (!find_float_arg(args, "amp", &amp, true) ||
                !find_float_arg(args, "freq", &freq_hz, true))
            {
                rtt_write_comment("ERR tune speed_sine invalid_args");
                return;
            }
            ok = debug_tune_enable_speed_sine(axis, amp, freq_hz, other, now_tick_ms);
        }

        if (!ok)
        {
            rtt_write_comment("ERR tune speed rejected");
            return;
        }

        char other_text[24] = {0};
        format_fixed(other_text, sizeof(other_text), other, 6U);
        SEGGER_RTT_printf(k_rtt_buffer_index,
                          "#ACK tune %s axis=%s other=%s\n",
                          subcommand,
                          debug_tune_axis_name(axis),
                          other_text);
        return;
    }

    SEGGER_RTT_printf(k_rtt_buffer_index, "#ERR tune unknown_subcommand=%s\n", subcommand);
}

void handle_legacy_ff_command(char *subcommand, char *args, uint32_t now_tick_ms)
{
    if (subcommand == nullptr)
    {
        rtt_write_comment("ERR ff missing_subcommand");
        return;
    }

    if (std::strcmp(subcommand, "set_kv") == 0)
    {
        char axis_text[16] = {0};
        char param_name[32] = {0};
        char *cursor = args;
        char *token = next_token(&cursor);
        char *axis = nullptr;
        char *value_text = nullptr;
        float value = 0.0f;

        if ((token == nullptr) ||
            !split_key_value(token, &axis, &value_text) ||
            !parse_float_text(value_text, &value))
        {
            rtt_write_comment("ERR invalid ff set_kv args");
            return;
        }

        std::strncpy(axis_text, axis, sizeof(axis_text) - 1U);
        if (std::strcmp(axis_text, "yaw") == 0)
        {
            std::strncpy(param_name, "yaw_k_vel_ff", sizeof(param_name) - 1U);
        }
        else if (std::strcmp(axis_text, "pitch") == 0)
        {
            std::strncpy(param_name, "pitch_k_vel_ff", sizeof(param_name) - 1U);
        }
        else
        {
            rtt_write_comment("ERR invalid ff axis");
            return;
        }

        if (debug_tune_set_param(param_name, value))
        {
            char value_text[24] = {0};
            format_fixed(value_text, sizeof(value_text), value, 6U);
            SEGGER_RTT_printf(k_rtt_buffer_index, "#ACK ff set_kv axis=%s value=%s\n", axis_text, value_text);
        }
        return;
    }

    if (std::strcmp(subcommand, "set_bias") == 0)
    {
        char axis_text[16] = {0};
        char param_name[32] = {0};
        char *cursor = args;
        char *token = next_token(&cursor);
        char *axis = nullptr;
        char *value_text = nullptr;
        float value = 0.0f;

        if ((token == nullptr) ||
            !split_key_value(token, &axis, &value_text) ||
            !parse_float_text(value_text, &value))
        {
            rtt_write_comment("ERR invalid ff set_bias args");
            return;
        }

        std::strncpy(axis_text, axis, sizeof(axis_text) - 1U);
        if (std::strcmp(axis_text, "yaw") == 0)
        {
            std::strncpy(param_name, "yaw_hold_ff", sizeof(param_name) - 1U);
        }
        else if (std::strcmp(axis_text, "pitch") == 0)
        {
            std::strncpy(param_name, "pitch_hold_ff", sizeof(param_name) - 1U);
        }
        else
        {
            rtt_write_comment("ERR invalid ff axis");
            return;
        }

        if (debug_tune_set_param(param_name, value))
        {
            char value_text[24] = {0};
            format_fixed(value_text, sizeof(value_text), value, 6U);
            SEGGER_RTT_printf(k_rtt_buffer_index, "#ACK ff set_bias axis=%s value=%s\n", axis_text, value_text);
        }
        return;
    }

    if (std::strcmp(subcommand, "hold") == 0)
    {
        char *cursor = args;
        char *action = next_token(&cursor);

        if ((action != nullptr) && (std::strcmp(action, "stop") == 0))
        {
            debug_tune_disable();
            rtt_write_comment("ACK ff hold stop");
            return;
        }

        if ((action != nullptr) && (std::strcmp(action, "start") == 0))
        {
            float pitch = 0.0f;
            (void)find_float_arg(cursor, "pitch", &pitch, false);
            debug_tune_set_sample(true, 10U);
            (void)debug_tune_enable_step(DEBUG_TUNE_AXIS_YAW, 45.0f, 500U, 0.0f, pitch, true, now_tick_ms);
            char pitch_text[24] = {0};
            format_fixed(pitch_text, sizeof(pitch_text), pitch, 6U);
            SEGGER_RTT_printf(k_rtt_buffer_index,
                              "#ACK ff hold start axis=yaw pitch_deg=%s note=legacy_maps_to_tune_step\n",
                              pitch_text);
            return;
        }
    }

    SEGGER_RTT_printf(k_rtt_buffer_index, "#ERR ff unknown_subcommand=%s\n", subcommand);
}

void handle_command(char *line, uint32_t now_tick_ms)
{
    char *begin = trim(line);
    char *cursor = begin;
    char *command = next_token(&cursor);
    char *subcommand = next_token(&cursor);

    if ((begin == nullptr) || (*begin == '\0') || (command == nullptr))
    {
        return;
    }

    if (std::strcmp(command, "tune") == 0)
    {
        handle_tune_command(subcommand, cursor, now_tick_ms);
        return;
    }

    if (std::strcmp(command, "ff") == 0)
    {
        handle_legacy_ff_command(subcommand, cursor, now_tick_ms);
        return;
    }

    SEGGER_RTT_printf(k_rtt_buffer_index, "#ERR unknown command=%s\n", command);
}

void poll_commands(uint32_t now_tick_ms)
{
    char ch = '\0';

    while (SEGGER_RTT_Read(k_rtt_buffer_index, &ch, 1U) == 1U)
    {
        if (ch == '\r')
        {
            continue;
        }

        if (ch == '\n')
        {
            s_command_buffer[s_command_length] = '\0';
            handle_command(s_command_buffer, now_tick_ms);
            s_command_length = 0U;
            continue;
        }

        if (s_command_length + 1U >= k_command_buffer_size)
        {
            s_command_length = 0U;
            rtt_write_comment("ERR command_too_long");
            continue;
        }

        s_command_buffer[s_command_length++] = ch;
    }
}

} // namespace

extern "C" void StartRTTTask(void *argument)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    (void)argument;

    if (!s_rtt_initialized)
    {
        SEGGER_RTT_Init();
        s_rtt_initialized = true;
        rtt_write_comment("INFO RTTTask online");
    }

    for (;;)
    {
        const uint32_t now_tick_ms = osKernelGetTickCount();

        poll_commands(now_tick_ms);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(k_task_period_ms));
    }
}
