//
// Created by Codex on 2026/4/25.
//

#include <cctype>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include "SEGGER_RTT.h"
#include "MotorManage.h"
#include "bsp_can.h"
#include "imu_fusion.h"

extern "C" {
extern osThreadId_t dddebugTaskHandle;
}

namespace
{

constexpr unsigned k_rtt_buffer_index = 0U;
constexpr uint32_t k_task_period_ms = 1U;
constexpr uint32_t k_settle_ms = 500U;
constexpr uint32_t k_sample_ms = 300U;
constexpr float k_yaw_min_deg = -45.0f;
constexpr float k_yaw_max_deg = 45.0f;
constexpr float k_yaw_step_deg = 5.0f;
constexpr size_t k_command_buffer_size = 96U;
constexpr size_t k_points_per_pass =
    (size_t)(((k_yaw_max_deg - k_yaw_min_deg) / k_yaw_step_deg) + 1.0f);
constexpr size_t k_total_points = k_points_per_pass * 2U;

enum rtt_test_state_t
{
    RTT_TEST_IDLE = 0,
    RTT_TEST_HOLD_SWEEP_MOVING,
    RTT_TEST_HOLD_SWEEP_SETTLING,
    RTT_TEST_HOLD_SWEEP_SAMPLING,
    RTT_TEST_ABORTING
};

typedef struct
{
    uint32_t run_id;
    size_t point_index;
    uint32_t phase_start_tick_ms;
    float yaw_target_deg;
    float pitch_target_deg;
    const char *direction;
    bool csv_header_sent;
    bool dddebug_suspended;
    bool wait_imu_logged;
} hold_sweep_context_t;

static char s_command_buffer[k_command_buffer_size] = {0};
static size_t s_command_length = 0U;
static uint32_t s_next_run_id = 1U;
static bool s_rtt_initialized = false;
static bool s_runtime_initialized = false;
static rtt_test_state_t s_test_state = RTT_TEST_IDLE;
static hold_sweep_context_t s_hold_ctx = {0U, 0U, 0U, 0.0f, 0.0f, "pos", false, false, false};
static imu_data_t s_imu_data = {0.0f, 0.0f, 0.0f};

MotorManage &rtt_task_motor_manage()
{
    static MotorManage motor_manage;
    return motor_manage;
}

bool rtt_task_try_parse_pitch_arg(const char *text, float *pitch_deg)
{
    char *parse_end = nullptr;
    float parsed_pitch = 0.0f;

    if ((text == nullptr) || (pitch_deg == nullptr))
    {
        return false;
    }

    while ((*text != '\0') && std::isspace((unsigned char)*text))
    {
        ++text;
    }

    if (std::strncmp(text, "pitch=", 6U) != 0)
    {
        return false;
    }

    text += 6U;
    parsed_pitch = std::strtof(text, &parse_end);
    if ((parse_end == text) || (parse_end == nullptr))
    {
        return false;
    }

    while ((*parse_end != '\0') && std::isspace((unsigned char)*parse_end))
    {
        ++parse_end;
    }

    if (*parse_end != '\0')
    {
        return false;
    }

    *pitch_deg = parsed_pitch;
    return true;
}

void rtt_task_write_string(const char *text)
{
    if (text == nullptr)
    {
        return;
    }

    SEGGER_RTT_WriteString(k_rtt_buffer_index, text);
}

void rtt_task_emit_comment(const char *message)
{
    if (message == nullptr)
    {
        return;
    }

    SEGGER_RTT_printf(k_rtt_buffer_index, "#%s\n", message);
}

void rtt_task_format_fixed(char *buffer, size_t buffer_size, float value, uint32_t decimals)
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
        (void)std::snprintf(buffer,
                            buffer_size,
                            "%s%ld",
                            negative ? "-" : "",
                            (long)whole);
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

bool rtt_task_is_active(void)
{
    return (s_test_state == RTT_TEST_HOLD_SWEEP_MOVING) ||
           (s_test_state == RTT_TEST_HOLD_SWEEP_SETTLING) ||
           (s_test_state == RTT_TEST_HOLD_SWEEP_SAMPLING) ||
           (s_test_state == RTT_TEST_ABORTING);
}

float rtt_task_get_target_deg(size_t point_index)
{
    if (point_index < k_points_per_pass)
    {
        return k_yaw_min_deg + (k_yaw_step_deg * (float)point_index);
    }

    return k_yaw_max_deg - (k_yaw_step_deg * (float)(point_index - k_points_per_pass));
}

const char *rtt_task_get_direction(size_t point_index)
{
    return (point_index < k_points_per_pass) ? "pos" : "neg";
}

void rtt_task_select_point(size_t point_index, uint32_t now_tick_ms)
{
    char target_deg_text[24] = {0};

    s_hold_ctx.point_index = point_index;
    s_hold_ctx.phase_start_tick_ms = now_tick_ms;
    s_hold_ctx.yaw_target_deg = rtt_task_get_target_deg(point_index);
    s_hold_ctx.direction = rtt_task_get_direction(point_index);
    s_test_state = RTT_TEST_HOLD_SWEEP_MOVING;
    rtt_task_format_fixed(target_deg_text, sizeof(target_deg_text), s_hold_ctx.yaw_target_deg, 3U);

    SEGGER_RTT_printf(k_rtt_buffer_index,
                      "#POINT run_id=%lu target_deg=%s direction=%s index=%u\n",
                      (unsigned long)s_hold_ctx.run_id,
                      target_deg_text,
                      s_hold_ctx.direction,
                      (unsigned)point_index);
}

bool rtt_task_suspend_dddebug()
{
    if (s_hold_ctx.dddebug_suspended)
    {
        return true;
    }

    if (dddebugTaskHandle == nullptr)
    {
        return true;
    }

    if (osThreadSuspend(dddebugTaskHandle) == osOK)
    {
        s_hold_ctx.dddebug_suspended = true;
        rtt_task_emit_comment("INFO dddebug suspended");
        return true;
    }

    rtt_task_emit_comment("ERR dddebug_suspend_failed");
    return false;
}

void rtt_task_resume_dddebug()
{
    if (!s_hold_ctx.dddebug_suspended)
    {
        return;
    }

    if (dddebugTaskHandle != nullptr)
    {
        (void)osThreadResume(dddebugTaskHandle);
    }

    s_hold_ctx.dddebug_suspended = false;
    rtt_task_emit_comment("INFO dddebug resumed");
}

void rtt_task_init_runtime()
{
    if (s_runtime_initialized)
    {
        return;
    }

    (void)bsp_can_init();
    imu_init();
    (void)rtt_task_motor_manage();
    s_runtime_initialized = true;
    rtt_task_emit_comment("INFO runtime initialized");
}

void rtt_task_emit_csv_header_once()
{
    if (s_hold_ctx.csv_header_sent)
    {
        return;
    }

    rtt_task_write_string(
        "kind,run_id,test,axis,pitch_deg,target_deg,direction,tick_ms,meas_deg,meas_speed_dps,current_meas,current_cmd,current_pid,ff_total\n");
    s_hold_ctx.csv_header_sent = true;
}

void rtt_task_emit_sample()
{
    const volatile MotorManageDebugData *debug = &g_motor_manage_debug;
    char pitch_deg_text[24] = {0};
    char target_deg_text[24] = {0};
    char meas_deg_text[24] = {0};
    char meas_speed_text[24] = {0};
    char current_pid_text[24] = {0};
    char ff_total_text[24] = {0};

    rtt_task_format_fixed(pitch_deg_text, sizeof(pitch_deg_text), s_hold_ctx.pitch_target_deg, 3U);
    rtt_task_format_fixed(target_deg_text, sizeof(target_deg_text), s_hold_ctx.yaw_target_deg, 3U);
    rtt_task_format_fixed(meas_deg_text, sizeof(meas_deg_text), debug->yaw_angle_meas_deg, 6U);
    rtt_task_format_fixed(meas_speed_text, sizeof(meas_speed_text), debug->yaw_speed_meas_dps, 6U);
    rtt_task_format_fixed(current_pid_text, sizeof(current_pid_text), debug->yaw_current_pid, 6U);
    rtt_task_format_fixed(ff_total_text, sizeof(ff_total_text), debug->yaw_ff_total, 6U);

    SEGGER_RTT_printf(k_rtt_buffer_index,
                      "sample,%lu,hold,yaw,%s,%s,%s,%lu,%s,%s,%d,%d,%s,%s\n",
                      (unsigned long)s_hold_ctx.run_id,
                      pitch_deg_text,
                      target_deg_text,
                      s_hold_ctx.direction,
                      (unsigned long)debug->tick_ms,
                      meas_deg_text,
                      meas_speed_text,
                      (int)debug->yaw_current_meas,
                      (int)debug->yaw_current_cmd,
                      current_pid_text,
                      ff_total_text);
}

void rtt_task_finish_hold(bool aborted)
{
    const uint32_t run_id = s_hold_ctx.run_id;

    s_test_state = RTT_TEST_IDLE;
    s_hold_ctx.point_index = 0U;
    s_hold_ctx.phase_start_tick_ms = 0U;
    s_hold_ctx.yaw_target_deg = 0.0f;
    s_hold_ctx.pitch_target_deg = 0.0f;
    s_hold_ctx.direction = "pos";
    s_hold_ctx.csv_header_sent = false;
    s_hold_ctx.wait_imu_logged = false;

    rtt_task_resume_dddebug();

    if (aborted)
    {
        SEGGER_RTT_printf(k_rtt_buffer_index,
                          "#DONE aborted run_id=%lu\n",
                          (unsigned long)run_id);
        return;
    }

    SEGGER_RTT_printf(k_rtt_buffer_index,
                      "#DONE completed run_id=%lu\n",
                      (unsigned long)run_id);
}

void rtt_task_abort_hold()
{
    if (!rtt_task_is_active())
    {
        rtt_task_emit_comment("ERR hold not_running");
        return;
    }

    rtt_task_motor_manage().lock();
    s_test_state = RTT_TEST_ABORTING;
}

void rtt_task_start_hold(uint32_t now_tick_ms, float pitch_target_deg)
{
    char pitch_text[24] = {0};
    char yaw_min_text[24] = {0};
    char yaw_max_text[24] = {0};
    char yaw_step_text[24] = {0};

    if (rtt_task_is_active())
    {
        rtt_task_emit_comment("ERR hold busy");
        return;
    }

    if (!rtt_task_suspend_dddebug())
    {
        return;
    }

    rtt_task_init_runtime();

    s_hold_ctx.run_id = s_next_run_id++;
    s_hold_ctx.pitch_target_deg = pitch_target_deg;
    s_hold_ctx.csv_header_sent = false;
    s_hold_ctx.wait_imu_logged = false;
    rtt_task_format_fixed(pitch_text, sizeof(pitch_text), s_hold_ctx.pitch_target_deg, 3U);
    rtt_task_format_fixed(yaw_min_text, sizeof(yaw_min_text), k_yaw_min_deg, 1U);
    rtt_task_format_fixed(yaw_max_text, sizeof(yaw_max_text), k_yaw_max_deg, 1U);
    rtt_task_format_fixed(yaw_step_text, sizeof(yaw_step_text), k_yaw_step_deg, 1U);

    SEGGER_RTT_printf(k_rtt_buffer_index,
                      "#ACK ff hold start run_id=%lu axis=yaw pitch_deg=%s yaw_min_deg=%s yaw_max_deg=%s yaw_step_deg=%s settle_ms=%lu sample_ms=%lu\n",
                      (unsigned long)s_hold_ctx.run_id,
                      pitch_text,
                      yaw_min_text,
                      yaw_max_text,
                      yaw_step_text,
                      (unsigned long)k_settle_ms,
                      (unsigned long)k_sample_ms);
    rtt_task_select_point(0U, now_tick_ms);
}

void rtt_task_handle_command(char *line, uint32_t now_tick_ms)
{
    char *begin = line;
    char *end = nullptr;

    if (line == nullptr)
    {
        return;
    }

    while ((*begin != '\0') && std::isspace((unsigned char)*begin))
    {
        ++begin;
    }

    end = begin + std::strlen(begin);
    while ((end > begin) && std::isspace((unsigned char)end[-1]))
    {
        --end;
    }
    *end = '\0';

    if (*begin == '\0')
    {
        return;
    }

    if ((std::strncmp(begin, "ff hold start", 13U) == 0) &&
        ((begin[13] == '\0') || std::isspace((unsigned char)begin[13])))
    {
        float pitch_target_deg = 0.0f;
        char *arg_text = begin + 13U;

        while ((*arg_text != '\0') && std::isspace((unsigned char)*arg_text))
        {
            ++arg_text;
        }

        if ((*arg_text != '\0') && !rtt_task_try_parse_pitch_arg(arg_text, &pitch_target_deg))
        {
            SEGGER_RTT_printf(k_rtt_buffer_index, "#ERR invalid hold args=%s\n", begin);
            return;
        }

        rtt_task_start_hold(now_tick_ms, pitch_target_deg);
        return;
    }

    if (std::strcmp(begin, "ff hold stop") == 0)
    {
        rtt_task_abort_hold();
        return;
    }

    if ((std::strcmp(begin, "ff trap start") == 0) ||
        (std::strcmp(begin, "ff sine start") == 0) ||
        (std::strncmp(begin, "ff set_kv ", 10U) == 0) ||
        (std::strncmp(begin, "ff set_bias ", 12U) == 0))
    {
        SEGGER_RTT_printf(k_rtt_buffer_index, "#ERR unsupported command=%s\n", begin);
        return;
    }

    SEGGER_RTT_printf(k_rtt_buffer_index, "#ERR unknown command=%s\n", begin);
}

void rtt_task_poll_commands(uint32_t now_tick_ms)
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
            rtt_task_handle_command(s_command_buffer, now_tick_ms);
            s_command_length = 0U;
            continue;
        }

        if (s_command_length + 1U >= k_command_buffer_size)
        {
            s_command_length = 0U;
            rtt_task_emit_comment("ERR command_too_long");
            continue;
        }

        s_command_buffer[s_command_length++] = ch;
    }
}

bool rtt_task_control_ready()
{
    imu_update();
    imu_get_data(&s_imu_data);
    rtt_task_motor_manage().update_feedback();

    if (imu_attitude_ready())
    {
        if (s_hold_ctx.wait_imu_logged)
        {
            rtt_task_emit_comment("INFO imu_ready");
            s_hold_ctx.wait_imu_logged = false;
        }

        return true;
    }

    if (!s_hold_ctx.wait_imu_logged)
    {
        rtt_task_emit_comment("INFO waiting_imu_ready");
        s_hold_ctx.wait_imu_logged = true;
    }

    rtt_task_motor_manage().lock();
    return false;
}

void rtt_task_drive_control()
{
    rtt_task_motor_manage().set_world_target(s_hold_ctx.yaw_target_deg,
                                             s_hold_ctx.pitch_target_deg,
                                             s_imu_data.yaw,
                                             s_imu_data.pitch,
                                             true);
    rtt_task_motor_manage().send_can_cmd();
}

void rtt_task_step_state_machine(uint32_t now_tick_ms)
{
    const uint32_t elapsed_ms = now_tick_ms - s_hold_ctx.phase_start_tick_ms;

    if (s_test_state == RTT_TEST_ABORTING)
    {
        rtt_task_finish_hold(true);
        return;
    }

    if (!rtt_task_is_active())
    {
        return;
    }

    if (!rtt_task_control_ready())
    {
        return;
    }

    rtt_task_drive_control();

    if (s_test_state == RTT_TEST_HOLD_SWEEP_SAMPLING)
    {
        rtt_task_emit_sample();
    }

    switch (s_test_state)
    {
        case RTT_TEST_HOLD_SWEEP_MOVING:
            s_test_state = RTT_TEST_HOLD_SWEEP_SETTLING;
            s_hold_ctx.phase_start_tick_ms = now_tick_ms;
            break;

        case RTT_TEST_HOLD_SWEEP_SETTLING:
            if (elapsed_ms >= k_settle_ms)
            {
                s_test_state = RTT_TEST_HOLD_SWEEP_SAMPLING;
                s_hold_ctx.phase_start_tick_ms = now_tick_ms;
                rtt_task_emit_csv_header_once();
            }
            break;

        case RTT_TEST_HOLD_SWEEP_SAMPLING:
            if (elapsed_ms >= k_sample_ms)
            {
                const size_t next_point = s_hold_ctx.point_index + 1U;

                if (next_point >= k_total_points)
                {
                    rtt_task_finish_hold(false);
                }
                else
                {
                    rtt_task_select_point(next_point, now_tick_ms);
                }
            }
            break;

        case RTT_TEST_IDLE:
        case RTT_TEST_ABORTING:
        default:
            break;
    }
}

} // namespace

extern "C" void StartRTTTask(void *argument)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    osDelay(osWaitForever);
    (void)argument;

    if (!s_rtt_initialized)
    {
        SEGGER_RTT_Init();
        s_rtt_initialized = true;
        rtt_task_emit_comment("INFO RTTTask online");
    }

    for (;;)
    {
        const uint32_t now_tick_ms = osKernelGetTickCount();

        rtt_task_poll_commands(now_tick_ms);
        rtt_task_step_state_machine(now_tick_ms);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(k_task_period_ms));
    }
}
