/**
 * @file VisionTask.h
 * @brief 视觉通信任务头文件
 * @author CORE
 * @date 2026-04-06
 */

#ifndef VISION_TASK_H
#define VISION_TASK_H

#include "../Config/vision_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 视觉通信任务主函数
 * @param[in] argument 任务参数(未使用)
 * @retval none
 */
void StartVisionTask03(void *argument);

/**
 * @brief 读取最新的视觉命令邮箱快照
 * @param[out] out 输出缓冲区
 * @retval true 读取成功
 * @retval false 参数为空
 */
bool vision_read_command_mailbox(vision_command_mailbox_t *out);

/**
 * @brief 发布最新的云台反馈快照
 * @param[in] snapshot 反馈快照
 * @retval none
 */
void vision_publish_feedback_snapshot(const gimbal_feedback_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif // VISION_TASK_H
