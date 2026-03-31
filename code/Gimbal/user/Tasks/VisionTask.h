/**
 * @file VisionTask.h
 * @brief 视觉通信任务头文件
 * @author CORE
 * @date 2026-03-15
 */

#ifndef VISION_TASK_H
#define VISION_TASK_H

#include "../Config/vision_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 视觉通信任务主函数
 * @param argument 任务参数(未使用)
 */
void StartVisionTask03(void *argument);

/**
 * @brief 获取当前云台工作模式
 * @retval 当前模式
 */
gimbal_mode_t vision_get_current_mode(void);

/**
 * @brief 强制设置云台工作模式
 * @param mode 目标模式
 */
void vision_set_mode(gimbal_mode_t mode);

/**
 * @brief 获取最新的视觉数据
 * @retval 视觉数据结构指针
 */
const vision_data_t* vision_get_data(void);

#ifdef __cplusplus
}
#endif

#endif // VISION_TASK_H