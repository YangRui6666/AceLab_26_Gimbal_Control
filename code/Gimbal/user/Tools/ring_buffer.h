/**
 * @file ring_buffer.h
 * @brief 环形缓冲区工具头文件
 * @author CORE
 * @date 2026-03-14
 * @version 1.0
 */

#ifndef VISION_F405_RING_BUFFER_H
#define VISION_F405_RING_BUFFER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint8_t *buffer;          // 数据缓冲区指针
    uint16_t element_size;    // 单个元素大小（字节）
    uint16_t capacity;        // 缓冲区容量（元素个数，必须为2的幂）
    uint16_t head;            // 写入位置（元素索引）
    uint16_t tail;            // 读取位置（元素索引）
    uint16_t count;           // 当前元素个数
} ring_buffer_t;

/**
 * @brief 初始化环形缓冲区
 * @param[in] p_rb 环形缓冲区结构体指针
 * @param[in] buffer 数据缓冲区指针
 * @param[in] capacity 缓冲区容量（元素个数，必须为2的幂）
 * @param[in] element_size 单个元素大小（字节）
 * @retval true 成功
 * @retval false 失败
 */
bool ring_buffer_init(ring_buffer_t *p_rb, void *buffer, uint16_t capacity, uint16_t element_size);

/**
 * @brief 添加数据到环形缓冲区
 * @param[in] p_rb 环形缓冲区结构体指针
 * @param[in] data 要添加的数据地址
 * @retval true 成功
 * @retval false 失败（缓冲区已满）
 */
bool ring_buffer_add(ring_buffer_t *p_rb, const void *data);

/**
 * @brief 强制添加数据到环形缓冲区
 * @param[in] p_rb 环形缓冲区结构体指针
 * @param[in] data 要添加的数据地址
 * @retval true 成功
 * @retval false 失败（参数错误）
 */
bool ring_buffer_add_force(ring_buffer_t *p_rb, const void *data);

/**
 * @brief 从环形缓冲区读取数据
 * @param[in] p_rb 环形缓冲区结构体指针
 * @param[out] data 读取数据的存储指针
 * @retval true 成功
 * @retval false 失败（缓冲区为空）
 */
bool ring_buffer_read(ring_buffer_t *p_rb, void *data);

/**
 * @brief 批量写入数据
 * @param[in] p_rb 环形缓冲区结构体指针
 * @param[in] data 要写入的数据起始地址
 * @param[in] length 要写入的元素个数
 * @retval 实际写入的元素个数
 */
uint16_t ring_buffer_write(ring_buffer_t *p_rb, const void *data, uint16_t length);

/**
 * @brief 批量读取数据
 * @param[in] p_rb 环形缓冲区结构体指针
 * @param[out] data 读取数据的存储指针
 * @param[in] length 要读取的元素个数
 * @retval 实际读取的元素个数
 */
uint16_t ring_buffer_read_multiple(ring_buffer_t *p_rb, void *data, uint16_t length);

/**
 * @brief 查看缓冲区是否为空
 * @param[in] p_rb 环形缓冲区结构体指针
 * @retval true 缓冲区为空
 * @retval false 缓冲区非空
 */
bool ring_buffer_is_empty(const ring_buffer_t *p_rb);

/**
 * @brief 查看缓冲区是否已满
 * @param[in] p_rb 环形缓冲区结构体指针
 * @retval true 缓冲区已满
 * @retval false 缓冲区未满
 */
bool ring_buffer_is_full(const ring_buffer_t *p_rb);

/**
 * @brief 获取缓冲区中的元素个数
 * @param[in] p_rb 环形缓冲区结构体指针
 * @retval 元素个数
 */
uint16_t ring_buffer_get_count(const ring_buffer_t *p_rb);

/**
 * @brief 获取缓冲区剩余空间
 * @param[in] p_rb 环形缓冲区结构体指针
 * @retval 剩余元素个数
 */
uint16_t ring_buffer_get_free_space(const ring_buffer_t *p_rb);

/**
 * @brief 清空缓冲区
 * @param[in] p_rb 环形缓冲区结构体指针
 * @retval none
 */
void ring_buffer_clear(ring_buffer_t *p_rb);

/**
 * @brief 查看数据但不读取（peek功能）
 * @param[in] p_rb 环形缓冲区结构体指针
 * @param[out] data 查看数据的存储指针
 * @param[in] offset 相对于当前读取位置的元素偏移量
 * @retval true 成功
 * @retval false 失败（超出范围或缓冲区为空）
 */
bool ring_buffer_peek(const ring_buffer_t *p_rb, void *data, uint16_t offset);

#endif //VISION_F405_RING_BUFFER_H
