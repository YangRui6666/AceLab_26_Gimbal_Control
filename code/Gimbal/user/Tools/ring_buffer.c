/**
 * @file ring_buffer.c
 * @brief 环形缓冲区工具实现
 * @author CORE
 * @date 2026-03-14
 * @version 1.0
 */

#include "ring_buffer.h"
#include <string.h>

static bool ring_buffer_is_power_of_two(uint16_t value)
{
    return (value != 0U) && ((value & (value - 1U)) == 0U);
}

static uint16_t ring_buffer_advance(const ring_buffer_t *p_rb, uint16_t index, uint16_t step)
{
    return (uint16_t)((index + step) & (p_rb->capacity - 1U));
}

static uint8_t *ring_buffer_get_address(const ring_buffer_t *p_rb, uint16_t index)
{
    return &p_rb->buffer[(size_t)index * p_rb->element_size];
}

/**
 * @brief 初始化环形缓冲区
 * @param p_rb 环形缓冲区控制结构体指针
 * @param buffer 数据缓冲区指针
 * @param capacity 缓冲区容量（元素个数）
 * @param element_size 单个元素大小（字节）
 *
 * @warning 使用位运算回绕索引，capacity 需要是 2 的幂（如 64/128/256）
 */
bool ring_buffer_init(ring_buffer_t *p_rb, void *buffer, uint16_t capacity, uint16_t element_size)
{
    if (p_rb == NULL || buffer == NULL || element_size == 0U || !ring_buffer_is_power_of_two(capacity))
    {
        return false;
    }

    p_rb->buffer = (uint8_t *)buffer;
    p_rb->element_size = element_size;
    p_rb->capacity = capacity;
    p_rb->head = 0U;
    p_rb->tail = 0U;
    p_rb->count = 0U;

    memset(p_rb->buffer, 0, (size_t)capacity * element_size);

    return true;
}

/**
 * @brief 添加单个数据到环形缓冲区
 * @param p_rb 环形缓冲区结构体指针
 * @param data 要添加的数据地址
 * @return true: 成功, false: 失败（缓冲区已满）
 */
bool ring_buffer_add(ring_buffer_t *p_rb, const void *data)
{
    if (p_rb == NULL || data == NULL || ring_buffer_is_full(p_rb))
    {
        return false;
    }

    memcpy(ring_buffer_get_address(p_rb, p_rb->head), data, p_rb->element_size);
    p_rb->head = ring_buffer_advance(p_rb, p_rb->head, 1U);
    p_rb->count++;

    return true;
}

/**
 * @brief 强制添加单个数据到环形缓冲区
 * @param p_rb 环形缓冲区结构体指针
 * @param data 要添加的数据地址
 * @return true: 成功, false: 失败（参数错误）
 */
bool ring_buffer_add_force(ring_buffer_t *p_rb, const void *data)
{
    if (p_rb == NULL || data == NULL)
    {
        return false;
    }

    if (ring_buffer_is_full(p_rb))
    {
        p_rb->tail = ring_buffer_advance(p_rb, p_rb->tail, 1U);
        p_rb->count--;
    }

    memcpy(ring_buffer_get_address(p_rb, p_rb->head), data, p_rb->element_size);
    p_rb->head = ring_buffer_advance(p_rb, p_rb->head, 1U);
    p_rb->count++;

    return true;
}

/**
 * @brief 从环形缓冲区读取单个数据
 * @param p_rb 环形缓冲区结构体指针
 * @param data 读取数据的存储指针
 * @return true: 成功, false: 失败（缓冲区为空）
 */
bool ring_buffer_read(ring_buffer_t *p_rb, void *data)
{
    if (p_rb == NULL || data == NULL || ring_buffer_is_empty(p_rb))
    {
        return false;
    }

    memcpy(data, ring_buffer_get_address(p_rb, p_rb->tail), p_rb->element_size);
    p_rb->tail = ring_buffer_advance(p_rb, p_rb->tail, 1U);
    p_rb->count--;

    return true;
}

/**
 * @brief 批量写入数据到环形缓冲区
 *
 * @param p_rb 环形缓冲区结构体指针
 * @param data 要写入的数据指针
 * @param length 要写入的元素个数
 *
 * @return 实际写入的元素个数
 */
uint16_t ring_buffer_write(ring_buffer_t *p_rb, const void *data, uint16_t length)
{
    if (p_rb == NULL || data == NULL || length == 0U)
    {
        return 0;
    }

    uint16_t free_space = ring_buffer_get_free_space(p_rb);
    uint16_t write_count = (length > free_space) ? free_space : length;
    uint16_t first_count;
    const uint8_t *src = (const uint8_t *)data;

    if (write_count == 0U)
    {
        return 0;
    }

    first_count = p_rb->capacity - p_rb->head;
    if (first_count > write_count)
    {
        first_count = write_count;
    }

    memcpy(ring_buffer_get_address(p_rb, p_rb->head), src, (size_t)first_count * p_rb->element_size);

    if (write_count > first_count)
    {
        uint16_t second_count = write_count - first_count;
        memcpy(ring_buffer_get_address(p_rb, 0U),
               src + ((size_t)first_count * p_rb->element_size),
               (size_t)second_count * p_rb->element_size);
    }

    p_rb->head = ring_buffer_advance(p_rb, p_rb->head, write_count);
    p_rb->count += write_count;

    return write_count;
}

/**
 * @brief 批量从环形缓冲区读取数据
 *
 * @param p_rb 环形缓冲区结构体指针
 * @param data 读取数据的存储指针
 * @param length 要读取的元素个数
 *
 * @return 实际读取的元素个数
 */
uint16_t ring_buffer_read_multiple(ring_buffer_t *p_rb, void *data, uint16_t length)
{
    uint16_t available;
    uint16_t read_count;
    uint16_t first_count;
    uint8_t *dst = (uint8_t *)data;

    if (p_rb == NULL || data == NULL || length == 0U)
    {
        return 0;
    }

    available = ring_buffer_get_count(p_rb);
    read_count = (length > available) ? available : length;

    if (read_count == 0U)
    {
        return 0;
    }

    first_count = p_rb->capacity - p_rb->tail;
    if (first_count > read_count)
    {
        first_count = read_count;
    }

    memcpy(dst, ring_buffer_get_address(p_rb, p_rb->tail), (size_t)first_count * p_rb->element_size);

    if (read_count > first_count)
    {
        uint16_t second_count = read_count - first_count;
        memcpy(dst + ((size_t)first_count * p_rb->element_size),
               ring_buffer_get_address(p_rb, 0U),
               (size_t)second_count * p_rb->element_size);
    }

    p_rb->tail = ring_buffer_advance(p_rb, p_rb->tail, read_count);
    p_rb->count -= read_count;

    return read_count;
}

/**
 * @brief 检查环形缓冲区是否为空
 *
 * @param p_rb 环形缓冲区结构体指针
 *
 * @return true: 空, false: 非空
 */
bool ring_buffer_is_empty(const ring_buffer_t *p_rb)
{
    if (p_rb == NULL)
    {
        return true;
    }
    return (p_rb->count == 0);
}

/**
 * @brief 检查环形缓冲区是否已满
 *
 * @param p_rb 环形缓冲区结构体指针
 *
 * @return true: 满, false: 非满
 */
bool ring_buffer_is_full(const ring_buffer_t *p_rb)
{
    if (p_rb == NULL)
    {
        return false;
    }
    return (p_rb->count >= p_rb->capacity);
}

/**
 * @brief 获取环形缓冲区中的数据个数
 *
 * @param p_rb 环形缓冲区结构体指针
 *
 * @return 元素个数
 */
uint16_t ring_buffer_get_count(const ring_buffer_t *p_rb)
{
    if (p_rb == NULL)
    {
        return 0;
    }
    return p_rb->count;
}

/**
 * @brief 获取环形缓冲区的剩余空间
 *
 * @param p_rb 环形缓冲区结构体指针
 *
 * @return 剩余元素个数
 */
uint16_t ring_buffer_get_free_space(const ring_buffer_t *p_rb)
{
    if (p_rb == NULL)
    {
        return 0;
    }
    return (p_rb->capacity - p_rb->count);
}

/**
 * @brief 清空环形缓冲区
 *
 * @param p_rb 环形缓冲区结构体指针
 */
void ring_buffer_clear(ring_buffer_t *p_rb)
{
    if (p_rb == NULL)
    {
        return;
    }

    p_rb->head = 0U;
    p_rb->tail = 0U;
    p_rb->count = 0U;
}

/**
 * @brief 查看数据但不读取（peek）
 *
 * @param p_rb 环形缓冲区结构体指针
 * @param data 查看数据的存储指针
 * @param offset 相对于当前读取位置的元素偏移量
 *
 * @return true: 成功, false: 失败（超出范围或缓冲区为空）
 */
bool ring_buffer_peek(const ring_buffer_t *p_rb, void *data, uint16_t offset)
{
    uint16_t peek_index;

    if (p_rb == NULL || data == NULL || ring_buffer_is_empty(p_rb) || offset >= p_rb->count)
    {
        return false;
    }

    peek_index = ring_buffer_advance(p_rb, p_rb->tail, offset);
    memcpy(data, ring_buffer_get_address(p_rb, peek_index), p_rb->element_size);

    return true;
}
