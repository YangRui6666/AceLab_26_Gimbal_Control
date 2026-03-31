/**
 * @file bsp_usb.h
 * @brief USB虚拟串口通信包装层
 * @author CORE
 * @date 2026-03-15
 */

#ifndef BSP_USB_H
#define BSP_USB_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "vision_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief USB数据接收回调函数类型
 * @param data 接收到的数据指针
 * @param len 数据长度
 */
typedef void (*usb_rx_callback_t)(uint8_t* data, uint16_t len);

/**
 * @brief 初始化USB包装层
 * @retval true 初始化成功
 * @retval false 初始化失败
 */
bool bsp_usb_init(void);

/**
 * @brief 注册USB数据接收回调函数
 * @param callback 回调函数指针
 */
void bsp_usb_register_rx_callback(usb_rx_callback_t callback);

/**
 * @brief 通过USB发送数据
 * @param data 要发送的数据指针
 * @param len 数据长度
 * @retval true 发送成功
 * @retval false 发送失败
 */
bool bsp_usb_transmit(uint8_t* data, uint16_t len);

/**
 * @brief 从接收缓冲区获取数据
 * @param buffer 存储接收数据的缓冲区
 * @param max_len 缓冲区最大长度
 * @retval 实际读取的数据长度
 */
uint16_t bsp_usb_get_rx_data(uint8_t* buffer, uint16_t max_len);

/**
 * @brief 检查接收缓冲区是否有数据
 * @retval true 有数据可读
 * @retval false 无数据
 */
bool bsp_usb_has_data(void);

/**
 * @brief 清空接收缓冲区
 */
void bsp_usb_clear_rx_buffer(void);

/**
 * @brief 获取接收缓冲区中可读数据的长度
 * @retval 可读数据长度
 */
uint16_t bsp_usb_get_rx_length(void);

/**
 * @brief USB接收数据处理函数(由USB CDC接口回调)
 * @param buf 接收数据缓冲区
 * @param len 数据长度
 * @note 此函数为弱函数，在usbd_cdc_if.c中被重定义
 */
void usb_cdc_rx_handler(uint8_t* buf, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif // BSP_USB_H