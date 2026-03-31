/**
 * @file bsp_usb.c
 * @brief USB虚拟串口通信包装层实现
 * @author CORE
 * @date 2026-03-15
 */

#include "bsp_usb.h"
#include "usbd_cdc_if.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "cmsis_os2.h"
#include "main.h"

// 私有变量
static uint8_t usb_rx_buffer[VISION_USB_RX_BUFFER_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;
static usb_rx_callback_t rx_callback = NULL;

// 外部信号量句柄(在main.c中定义)
extern osSemaphoreId_t VisionBinarySemHandle;

/**
 * @brief USB包装层初始化
 * @retval true 初始化成功
 * @retval false 初始化失败
 */
bool bsp_usb_init(void) {
    // 初始化环形缓冲区
    rx_head = 0;
    rx_tail = 0;
    rx_callback = NULL;

    return true;
}

/**
 * @brief 注册USB数据接收回调函数
 * @param callback 回调函数指针
 */
void bsp_usb_register_rx_callback(usb_rx_callback_t callback) {
    rx_callback = callback;
}

/**
 * @brief 通过USB发送数据
 * @param data 要发送的数据指针
 * @param len 数据长度
 * @retval true 发送成功
 * @retval false 发送失败
 */
bool bsp_usb_transmit(uint8_t* data, uint16_t len) {
    if (data == NULL || len == 0) {
        return false;
    }

    // 调用USB CDC发送函数
    uint8_t result = CDC_Transmit_FS(data, len);
    return (result == USBD_OK);
}

/**
 * @brief 从接收缓冲区获取数据
 * @param buffer 存储接收数据的缓冲区
 * @param max_len 缓冲区最大长度
 * @retval 实际读取的数据长度
 */
uint16_t bsp_usb_get_rx_data(uint8_t* buffer, uint16_t max_len) {
    if (buffer == NULL || max_len == 0) {
        return 0;
    }

    uint16_t count = 0;

    // 关中断保证数据一致性
    taskENTER_CRITICAL();

    while (rx_tail != rx_head && count < max_len) {
        buffer[count++] = usb_rx_buffer[rx_tail];
        rx_tail = (rx_tail + 1) % VISION_USB_RX_BUFFER_SIZE;
    }

    taskEXIT_CRITICAL();

    return count;
}

/**
 * @brief 检查接收缓冲区是否有数据
 * @retval true 有数据可读
 * @retval false 无数据
 */
bool bsp_usb_has_data(void) {
    return (rx_head != rx_tail);
}

/**
 * @brief 清空接收缓冲区
 */
void bsp_usb_clear_rx_buffer(void) {
    taskENTER_CRITICAL();
    rx_head = rx_tail = 0;
    taskEXIT_CRITICAL();
}

/**
 * @brief 获取接收缓冲区中可读数据的长度
 * @retval 可读数据长度
 */
uint16_t bsp_usb_get_rx_length(void) {
    uint16_t head = rx_head;
    uint16_t tail = rx_tail;

    if (head >= tail) {
        return head - tail;
    } else {
        return VISION_USB_RX_BUFFER_SIZE - tail + head;
    }
}

/**
 * @brief USB接收数据处理函数(弱函数实现)
 * @param buf 接收数据缓冲区
 * @param len 数据长度
 * @note 此函数在usbd_cdc_if.c中被调用
 */
__weak void usb_cdc_rx_handler(uint8_t* buf, uint32_t len) {
    if (buf == NULL || len == 0) {
        return;
    }

    // 存储到环形缓冲区
    for (uint32_t i = 0; i < len; i++) {
        uint16_t next_head = (rx_head + 1) % VISION_USB_RX_BUFFER_SIZE;

        // 检查缓冲区是否满
        if (next_head != rx_tail) {
            usb_rx_buffer[rx_head] = buf[i];
            rx_head = next_head;
        } else {
            // 缓冲区满，丢弃数据(或者覆盖最旧的数据)
            break;
        }
    }

    // 触发回调函数
    if (rx_callback != NULL) {
        rx_callback(buf, (uint16_t)len);
    }

    // 释放信号量通知任务有新数据
    if (VisionBinarySemHandle != NULL) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        osSemaphoreRelease(VisionBinarySemHandle);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

