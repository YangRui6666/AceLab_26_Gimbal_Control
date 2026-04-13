// //
// // Created by CORE on 2026/3/4.
// //

// #include <stdbool.h>
// #include <stdint.h>
// #include <string.h>

// #include "../Inc/bsp_can.h"
// #include "../../Config/can_id.h"
// #include "main.h"
// #include "FreeRTOS.h"
// #include "task.h"
// #include "stm32f4xx_hal.h"
// extern CAN_HandleTypeDef hcan1;

// // 全局状态管理
// static gm6020_state_t motor_states[2];      // [0]=Yaw, [1]=Pitch
// static ring_buffer_t can_rx_buffer;
// static can_rx_msg_t can_rx_data[64];        // 环形缓冲区存储空间(64个元素)

// // 静态函数声明
// static void parse_motor_feedback(const can_rx_msg_t *msg);
// static void handle_motor_unwrap(gm6020_state_t *motor, int16_t raw_pos);
// static bool check_motor_data(const can_rx_msg_t *msg);

// typedef struct
// {
//     int16_t canid;
//     int16_t data_1;
//     int16_t data_2;
//     int16_t data_3;
//     int16_t data_4;
// }gm6020_can_ctrl_t;

// /**
//  * @brief       can初始化函数入口,用于
//  *
//  * @date        2026-03-13
//  * @author      Rui.
//  *
//  */
// bool bsp_can_init(void)
// {
//     // 初始化环形缓冲区
//     if (!ring_buffer_init(&can_rx_buffer, can_rx_data, 64, sizeof(can_rx_msg_t)))
//     {
//         return false;
//     }

//     // 配置CAN过滤器 - 接收GM6020反馈
//     CAN_FilterTypeDef can_filter;
//     can_filter.FilterBank = 0;
//     can_filter.FilterMode = CAN_FILTERMODE_IDLIST;
//     can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
//     // 32-bit ID list mode uses FR1 as the first 32-bit ID and FR2 as the second.
//     // For standard frames, the 11-bit ID is placed in the high halfword (shifted by 5).
//     can_filter.FilterIdHigh = (CAN_STDID_GIMBAL_YAW_FB << 5);
//     can_filter.FilterIdLow = 0x0000;
//     can_filter.FilterMaskIdHigh = (CAN_STDID_GIMBAL_PITCH_FB << 5);
//     can_filter.FilterMaskIdLow = 0x0000;
//     can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
//     can_filter.FilterActivation = ENABLE;
//     can_filter.SlaveStartFilterBank = 14;

//     if (HAL_CAN_ConfigFilter(&hcan2, &can_filter) != HAL_OK)
//     {
//         return false;
//     }

//     // 启动CAN并启用FIFO0接收中断
//     if (HAL_CAN_Start(&hcan2) != HAL_OK)
//     {
//         return false;
//     }

//     if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
//     {
//         return false;
//     }

//     // 初始化电机状态
//     for (int i = 0; i < 2; i++)
//     {
//         memset(&motor_states[i], 0, sizeof(gm6020_state_t));
//     }

//     return true;
// }

// bool bsp_can_send_std(uint16_t std_id, const uint8_t data[8], uint8_t dlc)
// {
//     CAN_TxHeaderTypeDef tx_header;
//     uint8_t tx_data[8] = {0};
//     uint32_t tx_mailbox;

//     if (data == NULL || dlc > 8U)
//     {
//         return false;
//     }

//     memcpy(tx_data, data, dlc);

//     tx_header.StdId = std_id;
//     tx_header.IDE = CAN_ID_STD;
//     tx_header.RTR = CAN_RTR_DATA;
//     tx_header.DLC = dlc;

//     return HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, &tx_mailbox) == HAL_OK;
// }

// bool bsp_can_pop_rx(can_rx_msg_t *out)
// {
//     if (out == NULL)
//     {
//         return false;
//     }

//     return ring_buffer_read(&can_rx_buffer, out);
// }

// /**
//  * @brief       用于控制电机的底层发送函数
//  *              设计思路：调用该函数，直接给can总线去发送消息
//  *              传入你需要操作的dji电机的控制帧以及各个电机的电流值
//  * 
//  * @date        2026-03-13
//  * @author      Rui.
//  * 
//  * @param tx_std_id 要发送的帧id
//  * @param data_1    逻辑顺序为1的电机电流
//  * @param data_2    逻辑顺序为2的电机电流
//  * @param data_3    逻辑顺序为3的电机电流
//  * @param data_4    逻辑顺序为4的电机电流
//  */
// void dji_motor_tx(uint16_t tx_std_id, const int16_t data_1, const int16_t data_2, const int16_t data_3, const int16_t data_4)
// {
//     uint8_t tx_data[8];

//     tx_data[0] = (uint8_t)(data_1 >> 8);        // 电机 1 高字节
//     tx_data[1] = (uint8_t)(data_1 & 0xFF);      // 电机 1 低字节
//     tx_data[2] = (uint8_t)(data_2 >> 8);        // 电机 2 高字节
//     tx_data[3] = (uint8_t)(data_2 & 0xFF);      // 电机 2 低字节
//     tx_data[4] = (uint8_t)(data_3 >> 8);        // 电机 3 高字节
//     tx_data[5] = (uint8_t)(data_3 & 0xFF);      // 电机 3 低字节
//     tx_data[6] = (uint8_t)(data_4 >> 8);        // 电机 4 高字节
//     tx_data[7] = (uint8_t)(data_4 & 0xFF);      // 电机 4 低字节

//     if (!bsp_can_send_std(tx_std_id, tx_data, 8U))
//     {
//         // 在此处添加错误处理逻辑，例如设置错误标志或重试
//     }
// }

// // CAN 批量控制电机函数 - 同时控制最多 8 个电机
// // motor_currents: 长度为 8 的数组，索引 0-7 对应电机 1-8，值为 0 表示不控制该电机
// /**
//  * @brief       CAN 批量控制电机函数
//  *              通过这个函数，我们可以直接控制GM6020的8组电机，同时控制这八组电机
//  *              组内会自动处理can帧的发送，并且没有被赋值的电机组会自动被设置为0
//  *              使用方法：
//  *              入参motor_currents：传入一个长度为8的数组的指针
//  *              效果：按照每一个电机的逻辑位置控制电流值
//  * @date        2026-04-01
//  * @author      Rui.
//  * 
//  * @param motor_currents 
//  */
// void bsp_ctrl_motor(const int16_t *motor_currents)
// {
//     if (motor_currents != NULL)
//     {
//         gm6020_can_ctrl_t motor_ctrl;
//         uint8_t has_data_group1 = 0;  // 组 1(电机 1-4) 是否有数据
//         uint8_t has_data_group2 = 0;  // 组 2(电机 5-8) 是否有数据

//         motor_ctrl.data_1 = 0;
//         motor_ctrl.data_2 = 0;
//         motor_ctrl.data_3 = 0;
//         motor_ctrl.data_4 = 0;

//         if (motor_currents[0] != 0)
//         {
//             motor_ctrl.data_1 = motor_currents[0];
//             has_data_group1 = 1;
//         }
//         if (motor_currents[1] != 0)
//         {
//             motor_ctrl.data_2 = motor_currents[1];
//             has_data_group1 = 1;
//         }
//         if (motor_currents[2] != 0)
//         {
//             motor_ctrl.data_3 = motor_currents[2];
//             has_data_group1 = 1;
//         }
//         if (motor_currents[3] != 0)
//         {
//             motor_ctrl.data_4 = motor_currents[3];
//             has_data_group1 = 1;
//         }

//         if (has_data_group1)
//         {
//             dji_motor_tx(CAN_STDID_DJI_GROUP1, motor_ctrl.data_1, motor_ctrl.data_2, motor_ctrl.data_3, motor_ctrl.data_4);
//         }

//         motor_ctrl.data_1 = 0;
//         motor_ctrl.data_2 = 0;
//         motor_ctrl.data_3 = 0;
//         motor_ctrl.data_4 = 0;
//         if (motor_currents[4] != 0)
//         {
//             motor_ctrl.data_1 = motor_currents[4];
//             has_data_group2 = 1;
//         }
//         if (motor_currents[5] != 0)
//         {
//             motor_ctrl.data_2 = motor_currents[5];
//             has_data_group2 = 1;
//         }
//         if (motor_currents[6] != 0)
//         {
//             motor_ctrl.data_3 = motor_currents[6];
//             has_data_group2 = 1;
//         }
//         if (motor_currents[7] != 0)
//         {
//             motor_ctrl.data_4 = motor_currents[7];
//             has_data_group2 = 1;
//         }

//         if (has_data_group2)
//         {
//             dji_motor_tx(CAN_STDID_DJI_GROUP2, motor_ctrl.data_1, motor_ctrl.data_2, motor_ctrl.data_3, motor_ctrl.data_4);
//         }
//     }
//     else
//     {

//     }
// }

// //此处编写can的接受中断，创建环形缓冲区并且写入原始数据

// /**
//  * @brief CAN接收中断回调函数、数据的直接写入
//  * @param[in] hcan CAN句柄
//  */

//  /**
//   * @brief      CAN接收中断回调函数
//   * @note       作用：读取接受can总线上传来的信息
//   *             此函数通过中断触发，无需手动调用
//   *             中断触发后，can消息会进入环形缓冲区
//   *             配置了过滤器，自动过滤
//   *             行为：
//   *             中断触发后，程序将电机消息添加到缓冲区中（覆盖模式）
//   * 
//   * 
//   * @date        2026-04-01
//   * @author      Rui.
//   * 
//   * @param hcan 
//   */
// void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
// {
//     if (hcan == &hcan2)
//     {
//         CAN_RxHeaderTypeDef rx_header;
//         can_rx_msg_t rx_msg;

//         if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx_header, rx_msg.data) == HAL_OK)
//         {
//             // 只过滤需要的电机ID，减少中断处理时间
//             if (rx_header.StdId == CAN_STDID_GIMBAL_YAW_FB ||
//                 rx_header.StdId == CAN_STDID_GIMBAL_PITCH_FB)
//             {
//                 rx_msg.std_id = rx_header.StdId;
//                 rx_msg.dlc = rx_header.DLC;
//                 rx_msg.timestamp = xTaskGetTickCountFromISR();

//                 // 存入环形缓冲区
//                 ring_buffer_add_force(&can_rx_buffer, &rx_msg);
//             }
//         }
//     }
// }

// /**
//  * @brief 处理CAN接收数据
//  *        暴露给外界函数的接口，需要周期性调用更新数据
//  */

//  /**
//   * @brief       外部函数，用于处理CAN接收数据
//   * 
//   * @note       该函数是一个触发器
//   *             用于触发"parse_motor_feedback"函数
//   *             函数作用是去阅读，解析，处理can消息
//   * 
//   * @date        2026-04-01
//   * @author      Rui.
//   * 
//   */
// void bsp_process_can_rx_data(void)
// {
//     can_rx_msg_t rx_msg;

//     // 批量处理缓冲区中的数据
//     while (ring_buffer_read(&can_rx_buffer, &rx_msg))
//     {
//         parse_motor_feedback(&rx_msg);
//     }
// }

// /**
//  * @brief 电机反馈解析函数 (包含解旋和滤波)
//  * @param[in] msg CAN接收消息
//  */

//  /**
//   * @brief       can消息解析函数
//   * 
//   * @note       该函数为内部函数
//   *             被对外函数"bsp_process_can_rx_data"调用后
//   *             函数会去读取处理can的环形缓冲区内部的数据
//   *             按照GM6020的解析协议去解读
//   *             解读完毕之后，写入"motor_states"中
//   * 
//   * @date        2026-04-01
//   * @author      Rui.
//   * 
//   * @param msg 
//   */
// static void parse_motor_feedback(const can_rx_msg_t *msg)
// {
//     // 数据校验
//     if (!check_motor_data(msg))
//     {
//         return;
//     }

//     gm6020_state_t *motor = NULL;

//     // 根据CAN ID确定电机
//     if (msg->std_id == CAN_STDID_GIMBAL_YAW_FB)
//     {
//         motor = &motor_states[0];  // Yaw电机
//     }
//     else if (msg->std_id == CAN_STDID_GIMBAL_PITCH_FB)
//     {
//         motor = &motor_states[1];  // Pitch电机
//     }
//     else
//     {
//         return;  // 未知ID
//     }

//     // GM6020数据格式解析
//     int16_t raw_pos = (int16_t)((msg->data[0] << 8) | msg->data[1]);
//     int16_t raw_speed = (int16_t)((msg->data[2] << 8) | msg->data[3]);
//     int16_t raw_current = (int16_t)((msg->data[4] << 8) | msg->data[5]);
//     int8_t raw_temp = (int8_t)msg->data[6];

//     // 解旋算法 (处理位置跳跃)
//     handle_motor_unwrap(motor, raw_pos);

//     // 更新其他原始数据
//     motor->pos = raw_pos;
//     motor->speed = raw_speed;
//     motor->current = raw_current;
//     motor->temp = raw_temp;
//     motor->last_recv_time = msg->timestamp;

//     // 电流低通滤波
//     motor->filtered_current = CURRENT_FILTER_ALPHA * raw_current +
//                              (1.0f - CURRENT_FILTER_ALPHA) * motor->filtered_current;
// }

// /**
//  * @brief 处理电机位置解旋
//  * @param[in,out] motor 电机状态结构体
//  * @param[in] raw_pos 原始位置
//  */
// static void handle_motor_unwrap(gm6020_state_t *motor, int16_t raw_pos)
// {
//     // 首次初始化
//     if (motor->last_recv_time == 0)
//     {
//         motor->last_pos = raw_pos;
//         motor->total_angle = raw_pos;
//         motor->turn_count = 0;
//         return;
//     }

//     // 计算位置差值
//     int16_t delta_pos = raw_pos - motor->last_pos;

//     // 检测跨零跳跃 (从8191跳到0或从0跳到8191)
//     if (delta_pos > MOTOR_POS_RANGE / 2)
//     {
//         // 逆时针跨零 (8191 -> 0)
//         motor->turn_count--;
//     }
//     else if (delta_pos < -MOTOR_POS_RANGE / 2)
//     {
//         // 顺时针跨零 (0 -> 8191)
//         motor->turn_count++;
//     }

//     // 更新累计角度
//     motor->total_angle = motor->turn_count * MOTOR_POS_RANGE + raw_pos;
//     motor->last_pos = raw_pos;
// }

// /**
//  * @brief 校验电机数据
//  * @param[in] msg CAN接收消息
//  * @retval true 数据有效
//  * @retval false 数据无效
//  */
// static bool check_motor_data(const can_rx_msg_t *msg)
// {
//     // 检查数据长度
//     if (msg->dlc != 8) return false;

//     // 检查位置范围 (0-8191)
//     int16_t pos = (int16_t)((msg->data[0] << 8) | msg->data[1]);
//     if (pos < 0 || pos >= MOTOR_POS_RANGE) return false;

//     return true;
// }

// /**
//  * @brief 获取电机结构体指针
//  * @param[in] motor_id 电机ID (0=Yaw, 1=Pitch)
//  * @retval 电机结构体指针，失败返回NULL
//  */
// const gm6020_state_t* bsp_can_get_motor_state(uint8_t motor_id)
// {
//     if (motor_id >= 2)
//     {
//         return NULL;
//     }
//     return &motor_states[motor_id];
// }

// /**
//  * @brief 获取电机累计角度 (含圈数)
//  * @param[in] motor_id 电机ID
//  * @retval 累计角度值
//  */
// int32_t get_can_get_motor_angle(uint8_t motor_id)
// {
//     if (motor_id >= 2)
//     {
//         return 0;
//     }
//     return motor_states[motor_id].total_angle;
// }

// /**
//  * @brief 检查电机通信超时
//  * @param[in] motor_id 电机ID
//  * @retval true 超时, false 正常
//  *
//  */
// bool bsp_can_motor_is_timeout(uint8_t motor_id)
// {
//     if (motor_id >= 2)
//     {
//         return true;
//     }
//     return (xTaskGetTickCount() - motor_states[motor_id].last_recv_time) > pdMS_TO_TICKS(MOTOR_TIMEOUT_MS);
// }

// /**
//  * @brief       重置电机的解旋计数
//  *              
//  * 
//  * @date        2026-04-01
//  * @author      Rui.
//  * 
//  * @param motor_id 
//  */
// void bsp_can_reset_motor(uint8_t motor_id)
// {
//     if (motor_id < 2)
//     {
//         motor_states[motor_id].turn_count = 0;
//         motor_states[motor_id].total_angle = motor_states[motor_id].pos;
//     }
// }
