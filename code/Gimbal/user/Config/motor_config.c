// //
// // Created by CORE on 2026/3/12.
// //
//
// #include "motor_config.h"
// #include "can_id.h"
// #include "Tools/ring_buffer.h"
//
// static motor_handle_t motor_table[MOTOR_ID_COUNT];
//
//
// /**
//  * @brief       初始化函数，调用此函数进行电机的创建
//  *
//  * @date        2026-03-14
//  * @author      Rui.
//  *
//  * @warning     注意更新can_id.h
//  */
// void motor_init(void)
// {
//     //yaw电机的init
//     motor_table[MOTOR_ID_YAW].id = MOTOR_ID_YAW;
//     motor_table[MOTOR_ID_YAW].cfg.rx_std_id = CAN_STDID_GIMBAL_YAW_FB;
//     motor_table[MOTOR_ID_YAW].cfg.tx_std_id = CAN_STDID_DJI_GROUP1;
//     motor_table[MOTOR_ID_YAW].cfg.tx_slot = 1U;
//     motor_table[MOTOR_ID_YAW].cfg.current_limit = 20000;
//
//     //pitch电机的init
//     motor_table[MOTOR_ID_PITCH].id = MOTOR_ID_PITCH;
//     motor_table[MOTOR_ID_PITCH].cfg.rx_std_id = CAN_STDID_GIMBAL_PITCH_FB;
//     motor_table[MOTOR_ID_PITCH].cfg.tx_std_id = CAN_STDID_DJI_GROUP1;
//     motor_table[MOTOR_ID_PITCH].cfg.tx_slot = 2U;
//     motor_table[MOTOR_ID_PITCH].cfg.current_limit = 20000;
// }
//
// /**
//  * @brief       返回一个总表
//  *
//  * @date        2026-03-14
//  * @author      Rui.
//  *
//  * @return      motor_handle_t*
//  */
// motor_handle_t *motor_get_table(void)
// {
//     return motor_table;
// }
//
// /**
//  * @brief       获取一个电机的只读句柄
//  *
//  * @date        2026-03-14
//  * @author      Rui.
//  *
//  * @param id    电机创建时的id
//  * @return const motor_handle_t*
//  *
//  */
// const motor_handle_t *motor_get_config(motor_logic_id id)
// {
//     if ((size_t)id >= MOTOR_ID_COUNT)
//     {
//         return NULL;
//     }
//     return &motor_table[id];
// }
//
// /**
//  * @brief       获取一个电机的句柄
//  *
//  * @date        2026-03-14
//  * @author      Rui.
//  *
//  * @param id    电机创建时的id
//  * @return motor_handle_t*
//  *
//  */
// motor_handle_t *motor_get_handle(motor_logic_id id)
// {
//     if ((size_t)id >= MOTOR_ID_COUNT)
//     {
//         return NULL;
//     }
//     return &motor_table[id];
// }
//
// uint8_t motor_count(void)
// {
//     return MOTOR_ID_COUNT;
// }
//
