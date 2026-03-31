// #ifndef USER_CONFIG_MOTOR_CONFIG_H
// #define USER_CONFIG_MOTOR_CONFIG_H
//
// #include <stdint.h>
// #include <stddef.h>
//
// typedef enum
// {
//     MOTOR_ID_YAW = 0,
//     MOTOR_ID_PITCH,
//     MOTOR_ID_COUNT
// } motor_logic_id;
//
// //电机的固定数据
// typedef struct
// {
//     uint16_t rx_std_id;      // 反馈使用的 CAN StdId
//     uint16_t tx_std_id;      // 控制帧使用的 CAN StdId（组 ID）
//     uint8_t  tx_slot;        // 在一帧中的位置：1..4
//     int16_t  current_limit;  // 目标电流限幅
// } motor_config_t;
//
// typedef struct
// {
//     // 运行态数据
//     int16_t raw_angle;      //角度
//     int16_t raw_speed;      //速度
//     int16_t raw_current;    //电流
//     uint8_t temperature;    //温度
//     uint32_t last_rx_tick;      //上次接受到的时间
//     int16_t target_current;     //电流
// }motor_data_t;
//
// typedef struct
// {
//     motor_logic_id id;      //在这里写入逻辑顺序
//     motor_config_t cfg;     //配置表，包含固定数据
//     motor_data_t data;
// } motor_handle_t;
//
// void motor_init(void);
// uint8_t motor_count(void);
// motor_handle_t *motor_get_table(void);
// const motor_handle_t *motor_get_config(motor_logic_id id);
// motor_handle_t *motor_get_handle(motor_logic_id id);
//
// #endif // USER_CONFIG_MOTOR_CONFIG_H
