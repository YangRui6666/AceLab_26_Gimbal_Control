#ifndef USER_CONFIG_CAN_ID_H
#define USER_CONFIG_CAN_ID_H

// 发命令：DJI GM6020 电机组 1 (电机1-4)
#define CAN_STDID_DJI_GROUP1 0x200
// 发命令：DJI GM6020 电机组 2 (电机5-8)
#define CAN_STDID_DJI_GROUP2 0x2FF

// 收反馈：云台 Yaw 电机
#define CAN_STDID_GIMBAL_YAW_FB   0x204
// 收反馈：云台 Pitch 电机
#define CAN_STDID_GIMBAL_PITCH_FB 0x206

#endif // USER_CONFIG_CAN_ID_H
