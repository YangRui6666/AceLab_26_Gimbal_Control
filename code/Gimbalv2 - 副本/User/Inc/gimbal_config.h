#ifndef GIMBAL_CONFIG_H
#define GIMBAL_CONFIG_H

#define CTRL_TASK_PERIOD_MS                    (1U)
#define CTRL_MSG_QUEUE_LENGTH                  (16U)
#define COMM_SEMAPHORE_MAX_COUNT               (64U)
#define USB_RX_RING_BUFFER_SIZE                (512U)

#define COMM_TIMEOUT_MS                        (500U)
#define LOCK_TIMEOUT_MS                        (60000U)
#define MOTOR_OFFLINE_MS                       (20U)
#define STARTUP_GRACE_MS                       (200U)
#define TELEMETRY_PERIOD_MS                    (20U)

#define YAW_LIMIT_POS_DEG                      (60.0f)
#define YAW_LIMIT_NEG_DEG                      (-60.0f)
#define PITCH_LIMIT_POS_DEG                    (45.0f)
#define PITCH_LIMIT_NEG_DEG                    (-15.0f)
#define MOTOR_LIMIT_ERROR_MARGIN_DEG           (3.0f)

#define YAW_MAX_CURRENT_DEFAULT                (16000.0f)
#define PITCH_MAX_CURRENT_DEFAULT              (16000.0f)

#define YAW_POS_KP_DEFAULT                     (8.0f)
#define YAW_POS_KI_DEFAULT                     (0.0f)
#define YAW_POS_KD_DEFAULT                     (0.2f)
#define YAW_POS_OUT_MIN_DEFAULT                (-300.0f)
#define YAW_POS_OUT_MAX_DEFAULT                (300.0f)

#define YAW_SPD_KP_DEFAULT                     (15.0f)
#define YAW_SPD_KI_DEFAULT                     (0.5f)
#define YAW_SPD_KD_DEFAULT                     (0.0f)
#define YAW_SPD_OUT_MIN_DEFAULT                (-16000.0f)
#define YAW_SPD_OUT_MAX_DEFAULT                (16000.0f)

#define PITCH_POS_KP_DEFAULT                   (8.0f)
#define PITCH_POS_KI_DEFAULT                   (0.0f)
#define PITCH_POS_KD_DEFAULT                   (0.2f)
#define PITCH_POS_OUT_MIN_DEFAULT              (-300.0f)
#define PITCH_POS_OUT_MAX_DEFAULT              (300.0f)

#define PITCH_SPD_KP_DEFAULT                   (15.0f)
#define PITCH_SPD_KI_DEFAULT                   (0.5f)
#define PITCH_SPD_KD_DEFAULT                   (0.0f)
#define PITCH_SPD_OUT_MIN_DEFAULT              (-16000.0f)
#define PITCH_SPD_OUT_MAX_DEFAULT              (16000.0f)

#define SEARCH_YAW_CENTER_DEG                  (0.0f)
#define SEARCH_PITCH_CENTER_DEG                (10.0f)
#define SEARCH_YAW_AMP_DEG                     (25.0f)
#define SEARCH_PITCH_AMP_DEG                   (8.0f)
#define SEARCH_W_RAD                           (2.5132741f)
#define SEARCH_PITCH_PHASE_RAD                 (1.5707963f)

#define YAW_MOTOR_CAN_ID                       (0x205U)
#define PITCH_MOTOR_CAN_ID                     (0x206U)
#define GIMBAL_CURRENT_TX_CAN_ID               (0x1FFU)
#define YAW_MOTOR_CTRL_SLOT                    (0U)
#define PITCH_MOTOR_CTRL_SLOT                  (1U)
#define YAW_ENCODER_ZERO_RAW                   (0U)
#define PITCH_ENCODER_ZERO_RAW                 (0U)

#define BMI088_ACCEL_CHIP_ID                   (0x1EU)
#define BMI088_GYRO_CHIP_ID                    (0x0FU)
#define BMI088_ACCEL_RANGE_G                   (6.0f)
#define BMI088_GYRO_RANGE_DPS                  (2000.0f)
#define BMI088_ACCEL_LSB_PER_G                 (5460.0f)
#define BMI088_GYRO_LSB_PER_DPS                (16.4f)
#define IMU_SAMPLE_FREQ_HZ                     (1000.0f)
#define IMU_INIT_RETRY_CNT                     (3U)
#define IMU_INIT_RETRY_DELAY_MS                (20U)

#define PROTOCOL_SOF0                          (0xAAU)
#define PROTOCOL_SOF1                          (0x55U)
#define PROTOCOL_EOF0                          (0x5AU)
#define PROTOCOL_EOF1                          (0xA5U)
#define PROTOCOL_MAX_DATA_LEN                  (32U)

#define CMD_ID_ENABLE_TELEMETRY                (0x82U)
#define CMD_ID_ENTER_SEARCH                    (0x83U)
#define CMD_ID_AUTO_AIM                        (0x84U)
#define CMD_ID_ENTER_LOCK                      (0x87U)
#define CMD_ID_EXIT_LOCK                       (0x88U)

#define CMD_ID_STATUS_FEEDBACK                 (0x03U)
#define CMD_ID_LOCK_FEEDBACK                   (0x08U)

#endif
