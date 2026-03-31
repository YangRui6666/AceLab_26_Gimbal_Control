//
// Created by CORE on 2026/3/12.
//

#include "../Inc/bsp_bmi088.h"

#include "main.h"
#include "stm32f4xx_hal.h"

#define BMI088_SPI_TIMEOUT_MS            10U
#define BMI088_RESET_DELAY_MS            50U
#define BMI088_REG_DELAY_MS              1U
#define BMI088_DATA_TIMEOUT_MS           50U
#define BMI088_READ_MASK                 0x80U

#define BMI088_ACC_CHIP_ID_REG           0x00U
#define BMI088_ACC_CHIP_ID_VALUE         0x1EU
#define BMI088_ACCEL_XOUT_L_REG          0x12U
#define BMI088_ACC_CONF_REG              0x40U
#define BMI088_ACC_RANGE_REG             0x41U
#define BMI088_ACC_PWR_CONF_REG          0x7CU
#define BMI088_ACC_PWR_CTRL_REG          0x7DU
#define BMI088_ACC_SOFTRESET_REG         0x7EU
#define BMI088_ACC_SOFTRESET_VALUE       0xB6U
#define BMI088_ACC_ENABLE_ACC_ON         0x04U
#define BMI088_ACC_PWR_ACTIVE_MODE       0x00U
#define BMI088_ACC_CONF_MUST_SET         0x80U
#define BMI088_ACC_NORMAL                0x20U
#define BMI088_ACC_800_HZ                0x0BU
#define BMI088_ACC_RANGE_3G              0x00U

#define BMI088_GYRO_CHIP_ID_REG          0x00U
#define BMI088_GYRO_CHIP_ID_VALUE        0x0FU
#define BMI088_GYRO_X_L_REG              0x02U
#define BMI088_GYRO_RANGE_REG            0x0FU
#define BMI088_GYRO_BANDWIDTH_REG        0x10U
#define BMI088_GYRO_LPM1_REG             0x11U
#define BMI088_GYRO_SOFTRESET_REG        0x14U
#define BMI088_GYRO_CTRL_REG             0x15U
#define BMI088_GYRO_SOFTRESET_VALUE      0xB6U
#define BMI088_GYRO_2000DPS              0x00U
#define BMI088_GYRO_BANDWIDTH_MUST_SET   0x80U
#define BMI088_GYRO_1000_116_HZ          0x02U
#define BMI088_GYRO_NORMAL_MODE          0x00U
#define BMI088_GYRO_DRDY_ON              0x80U

extern SPI_HandleTypeDef hspi1;

typedef enum
{
    BMI088_SENSOR_ACCEL = 0,
    BMI088_SENSOR_GYRO
} bmi088_sensor_t;

typedef struct
{
    uint8_t reg;
    uint8_t value;
} bmi088_reg_cfg_t;

static bool bmi088_ready = false;
static uint32_t bmi088_last_tick = 0U;
static bmi088_data_t bmi088_last_data = {0};

static void bmi088_release_bus(void);
static void bmi088_select_sensor(bmi088_sensor_t sensor);
static HAL_StatusTypeDef bmi088_write_reg(bmi088_sensor_t sensor, uint8_t reg, uint8_t value);
static HAL_StatusTypeDef bmi088_read_reg(bmi088_sensor_t sensor, uint8_t reg, uint8_t *value);
static HAL_StatusTypeDef bmi088_read_regs(bmi088_sensor_t sensor, uint8_t reg, uint8_t *buffer, uint16_t length);
static bool bmi088_read_chip_id(bmi088_sensor_t sensor, uint8_t expected_id);
static bool bmi088_apply_config(bmi088_sensor_t sensor, const bmi088_reg_cfg_t *config, uint32_t config_num);
static bool bmi088_accel_init(void);
static bool bmi088_gyro_init(void);
static int16_t bmi088_parse_axis(uint8_t low_byte, uint8_t high_byte);

/**
 * @brief       BMI088 初始化
 * @retval      true 初始化成功
 * @retval      false 初始化失败
 */
bool bsp_imu_init(void)
{
    bmi088_ready = false;
    bmi088_last_tick = 0U;
    bmi088_last_data = (bmi088_data_t){0};

    bmi088_release_bus();

    if (!bmi088_accel_init())
    {
        return false;
    }

    if (!bmi088_gyro_init())
    {
        return false;
    }

    bmi088_ready = true;
    return true;
}

/**
 * @brief       读取 BMI088 六轴原始数据
 * @param[out]  imu_data 六轴原始数据输出结构体
 * @retval      true 读取成功
 * @retval      false 读取失败
 */
bool bsp_imu_get(bmi088_data_t *imu_data)
{
    uint8_t accel_buffer[6];
    uint8_t gyro_buffer[6];
    bmi088_data_t raw_data;

    if (imu_data == NULL || !bmi088_ready)
    {
        return false;
    }

    if (bmi088_read_regs(BMI088_SENSOR_ACCEL, BMI088_ACCEL_XOUT_L_REG, accel_buffer, sizeof(accel_buffer)) != HAL_OK)
    {
        return false;
    }

    if (bmi088_read_regs(BMI088_SENSOR_GYRO, BMI088_GYRO_X_L_REG, gyro_buffer, sizeof(gyro_buffer)) != HAL_OK)
    {
        return false;
    }

    raw_data.accel_x = bmi088_parse_axis(accel_buffer[0], accel_buffer[1]);
    raw_data.accel_y = bmi088_parse_axis(accel_buffer[2], accel_buffer[3]);
    raw_data.accel_z = bmi088_parse_axis(accel_buffer[4], accel_buffer[5]);
    raw_data.gyro_x = bmi088_parse_axis(gyro_buffer[0], gyro_buffer[1]);
    raw_data.gyro_y = bmi088_parse_axis(gyro_buffer[2], gyro_buffer[3]);
    raw_data.gyro_z = bmi088_parse_axis(gyro_buffer[4], gyro_buffer[5]);
    raw_data.tick = HAL_GetTick();

    bmi088_last_tick = raw_data.tick;
    bmi088_last_data = raw_data;
    *imu_data = raw_data;

    return true;
}

/**
 * @brief       检查 BMI088 最近一次读取是否超时
 * @retval      true 设备在线
 * @retval      false 设备离线或未初始化
 */
bool bsp_imu_check(void)
{
    if (!bmi088_ready || bmi088_last_tick == 0U)
    {
        return false;
    }

    return (HAL_GetTick() - bmi088_last_tick) <= BMI088_DATA_TIMEOUT_MS;
}

static void bmi088_release_bus(void)
{
    HAL_GPIO_WritePin(BMI088_ACCEL_CS_GPIO_Port, BMI088_ACCEL_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_SET);
}

static void bmi088_select_sensor(bmi088_sensor_t sensor)
{
    bmi088_release_bus();

    if (sensor == BMI088_SENSOR_ACCEL)
    {
        HAL_GPIO_WritePin(BMI088_ACCEL_CS_GPIO_Port, BMI088_ACCEL_CS_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_RESET);
    }
}

static HAL_StatusTypeDef bmi088_write_reg(bmi088_sensor_t sensor, uint8_t reg, uint8_t value)
{
    uint8_t tx_buffer[2] = {reg, value};
    HAL_StatusTypeDef status;

    bmi088_select_sensor(sensor);
    status = HAL_SPI_Transmit(&hspi1, tx_buffer, sizeof(tx_buffer), BMI088_SPI_TIMEOUT_MS);
    bmi088_release_bus();

    return status;
}

static HAL_StatusTypeDef bmi088_read_reg(bmi088_sensor_t sensor, uint8_t reg, uint8_t *value)
{
    return bmi088_read_regs(sensor, reg, value, 1U);
}

static HAL_StatusTypeDef bmi088_read_regs(bmi088_sensor_t sensor, uint8_t reg, uint8_t *buffer, uint16_t length)
{
    uint8_t address = reg | BMI088_READ_MASK;
    uint8_t dummy_byte = 0x55U;
    HAL_StatusTypeDef status;

    if (buffer == NULL || length == 0U)
    {
        return HAL_ERROR;
    }

    bmi088_select_sensor(sensor);

    status = HAL_SPI_Transmit(&hspi1, &address, 1U, BMI088_SPI_TIMEOUT_MS);
    if (status == HAL_OK && sensor == BMI088_SENSOR_ACCEL)
    {
        status = HAL_SPI_Transmit(&hspi1, &dummy_byte, 1U, BMI088_SPI_TIMEOUT_MS);
    }
    if (status == HAL_OK)
    {
        status = HAL_SPI_Receive(&hspi1, buffer, length, BMI088_SPI_TIMEOUT_MS);
    }

    bmi088_release_bus();
    return status;
}

static bool bmi088_read_chip_id(bmi088_sensor_t sensor, uint8_t expected_id)
{
    uint8_t chip_id = 0U;

    if (bmi088_read_reg(sensor, BMI088_ACC_CHIP_ID_REG, &chip_id) != HAL_OK)
    {
        return false;
    }

    return chip_id == expected_id;
}

static bool bmi088_apply_config(bmi088_sensor_t sensor, const bmi088_reg_cfg_t *config, uint32_t config_num)
{
    uint32_t index;
    uint8_t readback = 0U;

    if (config == NULL)
    {
        return false;
    }

    for (index = 0U; index < config_num; index++)
    {
        if (bmi088_write_reg(sensor, config[index].reg, config[index].value) != HAL_OK)
        {
            return false;
        }

        HAL_Delay(BMI088_REG_DELAY_MS);

        if (bmi088_read_reg(sensor, config[index].reg, &readback) != HAL_OK)
        {
            return false;
        }

        if (readback != config[index].value)
        {
            return false;
        }
    }

    return true;
}

static bool bmi088_accel_init(void)
{
    static const bmi088_reg_cfg_t accel_config[] =
    {
        {BMI088_ACC_PWR_CTRL_REG, BMI088_ACC_ENABLE_ACC_ON},
        {BMI088_ACC_PWR_CONF_REG, BMI088_ACC_PWR_ACTIVE_MODE},
        {BMI088_ACC_CONF_REG, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_SET},
        {BMI088_ACC_RANGE_REG, BMI088_ACC_RANGE_3G}
    };

    if (!bmi088_read_chip_id(BMI088_SENSOR_ACCEL, BMI088_ACC_CHIP_ID_VALUE))
    {
        return false;
    }

    if (bmi088_write_reg(BMI088_SENSOR_ACCEL, BMI088_ACC_SOFTRESET_REG, BMI088_ACC_SOFTRESET_VALUE) != HAL_OK)
    {
        return false;
    }

    HAL_Delay(BMI088_RESET_DELAY_MS);

    if (!bmi088_read_chip_id(BMI088_SENSOR_ACCEL, BMI088_ACC_CHIP_ID_VALUE))
    {
        return false;
    }

    return bmi088_apply_config(BMI088_SENSOR_ACCEL, accel_config, sizeof(accel_config) / sizeof(accel_config[0]));
}

static bool bmi088_gyro_init(void)
{
    static const bmi088_reg_cfg_t gyro_config[] =
    {
        {BMI088_GYRO_RANGE_REG, BMI088_GYRO_2000DPS},
        {BMI088_GYRO_BANDWIDTH_REG, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_SET},
        {BMI088_GYRO_LPM1_REG, BMI088_GYRO_NORMAL_MODE},
        {BMI088_GYRO_CTRL_REG, BMI088_GYRO_DRDY_ON}
    };

    if (!bmi088_read_chip_id(BMI088_SENSOR_GYRO, BMI088_GYRO_CHIP_ID_VALUE))
    {
        return false;
    }

    if (bmi088_write_reg(BMI088_SENSOR_GYRO, BMI088_GYRO_SOFTRESET_REG, BMI088_GYRO_SOFTRESET_VALUE) != HAL_OK)
    {
        return false;
    }

    HAL_Delay(BMI088_RESET_DELAY_MS);

    if (!bmi088_read_chip_id(BMI088_SENSOR_GYRO, BMI088_GYRO_CHIP_ID_VALUE))
    {
        return false;
    }

    return bmi088_apply_config(BMI088_SENSOR_GYRO, gyro_config, sizeof(gyro_config) / sizeof(gyro_config[0]));
}

static int16_t bmi088_parse_axis(uint8_t low_byte, uint8_t high_byte)
{
    return (int16_t)(((uint16_t)high_byte << 8) | low_byte);
}

