#include "bsp_imu.h"

#include "FreeRTOS.h"
#include "gimbal_config.h"
#include "gpio.h"
#include "spi.h"
#include "task.h"

#include <string.h>

#define BMI088_ACCEL_REG_CHIP_ID              (0x00U)
#define BMI088_ACCEL_REG_X_LSB                (0x12U)
#define BMI088_ACCEL_REG_RANGE                (0x41U)
#define BMI088_ACCEL_REG_CONF                 (0x40U)
#define BMI088_ACCEL_REG_PWR_CONF             (0x7CU)
#define BMI088_ACCEL_REG_PWR_CTRL             (0x7DU)
#define BMI088_ACCEL_REG_SOFTRESET            (0x7EU)

#define BMI088_GYRO_REG_CHIP_ID               (0x00U)
#define BMI088_GYRO_REG_X_LSB                 (0x02U)
#define BMI088_GYRO_REG_RANGE                 (0x0FU)
#define BMI088_GYRO_REG_BW                    (0x10U)
#define BMI088_GYRO_REG_LPM1                  (0x11U)
#define BMI088_GYRO_REG_SOFTRESET             (0x14U)

static bool s_imu_initialized = false;
static bool s_imu_last_ok = false;
static uint32_t s_imu_last_update_ms = 0U;

static void imu_delay_ms(uint32_t delay_ms)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    } else {
        HAL_Delay(delay_ms);
    }
}

static bool bmi088_spi_write(GPIO_TypeDef *port, uint16_t pin, uint8_t reg, uint8_t value)
{
    uint8_t tx_buf[2];

    tx_buf[0] = reg & 0x7FU;
    tx_buf[1] = value;

    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(&hspi1, tx_buf, (uint16_t)sizeof(tx_buf), HAL_MAX_DELAY) != HAL_OK) {
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
        return false;
    }
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    return true;
}

static bool bmi088_spi_read(GPIO_TypeDef *port, uint16_t pin, uint8_t reg, uint8_t *data, uint16_t len)
{
    uint8_t tx_buf[16] = {0};
    uint8_t rx_buf[16] = {0};
    uint16_t transfer_len = (uint16_t)(len + 2U);

    if ((data == NULL) || (transfer_len > sizeof(tx_buf))) {
        return false;
    }

    tx_buf[0] = reg | 0x80U;
    tx_buf[1] = 0x00U;

    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    if (HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, transfer_len, HAL_MAX_DELAY) != HAL_OK) {
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
        return false;
    }
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);

    memcpy(data, &rx_buf[2], len);
    return true;
}

static bool bmi088_accel_read(uint8_t reg, uint8_t *data, uint16_t len)
{
    return bmi088_spi_read(BMI088_ACCEL_CS_GPIO_Port, BMI088_ACCEL_CS_Pin, reg, data, len);
}

static bool bmi088_gyro_read(uint8_t reg, uint8_t *data, uint16_t len)
{
    return bmi088_spi_read(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, reg, data, len);
}

static bool bmi088_accel_write(uint8_t reg, uint8_t value)
{
    return bmi088_spi_write(BMI088_ACCEL_CS_GPIO_Port, BMI088_ACCEL_CS_Pin, reg, value);
}

static bool bmi088_gyro_write(uint8_t reg, uint8_t value)
{
    return bmi088_spi_write(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, reg, value);
}

static bool bmi088_probe_ids(void)
{
    uint8_t accel_id = 0U;
    uint8_t gyro_id = 0U;

    if (!bmi088_accel_read(BMI088_ACCEL_REG_CHIP_ID, &accel_id, 1U)) {
        return false;
    }

    if (!bmi088_gyro_read(BMI088_GYRO_REG_CHIP_ID, &gyro_id, 1U)) {
        return false;
    }

    return (accel_id == BMI088_ACCEL_CHIP_ID) && (gyro_id == BMI088_GYRO_CHIP_ID);
}

static bool bmi088_configure(void)
{
    HAL_GPIO_WritePin(BMI088_ACCEL_CS_GPIO_Port, BMI088_ACCEL_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_SET);
    imu_delay_ms(1U);

    if (!bmi088_accel_write(BMI088_ACCEL_REG_SOFTRESET, 0xB6U)) {
        return false;
    }
    imu_delay_ms(5U);

    if (!bmi088_gyro_write(BMI088_GYRO_REG_SOFTRESET, 0xB6U)) {
        return false;
    }
    imu_delay_ms(30U);

    if (!bmi088_probe_ids()) {
        return false;
    }

    if (!bmi088_accel_write(BMI088_ACCEL_REG_PWR_CONF, 0x00U)) {
        return false;
    }
    if (!bmi088_accel_write(BMI088_ACCEL_REG_PWR_CTRL, 0x04U)) {
        return false;
    }
    if (!bmi088_accel_write(BMI088_ACCEL_REG_CONF, 0xA8U)) {
        return false;
    }
    if (!bmi088_accel_write(BMI088_ACCEL_REG_RANGE, 0x01U)) {
        return false;
    }

    if (!bmi088_gyro_write(BMI088_GYRO_REG_RANGE, 0x00U)) {
        return false;
    }
    if (!bmi088_gyro_write(BMI088_GYRO_REG_BW, 0x02U)) {
        return false;
    }
    if (!bmi088_gyro_write(BMI088_GYRO_REG_LPM1, 0x00U)) {
        return false;
    }

    return true;
}

bool bsp_imu_init(void)
{
    uint32_t retry = 0U;

    for (retry = 0U; retry < IMU_INIT_RETRY_CNT; ++retry) {
        if (bmi088_configure()) {
            s_imu_initialized = true;
            s_imu_last_ok = true;
            s_imu_last_update_ms = HAL_GetTick();
            return true;
        }

        imu_delay_ms(IMU_INIT_RETRY_DELAY_MS);
    }

    s_imu_initialized = false;
    s_imu_last_ok = false;
    return false;
}

bool bsp_imu_update(IMURawData *raw)
{
    uint8_t accel_buf[6];
    uint8_t gyro_buf[6];
    int16_t ax = 0;
    int16_t ay = 0;
    int16_t az = 0;
    int16_t gx = 0;
    int16_t gy = 0;
    int16_t gz = 0;

    if ((!s_imu_initialized) || (raw == NULL)) {
        s_imu_last_ok = false;
        return false;
    }

    if (!bmi088_accel_read(BMI088_ACCEL_REG_X_LSB, accel_buf, sizeof(accel_buf))) {
        s_imu_last_ok = false;
        return false;
    }

    if (!bmi088_gyro_read(BMI088_GYRO_REG_X_LSB, gyro_buf, sizeof(gyro_buf))) {
        s_imu_last_ok = false;
        return false;
    }

    ax = (int16_t)(((uint16_t)accel_buf[1] << 8U) | accel_buf[0]);
    ay = (int16_t)(((uint16_t)accel_buf[3] << 8U) | accel_buf[2]);
    az = (int16_t)(((uint16_t)accel_buf[5] << 8U) | accel_buf[4]);
    gx = (int16_t)(((uint16_t)gyro_buf[1] << 8U) | gyro_buf[0]);
    gy = (int16_t)(((uint16_t)gyro_buf[3] << 8U) | gyro_buf[2]);
    gz = (int16_t)(((uint16_t)gyro_buf[5] << 8U) | gyro_buf[4]);

    raw->accel_x_g = ((float)ax) / BMI088_ACCEL_LSB_PER_G;
    raw->accel_y_g = ((float)ay) / BMI088_ACCEL_LSB_PER_G;
    raw->accel_z_g = ((float)az) / BMI088_ACCEL_LSB_PER_G;

    raw->gyro_x_rad_s = (((float)gx) / BMI088_GYRO_LSB_PER_DPS) * 0.0174532925f;
    raw->gyro_y_rad_s = (((float)gy) / BMI088_GYRO_LSB_PER_DPS) * 0.0174532925f;
    raw->gyro_z_rad_s = (((float)gz) / BMI088_GYRO_LSB_PER_DPS) * 0.0174532925f;

    s_imu_last_ok = true;
    s_imu_last_update_ms = HAL_GetTick();
    return true;
}

bool imu_check(void)
{
    return s_imu_initialized && s_imu_last_ok && ((HAL_GetTick() - s_imu_last_update_ms) <= 50U);
}
