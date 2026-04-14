#include "bsp_imu.h"

#include "cmsis_os2.h"
#include "main.h"

#include <string.h>

extern SPI_HandleTypeDef hspi1;

#define BMI088_ACCEL_CHIP_ID_REG          0x00U
#define BMI088_ACCEL_CHIP_ID_VALUE        0x1EU
#define BMI088_ACCEL_X_LSB_REG            0x12U
#define BMI088_ACCEL_TEMP_MSB_REG         0x22U
#define BMI088_ACCEL_CONF_REG             0x40U
#define BMI088_ACCEL_RANGE_REG            0x41U
#define BMI088_ACCEL_PWR_CONF_REG         0x7CU
#define BMI088_ACCEL_PWR_CTRL_REG         0x7DU

#define BMI088_GYRO_CHIP_ID_REG           0x00U
#define BMI088_GYRO_CHIP_ID_VALUE         0x0FU
#define BMI088_GYRO_X_LSB_REG             0x02U
#define BMI088_GYRO_RANGE_REG             0x0FU
#define BMI088_GYRO_BANDWIDTH_REG         0x10U
#define BMI088_GYRO_LPM1_REG              0x11U
#define BMI088_GYRO_SOFTRESET_REG         0x14U

#define BMI088_SPI_READ_MASK              0x80U
#define BMI088_SPI_WRITE_MASK             0x7FU

#define BMI088_ACCEL_CONF_400HZ_NORMAL    0xAAU
#define BMI088_ACCEL_RANGE_12G            0x02U
#define BMI088_ACCEL_PWR_CONF_ACTIVE      0x00U
#define BMI088_ACCEL_PWR_CTRL_ON          0x04U

#define BMI088_GYRO_RANGE_1000DPS         0x01U
#define BMI088_GYRO_BANDWIDTH_1000HZ      0x02U
#define BMI088_GYRO_NORMAL_MODE           0x00U
#define BMI088_SOFTRESET_CMD              0xB6U

#define BSP_IMU_OFFLINE_TIMEOUT_TICK      50U

enum
{
    IMU_AXIS_X = 0,
    IMU_AXIS_Y = 1,
    IMU_AXIS_Z = 2
};

static const uint8_t s_axis_source[3] = {IMU_AXIS_Y, IMU_AXIS_X, IMU_AXIS_Z};
static const int8_t s_axis_sign[3] = {1, -1, 1};

static bool s_bsp_imu_initialized = false;
static bool s_bsp_imu_online = false;
static uint8_t s_bsp_imu_accel_chip_id = 0U;
static uint8_t s_bsp_imu_gyro_chip_id = 0U;
static uint32_t s_bsp_imu_last_update_tick = 0U;

static void bmi088_accel_select(bool active);
static void bmi088_gyro_select(bool active);
static bool bmi088_spi_txrx(const uint8_t *tx, uint8_t *rx, uint16_t len);
static bool bmi088_accel_read_reg(uint8_t reg, uint8_t *value);
static bool bmi088_accel_write_reg(uint8_t reg, uint8_t value);
static bool bmi088_accel_read_burst(uint8_t start_reg, uint8_t *data, uint16_t len);
static bool bmi088_gyro_read_reg(uint8_t reg, uint8_t *value);
static bool bmi088_gyro_write_reg(uint8_t reg, uint8_t value);
static bool bmi088_gyro_read_burst(uint8_t start_reg, uint8_t *data, uint16_t len);
static bool bmi088_accel_init(void);
static bool bmi088_gyro_init(void);
static bool bmi088_read_raw(IMURawData *raw);
static void bsp_imu_apply_axis_map(IMURawData *raw);
static int16_t bmi088_read_temp_raw(const uint8_t *temp_data);

static void bmi088_accel_select(bool active)
{
    HAL_GPIO_WritePin(BMI088_ACCEL_CS_GPIO_Port,
                      BMI088_ACCEL_CS_Pin,
                      active ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void bmi088_gyro_select(bool active)
{
    HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port,
                      BMI088_GYRO_CS_Pin,
                      active ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static bool bmi088_spi_txrx(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    return HAL_SPI_TransmitReceive(&hspi1,
                                   (uint8_t *)tx,
                                   rx,
                                   len,
                                   10U) == HAL_OK;
}

static bool bmi088_accel_read_reg(uint8_t reg, uint8_t *value)
{
    uint8_t tx[3] = {reg | BMI088_SPI_READ_MASK, 0x55U, 0x55U};
    uint8_t rx[3] = {0};

    if (value == NULL) {
        return false;
    }

    bmi088_accel_select(true);
    if (!bmi088_spi_txrx(tx, rx, sizeof(tx))) {
        bmi088_accel_select(false);
        return false;
    }
    bmi088_accel_select(false);

    *value = rx[2];
    return true;
}

static bool bmi088_accel_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg & BMI088_SPI_WRITE_MASK, value};

    bmi088_accel_select(true);
    if (HAL_SPI_Transmit(&hspi1, tx, sizeof(tx), 10U) != HAL_OK) {
        bmi088_accel_select(false);
        return false;
    }
    bmi088_accel_select(false);
    return true;
}

static bool bmi088_accel_read_burst(uint8_t start_reg, uint8_t *data, uint16_t len)
{
    uint8_t tx[10] = {0};
    uint8_t rx[10] = {0};
    uint16_t i;

    if ((data == NULL) || (len == 0U) || (len > 8U)) {
        return false;
    }

    tx[0] = start_reg | BMI088_SPI_READ_MASK;
    for (i = 1U; i < (uint16_t)(len + 2U); ++i) {
        tx[i] = 0x55U;
    }

    bmi088_accel_select(true);
    if (!bmi088_spi_txrx(tx, rx, (uint16_t)(len + 2U))) {
        bmi088_accel_select(false);
        return false;
    }
    bmi088_accel_select(false);

    memcpy(data, &rx[2], len);
    return true;
}

static bool bmi088_gyro_read_reg(uint8_t reg, uint8_t *value)
{
    uint8_t tx[2] = {reg | BMI088_SPI_READ_MASK, 0x55U};
    uint8_t rx[2] = {0};

    if (value == NULL) {
        return false;
    }

    bmi088_gyro_select(true);
    if (!bmi088_spi_txrx(tx, rx, sizeof(tx))) {
        bmi088_gyro_select(false);
        return false;
    }
    bmi088_gyro_select(false);

    *value = rx[1];
    return true;
}

static bool bmi088_gyro_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg & BMI088_SPI_WRITE_MASK, value};

    bmi088_gyro_select(true);
    if (HAL_SPI_Transmit(&hspi1, tx, sizeof(tx), 10U) != HAL_OK) {
        bmi088_gyro_select(false);
        return false;
    }
    bmi088_gyro_select(false);
    return true;
}

static bool bmi088_gyro_read_burst(uint8_t start_reg, uint8_t *data, uint16_t len)
{
    uint8_t tx[8] = {0};
    uint8_t rx[8] = {0};
    uint16_t i;

    if ((data == NULL) || (len == 0U) || (len > 7U)) {
        return false;
    }

    tx[0] = start_reg | BMI088_SPI_READ_MASK;
    for (i = 1U; i < (uint16_t)(len + 1U); ++i) {
        tx[i] = 0x55U;
    }

    bmi088_gyro_select(true);
    if (!bmi088_spi_txrx(tx, rx, (uint16_t)(len + 1U))) {
        bmi088_gyro_select(false);
        return false;
    }
    bmi088_gyro_select(false);

    memcpy(data, &rx[1], len);
    return true;
}

static bool bmi088_accel_init(void)
{
    uint8_t chip_id = 0U;

    if (!bmi088_accel_read_reg(BMI088_ACCEL_CHIP_ID_REG, &chip_id)) {
        return false;
    }

    if (!bmi088_accel_read_reg(BMI088_ACCEL_CHIP_ID_REG, &chip_id)) {
        return false;
    }

    s_bsp_imu_accel_chip_id = chip_id;
    if (chip_id != BMI088_ACCEL_CHIP_ID_VALUE) {
        return false;
    }

    if (!bmi088_accel_write_reg(BMI088_ACCEL_PWR_CONF_REG, BMI088_ACCEL_PWR_CONF_ACTIVE)) {
        return false;
    }
    osDelay(1U);

    if (!bmi088_accel_write_reg(BMI088_ACCEL_PWR_CTRL_REG, BMI088_ACCEL_PWR_CTRL_ON)) {
        return false;
    }
    osDelay(5U);

    if (!bmi088_accel_write_reg(BMI088_ACCEL_CONF_REG, BMI088_ACCEL_CONF_400HZ_NORMAL)) {
        return false;
    }

    if (!bmi088_accel_write_reg(BMI088_ACCEL_RANGE_REG, BMI088_ACCEL_RANGE_12G)) {
        return false;
    }

    return true;
}

static bool bmi088_gyro_init(void)
{
    uint8_t chip_id = 0U;

    if (!bmi088_gyro_write_reg(BMI088_GYRO_SOFTRESET_REG, BMI088_SOFTRESET_CMD)) {
        return false;
    }
    osDelay(30U);

    if (!bmi088_gyro_read_reg(BMI088_GYRO_CHIP_ID_REG, &chip_id)) {
        return false;
    }

    s_bsp_imu_gyro_chip_id = chip_id;
    if (chip_id != BMI088_GYRO_CHIP_ID_VALUE) {
        return false;
    }

    if (!bmi088_gyro_write_reg(BMI088_GYRO_RANGE_REG, BMI088_GYRO_RANGE_1000DPS)) {
        return false;
    }

    if (!bmi088_gyro_write_reg(BMI088_GYRO_BANDWIDTH_REG, BMI088_GYRO_BANDWIDTH_1000HZ)) {
        return false;
    }

    if (!bmi088_gyro_write_reg(BMI088_GYRO_LPM1_REG, BMI088_GYRO_NORMAL_MODE)) {
        return false;
    }

    return true;
}

static int16_t bmi088_read_temp_raw(const uint8_t *temp_data)
{
    int16_t temp_raw;

    temp_raw = (int16_t)(((uint16_t)temp_data[0] << 3U) | ((uint16_t)temp_data[1] >> 5U));
    if (temp_raw > 1023) {
        temp_raw -= 2048;
    }

    return temp_raw;
}

static void bsp_imu_apply_axis_map(IMURawData *raw)
{
    int16_t acc_src[3];
    int16_t gyro_src[3];
    int16_t acc_dst[3];
    int16_t gyro_dst[3];
    uint32_t i;

    acc_src[0] = raw->acc_x;
    acc_src[1] = raw->acc_y;
    acc_src[2] = raw->acc_z;
    gyro_src[0] = raw->gyro_x;
    gyro_src[1] = raw->gyro_y;
    gyro_src[2] = raw->gyro_z;

    for (i = 0U; i < 3U; ++i) {
        acc_dst[i] = (int16_t)(acc_src[s_axis_source[i]] * s_axis_sign[i]);
        gyro_dst[i] = (int16_t)(gyro_src[s_axis_source[i]] * s_axis_sign[i]);
    }

    raw->acc_x = acc_dst[0];
    raw->acc_y = acc_dst[1];
    raw->acc_z = acc_dst[2];
    raw->gyro_x = gyro_dst[0];
    raw->gyro_y = gyro_dst[1];
    raw->gyro_z = gyro_dst[2];
}

static bool bmi088_read_raw(IMURawData *raw)
{
    uint8_t accel_data[6] = {0};
    uint8_t gyro_data[6] = {0};
    uint8_t temp_data[2] = {0};

    if (raw == NULL) {
        return false;
    }

    if (!bmi088_accel_read_burst(BMI088_ACCEL_X_LSB_REG, accel_data, sizeof(accel_data))) {
        return false;
    }

    if (!bmi088_accel_read_burst(BMI088_ACCEL_TEMP_MSB_REG, temp_data, sizeof(temp_data))) {
        return false;
    }

    if (!bmi088_gyro_read_burst(BMI088_GYRO_X_LSB_REG, gyro_data, sizeof(gyro_data))) {
        return false;
    }

    raw->acc_x = (int16_t)((uint16_t)accel_data[1] << 8U | accel_data[0]);
    raw->acc_y = (int16_t)((uint16_t)accel_data[3] << 8U | accel_data[2]);
    raw->acc_z = (int16_t)((uint16_t)accel_data[5] << 8U | accel_data[4]);

    raw->gyro_x = (int16_t)((uint16_t)gyro_data[1] << 8U | gyro_data[0]);
    raw->gyro_y = (int16_t)((uint16_t)gyro_data[3] << 8U | gyro_data[2]);
    raw->gyro_z = (int16_t)((uint16_t)gyro_data[5] << 8U | gyro_data[4]);
    raw->temp = bmi088_read_temp_raw(temp_data);

    bsp_imu_apply_axis_map(raw);
    return true;
}

bool bsp_imu_init(void)
{
    s_bsp_imu_initialized = false;
    s_bsp_imu_online = false;
    s_bsp_imu_last_update_tick = 0U;

    if (!bmi088_accel_init()) {
        return false;
    }

    if (!bmi088_gyro_init()) {
        return false;
    }

    s_bsp_imu_initialized = true;
    s_bsp_imu_online = true;
    s_bsp_imu_last_update_tick = osKernelGetTickCount();
    return true;
}

bool bsp_imu_update(IMURawData *raw)
{
    if ((raw == NULL) || (!s_bsp_imu_initialized)) {
        return false;
    }

    if (!bmi088_read_raw(raw)) {
        s_bsp_imu_online = false;
        return false;
    }

    s_bsp_imu_online = true;
    s_bsp_imu_last_update_tick = osKernelGetTickCount();
    return true;
}

bool imu_check(void)
{
    uint32_t now_tick;

    if ((!s_bsp_imu_initialized) || (!s_bsp_imu_online)) {
        return false;
    }

    now_tick = osKernelGetTickCount();
    return (uint32_t)(now_tick - s_bsp_imu_last_update_tick) <= BSP_IMU_OFFLINE_TIMEOUT_TICK;
}
