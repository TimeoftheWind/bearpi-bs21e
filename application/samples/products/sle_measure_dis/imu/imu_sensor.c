#include <stdbool.h>
#include <stdint.h>
#include "gpio.h"
#include "i2c.h"
#include "i2c_porting.h"
#include "pinctrl.h"
#include "soc_osal.h"
#include "tcxo.h"
#include "imu_sensor.h"

#define IMU_CLK_PIN 14
#define IMU_CS_PIN 13
#define IMU_DI_PIN 12
#define IMU_DO_PIN 11
#define IMU_SPI_HALF_PERIOD_US 1U
#define MAG_I2C_BUS 0
#define MAG_I2C_BAUD 50000
#define MAG_ADDR 44
#define MAG_SCL_PIN 22
#define MAG_SDA_PIN 23
#define MAG_SCL_MODE 26
#define MAG_SDA_MODE 27

#define REG_SIGNAL_PATH_RESET 0x02
#define REG_TEMP_DATA1 0x09
#define REG_PWR_MGMT0 0x1F
#define REG_GYRO_CONFIG0 0x20
#define REG_ACCEL_CONFIG0 0x21
#define REG_TEMP_CONFIG0 0x22
#define REG_GYRO_CONFIG1 0x23
#define REG_ACCEL_CONFIG1 0x24
#define REG_WHO_AM_I 0x75
#define ICM_WHO_AM_I 0x60
#define SPI_READ_MASK 0x80
#define RAW_LEN 14

/* Values match the 0x68 full-scale configuration used by the original test. */
#define ACC_LSB_PER_G 16384.0f
#define GYRO_LSB_PER_DPS 131.0f
#define MAG_LSB_PER_UT 1.0f

static uint8_t soft_spi_transfer(uint8_t tx)
{
    uint8_t rx = 0;
    for (uint8_t mask = 0x80U; mask != 0U; mask >>= 1U) {
        (void)uapi_gpio_set_val(IMU_DO_PIN, (tx & mask) ? GPIO_LEVEL_HIGH : GPIO_LEVEL_LOW);
        uapi_tcxo_delay_us(IMU_SPI_HALF_PERIOD_US);
        (void)uapi_gpio_set_val(IMU_CLK_PIN, GPIO_LEVEL_HIGH);
        rx <<= 1U;
        if (uapi_gpio_get_val(IMU_DI_PIN) == GPIO_LEVEL_HIGH) {
            rx |= 1U;
        }
        uapi_tcxo_delay_us(IMU_SPI_HALF_PERIOD_US);
        (void)uapi_gpio_set_val(IMU_CLK_PIN, GPIO_LEVEL_LOW);
    }
    return rx;
}

static errcode_t imu_soft_spi_init(void)
{
    uapi_gpio_init();
    (void)uapi_pin_set_mode(IMU_CLK_PIN, HAL_PIO_FUNC_GPIO);
    (void)uapi_pin_set_mode(IMU_CS_PIN, HAL_PIO_FUNC_GPIO);
    (void)uapi_pin_set_mode(IMU_DI_PIN, HAL_PIO_FUNC_GPIO);
    (void)uapi_pin_set_mode(IMU_DO_PIN, HAL_PIO_FUNC_GPIO);
    (void)uapi_pin_set_pull(IMU_CLK_PIN, PIN_PULL_NONE);
    (void)uapi_pin_set_pull(IMU_CS_PIN, PIN_PULL_UP);
    (void)uapi_pin_set_pull(IMU_DI_PIN, PIN_PULL_NONE);
    (void)uapi_pin_set_pull(IMU_DO_PIN, PIN_PULL_NONE);
    (void)uapi_gpio_set_dir(IMU_CLK_PIN, GPIO_DIRECTION_OUTPUT);
    (void)uapi_gpio_set_dir(IMU_CS_PIN, GPIO_DIRECTION_OUTPUT);
    (void)uapi_gpio_set_dir(IMU_DI_PIN, GPIO_DIRECTION_INPUT);
    (void)uapi_gpio_set_dir(IMU_DO_PIN, GPIO_DIRECTION_OUTPUT);
    (void)uapi_gpio_set_val(IMU_CLK_PIN, GPIO_LEVEL_LOW);
    (void)uapi_gpio_set_val(IMU_DO_PIN, GPIO_LEVEL_LOW);
    return uapi_gpio_set_val(IMU_CS_PIN, GPIO_LEVEL_HIGH);
}

static int16_t i16_be(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static errcode_t spi_write(uint8_t reg, uint8_t value)
{
    (void)uapi_gpio_set_val(IMU_CS_PIN, GPIO_LEVEL_LOW);
    (void)soft_spi_transfer(reg & (uint8_t)(~SPI_READ_MASK));
    (void)soft_spi_transfer(value);
    (void)uapi_gpio_set_val(IMU_CS_PIN, GPIO_LEVEL_HIGH);
    return ERRCODE_SUCC;
}

static errcode_t spi_read(uint8_t reg, uint8_t *buf, uint32_t len)
{
    if ((buf == NULL) || (len == 0U)) {
        return ERRCODE_INVALID_PARAM;
    }
    (void)uapi_gpio_set_val(IMU_CS_PIN, GPIO_LEVEL_LOW);
    (void)soft_spi_transfer(reg | SPI_READ_MASK);
    for (uint32_t i = 0; i < len; i++) {
        buf[i] = soft_spi_transfer(0U);
    }
    (void)uapi_gpio_set_val(IMU_CS_PIN, GPIO_LEVEL_HIGH);
    return ERRCODE_SUCC;
}

static errcode_t mag_write(uint8_t reg, uint8_t value)
{
    uint8_t b[2] = { reg, value };
    i2c_data_t d = { .send_buf = b, .send_len = 2, .receive_buf = NULL, .receive_len = 0 };
    return uapi_i2c_master_write(MAG_I2C_BUS, MAG_ADDR, &d);
}

static errcode_t mag_read(float *mx, float *my, float *mz)
{
    uint8_t reg = 0x01;
    uint8_t b[9] = {0};
    i2c_data_t d = { .send_buf = &reg, .send_len = 1, .receive_buf = b, .receive_len = sizeof(b) };
    errcode_t ret = uapi_i2c_master_writeread(MAG_I2C_BUS, MAG_ADDR, &d);
    if (ret != ERRCODE_SUCC) return ret;
    *mx = (float)(int16_t)(((uint16_t)b[1] << 8) | b[0]) / MAG_LSB_PER_UT;
    *my = (float)(int16_t)(((uint16_t)b[3] << 8) | b[2]) / MAG_LSB_PER_UT;
    *mz = (float)(int16_t)(((uint16_t)b[5] << 8) | b[4]) / MAG_LSB_PER_UT;
    return ERRCODE_SUCC;
}

errcode_t sle_imu_sensor_init(void)
{
    uint8_t id = 0;
    if (imu_soft_spi_init() != ERRCODE_SUCC) return ERRCODE_FAIL;
    osal_printk("IMU software SPI ready\r\n");
    spi_write(REG_SIGNAL_PATH_RESET, 0x10); uapi_tcxo_delay_ms(10);
    if (spi_read(REG_WHO_AM_I, &id, 1) != ERRCODE_SUCC || id != ICM_WHO_AM_I) return ERRCODE_FAIL;
    osal_printk("ICM42607P id ok\r\n");
    spi_write(REG_TEMP_CONFIG0, 0x30); spi_write(REG_GYRO_CONFIG1, 0x06);
    spi_write(REG_GYRO_CONFIG0, 0x68); spi_write(REG_ACCEL_CONFIG0, 0x68);
    spi_write(REG_ACCEL_CONFIG1, 0x46); spi_write(REG_PWR_MGMT0, 0x0F); uapi_tcxo_delay_ms(50);
    uapi_pin_set_mode(MAG_SCL_PIN, MAG_SCL_MODE); uapi_pin_set_mode(MAG_SDA_PIN, MAG_SDA_MODE);
    uapi_pin_set_pull(MAG_SCL_PIN, PIN_PULL_UP); uapi_pin_set_pull(MAG_SDA_PIN, PIN_PULL_UP);
    i2c_port_clock_enable(MAG_I2C_BUS, true);
    uapi_i2c_deinit(MAG_I2C_BUS);
    if (uapi_i2c_master_init(MAG_I2C_BUS, MAG_I2C_BAUD, I2C_SPEED_MODE_FS) != ERRCODE_SUCC) return ERRCODE_FAIL;
    osal_printk("IMU I2C ready\r\n");
    if (mag_write(0x29, 0x06) != ERRCODE_SUCC || mag_write(0x0B, 0x08) != ERRCODE_SUCC ||
        mag_write(0x0A, 0xC3) != ERRCODE_SUCC) return ERRCODE_FAIL;
    osal_printk("QMC5883P configured\r\n");
    return ERRCODE_SUCC;
}

errcode_t sle_imu_sensor_read(sle_imu_packet_t *p)
{
    uint8_t b[RAW_LEN];
    errcode_t ret;
    if (p == NULL) return ERRCODE_INVALID_PARAM;
    p->timestamp_us = uapi_tcxo_get_us();
    ret = spi_read(REG_TEMP_DATA1, b, sizeof(b));
    if (ret != ERRCODE_SUCC) {
        osal_printk("IMU SPI sample failed, ret:0x%x\r\n", ret);
        return ret;
    }
    ret = mag_read(&p->mx, &p->my, &p->mz);
    if (ret != ERRCODE_SUCC) {
        osal_printk("IMU MAG sample failed, ret:0x%x\r\n", ret);
        return ret;
    }
    p->ax=(float)i16_be(&b[2])/ACC_LSB_PER_G; p->ay=(float)i16_be(&b[4])/ACC_LSB_PER_G;
    p->az=(float)i16_be(&b[6])/ACC_LSB_PER_G; p->gx=(float)i16_be(&b[8])/GYRO_LSB_PER_DPS;
    p->gy=(float)i16_be(&b[10])/GYRO_LSB_PER_DPS; p->gz=(float)i16_be(&b[12])/GYRO_LSB_PER_DPS;
    return ERRCODE_SUCC;
}
