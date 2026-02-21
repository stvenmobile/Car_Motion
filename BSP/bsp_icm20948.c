#include "bsp_icm20948.h"
#include "spi.h"
#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

/* --- Hardware Redirection --- */
#define ICM20948_SPI           &hspi2
#define ICM20948_CS_GPIO_Port  GPIOB
#define ICM20948_CS_Pin        GPIO_PIN_12

#define READ   0x80
#define WRITE  0x00

/* --- Register Definitions --- */
#define REG_BANK_SEL      0x7F
#define B0_WHO_AM_I       0x00
#define B0_USER_CTRL      0x03
#define B0_PWR_MGMT_1     0x06
#define B0_ACCEL_XOUT_H   0x2D
#define B0_GYRO_XOUT_H    0x33
#define B0_EXT_SENS_DATA  0x3B // AK09916 data automatically lands here

#define B2_GYRO_CONFIG_1  0x01
#define B2_ACCEL_CONFIG   0x14

#define B3_I2C_MST_CTRL   0x01
#define B3_I2C_SLV0_ADDR  0x03
#define B3_I2C_SLV0_REG   0x04
#define B3_I2C_SLV0_CTRL  0x05
#define B3_I2C_SLV4_ADDR  0x13
#define B3_I2C_SLV4_REG   0x14
#define B3_I2C_SLV4_CTRL  0x15
#define B3_I2C_SLV4_DO    0x16

/* AK09916 Definitions */
#define MAG_I2C_ADDR      0x0C
#define MAG_CNTL2         0x31
#define MAG_ST1           0x10

/* --- Global Data --- */
raw_data_t g_raw_accel, g_raw_gyro, g_raw_mag;
axises_t g_imu_accel, g_imu_gyro, g_imu_mag;
float g_scale_accel = 16384.0f; // Matching +/-2g
float g_scale_gyro  = 16.4f;    // Matching +/-2000dps
float gyro_offset_x = 0, gyro_offset_y = 0, gyro_offset_z = 0;
float g_current_yaw = 0.0f;
float g_last_loop_time = 0.0f;

/* --- SPI Helpers --- */
static void ICM_Active()   { HAL_GPIO_WritePin(ICM20948_CS_GPIO_Port, ICM20948_CS_Pin, GPIO_PIN_RESET); }
static void ICM_NoActive() { HAL_GPIO_WritePin(ICM20948_CS_GPIO_Port, ICM20948_CS_Pin, GPIO_PIN_SET); }

static void select_user_bank(userbank_t ub) {
    uint8_t write_reg[2] = {REG_BANK_SEL | WRITE, (uint8_t)ub << 4};
    ICM_Active();
    HAL_SPI_Transmit(ICM20948_SPI, write_reg, 2, 10);
    ICM_NoActive();
}

static void write_single_reg(userbank_t ub, uint8_t reg, uint8_t val) {
    uint8_t write_reg[2] = {WRITE | reg, val};
    select_user_bank(ub);
    ICM_Active();
    HAL_SPI_Transmit(ICM20948_SPI, write_reg, 2, 10);
    ICM_NoActive();
}

static uint8_t read_single_reg(userbank_t ub, uint8_t reg) {
    uint8_t read_reg = READ | reg, val;
    select_user_bank(ub);
    ICM_Active();
    HAL_SPI_Transmit(ICM20948_SPI, &read_reg, 1, 10);
    HAL_SPI_Receive(ICM20948_SPI, &val, 1, 10);
    ICM_NoActive();
    return val;
}

static uint8_t* read_multiple_reg(userbank_t ub, uint8_t reg, uint8_t len) {
    uint8_t read_reg = READ | reg;
    static uint8_t reg_val[10];
    select_user_bank(ub);
    ICM_Active();
    HAL_SPI_Transmit(ICM20948_SPI, &read_reg, 1, 10);
    HAL_SPI_Receive(ICM20948_SPI, reg_val, len, 10);
    ICM_NoActive();
    return reg_val;
}

/* --- Magnetometer Proxy Helpers --- */
static void write_mag_reg(uint8_t reg, uint8_t val) {
    write_single_reg(ub_3, B3_I2C_SLV4_ADDR, MAG_I2C_ADDR);
    write_single_reg(ub_3, B3_I2C_SLV4_REG, reg);
    write_single_reg(ub_3, B3_I2C_SLV4_DO, val);
    write_single_reg(ub_3, B3_I2C_SLV4_CTRL, 0x80); // Enable Slave 4 write
    HAL_Delay(10);
}

/* --- Initialization --- */
void ICM20948_init() {
    while(read_single_reg(ub_0, B0_WHO_AM_I) != 0xEA) { HAL_Delay(100); }
    write_single_reg(ub_0, B0_PWR_MGMT_1, 0x80); // Reset
    HAL_Delay(100);
    write_single_reg(ub_0, B0_PWR_MGMT_1, 0x01); // Wake
    HAL_Delay(50);
    write_single_reg(ub_0, B0_USER_CTRL, 0x30);   // Enable I2C Master
    write_single_reg(ub_3, B3_I2C_MST_CTRL, 0x07); // Set Clock ~345kHz

    write_single_reg(ub_2, B2_GYRO_CONFIG_1, (3 << 1) | 0x01); // 2000dps
    write_single_reg(ub_2, B2_ACCEL_CONFIG, (0 << 1) | 0x01);  // 2g
}

void AK09916_init() {
    write_mag_reg(0x32, 0x01); // Software Reset
    HAL_Delay(50);
    write_mag_reg(MAG_CNTL2, 0x08); // Continuous Mode 100Hz

    write_single_reg(ub_3, B3_I2C_SLV0_ADDR, MAG_I2C_ADDR | READ);
    write_single_reg(ub_3, B3_I2C_SLV0_REG, MAG_ST1);
    write_single_reg(ub_3, B3_I2C_SLV0_CTRL, 0x89); // Auto-read 9 bytes
}

/* --- Data Handlers --- */
void ICM20948_Read_Data_Handle(void) {
    static uint32_t last_tick = 0;
    uint32_t current_tick = HAL_GetTick();
    if (last_tick == 0) { last_tick = current_tick; return; }

    uint32_t delta = current_tick - last_tick;
    g_last_loop_time = (float)delta;
    float dt = (float)delta / 1000.0f;
    last_tick = current_tick;
    if (dt > 0.1f) dt = 0.011f; // Target 11ms based on your hardware tests

    ICM20948_accel_read_g(&g_imu_accel);
    ICM20948_gyro_read_dps(&g_imu_gyro);

    // Process Magnetometer from external sensor registers
    uint8_t* m = read_multiple_reg(ub_0, B0_EXT_SENS_DATA, 9);
    if (m[0] & 0x01) { // ST1 DRDY check
        g_raw_mag.x = (int16_t)(m[2] << 8 | m[1]);
        g_raw_mag.y = (int16_t)(m[4] << 8 | m[3]);
        g_raw_mag.z = (int16_t)(m[6] << 8 | m[5]);
        // AK09916 sensitivity is 0.15uT/LSB
        g_imu_mag.x = (float)g_raw_mag.x * 0.15f;
        g_imu_mag.y = (float)g_raw_mag.y * 0.15f;
        g_imu_mag.z = (float)g_raw_mag.z * 0.15f;
    }

    g_current_yaw -= g_imu_gyro.z * dt;
    if (g_current_yaw >= 360.0f) g_current_yaw -= 360.0f;
    if (g_current_yaw < 0.0f) g_current_yaw += 360.0f;
}

void ICM20948_accel_read_g(axises_t* data) {
    uint8_t* t = read_multiple_reg(ub_0, B0_ACCEL_XOUT_H, 6);
    data->x = (float)((int16_t)(t[0] << 8 | t[1])) / g_scale_accel;
    data->y = (float)((int16_t)(t[2] << 8 | t[3])) / g_scale_accel;
    data->z = (float)((int16_t)(t[4] << 8 | t[5])) / g_scale_accel;
}

void ICM20948_gyro_read_dps(axises_t* data) {
    uint8_t* t = read_multiple_reg(ub_0, B0_GYRO_XOUT_H, 6);
    data->x = (float)((int16_t)(t[0] << 8 | t[1]) - gyro_offset_x) / g_scale_gyro;
    data->y = (float)((int16_t)(t[2] << 8 | t[3]) - gyro_offset_y) / g_scale_gyro;
    data->z = (float)((int16_t)(t[4] << 8 | t[5]) - gyro_offset_z) / g_scale_gyro;
}

void ICM20948_gyro_calibration(void) {
    long sx = 0, sy = 0, sz = 0;
    // 🏁 Warning Fix: Variable 'd' removed
    for(int i=0; i<500; i++) {
        uint8_t* t = read_multiple_reg(ub_0, B0_GYRO_XOUT_H, 6);
        sx += (int16_t)(t[0] << 8 | t[1]);
        sy += (int16_t)(t[2] << 8 | t[3]);
        sz += (int16_t)(t[4] << 8 | t[5]);
        HAL_Delay(5);
    }
    gyro_offset_x = (float)sx/500; gyro_offset_y = (float)sy/500; gyro_offset_z = (float)sz/500;
}
