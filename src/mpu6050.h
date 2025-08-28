#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

// Change these to match your wiring (example pins shown)
#ifndef MPU6050_I2C_PORT
#define MPU6050_I2C_PORT         I2C_NUM_0
#endif

#define I2C_SDA 22
#define I2C_SCL 23

#ifndef MPU6050_I2C_FREQ_HZ
#define MPU6050_I2C_FREQ_HZ      100000
#endif

// Default addresses: AD0 low -> 0x68, AD0 high -> 0x69
#define MPU6050_ADDR_DEFAULT     0x69

// Registers
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_WHO_AM_I     0x75

#define MPU6050_TASK_PERIOD_MS 10   // ~100 Hz

typedef struct {
    i2c_port_t i2c_port;
    uint8_t    addr;

    // Config (derived scales)
    float accel_lsb_per_g;     // e.g. 16384 for ±2g
    float gyro_lsb_per_dps;    // e.g. 131 for ±250 dps

    // Biases (in g for accel, dps for gyro)
    float ax_bias, ay_bias, az_bias;
    float gx_bias, gy_bias, gz_bias;

    // Mahony filter state
    float q0, q1, q2, q3;      // unit quaternion
    float exInt, eyInt, ezInt; // integral error terms
    float kp, ki;              // gains

    // Timing
    uint64_t last_update_us;   // for internal dt calculation
} mpu6050_t;


// I2C setup helper (install driver). Call once at startup.
esp_err_t mpu6050_i2c_init(i2c_port_t port, gpio_num_t sda, gpio_num_t scl, uint32_t freq_hz);

// Device init + basic configuration (±2g, ±250 dps, DLPF=42 Hz, ~200 Hz sample rate)
esp_err_t mpu6050_init(mpu6050_t *dev, i2c_port_t port, uint8_t addr);

// Optional: change full-scale ranges and rates
esp_err_t mpu6050_config(mpu6050_t *dev,
                         uint8_t accel_fs_sel,   // 0:±2g,1:±4g,2:±8g,3:±16g
                         uint8_t gyro_fs_sel,    // 0:±250,1:±500,2:±1000,3:±2000 dps
                         uint8_t dlpf_cfg,       // 0..6 (0=260Hz,3=42Hz,6=5Hz)
                         uint8_t smplrt_div);    // SampleRate = 1kHz/(1+div) when DLPF on

// Quick WHO_AM_I check (should be 0x68)
esp_err_t mpu6050_whoami(mpu6050_t *dev, uint8_t *out);

// Calibrate biases (keep device still on a flat surface).
// samples: e.g. 500. Takes ~samples/200 seconds at 200 Hz.
esp_err_t mpu6050_calibrate(mpu6050_t *dev, int samples);

// Read raw sensor values
esp_err_t mpu6050_read_raw(mpu6050_t *dev,
                           int16_t *ax, int16_t *ay, int16_t *az,
                           int16_t *gx, int16_t *gy, int16_t *gz);

// Update internal filter and return yaw/pitch/roll in degrees.
// Uses internal dt from esp_timer; call at a fixed-ish rate (100–500 Hz).
esp_err_t mpu6050_update_ypr(mpu6050_t *dev, float *yaw_deg, float *pitch_deg, float *roll_deg);

// Reset filter orientation to identity (yaw=0, pitch=0, roll=0)
void mpu6050_reset_filter(mpu6050_t *dev);

// Helper: convert quaternion -> yaw/pitch/roll (deg)
void mpu6050_quat_to_ypr(float q0, float q1, float q2, float q3,
                         float *yaw_deg, float *pitch_deg, float *roll_deg);

void mpu6050_task(void *pvParameters);

esp_err_t imu_boot(mpu6050_t *imu);

#ifdef __cplusplus
}
#endif
