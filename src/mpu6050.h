#pragma once
#include "esp_err.h"
#include "driver/i2c.h"
#include "dmp.h"
#include "i2c.h"

//MPU reg & addresses
#define MPU_ADDR_LOW            0x68
#define MPU_ADDR_HIGH           0x69
#define MPU_REG_WHO_AM_I        0x75
#define MPU_REG_PWR_MGMT_1      0x6B
#define MPU_REG_SMPLRT_DIV      0x19
#define MPU_REG_CONFIG          0x1A
#define MPU_REG_GYRO_CONFIG     0x1B
#define MPU_REG_ACCEL_CONFIG    0x1C
#define MPU_REG_INT_ENABLE      0x38
#define MPU_REG_INT_STATUS      0x3A
#define MPU_REG_USER_CTRL       0x6A
#define MPU_REG_FIFO_EN         0x23
#define MPU_REG_FIFO_COUNTH     0x72
#define MPU_REG_FIFO_COUNTL     0x73
#define MPU_REG_FIFO_RW         0x74
#define MPU_REG_SIG_PATH_RST    0x68

// DMP memory access
#define MPU_REG_BANK_SEL        0x6D
#define MPU_REG_MEM_START_ADDR  0x6E
#define MPU_REG_MEM_R_W         0x6F
#define MPU_REG_DMP_CFG_1       0x70
#define MPU_REG_DMP_CFG_2       0x71

// USER_CTRL bits
#define BIT_DMP_EN          0x80
#define BIT_FIFO_EN         0x40
#define BIT_I2C_MST_EN      0x20
#define BIT_I2C_IF_DIS      0x10
#define BIT_FIFO_RST        0x04
#define BIT_I2C_MST_RST     0x02
#define BIT_SIG_COND_RST    0x01

// INT_ENABLE bits
#define BIT_DMP_INT_EN      0x02

// DMP packet size
#define DMP_PACKET_SIZE     42

#define MAX_BATCH_PACKETS  8         // how many packets per burst read
#define POLL_MS            3         // for ~100 Hz DMP, poll every ~3–5 ms

#define MPU_REG_ACCEL_XOUT_H     0x3B

#define ACCEL_SENS_2G     16384.0f   // LSB per g
#define GYRO_SENS_250DPS  131.0f     // LSB per deg/s
#define RAD2DEG           57.2957795f

extern uint8_t mpu_addr;

esp_err_t mpu_detect(void);

esp_err_t mpu_device_init(void);

esp_err_t mpu_raw_init();

esp_err_t mpu_dmp_init(void);

esp_err_t mpu_dmp_initialize(void);

void dmp_task_polling(void *arg);

void ypr_task_polling(void *arg);