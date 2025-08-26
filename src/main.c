#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c.h"
#include "nvs.h"

#include "pwm.h"
#include "mpu6050.h"
#include "server.h"

#define I2C_SDA 22
#define I2C_SCL 23

#define D0_PIN 0
#define D2_PIN 2
#define D3_PIN 21
#define D8_PIN 19

void app_main(void) {
    ESP_ERROR_CHECK(mpu6050_i2c_init(MPU6050_I2C_PORT, I2C_SDA, I2C_SCL, MPU6050_I2C_FREQ_HZ));

    mpu6050_t imu;
    ESP_ERROR_CHECK(mpu6050_init(&imu, MPU6050_I2C_PORT, MPU6050_ADDR_DEFAULT));

    // Optional: verify device
    uint8_t who = 0;
    ESP_ERROR_CHECK(mpu6050_whoami(&imu, &who));
    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X (expect 0x68)", who);

    // Optional: tweak filter gains
    imu.kp = 3.0f;   // 2.0–5.0 good for snappy response
    imu.ki = 0.02f;  // small integral helps gyro bias; set 0.0f if you see drift overshoot

    // Calibrate (keep the board flat and still)
    ESP_ERROR_CHECK(mpu6050_calibrate(&imu, 500));

    // Main loop at ~200 Hz
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10);

    while (1) {
        float yaw, pitch, roll;
        esp_err_t err = mpu6050_update_ypr(&imu, &yaw, &pitch, &roll);
        if (err == ESP_OK) {
            // Yaw will drift without a magnetometer
            ESP_LOGI(TAG, "YPR: yaw=%7.2f°, pitch=%7.2f°, roll=%7.2f°", yaw, pitch, roll);
        } else {
            ESP_LOGW(TAG, "update error: %s", esp_err_to_name(err));
        }
        vTaskDelayUntil(&last, period);
    }
}