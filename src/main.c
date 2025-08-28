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

#define D0_PIN 0
#define D2_PIN 2
#define D3_PIN 21
#define D8_PIN 19
mpu6050_t imu;

void app_main(void) {
    ESP_ERROR_CHECK(nvs_init());
    ESP_ERROR_CHECK(imu_boot(&imu));
    ESP_ERROR_CHECK(server_init());

    TaskHandle_t imu_task = NULL;
    xTaskCreate(mpu6050_task, "mpu6050_task", 3072, (void *)&imu, 5, &imu_task);
}