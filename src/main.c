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
#include "log.h"
#include "mpu6050_dmp.h"

#define D0_PIN 0
#define D2_PIN 2
#define D3_PIN 21
#define D8_PIN 19
mpu6050_t imu;
static const char *TAGMAIN = "MAIN";

void app_main(void) {
    ESP_LOGI(TAGMAIN, "Hello world");
    ESP_ERROR_CHECK(nvs_init());
    ESP_ERROR_CHECK(imu_boot(&imu));
    ESP_ERROR_CHECK(server_init());
    ESP_ERROR_CHECK(log_init());

    TaskHandle_t mpu6050_task_handle = NULL;
    TaskHandle_t pwm_setter_task_handle = NULL;
    TaskHandle_t log_output_task_handle = NULL;

    xTaskCreate(mpu6050_task, "mpu6050_task", 3072, (void *)&imu, 5, &mpu6050_task_handle);
    xTaskCreate(pwm_setter_task, "pwm_setter_task", 4096, NULL, 4, &pwm_setter_task_handle);
    xTaskCreate(log_output_task, "log_output_task", 4096, NULL, 3, &log_output_task_handle);
    
}