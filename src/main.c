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

static const char *TAGMAIN = "MAIN_RX";

#define D0_PIN 0
#define D2_PIN 2
#define D3_PIN 21
#define D8_PIN 19

void app_main(void) {

    ESP_ERROR_CHECK(nvs_init());

    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 1); // OFF (active-LOW)
    
    ESP_ERROR_CHECK (mpu_raw_init(400000));
    ESP_ERROR_CHECK(server_init());
    xTaskCreate(ypr_task_polling, "ypr_poll", 4096, NULL, 5, NULL);
    // xTaskCreate(dmp_task_polling, "dmp_poll", 4096, NULL, 5, NULL);
    // xTaskCreate(pwm_cycle_task, "pwm_cycle_task", 4096, NULL, 5, NULL);
}
