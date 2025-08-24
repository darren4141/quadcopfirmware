#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c.h"
#include "nvs_flash.h"

#include "pwm.h"
#include "mpu6050.h"
#include "server.h"

static const char *TAGMAIN = "MAIN_RX";

#define D0_PIN 0
#define D2_PIN 2
#define D3_PIN 21
#define D8_PIN 19

void app_main(void) {

    // I2C master init
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ_INIT,
        .clk_flags = 0
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0));

    // Detect and basic init
    ESP_ERROR_CHECK(mpu_detect());
    ESP_LOGI(TAGMAIN, "MPU6050 detected at 0x%02X", mpu_addr);
    ESP_ERROR_CHECK(mpu_basic_init());

    // DMP init
    ESP_ERROR_CHECK(mpu_dmp_initialize());
    ESP_LOGI(TAGMAIN, "DMP ready; packet size = %d", DMP_PACKET_SIZE);

    i2c_driver_delete(I2C_PORT);

    i2c_config_t conf2 = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,  // 400 kHz
        .clk_flags = 0
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf2));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, conf2.mode, 0, 0, 0));
    
    // NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // LED
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 1); // OFF (active-LOW)

    ESP_ERROR_CHECK(server_init());
    xTaskCreate(ypr_task_polling, "ypr_poll", 4096, NULL, 5, NULL);
    // xTaskCreate(dmp_task_polling, "dmp_poll", 4096, NULL, 5, NULL);
    // xTaskCreate(pwm_cycle_task, "pwm_cycle_task", 4096, NULL, 5, NULL);
}
