#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "mpu6050.c"
#include "pwm.c"
#include "espnow.c"
#include "server.c"

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

    //espnow reciever
    // uint8_t self[6];
    // esp_read_mac(self, ESP_MAC_WIFI_STA);
    // ESP_LOGI(TAGMAIN, "C6 RX MAC: %02X:%02X:%02X:%02X:%02X:%02X",
    //          self[0],self[1],self[2],self[3],self[4],self[5]);

    // espnow_init();

    // // Receive callbacks
    // ESP_ERROR_CHECK(esp_now_register_recv_cb(recv_cb));

    // // If you want to restrict accepted peers, call esp_now_add_peer() with the sender MAC.
    // // Otherwise, broadcast works too, but unicast is more reliable.

    // // Create a 1-deep queue so producers never block; we only care about the latest packet
    // q_inputs = xQueueCreate(1, sizeof(controller_pkt_t));

    // // Start your consumer and your existing DMP task
    // xTaskCreate(control_consumer_task, "ctrl_cons", 4096, NULL, 5, NULL);

    // NVS init for Wi-Fi
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // LED
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 1); // OFF (active-LOW)

    wifi_init_softap();
    s_server = start_webserver();

    // xTaskCreate(pwm_cycle_task, "pwm_cycle_task", 4096, NULL, 5, NULL);
    // xTaskCreate(dmp_task_polling, "dmp_poll", 4096, NULL, 5, NULL);
}
