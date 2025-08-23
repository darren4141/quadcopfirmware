#pragma once
#include <stdio.h>
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_system.h"
#include "packet.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"


static inline void wifi_init_sta_channel(uint8_t channel){
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));      // ESPNOW needs Wi-Fi driver, not a connection
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(false));
}

static inline void espnow_init(void){
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init_sta_channel(ESPNOW_WIFI_CHANNEL);
    ESP_ERROR_CHECK(esp_now_init());
    // (Optional) set a Primary Master Key if you plan to use encrypted peers
    // const uint8_t pmk[16] = { /* 16 bytes */ };
    // ESP_ERROR_CHECK(esp_now_set_pmk(pmk));
}

static const char *TAGESPNOW = "ESPNOW_RX";

static QueueHandle_t q_inputs; // queue of controller_pkt_t

static void recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len){
    if (len != sizeof(controller_pkt_t)) return;
    controller_pkt_t pkt;
    memcpy(&pkt, data, sizeof(pkt));
    (void)info; // you can inspect info->src_addr, rssi, etc.

    if (q_inputs) {
        xQueueOverwrite(q_inputs, &pkt); // keep only the latest
    }
}

static void control_consumer_task(void *arg){
    controller_pkt_t pkt = {0};
    for(;;) {
        if (xQueueReceive(q_inputs, &pkt, pdMS_TO_TICKS(100))) {
            ESP_LOGI("CTRL", "seq=%u LX=%d LY=%d RX=%d RY=%d BTN=0x%04X",
                     pkt.seq, pkt.lx, pkt.ly, pkt.rx, pkt.ry, pkt.buttons);
        }
    }
}