#include "server.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_err.h"
#include "packet.h"
#include "pwm.h"
#include "server_html.h"
#include "mpu6050.h"

// ----------- YPR ring buffer + offsets -----------
static float ring[RING_N][3];    // [i][0]=yaw, [1]=pitch, [2]=roll
static int   ring_head = 0;
static int   ring_count = 0;
static float off[3] = {0,0,0};   // zero offsets: yaw,pitch,roll
static int pwm_cur[5]  = {0,0,0,0,0};
static SemaphoreHandle_t ypr_mtx;
static SemaphoreHandle_t pwm_mtx;

// Call this from your DMP polling task when you have a new YPR (radians or degrees—your choice)
void imu_push_ypr_to_server(const float yaw, const float pitch, const float roll) {
    if (!ypr_mtx) return;
    xSemaphoreTake(ypr_mtx, portMAX_DELAY);
    ring[ring_head][0] = yaw;
    ring[ring_head][1] = pitch;
    ring[ring_head][2] = roll;
    ring_head = (ring_head + 1) % RING_N;
    if (ring_count < RING_N) ring_count++;
    xSemaphoreGive(ypr_mtx);
}

void pwm_push_to_server(const int *vals, const int mode){
    if (!pwm_mtx) return;
    xSemaphoreTake(pwm_mtx, portMAX_DELAY);
    pwm_cur[0] = vals[0]; 
    pwm_cur[1] = vals[1]; 
    pwm_cur[2] = vals[2]; 
    pwm_cur[3] = vals[3];
    pwm_cur[4] = mode;
    xSemaphoreGive(pwm_mtx);
}

// Average last N samples (clamped to available)
static int avg_last(int N, float out[3]) {
    if (!ypr_mtx) return 0;
    xSemaphoreTake(ypr_mtx, portMAX_DELAY);
    if (ring_count == 0) { xSemaphoreGive(ypr_mtx); return 0; }
    if (N > ring_count) N = ring_count;

    double s0=0, s1=0, s2=0;
    int idx = (ring_head - 1 + RING_N) % RING_N;
    for (int i=0; i<N; i++) {
        s0 += ring[idx][0];
        s1 += ring[idx][1];
        s2 += ring[idx][2];
        idx = (idx - 1 + RING_N) % RING_N;
    }
    out[0] = (float)(s0 / N);
    out[1] = (float)(s1 / N);
    out[2] = (float)(s2 / N);
    xSemaphoreGive(ypr_mtx);
    return N;
}

// Set zero offsets = avg of last 20 (or fewer if not enough yet)
static void do_zero(void) {
    float m[3];
    int used = avg_last(RING_N, m);
    if (used) {
        off[0]=m[0]; off[1]=m[1]; off[2]=m[2];
        ESP_LOGI(TAG, "Zeroed YPR using last %d samples: off=[%.3f, %.3f, %.3f]", used, off[0], off[1], off[2]);
    } else {
        ESP_LOGW(TAG, "Zero failed: no samples yet");
    }
}

// Read most recent sample (minus offsets). Returns 1 if ok.
static int latest_zeroed(float z[3]) {
    if (!ypr_mtx) return 0;
    xSemaphoreTake(ypr_mtx, portMAX_DELAY);
    if (ring_count == 0) { xSemaphoreGive(ypr_mtx); return 0; }
    int idx = (ring_head - 1 + RING_N) % RING_N;
    z[0] = ring[idx][0] - off[0];
    z[1] = ring[idx][1] - off[1];
    z[2] = ring[idx][2] - off[2];
    xSemaphoreGive(ypr_mtx);
    return 1;
}

// ---------------- HTTP handlers ----------------
static esp_err_t root_get(httpd_req_t *req) {
    // Minimal page: shows YPR and listens for keys; T triggers zero.
    const char *html = server_html;
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t imu_json(httpd_req_t *req) {
    float z[3];
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");

    if (latest_zeroed(z)) {
        char buf[128];
        int n = snprintf(buf, sizeof(buf),
                         "{\"yaw\":%.4f,\"pitch\":%.4f,\"roll\":%.4f}", z[0], z[1], z[2]);
        return httpd_resp_send(req, buf, n);
    } else {
        return httpd_resp_sendstr(req, "{\"err\":\"no data\"}");
    }
}

static esp_err_t pwm_json(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");

    int c[5];
    if (!pwm_mtx) return httpd_resp_sendstr(req, "{\"err\":\"uninit\"}");

    xSemaphoreTake(pwm_mtx, portMAX_DELAY);
    for (int i = 0; i < 5; i++){ 
        c[i] = pwm_cur[i];
    }
    xSemaphoreGive(pwm_mtx);

    char buf[160];
    int n = snprintf(buf, sizeof(buf),
                    "{\"mot1\":%d,\"mot2\":%d,\"mot3\":%d,\"mot4\":%d,\"mode\":%d}",
                    c[0], c[1], c[2], c[3], c[4]);
    return httpd_resp_send(req, buf, n);
}

static esp_err_t zero_post(httpd_req_t *req) {
    do_zero();
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_sendstr(req, "ZEROED");
}

// ---------------- Wi-Fi (AP) + server ----------------
void wifi_ap_start(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t ap = { .ap = {
        .ssid = "ESP32C6_AP",
        .password = "esp32c6pass",
        .channel = 1,
        .max_connection = 4,
        .authmode = WIFI_AUTH_WPA_WPA2_PSK
    }};

    // IP: 192.168.4.1
    if (strlen((char*)ap.ap.password) == 0) ap.ap.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// ---------------- WebSocket ----------------
esp_err_t ws_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET){
        return ESP_OK;
    }

    httpd_ws_frame_t f = {0};
    f.type = HTTPD_WS_TYPE_TEXT;

    //peek at length
    ESP_ERROR_CHECK(httpd_ws_recv_frame(req, &f, 0));
    if (f.len == 0) return ESP_OK;

    f.payload = malloc(f.len + 1);
    if (!f.payload) return ESP_ERR_NO_MEM;
    ESP_ERROR_CHECK(httpd_ws_recv_frame(req, &f, f.len));
    f.payload[f.len] = 0;
    
    if (f.len >= 5 && f.payload[0]=='P' && f.payload[1]==':') {
        // Decode three hex digits into uint8_t values
        key_packet_t pkt = {
            .wasd = (uint8_t)strtol((char[]){f.payload[2],0}, NULL, 16) & 0xF,
            .tfgh = (uint8_t)strtol((char[]){f.payload[4],0}, NULL, 16) & 0xF,
            .ijkl = (uint8_t)strtol((char[]){f.payload[3],0}, NULL, 16) & 0xF
        };

        if((pkt.wasd & 0x00) && (pkt.ijkl & 0x00)){
            ESP_LOGI(TAG, "wasd and ijkl empty, hovering");
            // setMode(0);
        }

        if(pkt.wasd & 0x01){ //W pressed
            ESP_LOGI(TAG, "W pressed");
            setMode(1);
        }
        
        if(pkt.wasd & 0x02){ //A pressed
            ESP_LOGI(TAG, "A pressed");
            setMode(3);
        }
        
        if(pkt.wasd & 0x04){ //S pressed
            ESP_LOGI(TAG, "S pressed");
            setMode(2);
        }
        
        if(pkt.wasd & 0x08){ //D pressed
            ESP_LOGI(TAG, "D pressed");
            setMode(4);
        }
        
        if(pkt.ijkl & 0x01){ //I pressed
            ESP_LOGI(TAG, "I pressed");
            setMode(5);
        }
        
        if(pkt.ijkl & 0x02){ //J pressed
            ESP_LOGI(TAG, "J pressed");
            setMode(7);
        }
        
        if(pkt.ijkl & 0x04){ //K pressed
            ESP_LOGI(TAG, "K pressed");
            setMode(6);
        }
        
        if(pkt.ijkl & 0x08){ //L pressed
            ESP_LOGI(TAG, "L pressed");
            setMode(8);
        }
        
        if(pkt.tfgh & 0x01){ //T pressed - Zeroing function
            ESP_LOGI(TAG, "T pressed");
            mpu6050_calibrate(500);
        }
        
        if(pkt.tfgh & 0x02){ //F pressed
            ESP_LOGI(TAG, "F pressed");
            setMode(9);
        }
        
        if(pkt.tfgh & 0x04){ //G pressed
            ESP_LOGI(TAG, "G pressed");
        }
        
        if(pkt.tfgh & 0x08){ //H pressed
            ESP_LOGI(TAG, "H pressed");
        }
    }

    free(f.payload);
    return ESP_OK;
}

esp_err_t server_init(void) {
    // --------- Initialize NVS ---------
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // --------- LED setup (optional indicator) ---------
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 1);   // active-LOW -> 1 = OFF

    // --------- Create mutex for YPR ring buffer ---------
    ypr_mtx = xSemaphoreCreateMutex();
    if (!ypr_mtx) {
        ESP_LOGE(TAG, "Failed to create ypr_mtx mutex");
        return ESP_ERR_NO_MEM;
    }

    pwm_mtx = xSemaphoreCreateMutex();
    if (!pwm_mtx) {
        ESP_LOGE(TAG, "Failed to create pwm_mtx mutex");
        return ESP_ERR_NO_MEM;
    }

    // --------- Wi-Fi Access Point ---------
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t ap_cfg = { .ap = {
        .ssid = "ESP32C6_AP",
        .password = "esp32c6pass",
        .channel = 1,
        .max_connection = 4,
        .authmode = WIFI_AUTH_WPA_WPA2_PSK
    }};
    if (strlen((char*)ap_cfg.ap.password) == 0) {
        ap_cfg.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Wi-Fi AP started, SSID:%s password:%s",
             ap_cfg.ap.ssid, ap_cfg.ap.password);

    // --------- HTTP server ---------
    httpd_handle_t srv = NULL;
    httpd_config_t http_cfg = HTTPD_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(httpd_start(&srv, &http_cfg));

    // Register endpoints
    httpd_uri_t root = { .uri="/", .method=HTTP_GET, .handler=root_get, .user_ctx=NULL };
    httpd_register_uri_handler(srv, &root);

    httpd_uri_t imu = { .uri="/imu.json", .method=HTTP_GET, .handler=imu_json, .user_ctx=NULL };
    httpd_register_uri_handler(srv, &imu);

    httpd_uri_t pwm = { .uri="/pwm.json", .method=HTTP_GET, .handler=pwm_json, .user_ctx=NULL };
    httpd_register_uri_handler(srv, &pwm);

    httpd_uri_t zero = { .uri="/zero", .method=HTTP_POST, .handler=zero_post, .user_ctx=NULL };
    httpd_register_uri_handler(srv, &zero);
    
    httpd_uri_t ws = {
        .uri       = "/ws",
        .method    = HTTP_GET,
        .handler   = ws_handler,
        .user_ctx  = NULL,
#if defined(CONFIG_HTTPD_WS_SUPPORT) && CONFIG_HTTPD_WS_SUPPORT
        .is_websocket = true,                // <-- required when WS is enabled
#endif
    };
    httpd_register_uri_handler(srv, &ws);

    ESP_LOGI(TAG, "HTTP server started on port %d", http_cfg.server_port);

    return ESP_OK;
}