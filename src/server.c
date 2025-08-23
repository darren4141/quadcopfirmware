#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_http_server.h"

#define TAG "APP"
#define LED_GPIO 15  // onboard LED (active-LOW)

// ---------------- HTTP + WS ----------------
static httpd_handle_t s_server = NULL;
static volatile uint8_t s_wasd = 0, s_ijkl = 0, s_tfgh = 0; // last packets from client

static esp_err_t root_get_handler(httpd_req_t *req) {
    // Tiny page with keyboard handling and WS client
    const char *html =
        "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'/>"
        "<title>ESP32-C6 Keyboard</title>"
        "<style>body{font-family:system-ui;margin:16px}code{background:#eee;padding:2px 6px;border-radius:4px}</style>"
        "</head><body>"
        "<h2>Keyboard → 3×4-bit packet</h2>"
        "<p>Press keys: <b>WASD</b>, <b>IJKL</b>, <b>TFGH</b>. Packets go to ESP via WebSocket.</p>"
        "<p>Packet format: <code>P:&lt;W&gt;&lt;I&gt;&lt;T&gt;</code> (each is a hex nibble).</p>"
        "<div id='state'></div>"
        "<script>\n"
        "let ws; let wasd=0, ijkl=0, tfgh=0; let dirty=false;\n"
        "const keyBit = {\n"
        "  'w':['wasd',0],'a':['wasd',1],'s':['wasd',2],'d':['wasd',3],\n"
        "  'i':['ijkl',0],'j':['ijkl',1],'k':['ijkl',2],'l':['ijkl',3],\n"
        "  't':['tfgh',0],'f':['tfgh',1],'g':['tfgh',2],'h':['tfgh',3]\n"
        "};\n"
        "function nibbleToHex(n){return n.toString(16).toUpperCase();}\n"
        "function sendPacket(){ if(!ws || ws.readyState!==1) return; const pkt=`P:${nibbleToHex(wasd)}${nibbleToHex(ijkl)}${nibbleToHex(tfgh)}`; ws.send(pkt); }\n"
        "function updateState(){document.getElementById('state').innerHTML=\n"
        "  `WASD: 0x${nibbleToHex(wasd)} &nbsp; IJKL: 0x${nibbleToHex(ijkl)} &nbsp; TFGH: 0x${nibbleToHex(tfgh)}`;}\n"
        "function setBit(setName, bit, on){\n"
        "  let v = (setName==='wasd')?wasd:(setName==='ijkl')?ijkl:tfgh;\n"
        "  v = on ? (v | (1<<bit)) : (v & ~(1<<bit));\n"
        "  if(setName==='wasd') wasd=v; else if(setName==='ijkl') ijkl=v; else tfgh=v;\n"
        "  dirty=true; updateState();\n"
        "}\n"
        "window.addEventListener('keydown', e=>{const k=e.key.toLowerCase(); if(keyBit[k]){setBit(keyBit[k][0], keyBit[k][1], true);}});\n"
        "window.addEventListener('keyup',   e=>{const k=e.key.toLowerCase(); if(keyBit[k]){setBit(keyBit[k][0], keyBit[k][1], false);}});\n"
        "function connect(){\n"
        "  const proto = location.protocol==='https:'?'wss':'ws';\n"
        "  ws = new WebSocket(`${proto}://${location.host}/ws`);\n"
        "  ws.onopen  = ()=>{console.log('WS open'); updateState();};\n"
        "  ws.onclose = ()=>{console.log('WS closed'); setTimeout(connect, 1000);};\n"
        "  ws.onmessage = (ev)=>{console.log('WS <-', ev.data);} // debug echo\n"
        "}\n"
        "connect();\n"
        "// Send on change, throttled ~60Hz\n"
        "setInterval(()=>{ if(dirty){ dirty=false; sendPacket(); } }, 16);\n"
        "</script>"
        "</body></html>";

    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}

// WebSocket endpoint
static esp_err_t ws_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        // Handshake complete
        return ESP_OK;
    }
    httpd_ws_frame_t frame = {
        .final = true,
        .fragmented = false,
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = NULL,
        .len = 0
    };
    esp_err_t ret = httpd_ws_recv_frame(req, &frame, 0);
    if (ret != ESP_OK) return ret;

    frame.payload = malloc(frame.len + 1);
    if (!frame.payload) return ESP_ERR_NO_MEM;

    ret = httpd_ws_recv_frame(req, &frame, frame.len);
    if (ret == ESP_OK) {
        frame.payload[frame.len] = 0;
        // Expect "P:<W><I><T>", where each <…> is hex nibble
        if (frame.len >= 5 && frame.payload[0]=='P' && frame.payload[1]==':') {
            char w = frame.payload[2], i = frame.payload[3], t = frame.payload[4];
            uint8_t wn = (uint8_t)strtoul((char[]){w,0}, NULL, 16) & 0xF;
            uint8_t in = (uint8_t)strtoul((char[]){i,0}, NULL, 16) & 0xF;
            uint8_t tn = (uint8_t)strtoul((char[]){t,0}, NULL, 16) & 0xF;
            s_wasd = wn; s_ijkl = in; s_tfgh = tn;
            ESP_LOGI(TAG, "WASD=0x%X IJKL=0x%X TFGH=0x%X", wn, in, tn);
            // Example: drive onboard LED on any key pressed (active-LOW)
            bool any = (wn|in|tn) != 0;
            gpio_set_level(LED_GPIO, any ? 0 : 1);
        }
        // Echo ack back (optional)
        const char *ok = "OK";
        httpd_ws_frame_t rsp = {
            .final = true, .fragmented = false, .type = HTTPD_WS_TYPE_TEXT,
            .payload = (uint8_t*)ok, .len = strlen(ok)
        };
        httpd_ws_send_frame(req, &rsp);
    }
    free(frame.payload);
    return ret;
}

static httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        // Page
        httpd_uri_t root = { .uri="/", .method=HTTP_GET, .handler=root_get_handler, .user_ctx=NULL };
        httpd_register_uri_handler(server, &root);
        // WebSocket
        httpd_uri_t ws = { .uri="/ws", .method=HTTP_GET, .handler=ws_handler, .user_ctx=NULL, .is_websocket=true };
        httpd_register_uri_handler(server, &ws);

        ESP_LOGI(TAG, "HTTP+WS server started");
    } else {
        ESP_LOGE(TAG, "HTTP server start failed");
    }
    return server;
}

// ---------------- Wi-Fi SoftAP ----------------
static void wifi_init_softap(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t ap_config = {
        .ap = {
            .ssid = "ESP32C6_AP",
            .ssid_len = 0,
            .channel = 1,
            .password = "esp32c6pass",
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        }
    };
    if (strlen((char*)ap_config.ap.password) == 0) ap_config.ap.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "SoftAP up: SSID:%s PASS:%s IP: 192.168.4.1", ap_config.ap.ssid, ap_config.ap.password);
}