#pragma once
#include <stdint.h>

#define ESPNOW_WIFI_CHANNEL   6     // pick any 1..13; both sides must match
#define ESPNOW_PAYLOAD_VER    1

typedef struct __attribute__((packed)) {
    uint8_t  ver;
    int16_t  lx;     // left X   (-32768..32767)
    int16_t  ly;     // left Y
    int16_t  rx;     // right X
    int16_t  ry;     // right Y
    uint16_t buttons; // bitfield
    uint32_t seq;     // sequence number
} controller_pkt_t;
