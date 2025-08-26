#pragma once
#include <stdint.h>

typedef struct {
    uint8_t wasd;   // from hex digit 1
    uint8_t tfgh;   // from hex digit 2
    uint8_t ijkl;   // from hex digit 3
} key_packet_t;