#pragma once
#include <stdint.h>

static const uint8_t PACKET_SIZE    = 16;
static const uint8_t OFF_TIMESTAMP  =  0;
static const uint8_t OFF_LAT        =  4;
static const uint8_t OFF_LNG        =  8;
static const uint8_t OFF_ALT        = 12;

struct TelemetryPacket {
    uint32_t timestamp;
    float    lat;
    float    lng;
    float    altitude;
    float    rssi;
    float    snr;
};
