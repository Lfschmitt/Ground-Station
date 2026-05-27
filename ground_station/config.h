#pragma once

// ── LoRa (Heltec LoRa32 v2) ───────────────────────────────────────
#define LORA_CS    18
#define LORA_RST   14
#define LORA_DIO0  26
#define PIN_SCK     5
#define PIN_MISO   19
#define PIN_MOSI   27

#define LORA_FREQ   915.0f
#define LORA_BW     500.0f
#define LORA_SF       9
#define LORA_CR       5
#define LORA_SYNC  0x12
#define LORA_POWER   17
#define LORA_PREAMBLE 8

// ── OLED SSD1306 ──────────────────────────────────────────────────
#define OLED_SDA    4
#define OLED_SCL   15
#define OLED_RST   16
#define OLED_ADDR  0x3C

// ── Serial para PC ────────────────────────────────────────────────
#define SERIAL_BAUD 115200
