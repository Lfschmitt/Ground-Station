/*****************************************************\
    LoRa Ground Station Receiver
    Hardware : Heltec LoRa32 v2 (ESP32 + SX1276 + SSD1306)

    Recebe pacotes de telemetria do flight computer,
    imprime no Serial e exibe no OLED a cada 5 segundos.
    Um asterisco (*) pisca no canto superior direito
    a cada atualização.

    RF idêntico ao transmissor:
      915.0 MHz | BW 500 kHz | SF 9 | CR 4/5 | Sync 0x12
\*****************************************************/

#include <SPI.h>
#include <RadioLib.h>
#include <Wire.h>

// ── Pinos LoRa (Heltec LoRa32 v2) ────────────────────────────────
#define LORA_CS   18
#define LORA_RST  14
#define LORA_DIO0 26
#define PIN_SCK    5
#define PIN_MISO  19
#define PIN_MOSI  27

// ── Pinos OLED (Heltec LoRa32 v2) ────────────────────────────────
#define OLED_SDA   4
#define OLED_SCL  15
#define OLED_RST  16
#define OLED_ADDR 0x3C

// ──────────────────────────────────────────────────────────────────
// Rádio
// ──────────────────────────────────────────────────────────────────
SX1276 radio = new Module(LORA_CS, LORA_DIO0, LORA_RST, RADIOLIB_NC, SPI);

volatile bool rxFlag = false;
void IRAM_ATTR onReceiveDone() { rxFlag = true; }

// ── Formato do pacote (16 bytes) ──────────────────────────────────
static const uint8_t PACKET_SIZE   = 16;
static const uint8_t OFF_TIMESTAMP = 0;
static const uint8_t OFF_LAT       = 4;
static const uint8_t OFF_LNG       = 8;
static const uint8_t OFF_ALT       = 12;

uint32_t readU32(const uint8_t* b, int o) { uint32_t v; memcpy(&v, b+o, 4); return v; }
float    readF32(const uint8_t* b, int o) { float    v; memcpy(&v, b+o, 4); return v; }

// ── Último pacote recebido ─────────────────────────────────────────
static uint32_t g_timestamp = 0;
static float    g_lat       = 0.0f;
static float    g_lng       = 0.0f;
static float    g_altitude  = 0.0f;
static float    g_rssi      = 0.0f;
static float    g_snr       = 0.0f;
static bool     g_hasData   = false;

// ──────────────────────────────────────────────────────────────────
// Font 5×7 — 5 bytes por char, ASCII 32–126
// Cada byte é uma coluna vertical de 8 pixels (bit0 = topo)
// ──────────────────────────────────────────────────────────────────
static const uint8_t FONT5X7[][5] PROGMEM = {
    {0x00,0x00,0x00,0x00,0x00}, // 32 ' '
    {0x00,0x00,0x5F,0x00,0x00}, // 33 '!'
    {0x00,0x07,0x00,0x07,0x00}, // 34 '"'
    {0x14,0x7F,0x14,0x7F,0x14}, // 35 '#'
    {0x24,0x2A,0x7F,0x2A,0x12}, // 36 '$'
    {0x23,0x13,0x08,0x64,0x62}, // 37 '%'
    {0x36,0x49,0x55,0x22,0x50}, // 38 '&'
    {0x00,0x05,0x03,0x00,0x00}, // 39 '\''
    {0x00,0x1C,0x22,0x41,0x00}, // 40 '('
    {0x00,0x41,0x22,0x1C,0x00}, // 41 ')'
    {0x14,0x08,0x3E,0x08,0x14}, // 42 '*'
    {0x08,0x08,0x3E,0x08,0x08}, // 43 '+'
    {0x00,0x50,0x30,0x00,0x00}, // 44 ','
    {0x08,0x08,0x08,0x08,0x08}, // 45 '-'
    {0x00,0x60,0x60,0x00,0x00}, // 46 '.'
    {0x20,0x10,0x08,0x04,0x02}, // 47 '/'
    {0x3E,0x51,0x49,0x45,0x3E}, // 48 '0'
    {0x00,0x42,0x7F,0x40,0x00}, // 49 '1'
    {0x42,0x61,0x51,0x49,0x46}, // 50 '2'
    {0x21,0x41,0x45,0x4B,0x31}, // 51 '3'
    {0x18,0x14,0x12,0x7F,0x10}, // 52 '4'
    {0x27,0x45,0x45,0x45,0x39}, // 53 '5'
    {0x3C,0x4A,0x49,0x49,0x30}, // 54 '6'
    {0x01,0x71,0x09,0x05,0x03}, // 55 '7'
    {0x36,0x49,0x49,0x49,0x36}, // 56 '8'
    {0x06,0x49,0x49,0x29,0x1E}, // 57 '9'
    {0x00,0x36,0x36,0x00,0x00}, // 58 ':'
    {0x00,0x56,0x36,0x00,0x00}, // 59 ';'
    {0x08,0x14,0x22,0x41,0x00}, // 60 '<'
    {0x14,0x14,0x14,0x14,0x14}, // 61 '='
    {0x00,0x41,0x22,0x14,0x08}, // 62 '>'
    {0x02,0x01,0x51,0x09,0x06}, // 63 '?'
    {0x32,0x49,0x79,0x41,0x3E}, // 64 '@'
    {0x7E,0x11,0x11,0x11,0x7E}, // 65 'A'
    {0x7F,0x49,0x49,0x49,0x36}, // 66 'B'
    {0x3E,0x41,0x41,0x41,0x22}, // 67 'C'
    {0x7F,0x41,0x41,0x22,0x1C}, // 68 'D'
    {0x7F,0x49,0x49,0x49,0x41}, // 69 'E'
    {0x7F,0x09,0x09,0x09,0x01}, // 70 'F'
    {0x3E,0x41,0x49,0x49,0x7A}, // 71 'G'
    {0x7F,0x08,0x08,0x08,0x7F}, // 72 'H'
    {0x00,0x41,0x7F,0x41,0x00}, // 73 'I'
    {0x20,0x40,0x41,0x3F,0x01}, // 74 'J'
    {0x7F,0x08,0x14,0x22,0x41}, // 75 'K'
    {0x7F,0x40,0x40,0x40,0x40}, // 76 'L'
    {0x7F,0x02,0x04,0x02,0x7F}, // 77 'M'
    {0x7F,0x04,0x08,0x10,0x7F}, // 78 'N'
    {0x3E,0x41,0x41,0x41,0x3E}, // 79 'O'
    {0x7F,0x09,0x09,0x09,0x06}, // 80 'P'
    {0x3E,0x41,0x51,0x21,0x5E}, // 81 'Q'
    {0x7F,0x09,0x19,0x29,0x46}, // 82 'R'
    {0x46,0x49,0x49,0x49,0x31}, // 83 'S'
    {0x01,0x01,0x7F,0x01,0x01}, // 84 'T'
    {0x3F,0x40,0x40,0x40,0x3F}, // 85 'U'
    {0x1F,0x20,0x40,0x20,0x1F}, // 86 'V'
    {0x3F,0x40,0x38,0x40,0x3F}, // 87 'W'
    {0x63,0x14,0x08,0x14,0x63}, // 88 'X'
    {0x07,0x08,0x70,0x08,0x07}, // 89 'Y'
    {0x61,0x51,0x49,0x45,0x43}, // 90 'Z'
    {0x00,0x7F,0x41,0x41,0x00}, // 91 '['
    {0x02,0x04,0x08,0x10,0x20}, // 92 '\'
    {0x00,0x41,0x41,0x7F,0x00}, // 93 ']'
    {0x04,0x02,0x01,0x02,0x04}, // 94 '^'
    {0x40,0x40,0x40,0x40,0x40}, // 95 '_'
    {0x00,0x01,0x02,0x04,0x00}, // 96 '`'
    {0x20,0x54,0x54,0x54,0x78}, // 97 'a'
    {0x7F,0x48,0x44,0x44,0x38}, // 98 'b'
    {0x38,0x44,0x44,0x44,0x20}, // 99 'c'
    {0x38,0x44,0x44,0x48,0x7F}, // 100 'd'
    {0x38,0x54,0x54,0x54,0x18}, // 101 'e'
    {0x08,0x7E,0x09,0x01,0x02}, // 102 'f'
    {0x0C,0x52,0x52,0x52,0x3E}, // 103 'g'
    {0x7F,0x08,0x04,0x04,0x78}, // 104 'h'
    {0x00,0x44,0x7D,0x40,0x00}, // 105 'i'
    {0x20,0x40,0x44,0x3D,0x00}, // 106 'j'
    {0x7F,0x10,0x28,0x44,0x00}, // 107 'k'
    {0x00,0x41,0x7F,0x40,0x00}, // 108 'l'
    {0x7C,0x04,0x18,0x04,0x78}, // 109 'm'
    {0x7C,0x08,0x04,0x04,0x78}, // 110 'n'
    {0x38,0x44,0x44,0x44,0x38}, // 111 'o'
    {0x7C,0x14,0x14,0x14,0x08}, // 112 'p'
    {0x08,0x14,0x14,0x18,0x7C}, // 113 'q'
    {0x7C,0x08,0x04,0x04,0x08}, // 114 'r'
    {0x48,0x54,0x54,0x54,0x20}, // 115 's'
    {0x04,0x3F,0x44,0x40,0x20}, // 116 't'
    {0x3C,0x40,0x40,0x20,0x7C}, // 117 'u'
    {0x1C,0x20,0x40,0x20,0x1C}, // 118 'v'
    {0x3C,0x40,0x30,0x40,0x3C}, // 119 'w'
    {0x44,0x28,0x10,0x28,0x44}, // 120 'x'
    {0x0C,0x50,0x50,0x50,0x3C}, // 121 'y'
    {0x44,0x64,0x54,0x4C,0x44}, // 122 'z'
    {0x00,0x08,0x36,0x41,0x00}, // 123 '{'
    {0x00,0x00,0x7F,0x00,0x00}, // 124 '|'
    {0x00,0x41,0x36,0x08,0x00}, // 125 '}'
    {0x10,0x08,0x08,0x10,0x08}, // 126 '~'
};

// ──────────────────────────────────────────────────────────────────
// Driver SSD1306 (sem biblioteca)
// Display 128×64 px = 8 páginas × 128 colunas
// Fonte 6×8 px (5 px char + 1 px espaço) → 21 chars × 8 linhas
// ──────────────────────────────────────────────────────────────────

static void oledCmd(uint8_t c) {
    Wire.beginTransmission(OLED_ADDR);
    Wire.write(0x00); // Co=0, D/C=0 → comando
    Wire.write(c);
    Wire.endTransmission();
}

static void oledInit() {
    pinMode(OLED_RST, OUTPUT);
    digitalWrite(OLED_RST, LOW);  delay(10);
    digitalWrite(OLED_RST, HIGH); delay(10);
    Wire.begin(OLED_SDA, OLED_SCL);

    oledCmd(0xAE);              // display off
    oledCmd(0xD5); oledCmd(0x80); // clock divider
    oledCmd(0xA8); oledCmd(0x3F); // multiplex = 63 (64 linhas)
    oledCmd(0xD3); oledCmd(0x00); // offset = 0
    oledCmd(0x40);              // start line = 0
    oledCmd(0x8D); oledCmd(0x14); // charge pump ON
    oledCmd(0x20); oledCmd(0x00); // modo de endereçamento horizontal
    oledCmd(0xA1);              // seg remap: col 127 → SEG0
    oledCmd(0xC8);              // COM scan decrescente
    oledCmd(0xDA); oledCmd(0x12); // COM pins
    oledCmd(0x81); oledCmd(0xCF); // contraste
    oledCmd(0xD9); oledCmd(0xF1); // pre-charge
    oledCmd(0xDB); oledCmd(0x40); // VCOMH
    oledCmd(0xA4);              // exibe RAM
    oledCmd(0xA6);              // display normal
    oledCmd(0xAF);              // display on
}

static void oledClear() {
    oledCmd(0x21); oledCmd(0); oledCmd(127); // colunas 0–127
    oledCmd(0x22); oledCmd(0); oledCmd(7);   // páginas 0–7
    // 128×8 = 1024 bytes, enviados em 32 blocos de 32 bytes
    for (int i = 0; i < 32; i++) {
        Wire.beginTransmission(OLED_ADDR);
        Wire.write(0x40); // Co=0, D/C=1 → dados
        for (int j = 0; j < 32; j++) Wire.write(0x00);
        Wire.endTransmission();
    }
}

static void oledSetCursor(uint8_t page, uint8_t col) {
    oledCmd(0x21); oledCmd(col);  oledCmd(127);
    oledCmd(0x22); oledCmd(page); oledCmd(7);
}

static void oledWriteChar(char c) {
    if (c < 32 || c > 126) c = 32;
    Wire.beginTransmission(OLED_ADDR);
    Wire.write(0x40);
    for (int i = 0; i < 5; i++) Wire.write(pgm_read_byte(&FONT5X7[c - 32][i]));
    Wire.write(0x00); // 1 px de espaço entre caracteres
    Wire.endTransmission();
}

static void oledWriteStr(const char* s) {
    while (*s) oledWriteChar(*s++);
}

// ──────────────────────────────────────────────────────────────────
// Atualização do display (chamada a cada 5 segundos)
// ──────────────────────────────────────────────────────────────────

static bool _dot = false;

static void updateDisplay() {
    _dot = !_dot; // alterna o ponto a cada chamada
    oledClear();
    char line[22]; // 21 chars + '\0'

    // Linha 0: título + ponto piscante no canto direito
    // "LoRa Receiver      *"  ou  "LoRa Receiver       "
    oledSetCursor(0, 0);
    snprintf(line, sizeof(line), "LoRa Receiver      %c", _dot ? '*' : ' ');
    oledWriteStr(line);

    if (!g_hasData) {
        oledSetCursor(3, 0);
        oledWriteStr("  Aguardando dados  ");
        return;
    }

    oledSetCursor(1, 0);
    snprintf(line, sizeof(line), "T: %lu ms", (unsigned long)g_timestamp);
    oledWriteStr(line);

    oledSetCursor(2, 0);
    snprintf(line, sizeof(line), "Lat: %.6f", (double)g_lat);
    oledWriteStr(line);

    oledSetCursor(3, 0);
    snprintf(line, sizeof(line), "Lon: %.6f", (double)g_lng);
    oledWriteStr(line);

    oledSetCursor(4, 0);
    snprintf(line, sizeof(line), "Alt: %.1f m", (double)g_altitude);
    oledWriteStr(line);

    oledSetCursor(5, 0);
    snprintf(line, sizeof(line), "RSSI: %.1f dBm", (double)g_rssi);
    oledWriteStr(line);

    oledSetCursor(6, 0);
    snprintf(line, sizeof(line), "SNR:  %.1f dB", (double)g_snr);
    oledWriteStr(line);
}

// ──────────────────────────────────────────────────────────────────
// setup() / loop()
// ──────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);

    oledInit();
    oledClear();
    oledSetCursor(0, 0); oledWriteStr("LoRa Receiver");
    oledSetCursor(1, 0); oledWriteStr("Iniciando...");

    SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, LORA_CS);

    int state = radio.begin(915.0, 500.0, 9, 5, 0x12, 17, 8);
    if (state != RADIOLIB_ERR_NONE) {
        char msg[22];
        snprintf(msg, sizeof(msg), "Erro LoRa: %d", state);
        oledSetCursor(2, 0); oledWriteStr(msg);
        Serial.printf("[ERRO] SX1276 falhou, codigo: %d\n", state);
        while (true);
    }

    radio.setDio0Action(onReceiveDone, RISING);
    radio.startReceive();

    oledSetCursor(2, 0); oledWriteStr("Aguardando pacotes");
    Serial.println("[OK] Receptor LoRa iniciado");
}

static unsigned long lastUpdate = 0;

void loop() {
    // Recebe pacote LoRa
    if (rxFlag) {
        rxFlag = false;
        uint8_t packet[PACKET_SIZE];
        int state = radio.readData(packet, PACKET_SIZE);

        if (state == RADIOLIB_ERR_NONE && radio.getPacketLength() == PACKET_SIZE) {
            g_timestamp = readU32(packet, OFF_TIMESTAMP);
            g_lat       = readF32(packet, OFF_LAT);
            g_lng       = readF32(packet, OFF_LNG);
            g_altitude  = readF32(packet, OFF_ALT);
            g_rssi      = radio.getRSSI();
            g_snr       = radio.getSNR();
            g_hasData   = true;

            Serial.println("===== Pacote recebido =====");
            Serial.printf("[DATA] T:    %lu ms\n",   (unsigned long)g_timestamp);
            Serial.printf("[DATA] Lat:  %.6f\n",     (double)g_lat);
            Serial.printf("[DATA] Lon:  %.6f\n",     (double)g_lng);
            Serial.printf("[DATA] Alt:  %.1f m\n",   (double)g_altitude);
            Serial.printf("[RF]   RSSI: %.1f dBm\n", (double)g_rssi);
            Serial.printf("[RF]   SNR:  %.1f dB\n",  (double)g_snr);
            Serial.println("---------------------------");

        } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
            Serial.printf("[AVISO] CRC erro. RSSI: %.1f dBm, SNR: %.1f dB\n",
                          (double)radio.getRSSI(), (double)radio.getSNR());
        } else {
            Serial.printf("[ERRO] readData codigo: %d\n", state);
        }

        radio.startReceive(); // re-arma sempre
    }

    // Atualiza display a cada 5 segundos
    unsigned long now = millis();
    if (now - lastUpdate >= 5000UL) {
        lastUpdate = now;
        updateDisplay();
    }
}