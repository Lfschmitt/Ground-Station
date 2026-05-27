# Rocket Ground Station

Estação receptora de telemetria para foguete de 1 km.

O **ESP32** (Heltec LoRa32 v2) recebe pacotes LoRa do flight computer e os encaminha via USB Serial para o PC. O **programa Python** no PC lê esses dados e exibe um monitor em tempo real com gráficos.

---

## Arquitetura

```
Flight Computer
     │  LoRa 915 MHz
     ▼
  ESP32 (Heltec LoRa32 v2)
  ├── Recebe pacote LoRa
  ├── Exibe status no OLED
  └── Envia dados pelo Serial (USB)
          │  USB
          ▼
    PC — Python
    ├── Lê porta serial
    └── Monitor em tempo real (PyQt6 + matplotlib)
```

---

## Hardware

| Componente | Modelo |
|---|---|
| Microcontrolador | ESP32 (Heltec LoRa32 v2) |
| Rádio | SX1276 — 915 MHz, BW 500 kHz, SF 9, CR 4/5 |
| Display | SSD1306 OLED 128×64 (I2C) — status de campo |

---

## Estrutura do projeto

```
Ground Station/
├── esp32/
│   └── ground_station/
│       ├── ground_station.ino   ← firmware: LoRa → OLED + Serial
│       ├── config.h             ← pinos e parâmetros RF
│       └── telemetry.h          ← struct TelemetryPacket
│
├── python/
│   ├── main.py                  ← entry point
│   ├── serial_reader.py         ← lê e parseia dados do Serial
│   ├── gui.py                   ← janela PyQt6 + gráficos matplotlib
│   └── requirements.txt
│
└── README.md
```

---

## Pacote de telemetria

O ESP32 envia uma linha de texto por pacote recebido via Serial (`115200 baud`):

```
[DATA] T:1234 Lat:-23.550520 Lon:-46.633308 Alt:1024.3 RSSI:-72.0 SNR:8.2
```

O `serial_reader.py` parseia essa linha e entrega um dicionário com os campos para a GUI.

### Struct no ESP32 (`telemetry.h`)

| Campo | Tipo | Offset |
|---|---|---|
| `timestamp` | uint32 | 0 |
| `lat` | float32 | 4 |
| `lng` | float32 | 8 |
| `altitude` | float32 | 12 |
| `rssi` | float | — (RadioLib) |
| `snr` | float | — (RadioLib) |

---

## Setup Python

```bash
cd python
pip install -r requirements.txt
python main.py
```

Dependências: `pyserial`, `PyQt6`, `matplotlib`

---

## Setup ESP32

1. Abra `esp32/ground_station/ground_station.ino` no Arduino IDE
2. Selecione a placa **Heltec WiFi LoRa 32(V2)**
3. Faça o upload
4. Conecte via USB ao PC — o ESP32 aparece como porta `COMx`

Dependência: [RadioLib](https://github.com/jgromes/RadioLib)

---

## Fluxo de dados

```
LoRa ISR → readData() → TelemetryPacket
                              ├── OLED (status de campo)
                              └── Serial.printf() → USB → Python
```
