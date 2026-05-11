# Remote Controlled Car

A two-wheel RC car built on the **ESP32** microcontroller, controlled in real time via **Classic Bluetooth (SPP)** from a browser-based web interface using the **Web Serial API**.

---

## Overview

```
┌─────────────────────┐        Bluetooth SPP         ┌──────────────────────┐
│   Web Controller    │ ◄──────────────────────────► │    ESP32 Firmware    │
│   (index.html)      │     Commands: F/B/L/R/S/1-9  │   Motor + BT Stack   │
└─────────────────────┘                              └──────────────────────┘
                                                              │
                                                    ┌─────────┴─────────┐
                                                    │   L298N / Motor   │
                                                    │  Driver + 2x DC   │
                                                    └───────────────────┘
```

---

## Features

- Real-time directional control — Forward, Backward, Left, Right
- 9-level speed control mapped to PWM duty cycle
- Auto-stop on button release (hold-to-move behavior)
- Browser UI with D-pad, speed slider, telemetry panel, and real-time log
- Keyboard control via arrow keys
- Touch support for mobile browsers
- Thread-safe command handling with FreeRTOS mutex

---


## Hardware

| Component | Details |
|-----------|---------|
| MCU | ESP32 (any variant with Classic BT) |
| Motor driver | L298N or equivalent H-bridge |
| Motors | 2× DC gear motors |
| Power | 7–12V for motors, 3.3V/5V for ESP32 |

### Pin Configuration

| Signal | Left Motor | Right Motor |
|--------|-----------|-------------|
| ENA (PWM) | GPIO 5 | GPIO 15 |
| IN1 | GPIO 18 | GPIO 2 |
| IN2 | GPIO 19 | GPIO 4 |

> Pin assignments are defined in `app_main()` inside `main.c` and can be changed freely.

---

## Firmware

### Requirements

- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/) v5.x
- Classic Bluetooth enabled in `sdkconfig` (`CONFIG_BT_CLASSIC_ENABLED=y`)

### Build & Flash

```bash
idf.py set-target esp32
idf.py menuconfig      # Enable Classic BT if not already set
idf.py build
idf.py flash monitor
```

### Command Protocol

Single-character ASCII commands sent over SPP:

| Char | Action |
|------|--------|
| `F`  | Forward |
| `B`  | Backward |
| `L`  | Turn left |
| `R`  | Turn right |
| `S`  | Stop |
| `1`–`9` | Set speed level (maps to PWM 644–996) |

Speed mapping formula: `PWM = 600 + level × 44` (10-bit resolution, 1 kHz).

### Motor Driver (`motor.c`)

```c
void motor_init(Motor_Typedef *motor, int ena_pin, int in1_pin, int in2_pin,
                ledc_channel_t channel, ledc_timer_t timer);
void motor_forward (Motor_Typedef *motor, uint32_t speed);
void motor_backward(Motor_Typedef *motor, uint32_t speed);
void motor_stop    (Motor_Typedef *motor);
```

Uses the ESP-IDF **LEDC** peripheral for PWM generation and standard **GPIO** for direction control.

### FreeRTOS Task Design

```
app_main()
  ├── motor_init()
  ├── xSemaphoreCreateMutex()       ← protects bt_command & speed_level
  ├── Bluetooth SPP stack init
  └── xTaskCreate(motor_task)       ← runs every 20 ms
        └── reads bt_command + speed_level under mutex
            └── drives motors accordingly

spp_callback (BT stack context)
  └── ESP_SPP_DATA_IND_EVT
        └── updates bt_command or speed_level under mutex
```

---

## Web Controller

`index.html` — a single self-contained file, no dependencies, no build step.

### Requirements

- **Chrome** or **Edge** (Web Serial API required — Firefox and Safari not supported)
- ESP32 paired and connected via Bluetooth; visible as a COM/serial port

### Usage

1. Open `index.html` in Chrome or Edge.
2. Click **⚡ Connect** and select the ESP32 COM port (baud rate: **115200**).
3. Use the D-pad or keyboard arrow keys to drive the car.
4. Adjust the speed slider (levels 1–9) at any time.

### Keyboard Shortcuts

| Key | Action |
|-----|--------|
| `↑` | Forward |
| `↓` | Backward |
| `←` | Left |
| `→` | Right |
| *(release)* | Auto-stop |

### Telemetry Panel

The UI shows:
- **CMD** — last direction command sent
- **PWM** — computed PWM value for current speed level
- **SENT** — total number of commands sent this session
- **BAUD** — fixed at 115200

---

## Known Limitations

- Classic Bluetooth only — BLE is not supported (BLE memory is released at startup)
- Web Serial API requires a Chromium-based browser
- One client connection at a time (SPP slave mode)

## Contribution
Contributions are welcome! Please feel free to submit issues or pull requests.
