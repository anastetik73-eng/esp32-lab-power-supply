# ESP32 Laboratory Power Supply

An open-source ESP32-based laboratory power supply controller with Modbus RTU communication, TFT display, web interface, and OTA firmware updates.

**Author:** FW190
**License:** MIT
**Platform:** ESP32 (Arduino Framework, PlatformIO)

---

## Overview

This project turns an off-the-shelf XY6020L DC-DC converter module into a full-featured laboratory power supply with local display, rotary encoder control, physical RUN/STOP button, and remote monitoring via Wi-Fi.

The ESP32 acts as the central controller: it communicates with the XY6020L over **Modbus RTU** (UART), renders live data on a **TFT ST7789** display, provides a **web interface** for remote control, and supports **Over-The-Air (OTA)** firmware updates.

---

## Features

### Control & Measurement
- **XY6020L control** via Modbus RTU (UART, 115200 baud)
- Real-time measurement of **Voltage, Current, Power, Temperature**
- Setpoint adjustment for **Uset (Voltage)** and **Iset (Current limit)** via rotary encoder
- **CC / CV mode** indication
- Automatic calculation of **Capacity (Ah)**, **Energy (Wh)** and **runtime**
- **RUN / STOP** output control via physical button (GPIO 25) or encoder long-press

### User Interface
- **TFT ST7789** display (320×240, SPI) with 4 screens:
  - **P1 — Main:** V / A / W, setpoints, temperature, statistics
  - **P2 — Analytics:** load resistance, current load bar, IP, status
  - **P3 — Graph:** power history (30 points, 3 s interval)
  - **P4 — Clock / Info:** time, date, status, Wi-Fi
- Screen switching via **TTP223 touch button**
- Bright, color-coded UI designed for readability at a glance

### Connectivity
- **Wi-Fi web interface** (HTTP) for monitoring and control from any browser
- **REST-like JSON endpoint** for live data (`/data`)
- **Remote commands** for setpoints and output control (`/set`)
- **OTA firmware update** via ArduinoOTA
- **NTP time synchronization** (configurable timezone)

### Reliability
- **Hardware watchdog** (`esp_task_wdt`, 15 s timeout)
- **Interrupt-driven button handling** for reliable press detection
- **Debouncing** for encoder and external buttons
- **Modbus CRC validation** and link timeout detection (`LINK` / `ERR` indicator)

---

## Hardware

### Components
| Component | Description |
|---|---|
| **ESP32 DevKit** | Main controller (Arduino Framework) |
| **XY6020L** | 60 V / 20 A DC-DC converter with Modbus RTU interface |
| **ST7789 TFT** | 2.8" SPI display, 320×240 |
| **EC11 Encoder** | Rotary encoder with push button |
| **TTP223** | Capacitive touch sensor (screen switching) |
| **WEIPENG microswitch** | External RUN / STOP button |
| **Industrial PSU** | 500 W AC-DC input stage |

### Pinout

| Function | GPIO | Notes |
|---|---|---|
| TFT CS | 15 | SPI chip select |
| TFT DC | 2 | Data / Command |
| TFT RST | 4 | Reset |
| XY6020L RX (ESP32 TX) | 16 | UART2 RX |
| XY6020L TX (ESP32 RX) | 17 | UART2 TX |
| Encoder CLK | 27 | A channel |
| Encoder DT | 14 | B channel |
| Encoder SW | 26 | Push button (ISR) |
| Touch (TTP223) | 13 | Screen switch |
| RUN/STOP button | 25 | External, ISR (pull-up to GND) |

*Note: SPI uses default VSPI pins on ESP32 (SCK = 18, MOSI = 23).*

---

## Software Stack

### Built with
- **Arduino Framework** for ESP32
- **PlatformIO** (VS Code)
- **C++17**

### Libraries
- `Adafruit GFX` — graphics primitives
- `Adafruit ST7735/ST7789` — TFT driver
- `WiFi` — ESP32 Wi-Fi stack
- `WebServer` — HTTP server
- `ArduinoOTA` — Over-the-Air updates
- `esp_task_wdt` — hardware watchdog

### Custom modules (in `main.cpp`)
- **Modbus RTU** — full implementation with CRC16, request/response state machine
- **Display manager** — 4 screens with partial redraw
- **Encoder handler** — rotation with debounce + button ISR
- **Web UI** — HTML + JSON endpoint
- **Statistics** — Ah / Wh / runtime accumulation

---

## How It Works

1. **Modbus polling** — the ESP32 requests 20 registers from the XY6020L every 250 ms via UART2.
2. **Data parsing** — voltage, current, power, temperature, and CC/CV mode are extracted from the response.
3. **Screen update** — the active screen refreshes only when values change (partial redraw).
4. **User input**:
   - **Encoder rotation** → adjusts active setpoint (Uset or Iset)
   - **Encoder short press** → switches between Uset and Iset
   - **Encoder long press (≥ 600 ms)** → toggles RUN / STOP
   - **External button (GPIO 25)** → toggles RUN / STOP
   - **Touch button (TTP223)** → cycles through 4 screens
5. **Setpoint write** — deferred by 150 ms after last encoder movement, then sent to XY6020L via Modbus.
6. **Web & OTA** — served when Wi-Fi is connected; `ArduinoOTA.handle()` runs in the main loop.

---

## Screenshots

> **Note:** Photos will be added after the final hardware assembly.

**Main screen (P1):**
- Voltage, current, power
- Uset / Iset with active edit marker
- LINK status indicator
- RUN / STOP button
- Capacity, energy, runtime
- Temperature and Wi-Fi icon

*(Screenshots coming soon)*

---

## Building & Flashing

### Prerequisites
- **VS Code** with **PlatformIO** extension
- **USB drivers** for ESP32 (CP210x or CH340)
- **USB cable** (data-capable, not charge-only)

### Steps

1. **Clone the repository:**
   ```bash
   git clone https://github.com/anastatik73-eng/esp32-lab-power-supply.git
   cd esp32-lab-power-supply