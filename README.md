# Web Based OBD2 CAN Sniffer

A real-time ESP32-S3 CAN bus logger with a browser-based live terminal, CSV export, PSRAM ring buffer, and native USB SLCAN output.

## Features

- Live web terminal over WiFi AP at `192.168.4.1`
- Real-time CAN frame polling with FPS, total frames, drops, and buffer usage
- 300,000-frame PSRAM ring buffer with microsecond timestamps
- CSV download of the full captured buffer
- CAN-ID filter in the browser UI
- Listen-only TWAI mode — no transmission on the bus
- Native USB CDC SLCAN output compatible with SavvyCAN, Wireshark (SocketCAN), and any SLCAN tool
- Dual-core FreeRTOS task split: CAN capture on Core 1 (priority 10), web server on Core 0 (priority 5)
- Atomic cross-core counters using GCC `__atomic` builtins — no mutex on the hot path

## Hardware

- ESP32-S3 N16R8 (8 MB OPI PSRAM, 16 MB Flash, Native USB-OTG)
- SN65HVD230 CAN transceiver
- OBD2 connector or direct CAN-H / CAN-L connection

## Default Pinout

```cpp
#define TX_PIN   GPIO_NUM_16
#define RX_PIN   GPIO_NUM_17
```

> In listen-only mode the TX pin is not driven, but the TWAI driver still requires it to be defined.

## WiFi Settings

```cpp
#define WIFI_SSID  "CAN-Logger"
#define WIFI_PASS  "12345678"
```

Connect your phone or laptop to this access point, then open `http://192.168.4.1` in a browser.

## Customization Options

### WiFi AP name and password
```cpp
#define WIFI_SSID  "MyCarSniffer"
#define WIFI_PASS  "mypassword"
```

### CAN transceiver GPIO pins
```cpp
#define TX_PIN   GPIO_NUM_16   // change to your TX pin
#define RX_PIN   GPIO_NUM_17   // change to your RX pin
```

### CAN bus speed
Edit this line inside `setup()`:
```cpp
twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
// Other options: 250KBITS, 125KBITS, 1MBITS
```

### Ring buffer size
```cpp
#define MAX_FRAMES   300000UL   // reduce if PSRAM is smaller
#define API_BATCH    300        // frames returned per web API call
#define TWAI_Q_LEN   128        // TWAI hardware RX queue depth
```

### Native USB SLCAN
```cpp
#define USE_NATIVE_USB  1   // 1 = USB-OTG CDC, 0 = UART Serial
```

## Web UI Controls

| Control | Description |
|---|---|
| Pause / Resume | Freeze the live terminal display |
| Clear | Clear on-screen rows (buffer untouched) |
| CSV Download | Download full PSRAM buffer as `.csv` |
| Reset Buffer | Wipe PSRAM buffer and restart capture |
| Filter input | Show only frames matching a CAN ID (e.g. `7E8`) |

The stats bar shows **FPS**, **total frames captured**, **dropped frames**, and a **buffer fill % bar** that turns amber above 70% and red above 90%.

## HTTP API

| Endpoint | Method | Description |
|---|---|---|
| `/` | GET | Serves the full web terminal UI |
| `/api/frames?since=N` | GET | Returns up to 300 frames as JSON, cursor-based |
| `/api/stats` | GET | Returns FPS, total, drops, buffer fill % |
| `/api/reset` | POST | Clears ring buffer and resets counters |
| `/download` | GET | Streams full buffer as CSV attachment |

## SLCAN USB Output

When `USE_NATIVE_USB 1` is set, the device appears as a CDC serial port on the host PC. Frames are output in SLCAN format:
- Standard frame: `t<3-hex-id><dlc><data>\r`
- Extended frame: `T<8-hex-id><dlc><data>\r`

## Arduino IDE Settings

| Setting | Value |
|---|---|
| Board | ESP32S3 Dev Module |
| PSRAM | OPI PSRAM |
| USB Mode | USB-OTG (TinyUSB) |
| CPU Frequency | 240 MHz |

## Architecture

- **`canTask` (Core 1, priority 10):** Blocks on `twai_read_alerts(portMAX_DELAY)` — sleeps between frames. On RX it drains the TWAI queue in a burst, writes each frame to the PSRAM ring buffer using atomic stores, and simultaneously formats + sends the frame as SLCAN over USB CDC.
- **`webTask` (Core 0, priority 5):** Runs `WebServer.handleClient()` on the same core as the WiFi stack to avoid cross-core TCP/IP overhead.
- The `loop()` task is immediately deleted to reclaim its stack.

## License

MIT
