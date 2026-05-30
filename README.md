# ESP32 SignalK Boat Sensor Puck

A compact multi-sensor boating instrument that combines GPS, magnetic compass, IMU, and environmental sensing into a single ESP32-based unit. Data is sent over serial to a [Signal K](https://signalk.org/) server.

## Hardware

| Component | Role | I2C Address |
|---|---|---|
| ESP32-S3 (Lonely Binary) | Microcontroller | — |
| Sequre M10-250 GPS | Position / GNSS | UART (pins 16/17) |
| QMC5883P (onboard GPS module) | Magnetic compass | 0x2C |
| BNO085 IMU | Roll, pitch, yaw | 0x4A or 0x4B |
| BME680 | Temp, pressure, VOC | 0x76 or 0x77 |

**Custom I2C pins:** SDA = GPIO8, SCL = GPIO9  
**BNO085:** INT = GPIO10, RST = GPIO11

## Output Rates

| Sensor | Rate |
|---|---|
| GPS (pass-through) | Native NMEA, 38400 baud |
| Compass (QMC5883P) | 10 Hz |
| IMU attitude (BNO085) | 10 Hz |
| Environmental (BME680) | 0.5 Hz (every 2 s) |

## Firmware Versions

### `/NMEA0183/` — NMEA0183 output (recommended for most Signal K setups)

Sends standard NMEA sentences that Signal K's built-in parser understands natively:

- `$HCHDG` — magnetic heading from QMC5883P
- `$IIXDR` — pitch, roll, and yaw from BNO085 (degrees)
- `$WIXDR` — pressure (bar), temperature (°C), VOC resistance (kΩ)
- Raw GPS NMEA sentences passed through directly

Connect to Signal K as a **Serial** connection provider, NMEA0183 type.

### `/JSON/` — Native Signal K JSON Delta output

Sends Signal K delta JSON objects directly over serial. Useful if you want to avoid NMEA translation or need radian-native values.

- `navigation.headingMagnetic` — from QMC5883P (radians)
- `navigation.attitude.roll` / `.pitch` / `.headingMagnetic` — from BNO085 (radians)
- `environment.outside.temperature` — Kelvin
- `environment.outside.pressure` — Pascals
- Raw GPS NMEA pass-through (Signal K handles mixed streams)

Connect to Signal K as a **Serial** connection provider, Signal K JSON type.

## Required Libraries

Install via Arduino Library Manager:

- `Adafruit BNO08x` (BNO085 IMU)
- `Adafruit BME680`
- `Adafruit QMC5883P`
- `Adafruit Unified Sensor`

## Signal K Connection

1. Open Signal K admin UI → **Server → Connections → Add**
2. Set type to **Serial**, select the correct port
3. NMEA0183 version: set data type to **NMEA0183**
4. JSON version: set data type to **Signal K JSON**
5. Baud rate: **115200**

> **Note:** Release the serial port from Arduino IDE before connecting Signal K, or data will not be received.

## License

MIT — use freely, attribution appreciated.
