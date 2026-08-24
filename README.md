# ESP32 SignalK Boat Sensor Puck

PLEASE NOTE THAT I AM NO LONGER USING I2C FOR THE IMU (BNO08x) FOR THIS PROJECT, but I have left the I2C code and details below.  This is due to some very frustrating, known I2C timing issues with this chip that I was never able to get stable.  I have since switched to UART for that sensor, and I have also replaced the BME680 with a BME280 (which loses VOC sensing but gains humidity sensing).  Please use the IMU UART code, which will be the code updated moving forward.

## What is this project?

A compact multi-sensor boating instrument that combines GPS, dual magnetometer (two compasses), IMU, and environmental sensing (temp/barometric pressure, and VCO) into a single ESP32-based unit. I have all of these sensors mounted in an enclosure about the size of a hocky puck which will be mounted above the transomway under a dodger.  All data is converted to json or NMEA0183 (version specific) over a single serial (USB) cable to the [Signal K](https://signalk.org/) server.  

See README_BNO08x for details related to IMU sensor. Note: let the BNO08x run for a while in motion before setting any offsets.  The board dymanically compensates for a lot of motion and attempts to find its baseline zero point relative to that motion (like waving an iphone in figure 8 movements to set the compass).  This is by design and you may find no offsets are necessary (the code works fine with zero values, ie., no changes to offsets in the code).

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

NOTE on wiring: 
- All I2C connected sensors are wired on the same I2C bus (I am using Lonely Binary ESP32-S3, which is particular about which pins can be used for I2C, thus if using the same board, pins GPIO8 and GPIO9 work, others not so much.
- I have the BNO08x reset pins wired, as there is some flukiness in its I2C timers, which will cause the board to eventually freeze, requiring a reset.  The code impliments a watchdog monitor of the sensor and will reset robustly if this happens (new in 2v7).
- ublox M10 GPS requires 5v in many packages, whereas the other sensors are all 3.3v.  The board I am using do not require level shifting and the like when wiring both 5v and 3.3v on the same ESP.  Simply wire the GPS power to 5v (if 5v in your packaging), and the other sensors to shared 3.3v and GND rails.

## Output Rates

| Sensor | Rate |
|---|---|
| GPS (pass-through) | Native NMEA, 38400 baud |
| Compass (QMC5883P) | 10 Hz |
| IMU attitude (BNO085) | 10 Hz |
| Environmental (BME680) | 0.5 Hz (every 2 s) |

## Firmware Versions

Version 2v7 changelog:

Changes to NMEA0183 version: 
- Added more robust sensor keepalives/watchguards (due to known I2C instability of BNO)
- Fixed other minor stability issues
- Added hardcoded offsets that can be applied (in degrees, conde converts to radians) to account for variations in the levelness of surfaces when mounting (eg., if when mounted, sensor baseline roll and pitch are -1.2 and -.8, resepectively, for example, positive 1.0 and positive 8 degree values can be added in the '// Modify these variables...' block to account for less than perfectly level surfaces (providing near-zero baseline values).  NOTE: these offsets do not replace the need for boat specific calibration and various declination adjustments, all of which can be done in signalk with its own and other tools.  Rather, these offsets simply allow for minor adjustments to baseline the 'pucks' data values near zero.

Changes to json version:
- None yet, need to update


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

## IMPORTANT:
- Once installed, paths must be created in signalk for the new data.  Download NMEA0183 XDR Sentence Parser plugin from the signalk main screen, then configure paths for each data source.
- When upgrading, disable the GPS (or however this connection is named in your instance) connection in the SERVER -> DATA CONNECTIONS page in signalk, select the proper serial port in the IDE, flash, then release serial port (change to some other port) in the IDE, then re-enable the connection in signalk.
- FYI: ubox GPS's take some time to obtain an inital fix from a cold start - upwards of 10-15 minutes (see their datasheets for details), whilst it established and saves a map of the 'visible' sat constellations.  Once a fix is established, it seems this map is cached in non-volitile memory, as new initial fixes after a reboot happen very quickly (nearly instantaneously).

## License

MIT — use freely, attribution appreciated.
