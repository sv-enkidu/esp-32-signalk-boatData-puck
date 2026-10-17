
# ESP32 SignalK Boat Sensor Puck

PLEASE NOTE THAT I AM NO LONGER USING I2C FOR THE IMU (BNO08x) FOR THIS PROJECT, instead, I am using UART, which is super stable, but I have left the I2C code in an earlier version in the repo, if anyone wants to attempt to connect the IMU this way (I strongly recommend that you to not do so).  Please see BNO08x readme for some details and context on that board.  Also, I have replaced the BME680 with a BME280 (which loses VOC sensing but gains humidity sensing), which is in the latest posted code.  

## What is this project?

A compact multi-sensor boating instrument that combines GPS, dual magnetometer (two compasses), IMU, and environmental sensing (temp/barometric pressure, and VCO) into a single ESP32-based unit. I have all of these sensors mounted in an enclosure about the size of a hocky puck which will be mounted above the transomway under a dodger.  All data is converted to json or NMEA0183 (version specific) over a single serial (USB) cable to the [Signal K](https://signalk.org/) server.  

Said another way, for about $100 you can have a marine grade GPS, IMU and environmental sensing in a form factor small enough to mount pretty much anywhere.    

See detailed features and hardware overview below.

See README_BNO08x for details related to IMU sensor. This is an excellent boating IMU, to my mind, perhaps the best off-the-shelf sensor available for this purpose. The board dynamically compensates between axis' of motion for all vectors sent to signalk, and the code is written so that both compass' inherit dynamic and real-time tilt compensation from the BNO.  Although I have included user-settable offsets in the code for roll (heel), pitch, yaw and compass alignment, most of the difficult and time consuming environmental compensation usually required with an IMU, or for tilt-compensation on the compass, is provided using native features of that sensor.  See BNO readme for details.   

Below are a couple examples of the puck hardware in various enclosures (enclosure stl file in the repo, in theory this can be resized for your hardware, but first seperatee the USB connected housing so that it maintains its proportions to fit the dimensions of the connector).  I am using the ESP32's I had handy, which are quite large, with a smaller ESP the overall size could be reduced significantly.  I also built a water resistant (it will leak if fully submerged) hole on the underside of the enclosure mounted near the BME280 to allow the temp/humidity sensor access to ambient air/pressure.

https://github.com/sv-enkidu/esp-32-signalk-boatData-puck/blob/main/IMG_5853.jpeg

## How to set up

After wiring (see below and optional BNO readme), this will need to be setup in Signalk.  The process is fairly straightforward and once set up, it should function very robustly and dependably.  

1. Download NMEA0183 XDR Sentence Parser plugin from the Signalk app store
2. Set up new data connection for the puck (a one time step when setting up for the first time) in Signalk main menus (data -> connection).  Name connection anything you want, specify 115200 baud rate, select the proper serial port and enable checksum.
3. Configure NMEA connections in the XDR parser plugin (Signalk main -> Apps and plugins -> configuration - NMEA0183 XDR Sentence Parser) and add the following:

### 1. Pitch
| Field | Value |
|---|---|
| Description | IMU Pitch |
| Category | Angle (or Navigation) |
| Units | deg |
| XDR Sensor ID | `PITCH` |
| Math expression | `x` |
| Fixed decimal | 1 |
| Signal K Path | `navigation.attitude.pitch` |

### 2. Roll / Heel
| Field | Value |
|---|---|
| Description | IMU Roll |
| Category | Angle (or Navigation) |
| Units | deg |
| XDR Sensor ID | `ROLL` |
| Math expression | `x` |
| Fixed decimal | 1 |
| Signal K Path | `navigation.attitude.roll` |

### 3. Yaw (Secondary Heading)
| Field | Value |
|---|---|
| Description | IMU Yaw |
| Category | Angle (or Navigation) |
| Units | deg |
| XDR Sensor ID | `YAW` |
| Math expression | `x` |
| Fixed decimal | 1 |
| Signal K Path | `navigation.attitude.yaw` |

### 4. Temperature
| Field | Value |
|---|---|
| Description | Air Temperature |
| Category | Angle (or Navigation) |
| Units | C |
| XDR Sensor ID | `TEMP` |
| Math expression | `x+273.15` |
| Fixed decimal | 2 |
| Signal K Path | `navigation.outside.temperature` |
---

### 5. Barometric Pressure
| Field | Value |
|---|---|
| Description | Humid |
| Category | pressure |
| Units | bar |
| XDR Sensor ID | `BARO` |
| Math expression | `x*100000` |
| Fixed decimal | 0 |
| Signal K Path | `environment.outside.humidity` |
---

### 4. (VOC - optiona, if using BME680)
| Field | Value |
|---|---|
| Description | VOC Gas Detection |
| Category | Other |
| Units | kOhm |
| XDR Sensor ID | `VOC` |
| Math expression | `x` |
| Fixed decimal | 1 |
| Signal K Path | `navigation.attitude.yaw` |

Note that the the various GPS paths and sources are default in Signalk and in ublox M10 GPS's, no connection must be set up in the NMEA sentence parser plugin (you should see about a dozen or so new GPS related paths/coursed once deployed). See browser view image beiow for default examples.

---

### Signalk Paths/Sources Browser view

Below is a screenshot of the paths and sources added to Signalk (actual GPS coordinates also added but offscreen).  Note if Signalk is configured to use GPS data and time, this path will also be included (as it is here). 

<img width="887" height="1027" alt="puck_signalk_pathView" src="https://github.com/user-attachments/assets/43fe2f24-91e0-47bb-a5c7-fada0820ef96" />

Note that the the various GPS paths and sources are default in Signalk and in ublox M10 GPS's, no connection must be set up in the NMEA sentence parser plugin (you should see about a dozen or so new GPS related paths/coursed once deployed).  These include GPS derived data/time, and navigation.magneticVariationAgeOfService, for example, as well as the various views of GPS system in use and number of sats visible/locked.

## Hardware and misc. feature details

## Dual-Compass Configuration, with vertical/tilt axis compensation on  both

When used alongside the onboard **QMC5883L** magnetometer (as on the Sequre M10-25Q GPS module), the code outputs both compass sources as distinct NMEA sentences:

- `$HCHDG` — from the QMC5883L (`HC` = Heading Compass)
- `$INHDG` or `$IIHDG` — from the BNO085 (`IN` = Integrated Navigation, `II` = Instrumentation)

Signal K sees these as two independent `navigation.headingMagnetic` keys from different sources. This lets you designate one as primary while retaining the other for cross-validation or backup — and it creates a solid baseline for comparing and validating compass accuracy before you leave the dock.

Since the QMC5883 compass is a three axis magnometer, but lacks any on-chip or firmware tilt compensation (ie., ordinarily adding tilt-compensation is quite a complex and variable process), I wrote the code so that the QMC compass uses the algo processed tilt-compensation dynamically from the BNO (free, highly accurate tilt-compensation)!  Note a bad bonus.

| Component | Role | I2C Address |
|---|---|---|
| ESP32-S3 (Lonely Binary) | Microcontroller | — |
| Sequre M10-250 GPS | Position / GNSS | UART (pins 16/17) |
| QMC5883P (onboard GPS module) | Magnetic compass | 0x2C |
| BNO085 IMU | Roll, pitch, yaw | 0x4A or 0x4B |
| BME680 (or BME280 in version4v7) | Temp, pressure, VOC/humidity | 0x76 or 0x77 |

**Custom I2C pins:** SDA = GPIO8, SCL = GPIO9  
**BNO085:** INT = GPIO10, RST = GPIO11
- See enclosure comments below for wiring of external USB connector, as shown in the image above (the connector is reset in the enclosure to reduce excess exposed wiring).

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

Version 2v9 changelog:

Changes to NMEA0183 version: 
- Fixed other minor stability issues (the current version is very stable, I am running an autopilot using this IMU and it works exceptionally well).
- Added hardcoded offsets that can be applied (in degrees, conde converts to radians) to account for variations in the levelness of surfaces when mounting (eg., if when mounted, sensor baseline roll and pitch are -1.2 and -.8, resepectively, for example, positive 1.0 and positive 8 degree values can be added in the '// Modify these variables...' block to account for less than perfectly level surfaces (providing near-zero baseline values).  NOTE: these offsets do not replace the need for boat specific calibration and various declination adjustments (although there is no reason they could not be used as such, assuming you don't mind changing the firmware to set), all of which can be done in signalk with its own and other tools.  Rather, these offsets simply allow for minor adjustments to baseline the 'pucks' data values near zero.
- Modified YAW offset and YAW functionality (see below YAW notes)
- TO DO: add 2-way comms between puck and signalk/Node Red via MQTT (so that offsets can be set either dymanically using signalk path data and Node Red logic, or via a simply MQTT out node, for example).

### Enclosure

- 3D printable enclosure example added to repo.  Ideally print with ASA filament, or keep the puck out of intense, direct sunlight if less environmentally robust filament is used.
- The USB connector I used looks like this (bought on amazon).  If you use these, wire the white and blue to D+/D- on the ESP, and connect power to the ESP 5V pin and GND (many GPS's like the one I used - a Sequre M10-250Q with integrated QMC5883 compass - require 5v).  the green wire is not needed, so clip it or leave it unwired.

<img width="298" height="494" alt="USB_connector_image" src="https://github.com/user-attachments/assets/ed174e0c-3217-401e-ac34-f72efbff573b" />


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

## Baseline Calibration: Aligning the Two Compasses

The BNO085 and QMC5883L will almost always be physically mounted at a slightly different angle inside the enclosure. Use Signal K's **math expression field** to correct this offset so that both sensors track each other cleanly across the full 360° arc.

**Procedure:**
1. Place the enclosure flat on a stable bench before installation.
2. Rotate it until **Sensor A** reads exactly 0° (North) in Signal K.
3. Note what **Sensor B** reads. The difference is your offset.
4. In Sensor B's Signal K math field, enter: `value + (OFFSET_DEGREES * Math.PI / 180)`

**Example:** If Sensor B reads 358° when Sensor A reads 0°, the offset is 2°:
```
value + (2 * Math.PI / 180)
```

After this one-time step, both sensors will track each other precisely — and you haven't yet left the dock.

## Enhanced YAW and YAW caplibration
(to add)

## IMPORTANT:
- Don't forget to set up in Signalk, and release serial port from your IDE after flashing, if the board is flashed using the same computer as your signalk server.

## License

MIT — use freely, attribution appreciated.
