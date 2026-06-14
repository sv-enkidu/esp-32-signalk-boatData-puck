# BNO08x IMU — Why This Sensor for Marine Use

## Overview

This project uses the **BNO085 (BNO08x series)** IMU rather than a simpler standalone compass or gyro. The short answer for why: no other readily available breakout board performs as many jobs simultaneously — tilt-compensated heading, pitch, roll, yaw, and active self-calibration — all in firmware, on the chip itself, before your code ever sees the data.

---

## What Makes It Different: Sensor Fusion

The BNO08x contains three separate physical sensors internally:

- **Accelerometer** — establishes a gravity vector, which is what allows pitch and roll to be computed accurately regardless of how the boat is oriented
- **Gyroscope** — measures fast rotational changes across all three axes, providing the high-frequency responsiveness that a compass alone cannot
- **Magnetometer** — provides a stable absolute reference to Magnetic North, used as a long-term correction anchor

The chip's onboard processor blends all three continuously using a **Kalman filter**, which is the same class of algorithm used in aircraft inertial navigation systems. The filter is always running equations — even when the sensor appears to be sitting still — micro-correcting the drifting gyro data against the stable magnetometer baseline. Because the code requests updates at up to 100Hz, you can observe this live in Signal K as the tiny, continuous self-corrections the filter makes against sensor noise. This is normal and expected behavior; it means the board is doing its job.

### Why This Matters on a Boat

A boat environment is particularly hostile to magnetometers. Engine blocks, alternators, wiring runs, and even the keel itself create local magnetic field distortions that cause conventional compass modules to read incorrectly. The BNO085 addresses this in a way that's uniquely well-suited to the marine environment: it **requires regular dynamic motion** to continuously map and calibrate away these local fields. A slow 360° turn or a figure-eight at the start of a passage is all it takes to give the chip enough data to characterize and subtract the local interference. After that, it continues refining its calibration model passively as the boat moves normally.

---

## Report Type Selection

The current code uses `SH2_ROTATION_VECTOR`. The table below explains what the alternatives offer and why this one was chosen:

| Report Type | Sensors Blended | Characteristics |
|---|---|---|
| **`SH2_ROTATION_VECTOR`** *(current)* | Mag + Gyro + Accel | Tilt-compensated, dynamically referenced to Magnetic North. Highly responsive. Best choice for boat heading. |
| `SH2_GAME_ROTATION_VECTOR` | Gyro + Accel only | Extremely smooth and immune to magnetic distortion, but heading is **relative** — it starts at a random value on boot because it ignores the magnetometer entirely. Designed for VR gaming. |
| `SH2_GEOMAGNETIC_ROTATION_VECTOR` | Mag + Accel only | Traditional tilt-compensated compass — stable over time, but **lacks the gyro**, so it appears noisy or laggy in heavy seas. |
| `SH2_MAGNETIC_FIELD_CALIBRATED` | Mag only | Raw 3-axis magnetic values. Equivalent to what the external QMC5883L provides — fully uncompensated for tilt or motion. |

---

## Understanding the Continuous Micro-Corrections in Signal K

You will notice the BNO085 heading value in Signal K is never completely static, even at anchor. This is not noise — it is the Kalman filter doing its job. Here is what each sensor contributes and what it costs:

- **The Gyroscope** is very fast and responsive, but suffers from *gyro drift*: even when completely still, it accumulates tiny fractional errors that make it think it is rotating slightly. Left uncorrected, it would slowly wander away from true heading.
- **The Magnetometer** is a stable absolute reference, but it is inherently noisy. Ambient electromagnetic radiation from nearby wiring, Earth's fluctuating field, and even cellular signals introduce constant background static.

The Kalman filter's job is to exploit the strengths of each sensor to cancel the weaknesses of the other. The result is a heading that is both fast-responding *and* anchored to Magnetic North — which neither sensor could achieve alone.

---

## Dual-Compass Configuration

When used alongside the onboard **QMC5883L** magnetometer (as on the Sequre M10-25Q GPS module), the code outputs both compass sources as distinct NMEA sentences:

- `$HCHDG` — from the QMC5883L (`HC` = Heading Compass)
- `$INHDG` or `$IIHDG` — from the BNO085 (`IN` = Integrated Navigation, `II` = Instrumentation)

Signal K sees these as two independent `navigation.headingMagnetic` keys from different sources. This lets you designate one as primary while retaining the other for cross-validation or backup — and it creates a solid baseline for comparing and validating compass accuracy before you leave the dock.

---

## Signal K Configuration

Configure XDR paths in Signal K as follows:

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

---

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

---

## Hardware Interface Options (PS0 / PS1 Pins)

The BNO08x is unusual in that it can communicate over several completely different physical interfaces. The **PS0** and **PS1** pins (Protocol Selection) tell the chip which interface to use at startup. The current code uses the default I2C mode (both pins LOW / unconnected).

| PS1 | PS0 | Interface | Notes |
|---|---|---|---|
| LOW | LOW | **I2C** *(default)* | Standard I2C using SHTP protocol |
| LOW | HIGH | UART-RVC | Simplified streaming serial — originally designed for robotic vacuum cleaners. Outputs Yaw/Pitch/Roll + XYZ acceleration at 115200 baud, 100Hz. No configuration needed. |
| HIGH | LOW | UART-SHTP | Full-featured serial using the SHTP packet protocol |
| HIGH | HIGH | SPI | High-speed SPI using SHTP protocol |

### Mode A: I2C / SPI / Full UART (SHTP)
In these modes the chip uses the **SHTP (Sensor Hub Transport Protocol)** — a bidirectional, packet-based system. Your host must actively send a configuration packet requesting specific *reports*. This is what allows the code to request `SH2_ROTATION_VECTOR` specifically, at a defined update rate, while ignoring other report types. The processed output is a 4-point **quaternion** (which prevents gimbal lock), which the code then converts to Euler angles (Pitch, Roll, Yaw).

### Mode B: UART-RVC ("Easy Mode")
With PS1=LOW and PS0=HIGH, the chip ignores I2C entirely and simply streams a fixed binary packet out of the TX pin at 100Hz — no configuration required. The packet already contains Yaw, Pitch, and Roll in degrees, plus XYZ acceleration. Useful for simple integrations that don't need the full SHTP feature set.

---

## Wiring

- **VCC**: 3.3V
- **GND**: Ground
- **SDA / SCL**: Shared I2C bus with other sensors (see code for GPIO pin assignments)
- **RST**: ESP32 GPIO (defined as `BNO_RST` in code)
- **INT**: ESP32 GPIO (defined as `BNO_INT` in code)
- PS0 and PS1 are left unconnected (pulled LOW internally = I2C default mode)

See the main sketch for exact GPIO assignments.
