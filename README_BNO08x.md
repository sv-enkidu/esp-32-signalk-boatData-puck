# BNO08x IMU — Why This Sensor for Marine Use

## Overview

This project uses the **BNO085 (BNO08x series)** IMU rather than other recommended IMU's commonly used with signalk, or a standalone compass/gyro. The short answer for why I am using this IMU is that it works great, and it can us used without pypilot for basic IMU (ie., not-autopilot related) calibration for roll, pitch and yaw.  The BNO is unique in that it has on-board firmware that runs the math to calculate the various interrelated values across its 9-axis', so no math is necessary for very accurate data for those values off-board or in firmware (the breakout board performs many jobs simultaneously — tilt-compensated heading, pitch, roll, yaw, and active self-calibration — all in firmware, on the chip itself, before your code ever sees the data.  It is this algorythmically processed data that the code outputs (options for unprocessed data modes exist, which I am not using as it works great as is, leveraging its native capibilities).  Details below.

Note: The downside is that this board has known I2C timing issues (it can violate I2C timing requirements in certain situations). This is not a problem with UART, which is very stable/robust (so much so that I removed the I2C watchguard functionality in the UART version of the code). You should be able to 'set it and forget it', plus I use the BNO08x tilt adjustment for the second compass such that both compasses are fully tilt-adjusted dynamically.

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

The current code uses `SH2_ROTATION_VECTOR` (the dynamically adjusted outputs). The table below explains what the alternatives offer and why this one was chosen:

| Report Type | Sensors Blended | Characteristics |
|---|---|---|
| **`SH2_ROTATION_VECTOR`** *(current)* | Mag + Gyro + Accel | Tilt-compensated, dynamically referenced to Magnetic North. Highly responsive. Best choice for boat heading. |
| `SH2_GAME_ROTATION_VECTOR` | Gyro + Accel only | Extremely smooth and immune to magnetic distortion, but heading is **relative** — it starts at a random value on boot because it ignores the magnetometer entirely. Designed for VR gaming. |
| `SH2_GEOMAGNETIC_ROTATION_VECTOR` | Mag + Accel only | Traditional tilt-compensated compass — stable over time, but **lacks the gyro**, so it appears noisy or laggy in heavy seas. |
| `SH2_MAGNETIC_FIELD_CALIBRATED` | Mag only | Raw 3-axis magnetic values. Equivalent to what the external QMC5883L provides — fully uncompensated for tilt or motion. |

---

## Understanding the Continuous Micro-Corrections in Signal K

You will notice the BNO085 (and often QMC compass) heading value in Signal K is never completely static, even at anchor. This is not noise — it is the Kalman filter doing its job. Here is what each sensor contributes and what it costs:

- **The Gyroscope** is very fast and responsive, but suffers from *gyro drift*: even when completely still, it accumulates tiny fractional errors that make it think it is rotating slightly. Left uncorrected, it would slowly wander away from true heading.
- **The Magnetometer** is a stable absolute reference, but it is inherently noisy. Ambient electromagnetic radiation from nearby wiring, Earth's fluctuating field, and even cellular signals introduce constant background static.

The Kalman filter's job is to exploit the strengths of each sensor to cancel the weaknesses of the other. The result is a heading that is both fast-responding *and* anchored to Magnetic North — which neither sensor could achieve alone.

---


## Hardware Interface Options (PS0 / PS1 Pins)

The BNO08x is unusual in that it can communicate over several completely different physical interfaces. The **PS0** and **PS1** pins (Protocol Selection) tell the chip which interface to use at startup. The current code uses UART, which is strongly recommended.  It turns out, this board has known timing issues such that its I2C implementation violates the standards in ways that make it unreliable.  I initially deployed using I2C and despite a great deal of effort to make stable (including the addition of watch-guard code to catch failures and reset the board, this sensor just does not work dependably with I2C.  I migrated the code to UART and it works great, with no need for the watch-guard code, however in some versions I have left it in the code commented out for others who may want to try to make I2C work.

## AN IMPORTANT NOTE ON UART WIRING with generic Chinese GY-BNO08x boards, or other boards where UART does not appear to work off-the-shelf

IMPORTANT:  Before reading below methods on connecting to ESP32, read this paragraph, it may save you a lot of the pain I went through getting UART to work with cheaper boards.    

The board comes in various form-factors, including versions by Sparkfun, Adafruit, and by various Chinese manufacturers and sellers (on Amazon, for example).   I used both the Sparkfun version (which works great and is well designed) and a cheap version like the one used in the UART example here:[   ](https://wolles-elektronikkiste.de/en/bno08x-9-dof-imus).  The code should work great with any version, so long as the PS1 jumper is closed (put solder blob across the PS1 jumper, and leave PS2 open), but in many cases, even with the jumper bridged, UART will work very poorly, or not at all, unless you wire PS1 high (to 3.3v, and optionally PS0 to GND.  UART is pulled high and although in theory, bridging that jumper should pull UART pins high, the signal is just not strong enough without hard wiring PS1 to VCC (see the UART examples in that URL).

I am using full-on UART (not the UART-RVC mode), since RVC (as I understand it), bypasses at least some of the on-board/in-firmware processing that is so central to the power of the sensor. 

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

(See the URL above, but here is a summary):
- **VCC**: 3.3V
- **GND**: Ground
- **SDA / SCL**: Shared I2C bus with other sensors (see code for GPIO pin assignments)
- **RST**: ESP32 GPIO (defined as `BNO_RST` in code) 
- **INT**: ESP32 GPIO (defined as `BNO_INT` in code)
- **PS0**: Unconnected, or wire to ESP GND
- **PS1**: Unconnected on the Sparkfun board, CONNECT TO 3.3V on the "GY-..." versions or if you have problems with UART
