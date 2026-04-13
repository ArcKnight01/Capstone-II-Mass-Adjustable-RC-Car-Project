

# SENSOR TECHNICAL SPECIFICATIONS DOCUMENT

This document includes information on all sensors and electronics used in this repository for the purpose of creating the sensor model. This information is summarized from the sensor datasheets and relevant documentation.

## RC CAR NAVIGATION SYSTEM



## 1. SYSTEM OVERVIEW

**Sensors:**
- GNSS: Adafruit Ultimate GPS Hat (FGPMMOPA6H / MTK3339)
- IMU: Bosch BNO055 (9-DOF)
- Control "Steering Angle" Input: RC Receiver (PWM via Arduino Nano)

### Project Frame Conventions

Body-frame axes used by the car:
- `+X`: forward
- `+Y`: left / port side
- `+Z`: up

Current steering sign convention:
- not yet fully confirmed in the project logs
- treat steering sign as a configurable mapping in animation and sensor-fusion code

Reference options to support during development:
- `negative-left`: negative steering angle = left turn, positive = right turn
- `positive-left`: positive steering angle = left turn, negative = right turn

---

# PART I — ORIGINAL COLLECTED DATA (RAW + DERIVED)

---

## 2. GPS MODULE

### General Specifications

| Parameter | Value |
|----------|------|
| Chipset | MTK3339 / PA1616D |
| Channels | 66 tracking, 210 PRN |
| Sensitivity (tracking) | -165 dBm |
| Sensitivity (acquire) | -148 dBm |
| Update Rate | 1 Hz (default), up to 10 Hz |
| Interface | UART (NMEA) |
| Power Consumption | ~20 mA |

### Position Performance

| Parameter | Value |
|----------|------|
| Position Accuracy | ~3.0 m (CEP) |
| SBAS Accuracy | ~2.5 m |
| Velocity Accuracy | ~0.1 m/s |
| PPS Timing Accuracy | ~10 ns |

### Derived Noise Model *(EKF Assumption)*

| Parameter | Value |
|----------|------|
| Position Std Dev (σ) | ~2.5 m |
| Velocity Std Dev (σ) | ~0.05 m/s |
| Update Interval | 0.1 – 1.0 s |

### NMEA Output

| Sentence | Description |
|----------|------------|
| GGA | Position, altitude, fix |
| RMC | Position, velocity, time |
| VTG | Ground speed |
| GSA/GSV | Satellite quality |

---

## 3. IMU (Bosch BNO055)

### General Specifications

| Parameter | Value |
|----------|------|
| Sensor Type | 9-DOF (Accel + Gyro + Mag) |
| Fusion | Onboard (Cortex-M0+) |
| Interface | I2C / UART |
| Output Types | Euler, Quaternion, Vectors |

### Output Data Rates

| Signal | Frequency |
|--------|----------|
| Orientation | 100 Hz |
| Gyroscope | 100 Hz |
| Linear Acceleration | 100 Hz |
| Magnetometer | 20 Hz |
| Temperature | 1 Hz |

### Accelerometer

| Parameter | Value |
|----------|------|
| Range | ±2g / ±4g / ±8g / ±16g |
| Resolution | 14-bit |
| Noise Density | ~150 µg/√Hz |
| Offset | ±80 mg |
| Temp Drift | ±3.5 mg/K |

**Derived RMS Noise (EKF):**  
σ ≈ 0.012 m/s²

### Gyroscope

| Parameter | Value |
|----------|------|
| Range | ±125 to ±2000 °/s |
| Noise Density | ~0.014 °/s/√Hz |
| Bias | ±1 °/s |
| Temp Drift | ±0.03 °/s/K |

**Derived RMS Noise (EKF):**  
σ ≈ 0.0014 rad/s

### Magnetometer

| Parameter | Value |
|----------|------|
| Range | ±1300 µT (xy), ±2500 µT (z) |
| Resolution | ~0.3 µT |
| Noise | 0.3–0.6 µT RMS |
| Heading Accuracy | ±2.5° |

### Orientation (Fused Output)

| Parameter | Value |
|----------|------|
| Output Type | Quaternion / Euler |
| Rate | 100 Hz |
| Approx Noise (σ) *(assumed)* | ~1.5° (0.026 rad) |

---

## 4. RC RECEIVER (PWM INPUT)

### Signal Characteristics

| Parameter | Value |
|----------|------|
| Frequency | ~50 Hz |
| Period | ~20 ms |
| Pulse Width Range | 1–2 ms (typical) |

### Project Measured Values

| Parameter | Value |
|----------|------|
| Measured Range | 942 – 1835 µs |
| Mapped Angle | -60° to +60° |

Current project interpretation of the mapped steering angle is still being
verified. For now, support is given both of these possibilities in code (configurable):
- Option A: `-60 deg` full left, `0 deg` centered, `+60 deg` full right
- Option B: `+60 deg` full left, `0 deg` centered, `-60 deg` full right

### Voltage

| Parameter | Value |
|----------|------|
| Receiver Voltage | ~6 V |
| Receiver PWM Voltage | ~3.3V |

### Noise Model *(assumed)*

| Parameter | Value |
|----------|------|
| Timer Resolution | ~4 µs |
| Jitter | ~1–3 µs |
| Angle Noise (σ) | ~0.5° |

---

## 5. SENSOR FUSION CHARACTERISTICS

### Update Rates

| Sensor | Rate |
|--------|------|
| IMU | 100 Hz |
| GPS | 1–10 Hz |
| RC Input | 50 Hz |

### Uncertainties

| Sensor | Dominant Error |
|--------|----------------|
| GPS | Multipath, latency |
| IMU | Bias drift, temperature |
| RC | PWM jitter, mapping error |

---

## 6. EKF PARAMETERS (ENGINEERING ASSUMPTIONS)

| Signal | Std Dev (σ) |
|--------|-------------|
| GPS Position | 2.5 m |
| GPS Velocity | 0.05 m/s |
| Acceleration | 0.012 m/s² |
| Gyroscope | 0.0014 rad/s |
| Orientation | 0.026 rad |
| Steering Angle | 0.0087 rad |

### EKF Convention Notes

Recommended starting convention for the filter:
- Use a local 2D world frame with `x = East`, `y = North`
- Keep the vehicle body frame as `+X forward`, `+Y left`, `+Z up`
- Convert steering measurements through a configurable sign mapping before using
  any steering geometry or bicycle-model relationship

---

# PART II — CLEANED DATASET (DATASHEET-CONSISTENT)

---

## GPS (Corrected Datasheet View)

| Parameter | Value |
|----------|------|
| Chipset | MT3339 |
| Channels | 210 PRN, 66 search, 22 tracking |
| Sensitivity | -148 dBm (acq), -165 dBm (track) |
| Update Rate | 1–10 Hz |
| Power Consumption | 25 mA (acq), 20 mA (tracking) |

---

## BNO055 (Datasheet Summary)

| Parameter | Value |
|----------|------|
| Outputs | Euler, Quaternion, Linear Accel, Gravity |
| Interface | I2C |
| Accel Range | ±2/4/8/16 g |
| Gyro Range | ±125–2000 °/s |
| Mag Range | ±1300 / ±2500 µT |
| Output Rate | up to 100 Hz |

---

## RC SYSTEM (Project-Specific)

| Parameter | Value |
|----------|------|
| PWM Rate | ~50 Hz |
| Pulse Range | 942–1835 µs |
| Voltage | ~3.3V |

---

# END OF DOCUMENT
