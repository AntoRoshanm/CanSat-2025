# 🛰️ CanSat Mission Code – IN-SPACe India 2024–25

## Overview
This repository contains the Arduino-based mission code developed for **Dr.MGR-ACS SAT**, the CanSat designed for the **IN-SPACe CanSat India Student Competition 2024–25**, organized by **IN-SPACe**, **ISRO**, and the **Astronautical Society of India**.

The firmware runs on a **Teensy 4.1 microcontroller**, integrating various sensors for environmental data collection, telemetry, and automated recovery logic.

---

## ✨ Features
- Multi-sensor integration (BNO055, BMP280, DPS310, INA219, MQ-131, GPS)
- Dual barometric altitude tracking
- Real-time telemetry via XBee
- SD card data logging (mission + GPS logs)
- Automatic state transitions based on altitude and motion
- Servo-based parachute deployment system
- Recovery beeper for post-landing identification
- RTC time synchronization for accurate mission timestamps

---

## ⚙️ Hardware Components
| Component | Function |
|------------|-----------|
| **Teensy 4.1** | Main processing unit |
| **BNO055** | Orientation and motion detection |
| **BMP280 / DPS310** | Barometric pressure & altitude measurement |
| **INA219** | Voltage & current monitoring |
| **MQ-131** | Ozone concentration sensing |
| **GPS (NavIC)** | Position and altitude data |
| **XBee / ESP8266** | Telemetry communication |
| **Servo Motor** | Parachute deployment mechanism |
| **Buzzer** | Landing recovery alert |

---

## 🧠 Mission States
| State | Description |
|--------|-------------|
| **BOOT** | System initialization |
| **ASCENT** | Launch and climb phase |
| **GYRO_ACTIVE** | Reaction wheel active (attitude control) |
| **DESCENT** | Parachute deployed |
| **LANDED** | Stable landing detected |
| **RECOVERY_BEEP** | Beacon active for retrieval |

---

## 🧩 Data Logging
Two log files are created automatically on the SD card:
1. `MISSION_x.csv` – Complete mission telemetry  
2. `GPS_LOGx.csv` – Raw GPS data snapshots

---

## 🛰️ Telemetry Format
Each telemetry packet sent via XBee follows this structure:
