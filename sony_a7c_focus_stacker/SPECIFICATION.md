# Sony α7C Focus Stacking System Technical Specification & Development Log

## 1. System Overview & Architecture
This project specifies an automated focus-stacking pipeline for a Sony α7C mirrorless camera paired with a 50mm macro lens under shallow depth-of-field constraints. Two primary control topologies were explored:
1. **Host-Tethered C++ / Python (USB-C)** via the official Sony Camera Remote SDK (CRSDK).
2. **Wireless Standalone Embedded (BLE)** via an Adafruit ESP32-S3 Feather emulating the Sony RMT-P1BT Bluetooth Remote Commander.

```
+-----------------------------------------------------------------------------------+
| PHYSICAL RIG & TRIGONOMETRIC CONVERSION                                          |
|                                                                                   |
| Specimen / Calibration Plane: 10 cm physical ruler inclined at 30° to lens axis.  |
|                                                                                   |
|   True Optical Depth (Z) = Ruler Distance (L) * sin(30°) = L * 0.5               |
|                                                                                   |
| Motor behavior across macro range is non-linear and parameterized via a lookup    |
| table (`macro_calibration.csv`) interpolated at runtime.                          |
+-----------------------------------------------------------------------------------+
```

---

## 2. Calibration Model: `macro_calibration.csv`
The calibration model maps physical ruler index positions ($L$) to true depth ($Z$) and dynamic step/pulse densities.

### Calibration Data Model

| Ruler Distance Index ($L$, cm) | True Depth Axis ($Z$, mm) | Calibrated Motor Steps / mm (USB CRSDK) | Calibrated Pulse Duration (ms/mm, BLE) |
|---|---|---|---|
| 2.0 | 10.0 | 24.5 | 320.0 |
| 3.0 | 15.0 | 18.2 | 240.0 |
| 4.0 | 20.0 | 14.1 | 185.0 |
| 6.0 | 30.0 | 9.4 | 125.0 |
| 8.0 | 40.0 | 6.8 | 90.0 |
| 10.0 | 50.0 | 5.1 | 68.0 |

### Linear Interpolation Algorithm
$$R(L) = R_0 + \frac{L - L_0}{L_1 - L_0} (R_1 - R_0)$$

---

## 3. Host-Tethered USB Architecture (CRSDK on ARM64 Linux)

### Hardware Constraints & Compatibility
* **Sony CRSDK Requirement:** Closed-source precompiled binaries (`libCrSDK.so`) require a full hosted OS environment with POSIX threads and dynamic linkers (glibc 2.28+, ARMv7-A/ARMv8-A or x86_64).
* **Incompatible Hardware:** Microcontrollers like the RP2040 cannot run CRSDK due to lack of an OS, limited SRAM (264 KB), and closed-source driver binaries.
* **Compatible Minimal SBCs:** Raspberry Pi Zero 2 W, Raspberry Pi 4B/5, Compute Module 4, or x86_64 Linux laptops.

### Directory Structure (`host_crsdk/`)
* `CMakeLists.txt`: Build configuration linking `libCrSDK.so` and `pthread`.
* `src/StackingEngine.h` & `src/StackingEngine.cpp`: Native C++ stacking controller with linear interpolation, thread-safe timestamped file/console logging, and backlash compensation.
* `src/main.cpp`: CLI entrypoint accepting `<range_mm> <frame_count> <ruler_cm>`.
* `python/focus_stacker.py`: Python prototype implementation.

---

## 4. Standalone Wireless Embedded Architecture (ESP32 BLE)

### Hardware Selection & Antenna Considerations
* **Adafruit ESP32-S3 Feather w/ w.FL (PID 5885):** Requires an external 2.4 GHz antenna. Uses a micro w.FL (MHF3/IPEX3) connector. PID 5477 is an adapter pigtail (w.FL to RP-SMA female jack) requiring an external antenna.
* **Adafruit Feather ESP32-S3 w/ Built-in Antenna (PID 5323 or 5401):** Integrated PCB trace antenna. Recommended for compact hot-shoe / cage installations.

### Sony BLE Protocol Specifications & Fixes
The Sony RMT-P1BT protocol uses GATT Service `8000FF00-FF00-FFFF-FFFF-FFFFFFFFFFFF` and Control Characteristic `0000FF01-0000-1000-8000-00805F9B34FB`:

* **Shutter & Autofocus (2-byte commands):**
  * AF / Half-Press Engage: `0x01, 0x07`
  * AF / Half-Press Release: `0x01, 0x06`
  * Full Shutter Down: `0x01, 0x09`
  * Full Shutter Up: `0x01, 0x08`
* **Manual Focus Stepping (3-byte commands):**
  * Focus Near (+) Press: `0x02, 0x6B, [Speed]` (Speed: `0x10` slow to `0x7F` fast)
  * Focus Near Release: `0x02, 0x6A, 0x00`
  * Focus Far (-) Press: `0x02, 0x6D, [Speed]`
  * Focus Far Release: `0x02, 0x6C, 0x00`
* **Device Filtering:**
  * Sony Company Identifier: `0x012D` in BLE Manufacturer Specific Data.
  * Filters non-Sony peripherals during scanning.

### Firmware Implementation (`firmware/BLE camera control/`)
* **PlatformIO Environment:** `adafruit_feather_esp32s3` with LittleFS, PSRAM, and native USB CDC.
* **Display & Physical Controls (FeatherWing OLED 128x32):**
  * `BUTTON_A` (GPIO 9): Start stacking sweep / Cancel running sweep / Start BLE scan if disconnected.
  * `BUTTON_B` (GPIO 6): Short press = Single shutter trigger; Long press (>600ms) = Focus Far (-100ms).
  * `BUTTON_C` (GPIO 5): Short press = Focus Near (+100ms); Long press (>600ms) = Focus Far (-100ms).
  * 128x32 OLED provides real-time connection telemetry, parameter display, and slice progress bar.
* **USB-CDC Serial Console Interface:**
  * `RUN [range_mm] [frames] [ruler_cm]`: Trigger sweep with active or specified parameters.
  * `SET [range_mm] [frames] [ruler_cm]`: Update parameters without triggering.
  * `SHUTTER` / `SNAP`: Trigger camera exposure.
  * `NEAR [ms]` / `FAR [ms]`: Nudge focus near/far.
  * `SCAN`: Restart camera discovery scan.
  * `STOP`: Abort running stack.
  * `HELP`: Display command summary.
* **Flash Storage & Logging (`LittleFS`):**
  * `macro_calibration.csv`: Embedded LittleFS stores calibrated ms/mm mapping curve with linear interpolation.
  * `stacker.log`: Persistent timestamped command and output logging (`[HH:MM:SS.mmm] [TAG] message`) with automatic log rotation at 64 KB (`stacker.log.old`).
  * Serial Log Tools: `LOG` / `DUMP_LOG` (read out log file), `CLEAR_LOG` (truncate/erase), `LOG_STATUS` (file size and flash space).

---

## 5. Architectural Comparison Summary

| Parameter | Linux Host (USB-C CRSDK) | Embedded BLE Remote (ESP32-S3) |
|---|---|---|
| **Control Interface** | USB 2.0 / 3.0 Type-C cable | 2.4 GHz Bluetooth Low Energy (BLE) |
| **Step Mechanism** | Discrete DSP micro-steps (`CrFocusPositionDrive`) | Timed pulse durations (ms active hold) or discrete 3-byte steps |
| **Repeatability** | Deterministic internal motor index | Subject to BLE connection interval jitter ($\pm 10\text{ ms}$) |
| **Telemetry** | Full two-way state and exposure control | One-way blind commands |
| **Hardware Form Factor** | RPi 4/5, Pi Zero 2 W, or Ubuntu Laptop | Adafruit ESP32-S3 Feather + LiPo battery |
| **Cabling** | Requires USB-C tether to camera | Fully wireless |
