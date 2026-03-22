# 🛠️ Platform & Eldriver Documentation [STM32F4]

## 📌 Introduction
This document provides the technical specifications and software architecture for the **STM32F407** platform used in the **EcoDrive** system. It focuses on power optimization (Eco-mode) and peripheral driver interfacing.

---

## 🏗️ 1. System Architecture
The firmware is structured in a layered approach to ensure modularity between the hardware and the eco-driving logic.

### Layer Stack:
- **Application Layer:** Eco-Drive Algorithm & Power Management.
- **Service Layer:** Data logging and system state configurations.
- **Eldrivers (Peripheral Drivers):** Abstraction layer for STM32F4 hardware.
- **Platform/Hardware:** STM32F407 (Cortex-M4) Microcontroller.

> **Architecture Diagram:**
> ![System Architecture]("\\wsl.localhost\Ubuntu\home\carol_nasser\EcoDrive\docs\EcoDrive.drawio.png")

---

## 💻 2. Platform Specifications (STM32F4)
| Feature | Details |
| :--- | :--- |
| **Microcontroller** | STM32F407VG (ARM Cortex-M4) |
| **Core Frequency** | 84 MHz (Configured for Power Saving) |
| **Flash Memory** | 1 MB |
| **SRAM** | 192 KB |
| **Toolchain** | GCC Arm Embedded / CMake |

---

## 🔌 3. Eldriver (Peripheral) Documentation
All drivers are implemented following the **Doxygen** commenting standard for automated API documentation.

### A. ADC Driver (Analog-to-Digital)
- **Files:** `platform/drivers/adc_driver.h`
- **Purpose:** Interfaces with the accelerator pedal and battery voltage sensors.
- **Key APIs:** - `ADC_Init()`: Configures 12-bit resolution.
    - `ADC_Read()`: Polls sensor data for the Eco-Logic.

### B. PWM Driver (Pulse Width Modulation)
- **Files:** `platform/drivers/pwm_driver.h`
- **Purpose:** Controls the inverter/motor power delivery.
- **Key APIs:** - `PWM_SetDutyCycle()`: Dynamically adjusts power to optimize energy.

### C. GPIO & Communication
- **GPIO:** Manages Eco-mode LED indicators and safety signals.
- **CAN/UART:** Telemetry data transmission to the vehicle dashboard.

---

## 📚 4. Code-Level Documentation (Doxygen)
To generate the full technical HTML report, navigate to the project root and run:
```bash
doxygen Doxyfile

Last updated: March 22, 2026