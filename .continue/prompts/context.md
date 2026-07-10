# EcoDrive Project Context

## Project Overview
EcoDrive is a modular PMSM (Permanent Magnet Synchronous Motor) control system for electric vehicle applications. It supports dual platforms: STM32F4 for embedded deployment and host (x86) for simulation/HIL testing.

## Tech Stack

### Languages
- **C++20** - Primary application code
- **C11** - Driver layer and HAL
- **ARM Assembly** - Startup code

### Frameworks & Libraries
- **CMSIS-DSP** - Digital signal processing (FOC, transforms, filters)
- **ImGui** - GUI for host simulation dashboard
- **ImPlot** - Real-time plotting
- **ImGuiFileDialog** - File selection
- **Eigen** - Linear algebra
- **HDF5** - Data logging
- **FreeRTOS** - RTOS (STM32F4)
- **TinyUSB** - USB CDC (STM32F4)

### Build System
- **CMake** with platform-specific toolchains
- **GCC ARM** for STM32F4
- **Native GCC** for host simulation

## Architecture

### Layer Structure


### Key Design Patterns
1. **Hardware Abstraction** - `eldriver_*` APIs abstract platform differences
2. **State Machine** - `MCMode` enum controls FOC/SComm/Trap states
3. **Callback-Based** - `onFrame`, `onError` callbacks for protocol handling
4. **Singleton/Static** - `PmsmControl` as global instance
5. **Friend Functions** - Platform callbacks access private members

## Key Entry Points
- **Main**: `application/main.cpp` → `platform_init()` → `gui_loop()` (host) or FreeRTOS (STM32)
- **PMSM Control**: `PmsmControl::init()` → configures motor control
- **Control Loops**: `pwmLoop()` (high frequency) and `xmcLoop()` (low frequency)
- **Communication**: ABFStream protocol over UART/USB/CDC

## Motor Control Modes
1. **MCMode::SComm** - Sensorless commutation (BLDC)
2. **MCMode::Foc** - Field Oriented Control (PMSM)
3. **MCMode::Trap** - Trapezoidal control (BLDC)

## Data Flow


## Communication Protocols
- **ABF** (Application Bus Frame) - Custom framing with CRC8
- **IDV** (Internal Data Value) - Serialization protocol for DAQ
- **USB-CDC** - Virtual serial over USB
- **UART** - Serial communication
- **TCP** - Host simulation communication


