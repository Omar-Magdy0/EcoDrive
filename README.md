---
# ⚡ EcoDrive

A **modular, dual-platform** open-source firmware platform for **PMSM motor control** in electric vehicle applications.  
Built for the **Cairo University Eco Racing Team** — validated in real-world racing.

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
![Platform: STM32F4 + Host](https://img.shields.io/badge/platform-STM32F4%20%7C%20Host%20(SIL)-blue)
---

## 🧠 Overview

EcoDrive implements **Field-Oriented Control (FOC)** and **Self-Commissioning (SComm)** for PMSM drives with a clean hardware abstraction layer (`eldriver`) that runs identically on:

| Platform | Purpose | Toolchain |
|----------|---------|-----------|
| **Host (x86)** | SIL simulation, HIL testing, GUI dashboard | Native GCC + CMake |
| **STM32F4** | Embedded deployment | GCC ARM Embedded + CMake |

### Core Capabilities

- ⚙️ **FOC** (Field-Oriented Control) with SVM
- 🔬 **SComm** (Self-Commissioning) — HFI-based parameter identification
- 📊 **Real-time Dashboard** — ImGui/ImPlot GUI with scope, triggers, and analytics
- 🧪 **SIL Simulation** — Full PMSM electrical model with HDF5 logging
- 🔄 **ABF Protocol** — Asynchronous Binary Frame over USB-CDC/UART/TCP
- 📈 **DAQ System** — Internal Data Value (IDV) serialization for telemetry
---

## 🏗️ Architecture

### Project Structure

| Directory | Contents |
|-----------|----------|
| `application/` | Main entry, PMSM control logic |
| `application/PmsmControl/` | FOC, SComm, position drivers, types |
| `core/` | Low-level utilities (rstream, smem, math) |
| `middleware/` | ABF/IDV protocols, DAQ, scope stream |
| `platform/host/` | Host eldrivers, SIL engine, GUI |
| `platform/stm32f4/` | STM32F4 HAL, USB, FreeRTOS config |
| `EcoTool/` | PC utility for USB communication |
| `test/` | SIL integration tests |

---

## 🎮 Control Modes

| Mode | Description | Status |
|------|-------------|--------|
| `Foc` | Field-Oriented Control with Space Vector PWM | ✅ Active |
| `SComm` | Self-Commissioning (R, Ld, Lq identification via HFI) | ✅ Active |
| `Idle` | Safe state, motor disabled | ✅ Active |

### Control Loop Architecture

```
pwmLoop() ─── PWM rate (e.g., 25 kHz)
  ├── Read ADC (phase currents, bus voltage)
  ├── Clarke/Park transforms
  ├── Current PI controllers (dq frame)
  ├── Inverse Park/Clarke
  └── Write SVM duty cycles

xmcLoop() ─── Slow loop (~1 kHz)
  ├── Position/speed estimation
  ├── Speed/position PI controllers
  └── Setpoint management
```

---

## 🔬 SIL Simulation

The Software-in-the-Loop engine models:

- **PMSM electrical dynamics** — saliency, mutual inductances, back-EMF
- **Inverter non-idealities** — dead-time, diode commutation, on/off resistance
- **Discontinuity handling** — sub-stepping for drive transitions
- **HDF5 logging** — compressed, chunked datasets with column metadata

Launch the GUI dashboard:
```bash
./build/bin/EcoDrive
```

---

## 📡 Communication Protocols

### ABF (Asynchronous Binary Frame)
- Sync byte + ServiceID + Payload + CRC-8 AUTOSAR
- MCOBS encoding for payload
- Supports UART, USB-CDC, and TCP transports

### IDV (Internal Data Value)
- Self-describing serialization for DAQ channels
- Auto-discovery with MARKER-based session/stream/channel hierarchy

---

## 📄 License

This project is **MIT licensed** — see [LICENSE](LICENSE) for details.

## 🤝 Sponsors

[![Shell](https://img.shields.io/badge/Shell-Sponsor-yellow)](https://www.shell.com)
[![EgyptAir](https://img.shields.io/badge/EgyptAir-Sponsor-red)](https://www.egyptair.com)

## 📬 Contact

**Maintainer**: Omar Magdy  
📧 [omar.magdy.om.om@gmail.com](mailto:omar.magdy.om.om@gmail.com)

---

*Built with passion by the **Cairo University Eco Racing Team** 🏁*

---
# ⚡ EcoDrive

A modular, open-source platform for developing **inverters**, **controllers**, and **embedded firmware** for electric vehicle applications.  
Built with robust engineering principles and driven by passion.

Maintained by **Omar Magdy** for the **Cairo University Eco Racing Team**.

---

## 🧠 Overview

EcoDrive implements a range of hardware and software solutions for electric drive systems.  
It includes:

- Custom inverters (multi-phase and bidirectional support)
- Firmware for Trapezoidal , FOC, SPWM, SVM, and advanced motor control
- Modular controllers and drivers
- Fully documented software and hardware stacks
- Design iterations built for real-world racing

---

## 🚀 Features

- ⚙️ **Multiple Inverter Designs**
- 🧾 **Well-documented Firmware** using Doxygen
- 💡 **Modular & Scalable Design** across generations
- 🛠️ **Interactive BOMs and Fabrication-Ready Gerbers**
- 🔧 **Configurable Build System** for firmware
- 💻 **Simulator & HIL Support**
- 🧪 **Validated by Real-World Team Use**

Specific features are documented in each design subfolder:

---

## 📦 Installation

- Build your own firmware using the instructions in each firmware folder.
- Or, download the **precompiled binaries** from the design folders.
- Configure and flash using the README-provided instructions.

---

## 🧑‍💻 Used By

- 🏁 **Cairo University Eco Racing Team** https://cu-eco.org

---

## 📄 License

This project is open source.  
📌 **License**:  MIT

---

## 💬 Support

You can support the project by:

- Contributing firmware or hardware improvements
- Reviewing documentation and build systems
- Participating in issue tracking and pull requests
- Providing feedback based on hardware tests
- Become a sponsor
---

## 🤝 Sponsors

- 🐚 **Shell**
- ✈️ **EgyptAir**

---

## 📬 Contact

Maintainer: **Omar Magdy**  
📧 [omar.magdy.om.om@gmail.com](mailto:omar.magdy.om.om@gmail.com)

---

## 🏷️ Acknowledgements

- All engineers and contributors from the Cairo University Eco Racing Team

---

