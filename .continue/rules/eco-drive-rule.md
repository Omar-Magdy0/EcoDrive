# EcoDrive Coding Rules

## Project-Specific Patterns

### 1. Hardware Abstraction Layer (HAL)

#### Driver Naming Convention
```cpp
// All platform drivers follow this pattern:
eldriver_[module]_[function]  // e.g., eldriver_mc3p_init()
eldriver_[module]_[action]    // e.g., eldriver_gpio_write()

// Use platform detection for host vs embedded
#ifdef PLATFORM_HOST
    // Host simulation implementation
#else
    // STM32F4 implementation
#endif

// FOC/SComm/Trap modes share this structure:
void [Mode]_init()              // Initialize mode
void [Mode]_onEnter(MCMode prev) // Enter mode
void [Mode]_onExit()            // Exit mode  
void [Mode]_pwmLoop()           // High-frequency (PWM rate)
void [Mode]_xmcLoop()           // Low-frequency (control rate)

// Use config structs with defaults
struct Config[Feature] {
    // Members with default values
};
ERR config[Feature](Config[Feature] cfg);

// Use q31_t for fixed-point operations
// Constants use _q31 suffix or q31_ prefix
q31_t angle_q31 = /* ... */;
float angle_float = angle_q31 * (M_PI / INT32_MAX);

// Q15 for PWM duty cycles (-1.0 to 1.0)
int16_t duty_q15; // Q15 format
// Scale to float: duty_float = duty_q15 / 32767.0f

enum class ERR : uint8_t {
    OK = 0,
    ERR_GENERIC,
    ERR_CONFIG,
    // ...
};

ERR function() {
    if (condition) return ERR::ERR_TYPE;
    return ERR::OK;
}

// Standard callback signatures
void onFrame(void* ctx, uint8_t id, uint8_t* payload, uint8_t payload_len);
void onError(void* ctx, uint8_t id);

// Register callbacks
void setOnFrame(void (*callback)(void* ctx, uint8_t id, uint8_t*, uint8_t));

// Use elcore_smem for static allocation
elcore_smem_t memory_pool;
elcore_smem_init(&memory_pool, buffer, buffer_size);
void* ptr = elcore_smem_alloc(&memory_pool, size);


// Use elcore_rstream for circular buffers
elcore_rstream_t stream;
elcore_rstream_init(&stream, buffer, buffer_size);
// Write: elcore_rstream_commitWrite()
// Read: elcore_rstream_releaseRead()

// Use Sil class for HDF5 logging
Sil sil;
sil.logStart("filename.h5");
sil.log(state, input);  // Log frame
sil.logStop();

// Separate control panel and signal displays
void drawControlPanel();    // Parameters & controls
void drawPhaseCurrentPlot(); // Real-time plots (ImPlot)
void drawMechanical();      // Speed, torque, etc.



---

## **`.continue/prompts/mcp-rules.md`**

```markdown
# MCP (Motor Control Protocol) Rules

## Core Modules

### 1. PMSM Control Core
**Files**: `application/PmsmControl/PmsmControlCore_*.cpp`

**Key Functions**:
- `Foc_pwmLoop()` - FOC high-frequency (current control)
- `Foc_xmcLoop()` - FOC low-frequency (speed/position)
- `SComm_pwmLoop()` - Sensorless commutation
- `Trap_pwmLoop()` - Trapezoidal control

**Dependencies**:
- `eldriver_mc3p_*` - ADC reading, PWM output
- `elmath.h` - Math utilities
- `CMSIS-DSP` - Clarke/Park transforms

### 2. Position/Speed Sensing
**Files**: `PosDriver.h`, `PosDriver.cpp`, `eldriver_hall.*`

**Sensor Types**:
- Hall sensors (`PosDriverType::Hall`)
- Sensorless (`PosDriverType::Open`)
- Resolver/Encoder (future)

### 3. PWM Generation
**Files**: `eldriver_mc3p.*`

**Modes**:
- **SVM** (Space Vector Modulation) - FOC mode
- **Trap** (Trapezoidal) - BLDC mode

**Key Functions**:
- `eldriver_mc3p_write_svm(alpha_q15, beta_q15)` - SVM output
- `eldriver_mc3p_write_trap(sector, duty_q15)` - Trapezoidal output

### 4. Data Acquisition
**Files**: `DAQStream.*`, `ABFStream.*`, `ScopeStream.h`

**Key Functions**:
- `DAQSession::process()` - Parse IDV frames
- `ABFStream::process()` - Parse ABF frames
- `ScopeStream::write()` - Buffer samples

## Critical Data Structures

### `PmsmControlTypes.h`
Contains all type definitions:
- `MCMode` - Control mode enumeration
- `ConfigPwm` - PWM configuration
- `ConfigFocPid` - PID gains for FOC
- `ConfigOlstup` - Open-loop startup parameters
- `Pid` - PID controller structure

### `Sil` (Host Only)
Software-in-the-Loop simulation:
- `state` - Motor state (current, speed, position)
- `input` - Control input (voltage, PWM)
- `electrical_solve()` - Electrical model
- `log()` - HDF5 logging

## Communication Protocols

### ABF Protocol
**Files**: `ABFStream.*`

# Whole Repo digest
This is under .continue/repomix-output-EcoDrive.xml


**Structure**: