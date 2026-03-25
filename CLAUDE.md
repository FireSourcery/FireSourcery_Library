# CLAUDE.md — FireSourcery_Library

## Project Overview

**FireSourcery_Library** is a modular, layered embedded systems library written in **C** for motor control applications targeting **ARM Cortex-M** microcontrollers (primarily NXP S32K / Kinetis KE0x families). It implements a full motor controller stack including FOC (Field-Oriented Control), sensor feedback, protocol communication, and peripheral abstraction.

## Core Principles
- **Simplicity First**: Make every change as simple as possible. Impact minimal code.
- **No Laziness**: Find root causes. No temporary fixes. Senior developer standards.
- **Minimal Impact**: Changes should only touch what's necessary. Avoid introducing bugs.

### Demand Elegance (Balanced)
- For non-trivial changes: pause and ask "is there a more elegant way?"
- If a fix feels hacky: "Knowing everything I know now, implement the elegant solution"
- Skip this for simple, obvious fixes – don't over-engineer
- Challenge your own work before presenting it
- Consider established design patterns to eliminate code duplication

### Self-Improvement Loop
- After ANY correction from the user: update `.claude/tasks/lessons.md` with the pattern
- Write rules for yourself that prevent the same mistake
- Ruthlessly iterate on these lessons until mistake rate drops
- Review lessons at session start for relevant project

## Task Management
1. **Plan First**: Write plan to `.claude/tasks/todo.md` with checkable items
2. **Verify Plan**: Check in before starting implementation
3. **Track Progress**: Mark items complete as you go
4. **Explain Changes**: High-level summary at each step
5. **Document Results**: Add review section to `.claude/tasks/todo.md`
6. **Capture Lessons**: Update `.claude/tasks/lessons.md` after corrections

## Build & Toolchain

- **Compiler:** ARM GCC (`arm-none-eabi-gcc` 15.2.1)
- **Target Architecture:** ARM Cortex-M4 (S32K142, KE0x)
- **Build System:** Makefile / IDE project-based (MCUXpresso / VS Code)
- **C Standard:** C11 (gnu11)
- **No dynamic allocation** — all memory is statically allocated
- **No standard library dependencies** beyond `<stdint.h>`, `<stdbool.h>`, `<stddef.h>`, `<string.h>`

## Key Conventions

### Architecture Patterns
- **Opaque struct pattern**: Modules expose a struct type and operate on pointers to it
- **Config struct pattern**: Modules use `const` config structs (`CONFIG` / `CONST`) separated from mutable state
- **HAL abstraction**: Hardware access through `HAL_*.h` headers with platform-specific implementations
- **Thread/ISR separation**: `_Thread.h` files define periodic processing functions; ISR-safe boundaries are explicit
- **State machines**: Hierarchical state machine framework in `Utility/StateMachine/` used extensively for motor control states
- **Fixed-point math**: 16-bit fractional (`fract16`) and Q16 fixed-point arithmetic — no floating point at runtime
- **Const struct descriptors pattern**:  "Static Polymorphism Pattern". Const struct handle, holds pointer to runtime state in RAM.
<!-- - **NvMemory pattern**: Configuration stored in Flash/EEPROM with structured read/write abstraction -->

### Important Rules
- **No heap allocation** (`malloc`/`free` are never used)
- **No floating point** in runtime code paths — all math is integer/fixed-point
- **ISR-safe**: Shared data between ISR and thread contexts uses volatile and critical sections (`System/Critical/`)
- **Const-correctness**: Config/calibration data is `const` qualified and stored in non-volatile memory regions
- **Reentrant-safe**: Module functions operate on explicit instance pointers, no hidden global state

### Code Style
- **PascalCase** for types, structs, enums, and function names: `Motor_FOC_ProcAngle()`, `MotorController_Init()`
- **UPPER_SNAKE_CASE** for macros, enum values, and constants: `MOTOR_STATE_RUN`, `HAL_ADC_CHANNEL_COUNT`
- **camelCase** for local variables and struct fields
- Keep short functions on one line
- Prefix functions with module name: `Motor_`, `Phase_`, `Encoder_`, `Serial_`, etc.
- Files prefixed with `_` (e.g., `_Motor_Config.h`) are internal/private headers not intended for external inclusion
- Public API headers match the module directory name: `Motor.h`, `StateMachine.h`, `Protocol.h`


## Directory Map

```
FireSourcery_Library/
│
├── External/                    # Third-party / vendor headers
<!-- │   ├── CMSIS/Core/Include/      #   ARM CMSIS core headers (Cortex-M4) -->
<!-- │   └── S32K142/include/         #   NXP S32K142 device headers & register maps -->
│
├── Math/                        # Fixed-point math primitives (no FPU)
│   ├── Accumulator/             #   Integrator / accumulator
│   ├── Angle/                   #   Angle representation, encoder-to-angle, speed calc
│   ├── Filter/                  #   Moving average filters
│   ├── Fixed/                   #   Fixed-point (Q16) and fract16 arithmetic
│   ├── Hysteresis/              #   Hysteresis comparators
│   ├── Linear/                  #   Linear interpolation / scaling (y = mx + b)
│   ├── PID/                     #   PID controller
│   ├── Ramp/                    #   Slew-rate limiter / ramp generator
│   ├── math_bits.h              #   Bit manipulation helpers
│   ├── math_edge.h              #   Edge detection (rising/falling)
│   └── math_general.h           #   General math macros (min, max, clamp, abs)
│
├── Motor/                       # *** Core motor control stack ***
│   ├── Motor/                   #   Single motor instance control
│   │   ├── Motor.h              #     Top-level motor API
│   │   ├── Motor_FOC.c/h        #     FOC algorithm entry (Park, Clarke, SVPWM)
│   │   ├── Motor_Commutation.h  #     Commutation mode dispatch
│   │   ├── Motor_StateMachine.c/h #   Motor state machine (Init→Stop→Run→Fault)
│   │   ├── Motor_User.c/h       #     User-facing getters/setters
│   │   ├── Motor_Var.c/h        #     Variable table (read/write by ID)
│   │   ├── Motor_Thread.h       #     Periodic thread processing functions
│   │   ├── Motor_Config.c/h     #     NvMemory-backed configuration
│   │   ├── Motor_Debug.h        #     Debug / diagnostic accessors
│   │   ├── Motor_Table.h        #     Multi motor array
│   │   ├── Analog/              #     Motor-specific ADC channels
│   │   ├── Math/                #     FOC math core (Clarke, Park, SVPWM, PI loops)
│   │   │   ├── FOC.c/h          #       Core FOC transform & control loop
│   │   │   ├── FOC_Feedback.h   #       Current feedback processing
│   │   │   ├── FOC_Ext.h        #       Extended FOC features
│   │   │   ├── math_foc.h       #       FOC math primitives
│   │   │   └── math_svpwm.h     #       Space-vector PWM sector math
│   │   ├── Phase/               #     3-phase voltage/current representation
│   │   ├── Phase_Input/         #     Phase voltage/current acquisition, calibration
│   │   ├── Sensor/              #     Rotor position sensor abstraction
│   │   │   ├── Encoder/         #       Incremental encoder sensor
│   │   │   ├── Hall/            #       Hall-effect sensor (6-step + interpolation)
│   │   │   └── SinCos/          #       Sin/Cos analog encoder
│   │   └── SubStates/           #     Calibration & open-loop sub-states
│   │
│   ├── MotorController/         #   System-level controller (multi-motor, app logic)
│   │   ├── MotorController.c/h  #     Top-level controller API & init
│   │   ├── MotorController_StateMachine.c/h # App-level state machine
│   │   ├── MotorController_Thread.h #   Main loop / periodic tasks
│   │   ├── MotorController_User.c/h #   User command interface
│   │   ├── MotorController_Var.c/h  #   Controller variable table
│   │   ├── MotorController_App.h    #   Application-specific hooks
│   │   ├── MotAnalogUser/       #     Analog input (throttle, brake) processing
│   │   ├── MotBuzzer/           #     Buzzer / audio feedback
│   │   ├── MotLimits/           #     Current/voltage/temperature limiting
│   │   ├── MotNvm/              #     Non-volatile memory management
│   │   └── Vehicle/             #     Traction controller specialized logic (speed, direction, gear)
│   │
│   └── MotProtocol/             #   Motor-specific protocol layer
│       ├── MotPacket.c/h        #     Packet format for motor variable read/write
│       ├── MotProtocol.c/h      #     Protocol command handler
│       └── MotVarId.h           #     Variable ID enumeration (shared with host tools)
│
├── Peripheral/                  # Hardware peripheral abstraction
│   ├── Analog/                  #   ADC driver (multi-channel, DMA-capable)
│   │   ├── Analog.h             #     Top-level analog API
│   │   ├── Analog_ADC.h         #     ADC conversion management
│   │   ├── Analog_ADC_Thread.h  #     ADC periodic scan
│   │   ├── Linear_ADC.h         #     Linear scaling of ADC values
│   │   └── HAL_ADC.h            #     HAL interface for ADC
│   ├── HAL/                     #   HAL platform implementations
│   │   ├── HAL_Peripheral.h     #     Common HAL type definitions
│   │   └── Platform/KE0x/       #     Kinetis KE0x HAL implementations
│   ├── NvMemory/                #   Non-volatile memory (Flash + EEPROM)
│   │   ├── EEPROM/              #     EEPROM driver
│   │   ├── Flash/               #     Flash programming driver
│   │   └── NvMemory/            #     Unified NvMemory abstraction
│   ├── Pin/                     #   GPIO pin abstraction
│   ├── PWM/                     #   PWM output driver (center-aligned for motor)
│   ├── Serial/                  #   UART serial driver (interrupt/DMA)
│   └── Xcvr/                    #   Transceiver abstraction (RX/TX buffer management)
│
├── System/                      # System-level services
│   ├── Critical/                #   Critical section (interrupt disable/enable)
│   ├── Reboot/                  #   System reset / bootloader entry
│   └── SysTime/                 #   System tick timer (millisecond timekeeping)
│
├── Transducer/                  # Sensor & actuator drivers
│   ├── Blinky/                  #   LED blink patterns
│   ├── Encoder/                 #   Quadrature encoder driver (DeltaD / DeltaT modes)
│   ├── Monitor/                 #   Monitoring / fault detection framework
│   │   ├── Base/                #     Generic range monitor, threshold, status
│   │   ├── Heat/                #     Temperature monitoring (thermistor → °C)
│   │   └── Voltage/             #     Voltage monitoring (divider → volts)
│   └── UserIn/                  #   User input (analog + digital with debounce)
│
├── Type/                        # Generic type utilities
│   ├── accessor.h               #   Accessor macros (getter/setter generation)
│   ├── mux.h                    #   Multiplexer pattern
│   ├── typeless.h               #   Type-erased operations
│   ├── Array/                   #   Typed array wrappers, void_array
│   ├── Bits/                    #   Bitfield manipulation
│   ├── Field/                   #   Struct field offset utilities
│   └── Word/                    #   Word-level operations, version struct
│
└── Utility/                     # Reusable utility frameworks
    ├── Protocol/                #   Generic request-response protocol engine
    │   ├── Protocol.c/h         #     Protocol state machine
    │   ├── Socket.c/h           #     Socket-style connection abstraction
    │   ├── Packet/              #     Packet parsing
    │   └── Request/             #     Request/response handling
    ├── Ring/                    #   Ring buffer (byte and typed variants)
    ├── StateMachine/            #   Hierarchical state machine framework
    │   ├── StateMachine.c/h     #     Public API
    │   ├── State.h              #     State definition macros
    │   └── Extension/           #     Composite state machine parts
    ├── Timer/                   #   Software timer (countdown, periodic)
    ├── BootRef/                 #   Bootloader reference / shared memory
    └── LimitArray/              #   Array with bounds-checked access
```

## Module Dependency Flow

```
Application (KMC)
    │
    ▼
MotorController  ←──  MotProtocol
    │
    ▼
Motor  ──→  FOC Math  ──→  Math/Fixed, Math/PID, Math/Angle
  │
  ├──→  Sensor (Hall / Encoder / SinCos)  ──→  Transducer/Encoder
  ├──→  Phase  ──→  Peripheral/PWM
  └──→  Motor_Analog  ──→  Peripheral/Analog
                                │
Transducer/Monitor  ──→  Math/Linear, Math/Filter
                                │
Peripheral/*  ──→  HAL/Platform/*  ──→  External/CMSIS, External/S32K142
                                │
System/*  ──→  HAL (Critical, Reboot, SysTime)
                                │
Utility/*  (StateMachine, Protocol, Ring, Timer) — framework, no HW dependency
Math/*  — pure computation, no HW dependency
Type/*  — pure type utilities, no dependency
```

<!-- ## Common Tasks -->

<!-- ### Adding a new motor variable
1. Add the variable ID to `Motor/MotProtocol/MotVarId.h`
2. Add getter/setter in `Motor/Motor/Motor_Var.c` (or `MotorController/MotorController_Var.c` for controller-level)
3. Add entry to the variable table in the corresponding `_AppTable.c`

### Adding a new peripheral driver
1. Create `HAL_<Periph>.h` interface in `Peripheral/<Module>/`
2. Implement platform-specific HAL in `Peripheral/HAL/Platform/<Platform>/HAL_<Periph>.h`
3. Create driver module with config struct + instance struct pattern

### Adding a new state to a state machine
1. Define state entry/exit/input handler functions
2. Add `State_T` definition using macros from `Utility/StateMachine/State.h`
3. Register in the parent state machine's state table

## Testing & Debugging

- **No unit test framework** in the embedded target — validation is done via:
  - Protocol variable read-back (`MotProtocol`)
  - Debug variables (`Motor_Debug.h`)
  - Host-side tools (Kelly user app — `kelly_user_app_project`)
- **Simulation**: Python scripts in `Motor/Docs/Simulation/` for algorithm validation (SVPWM, atan2) -->



