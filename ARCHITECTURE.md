# FireSourcery Library Architecture

Embedded C library for bare-metal motor controllers on ARM Cortex-M. It provides a complete field-oriented motor control stack (control loops, rotor sensing, protection, CANopen/CiA 402 and serial protocols), built from reusable math, state machine and protocol modules that also work on their own.

- **Language**: C23, built with `arm-none-eabi-gcc` for Cortex-M0+ and Cortex-M4.
- **Runtime**: bare metal. No RTOS, no heap after init, no `stdio`, no blocking in the main loop.
- **Arithmetic**: fixed-point at runtime; no FPU required.
- **Platforms**: NXP Kinetis KE0x in tree. Other platforms plug in from the application.

## Motor control

- **Field-oriented control**: Clarke/Park transforms, SVPWM, d/q current PI loops, speed PID. Feedback modes are voltage, current, or a speed loop over either.
- **Rotor sensing**: Hall, quadrature encoder, analog sin/cos, and a sensorless sliding-mode observer with PLL (in development). All sit behind one `RotorSensor_T` interface, selectable per motor at runtime.
- **Motor states**: passive (freewheel or hold), run, open loop with alignment and sensor start-up sequences, calibration, intervention (zero torque, ramp-down, safe stop), fault.
- **Calibration**: phase current zero offsets, Hall table, encoder homing and alignment, sin/cos, standstill Rs/Ld/Lq identification.
- **Protection**: DC bus over/under-voltage, supply discharge and regen current limits (mapped onto the q axis by power balance), thermal derating for the motor, MOSFETs and PCB.
- **Controller**: one `MotorController` runs a table of motors. Operating modes plug into its state machine as subtrees: direct motor command, or traction (throttle, brake, direction, park).
- **Communication**: serial packet protocol with variable read/write by ID and a flash loader; CANopen SDO/PDO with the CiA 402 drive profile. Every motor variable is also reachable over SDO.
- **Configuration**: parameters persist in Flash or EEPROM and are addressed by variable ID.

## Design

- **Compile-time HAL**: the platform is selected by include path, so `HAL_*.h` functions inline down to register access with no runtime dispatch.
- **Const descriptors**: hardware-bound modules are `const` structs in ROM (HAL handles, pin IDs, NVM config address) that point at their mutable state in RAM.
- **Hierarchical state machine**: states are `const` tree nodes in ROM; an instance's RAM holds the active-state pointer and input buffers. Other translation units can mount substates onto a published state.
- **Fixed-point quantities**: Q1.15 `fract16`, `angle16` and per-unit motor values. The type names the representation, the identifier suffix names the unit. Floating point appears only in compile-time constant macros.
- **Pure math layer**: control and physics equations are stateless `*_math.h` functions over scalars, kept apart from the modules that hold state.
- **Decoupled protocols**: the serial, CANopen and CiA 402 engines depend on neither the controller nor a specific transport. Small binding files map their requests onto controller and motor variables.

## Dependencies

Arrows point from a module to what it depends on. A layer may use anything below it; nothing includes upward.

### Layers

```mermaid
---
config:
  flowchart:
    wrappingWidth: 600
---
flowchart TB
    App(["Application<br/>board descriptors · ISR vectors · main loop"])
    Motor["<b>Motor</b><br/>MotorController · Motor · MotProtocol"]
    Transducer["<b>Transducer</b><br/>Encoder · Monitor · UserIn · Pulse · Blinky"]
    Peripheral["<b>Peripheral</b><br/>ADC · PWM · CAN · UART · SPI<br/>Pin · NvMemory · ClockTimer · Xcvr"]
    Framework["<b>Framework</b><br/>StateMachine · Protocol · Ring<br/>Timer · LimitArray · Transient"]
    HAL["<b>Peripheral/HAL</b><br/>HAL_*.h per platform"]
    System["<b>System</b><br/>Critical · SysTime<br/>Reboot · BootRef"]
    Math["<b>Math</b><br/>Fixed · Angle · PID · Ramp<br/>Filter · Linear · Accumulator<br/>Hysteresis · Threshold"]
    Type["<b>Framework/Type</b><br/>Array · Bits<br/>Field · Word"]
    MCU[("MCU")]

    App --> Motor
    Motor --> Transducer
    Transducer --> Peripheral
    Transducer --> Framework
    Peripheral --> HAL
    Peripheral --> System
    Framework --> System
    Framework --> Math
    Framework --> Type
    HAL --> MCU
    System --> MCU
```

`Transducer` turns peripheral signals into measurements and user commands. `Framework` holds reusable engines with no peripheral access. `Math` covers numeric values and `Framework/Type` the shape of memory; neither depends on anything. Register access is confined to `Peripheral/HAL/Platform` and to `System`, which wraps the Cortex-M core: critical sections, SysTick time, reset. Vendor headers (CMSIS, device registers) are in `External/`.

### Motor stack

```mermaid
---
config:
  flowchart:
    wrappingWidth: 600
---
flowchart TB
    App(["Application"])
    Bind["<b>Bindings</b><br/>MotorController_MotProtocol<br/>MotorController_Cia402<br/>MotorController_MotCan"]

    subgraph Controller["MotorController"]
        MC["<b>MotorController</b><br/>system state machine · NVM<br/>monitors · user inputs"]
        Traction["<b>Traction</b><br/>throttle · brake · direction"]
    end

    subgraph Engines["MotProtocol: wire engines"]
        Serial["<b>MotPacket</b> · MotProtocol<br/>serial packets · var access<br/>flash loader"]
        CANopen["<b>CANopen</b> · Cia402 · MotCan<br/>SDO · PDO · drive profile"]
    end

    subgraph Axis["Motor: one instance per motor"]
        Motor["<b>Motor</b><br/>User · Var · Config"]
        MSM["<b>Motor_StateMachine</b>"]
        FOC["<b>FOC</b><br/>Clarke · Park<br/>SVPWM · PI"]
        Sensor["<b>RotorSensor</b><br/>Hall · Encoder<br/>SinCos · Sensorless"]
        Phase["<b>Phase</b><br/>PWM output"]
        PhaseIn["<b>Phase_Input</b><br/>phase I/V"]
        VBus["<b>VBus</b><br/>DC bus"]
    end

    SM["Framework/StateMachine"]
    Proto["Framework/Protocol"]
    MathL["Math<br/>Fixed · PID · Ramp · Angle"]
    Tdc["Transducer<br/>Encoder · Pulse · Monitor"]
    Per["Peripheral<br/>PWM · ADC"]

    App --> MC
    Bind --> MC & Serial & CANopen
    MC --> Traction & Motor
    Traction --> Motor
    Motor --> MSM & FOC & Sensor & Phase & PhaseIn & VBus
    MSM --> SM
    Serial --> Proto
    FOC --> MathL
    Sensor --> Tdc
    VBus --> Tdc
    Phase --> Per
    PhaseIn --> Per
```

Each `Motor` composes its own state machine, FOC loop, sensor table and phase I/O. `MotorController` owns the motor table, the system state machine, monitors, NVM and communication ports; both levels run on `Framework/StateMachine`. The wire engines know nothing of the controller, so binding files supply the mapping.

## Integration

1. Put the library root on the include path. Includes are root-relative, e.g. `#include "Motor/MotorController/MotorController.h"`.
2. Select the HAL with one define: `HAL_PERIPHERAL_PATH_PLATFORM=KE0x` for an in-tree platform, or `HAL_PERIPHERAL_PATH_DIRECTORY=<dir>` for a platform directory the application supplies.
3. Define the board as `const` descriptors: peripherals, motors, and the `MotorController_T`.
4. Call the thread entry points from their execution contexts:

| Context | Rate | Call | Work |
|---|---|---|---|
| PWM ISR | PWM frequency, typically 20 kHz | `MotorController_PWM_Thread` | ADC sampling, rotor angle capture, motor state machine: FOC and duty update |
| Timer ISR | 1 kHz | `SysTime_CaptureMillis_ISR`, `MotorController_Timer1Ms_Thread` | Millisecond time base, DC bus over/under-voltage |
| Main loop | Free-running | `MotorController_Main_Thread` | 1 ms: controller state machine, protocols, user inputs. 10 ms: supply limits. ~1 s: thermal, derating, fault roll-up |

Peripheral interrupts (UART, CAN, encoder edges) call their module's `*_ISR` handlers.

## Directory Map

```mermaid
---
config:
  treeView:
    showIcons: true
---
treeView-beta
├── Docs/                                      ## conventions, datasheets, references
├── External/                                  ## vendor headers
│   ├── CMSIS/                                 ## Cortex-M core
│   └── S32K142/                               ## NXP S32K142 device
├── Framework/                                 ## reusable engines, no peripheral access
│   ├── LimitArray/                            ## multi-source derate arbitration
│   ├── Protocol/                              ## request/response protocol engine
│   │   └── Extension/                         ## data mode, flash loader, datagram
│   ├── Ring/                                  ## ring buffer
│   ├── StateMachine/                          ## hierarchical state machine
│   │   └── Extension/                         ## composite machine parts
│   ├── Timer/                                 ## software timers
│   ├── Transient/                             ## triggered sample recorder
│   └── Type/                                  ## shape of memory
│       ├── Array/                             ## typed arrays
│       ├── Bits/                              ## bit fields
│       ├── Field/                             ## struct field get/set
│       ├── Word/                              ## packed words, version
│       ├── accessor.h                         ## accessor macros
│       └── void_pointer.h                     ## pointer casts and copies
├── Math/                                      ## numeric values, no hardware
│   ├── Accumulator/                           ## saturating integrator
│   ├── Angle/                                 ## angle and speed state, PLL
│   ├── Filter/                                ## moving average
│   ├── Fixed/                                 ## fract16 Q1.15, angle16
│   ├── HAL/                                   ## fixed or float representation
│   ├── Hysteresis/                            ## hysteresis band
│   ├── Linear/                                ## linear maps, unit scaling
│   ├── PID/                                   ## PID controller
│   ├── Ramp/                                  ## slew-rate limited setpoint
│   ├── Threshold/                             ## threshold levels
│   ├── bits_math.h                            ## bit manipulation
│   ├── edge_fn.h                              ## edge detection
│   └── math_general.h                         ## min, max, clamp
├── Motor/                                     ## motor control stack
│   ├── Docs/                                  ## fixed-point notes, simulations
│   ├── Motor/                                 ## one motor instance
│   │   ├── Analog/                            ## phase ADC capture, offset calibration
│   │   ├── Extension/                         ## motor table, debug hooks
│   │   ├── Math/                              ## FOC math
│   │   │   ├── FOC.c/h                        ## FOC state over the pure math
│   │   │   ├── FOC_Ext.h                      ## current capture, loop helpers
│   │   │   ├── foc_math.h                     ## Clarke, Park, limits
│   │   │   ├── FOC_Sensorless.c/h             ## rotor position observer, superseded by Sensor/Sensorless/EmfObserver.h
│   │   │   ├── foc_sensorless_math.h          ## back-EMF, SMO, PLL detector
│   │   │   ├── motor_electrical_math.h        ## per-unit parameters, identification
│   │   │   ├── smo_math.h                     ## sliding-mode switching function
│   │   │   └── svpwm_math.h                   ## space vector modulation
│   │   ├── Phase/                             ## 3-phase PWM output
│   │   ├── Phase_Input/                       ## phase current and voltage capture
│   │   ├── Sensor/                            ## RotorSensor interface, sensor table
│   │   │   ├── Encoder/                       ## quadrature encoder
│   │   │   ├── Hall/                          ## Hall sensors
│   │   │   ├── Sensorless/                    ## stator as angle source: EmfObserver (Stator image, EMF estimate, AnglePll)
│   │   │   └── SinCos/                        ## analog sin/cos
│   │   ├── StateMachine/                      ## substates
│   │   │   ├── Motor_Calibration.c/h          ## calibration entry
│   │   │   ├── Motor_CalibrationElectrical.c  ## Rs, Ld, Lq identification
│   │   │   ├── Motor_Intervention.c/h         ## zero torque, ramp-down, safe stop
│   │   │   └── Motor_OpenLoop.c/h             ## alignment, open-loop start-up
│   │   ├── VBus/                              ## DC bus voltage, supply limits
│   │   ├── Motor.c/h                          ## Motor_T
│   │   ├── Motor_Clock.h                      ## time references
│   │   ├── Motor_Config.c/h                   ## configuration fields
│   │   ├── Motor_Electrical.h                 ## speed base, electrical parameters
│   │   ├── Motor_FOC.c/h                      ## FOC control loop
│   │   ├── Motor_Include.h                    ## all Motor headers
│   │   ├── Motor_StateMachine.c/h             ## motor states
│   │   ├── Motor_Thread.h                     ## thread entry points
│   │   ├── Motor_User.c/h                     ## user commands and status
│   │   └── Motor_Var.c/h                      ## variables by ID
│   ├── MotorController/                       ## system controller
│   │   ├── AppTable/                          ## operating mode table
│   │   ├── MotAnalogUser/                     ## throttle, brake, shifter, option pins
│   │   ├── MotBuzzer/                         ## buzzer patterns
│   │   ├── MotLimits/                         ## limit arbitration, disconnected
│   │   ├── MotNvm/                            ## save and load configuration
│   │   ├── Traction/                          ## traction operating mode
│   │   ├── MotorController.c/h                ## MotorController_T
│   │   ├── MotorController_Analog.c/h         ## ADC offset calibration
│   │   ├── MotorController_App.h              ## operating mode interface
│   │   ├── MotorController_Include.h          ## all controller headers
│   │   ├── MotorController_MotProtocol.c/h    ## serial request table
│   │   ├── MotorController_StateMachine.c/h   ## system states
│   │   ├── MotorController_Thread.h           ## thread entry points
│   │   ├── MotorController_User.c/h           ## user commands
│   │   └── MotorController_Var.c/h            ## variables by ID
│   ├── MotProtocol/                           ## wire protocols
│   │   ├── CANopen/                           ## object dictionary, SDO, PDO
│   │   ├── Cia402/                            ## CiA 402 drive profile
│   │   ├── MotCan/                            ## motor variables over SDO
│   │   ├── MotVarId/                          ## variable IDs shared with host tools
│   │   ├── MotPacket.c/h                      ## serial packet format
│   │   └── MotProtocol.c/h                    ## serial requests, flash loader
│   └── Version.h                              ## library version
├── Peripheral/                                ## peripheral abstraction
│   ├── ADC/                                   ## converters, channel sets, conversions
│   ├── CAN/                                   ## CAN driver and services
│   ├── ClockTimer/                            ## hardware timer
│   ├── HAL/                                   ## HAL path selection
│   │   └── Platform/                          ## register access per MCU
│   │       └── KE0x/                          ## Kinetis KE0x
│   ├── NvMemory/                              ## non-volatile memory
│   │   ├── EEPROM/                            ## EEPROM driver
│   │   ├── Flash/                             ## Flash driver
│   │   └── NvMemory/                          ## common operations
│   ├── Pin/                                   ## GPIO pin
│   ├── PWM/                                   ## PWM output
│   ├── SPI/                                   ## SPI master
│   ├── UART/                                  ## buffered UART
│   └── Xcvr/                                  ## message transport interface
├── System/                                    ## Cortex-M core services
│   ├── BootRef/                               ## bootloader shared reference
│   ├── Critical/                              ## critical sections
│   ├── Reboot/                                ## reset
│   └── SysTime/                               ## SysTick millisecond time
└── Transducer/                                ## sensors and actuators
    ├── Blinky/                                ## indicator pin patterns
    ├── Encoder/                               ## quadrature decode, M/T speed
    ├── Monitor/                               ## threshold monitors
    │   ├── Base/                              ## range monitor, derate
    │   ├── Heat/                              ## thermistor temperature
    │   └── Voltage/                           ## voltage divider
    ├── Pulse/                                 ## pulse edge timing
    └── UserIn/                                ## analog and digital user inputs
```

