# FireSourcery Library
**Embedded C library** with a **Motor Control** stack for bare-metal MCU targets.
A field-oriented control stack built from layers that also stand alone.

C23, `arm-none-eabi-gcc`, Cortex-M0+ and Cortex-M4. No RTOS, no heap after init, no `stdio`. Fixed-point at runtime. In-tree platform: Kinetis KE0x.

- **Modular** — independently functional modules with minimized coupling.
- **Portable** — hardware-independent core; chip specifics live behind a HAL.
- **Reusable** — generic C primitives composed into higher-level behaviors.
- **OOP in C** — encapsulation, const correctness, clear ownership of state.
- **Disciplined** — strong naming conventions, layered design, minimal duplication.

## Architecture
[ARCHITECTURE.md](ARCHITECTURE.md) covers the full feature set, design patterns, layer and dependency diagrams, and integration.


## Motor Control Features
- Field-Oriented Control (FOC).
- Sensor support: Hall, Quadrature Encoder, Sine/Cosine.
- Speed and current feedback loops.
- Layered control behaviors built up from shared primitives.
- Standard wire protocol for GUI / host tooling, with an Arduino-compatible wrapper.
-
## Layout
### Motor — Motor control stack
- `Motor/Motor` — per-motor functions and state for operating a single motor.
- `Motor/MotorController` — multi-motor coordination, user I/O, communication.
- `Motor/MotProtocol` — wire protocol for host/GUI integration.

### Peripheral — Peripheral abstraction
Common peripheral interface called by upper layers: `Analog`, `CAN`, `ClockTimer`, `NvMemory`, `PWM`, `Pin`, `SPI`, `Serial`, `Xcvr`.
- `Peripheral/HAL` — hardware abstraction layer; register-level code per chip.

### System
Common interface for system-level functions; thin abstraction over remaining hardware dependencies.

### Transducer — Sensor & actuator algorithms
`Encoder`, `Monitor`, `Pulse`, `UserIn`, `Blinky`. Sits on top of `Peripheral`.

### Generic layers
- `Math` — numerical value: `Fixed`, `Linear`, `PID`, `Ramp`, `Filter`, `Angle`, `Accumulator`, `Hysteresis`, `Threshold`.
- `Framework` — software tools and algorithms independent of hardware (e.g. `StateMachine`, `Protocol`).
- `Framework/Type` — shape of memory; type definitions and containers.

## Further reading
- [State machine architecture](Framework/StateMachine/Architecture.md): state tree, transitions, inputs, threading, adding a state
- [Protocol engine](Framework/Protocol/README.md): receive pipeline and request flow traces
- [MotVar over CANopen SDO](Motor/MotProtocol/MotCan/README.md): object dictionary contract and frame layout
- [Fixed-point quantities](Motor/Docs/FixedPoint.md): representation, units, naming
- [Per-unit electrical bases](Motor/Motor/Math/motor_electrical_math.md): ψ and L scaling
- [ADC conversion sets](Peripheral/ADC/Notes.md): coordinating conversions across converters
- [Config, State, Ref](Docs/Conventions/Config_State_Ref.md): one writable copy per persisted value

## License
GPL-3.0-or-later. See [LICENSE](LICENSE).
