# FireSourcery Library

A general-purpose **Embedded C library** with a **Motor Control** stack for bare-metal MCU targets.

## Highlights

- **Modular** — independently functional modules with minimized coupling.
- **Portable** — hardware-independent core; chip specifics live behind a HAL.
- **Reusable** — generic C primitives composed into higher-level behaviors.
- **OOP in C** — encapsulation, const correctness, clear ownership of state.
- **Disciplined** — strong naming conventions, layered design, minimal duplication.

## Toolchain & Language

- **Language**: C23.
- **Compiler**: GCC ARM Embedded (`arm-none-eabi-gcc`).
- **Build**: GNU Make; PlatformIO supported via `library.json`.
- **Runtime**: bare metal, no RTOS required.
- **Constraints**: no dynamic allocation after init (`malloc`/`free` forbidden); no `stdio` (`printf` etc.); no blocking in the main loop.

## Architecture

App → Motor → Transducer → Peripheral → HAL → Hardware

## Motor Control Features

- Field-Oriented Control (FOC).
- Sensor support: Hall, Quadrature Encoder, Sine/Cosine.
- Speed and current feedback loops.
- Layered control behaviors built up from shared primitives.
- Standard wire protocol for GUI / host tooling, with an Arduino-compatible wrapper.

## Layout

### Motor — Motor control stack

- `Motor/Motor` — per-motor functions and state for operating a single motor.
- `Motor/MotorController` — multi-motor coordination, user I/O, communication.
- `Motor/MotProtocol` — wire protocol for host/GUI integration.

### Peripheral — Peripheral abstraction

Common peripheral interface called by upper layers: `Analog`, `CanBus`, `ClockTimer`, `NvMemory`, `PWM`, `Pin`, `SPI`, `Serial`, `Xcvr`.

- `Peripheral/HAL` — hardware abstraction layer; register-level code per chip.

### System

Common interface for system-level functions; thin abstraction over remaining hardware dependencies.

### Transducer — Sensor & actuator algorithms

`Encoder`, `Monitor`, `Pulse`, `UserIn`, `Blinky`. Sits on top of `Peripheral`.

### Generic layers

- `Math` — numerical value: `Fixed`, `Linear`, `PID`, `Ramp`, `Filter`, `Angle`, `Accumulator`, `Hysteresis`, `Threshold`.
- `Type` — shape of memory; type definitions and containers.
- `Framework` — software tools and algorithms independent of hardware (e.g. `StateMachine`, `Protocol`).

## License

GNU — see [LICENSE](LICENSE).
