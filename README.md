# FireSourcery_Library

A **General Embedded C Library** and **Motor Control Library**

## Overview and Features

* Modular Design
  - Independently functional modules. Minimized dependencies/coupling of modules.
  - Hardware independent structure, using abstraction layers.
  - Generalized reusable C code.
  - OOP principles via encapsulation in C.
  - Follows good coding practices. E.g strong naming conventions, minimal code duplication, design principles/patterns, const correctness.

* Motor Library Features
  - FOC.
  - Sensors: Hall, Quadrature Encoder, Sine Cosine.
  - Speed and current feedback loop.
  - Layered implementation of control behaviors with code reuse.
  - Standard protocol for GUI side. Arduino-compatible wrapper.

* TODO
  - Encoder HFI align
  - Sensorless
  - Input Devices: 1-D, 2-D, Joystick.
  - Misra compliance

### Directory Contents Overview

* Math - General math functions

* Motor - Motor Control Library
  - Motor - Functions and state information for the operation of a single unique motor.
  - MotorController - Functions and state information common to all motors, e.g. handling of communication, and user input/outputs.

* Peripheral - Peripheral abstraction layer. A common peripheral interface called by upper layers.
  - HAL - Hardware abstraction layer. Handling of hardware registers, per chip implementation.

* System - Common interface for system functions. Abstraction layer over some hardware dependencies.

* Transducers - Sensors and Actuators algorithms layer. Utilizes Peripheral layer.

* Utility - Software tools and algorithms independent of hardware.
