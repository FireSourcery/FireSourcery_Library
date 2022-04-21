# FireSourcery_Library

A **General Embedded C Library** and **Motor Control Library**

## Overview and Features 

* Modular Design  
Independently functional modules. Minimized dependency / coupling of modules.  
Hardware independent structure, using abstraction layers.  
Generalized reusable C code.  
OOP principles via encapsulation in C.  
Following good coding practices, strong naming conventions, minimal code redundancy, design principles/patterns.  

### Directory Contents Overview

* Math - General math functions

* Motor - Motor Control Library
  - Motor - Functions and state information for the operation of a single unique motor.
  - Motor Controller - Functions and state information common to all motors, e.g. handling of communication, user input/outputs. 
		
* Peripheral - Peripheral abstraction layer. Common peripheral interface called by upper layers. 
  - HAL - Hardware abstraction layer. Handling of hardware register, per chip implementation.
 	
* System - Common interface for system functions. Abstraction layer over some hardware dependencies. 

* Transducers - Sensors and Actuators algorithms layer. Utilizes Peripheral layer. 
	
* Utility - Software tools and algorithms independent of hardware. 
