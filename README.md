# FireSourcery_Library

A General Embedded C Library and Motor Control Library

Overview and Features:

Modular Design - Independently functional 
Following good coding practices, strong n

Follow OOP principles via encapsulation in C


Math: General math functions

Motor
- Motor: Algorithms and state information for the operation of a single unique motor.
- Motor Controller: Functions and state common to all motors, e.g. handling of communication, user input/outputs. 
		
Peripheral: Peripheral abstraction layer. Peripheral algorithms. Common peripheral interface called by upper layers. 
- HAL: Hardware abstraction layer. Handling of hardware register. Per chip implementation.
 	
System: Common interface for system functions. Abstraction layer over some hardware dependencies. 

Transducers: Sensors and Actuators algorithm layer. Calls Peripheral layer. 
	
Utility: Software tools and algorithms independent of hardware.