



struct defines the shape of memory, module design wraps semantics
add semantics: "types" of structs by purpose and access patterns

static polymorphism context / compile time configuration constants
    TimerT,
HAL wrapper - configuration mix in to register access
    PWM_T

Runtime Configuration - runtime selectable configuration. mostly transparent data - in user domain units or internal domain units?
    MotorController_Config_T

Runtime optimization - derived
    Angle_SpeedFractRef_T
    Linear_T

General runtime model - most common
    Motor_State_T
    Angle_T

Data Transfer Interface - data is the interface

    almost primitive - form of 2 register or less pass by value or static inline
        foc_abc

    Transparent Data / Data Model
        Phase_Data_T

    Packets - also transparent data
        meta data encoded in struct shape.
        placement offset, size, matters

