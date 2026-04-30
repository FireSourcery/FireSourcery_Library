#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2026 FireSourcery

    This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
*/
/******************************************************************************/
/******************************************************************************/
/*!
    @file   SwitchBrake.h
    @author FireSourcery
    @brief  Digital "handbrake" augment. When the pin is active, the module
            reports a NVM-configured Percent16 brake floor that a policy layer
            can fuse (max) with the analog brake pedal value.

            Pure pin owner — no arbitration or drive-state awareness.
*/
/******************************************************************************/
#include "../../MotorController_StateMachine.h"

#include <stdint.h>
#include <stdbool.h>

typedef struct SwitchBrake_Config
{
    uint16_t Value_Percent16;   /* Brake value applied while pin is active, [0:65535] */
    bool  IsEnabled;    /* NVM runtime enable (SKU has pin but unused) */
}
SwitchBrake_Config_T;

static inline void _MotorController_CallSwitchBrakePin(MotorController_T * p_dev, SwitchBrake_Config_T * p_config, UserDIn_Edge_T edge)
{
    switch (edge)
    {
        // case USER_DIN_EDGE_RISING: math_max(_MotAnalogUser_GetBrake(&p_dev->ANALOG_USER), p_config->Value_Percent16); break;
        // case USER_DIN_EDGE_FALLING: MotorController_EnterMain(p_dev); break;
        // default: break;
    }
}

static inline void  MotorController_CallSwitchBrakePin(MotorController_T * p_dev,  UserDIn_Edge_T edge)
{
    // _MotorController_CallSwitchBrakePin(p_dev, &p_dev->P_NVM_CONFIG->ANALOG_USER_SWITCH_BRAKE_CONFIG, edge);
}

// static inline UserDIn_T * MotorController_SwitchBrakePin(const MotorController_T * p_dev) { return (UserDIn_T *)&p_dev->OPT_DIN; }

// static inline void MotorController_ProcSwitchBrakePin(const MotorController_T * p_dev)
// {
//     UserDIn_PollEdge(MotorController_SwitchBrakePin(p_dev));
// }