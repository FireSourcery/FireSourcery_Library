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
    @file   MotDrive.h
    @author FireSourcery
    @brief  Drive-input primitives common across behavior modes.
*/
/******************************************************************************/
#include "../../MotAnalogUser/Pedals/Pedals.h"
#include "../../MotAnalogUser/Shifter/Shifter.h"
#include "../../MotAnalogUser/MotAnalogUser.h"
#include "Motor/Motor/Motor_User.h"  /* Motor_Direction_T */
#include "Math/math_general.h"       /* sign_t */
#include <stdint.h>
#include <stdbool.h>

typedef struct MotDrive_Config
{
    uint8_t RequireZeroOnEntry : 1U;
}
MotDrive_Config_T;

typedef struct MotDrive_State
{
    bool ThrottleZeroed;
    MotDrive_Config_T Config;
}
MotDrive_State_T;

typedef const struct MotDrive
{
    MotDrive_State_T * P_STATE;
    const MotDrive_Config_T * P_NVM_CONFIG;
}
MotDrive_T;