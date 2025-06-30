
#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery

    This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
/******************************************************************************/
/******************************************************************************/
/*!
    @file   MotAnalogUser_Conversion.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/

#include "Peripheral/Analog/Analog.h"

typedef const struct MotAnalogUser_Conversion
{
    const Analog_Conversion_T THROTTLE;
    const Analog_Conversion_T BRAKE;
}
MotAnalogUser_Conversion_T;

static inline void MotAnalogUser_Conversion_Mark(const MotAnalogUser_Conversion_T * p_this)
{
    Analog_Conversion_Mark(&p_this->THROTTLE);
    Analog_Conversion_Mark(&p_this->BRAKE);
}

static inline uint16_t MotAnalogUser_Conversion_GetThrottle(const MotAnalogUser_Conversion_T * p_const) { return Analog_Conversion_GetResult(&p_const->THROTTLE); }
static inline uint16_t MotAnalogUser_Conversion_GetBrake(const MotAnalogUser_Conversion_T * p_const) { return Analog_Conversion_GetResult(&p_const->BRAKE); }