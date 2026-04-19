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
    @file   Motor_Limits.h
    @author FireSourcery
    @brief  Motor-local arbitration sources (per-motor scope).

    Complements system-scope MotLimits.h. Per-motor sources write to a
    Motor-owned LimitArray so one motor's derate does not broadcast to
    peers sharing the same controller.
*/
/******************************************************************************/
#include "Type/Array/LimitArray/LimitArray.h"

/* Per-motor motoring-current sources */
typedef enum Motor_ILimitId
{
    MOTOR_I_LIMIT_HEAT_WINDING,     /* this motor's winding NTC / I^2t */
    MOTOR_I_LIMIT_STALL,            /* startup / lock current cap */
    MOTOR_I_LIMIT_COUNT,
}
Motor_ILimitId_T;

/* Per-motor generating-current sources */
typedef enum Motor_IGenLimitId
{
    MOTOR_I_GEN_LIMIT_HEAT_WINDING,
    MOTOR_I_GEN_LIMIT_COUNT,
}
Motor_IGenLimitId_T;

/* Per-motor speed sources */
typedef enum Motor_SpeedLimitId
{
    MOTOR_SPEED_LIMIT_FIELD_WEAKEN, /* optional per-motor FW clamp */
    MOTOR_SPEED_LIMIT_COUNT,
}
Motor_SpeedLimitId_T;
