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
#include "Framework/LimitArray/LimitArray.h"

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


// /*
//     Motor-local arbitration buffers. Handles live in Motor_T (P_I_LIMITS_LOCAL, etc.).
//     Per-motor sources (winding heat, stall, field-weaken) write here so a single
// */
// typedef struct Motor_Limits
// {
//     limit_t                 ILimits[MOTOR_I_LIMIT_COUNT];
//     LimitArray_Augments_T   ILimitsActive;
//     limit_t                 IGenLimits[MOTOR_I_GEN_LIMIT_COUNT];
//     LimitArray_Augments_T   IGenLimitsActive;
//     limit_t                 SpeedLimits[MOTOR_SPEED_LIMIT_COUNT];
//     LimitArray_Augments_T   SpeedLimitsActive;

// } Motor_Limits_T;



// void Motor_SetSpeedLimitWith(Motor_SpeedControl_T * p_motor, Motor_Limits_T * p_motor, LimitArray_T * p_system)
// {
//     // if (LimitArray_IsUpperActive(p_local) == true) { Motor_TrySpeedLimit(p_motor, LimitArray_Upper(p_local)); }
//     // else { Motor_TryClearSpeedLimit(p_motor); }
//     Motor_SetSpeedLimits(p_motor, math_min(LimitArray_Upper(p_system), LimitArray_Upper(p_local)));
// }
