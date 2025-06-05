/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

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
    @file   .h
    @author FireSourcery

    @brief  Limits Input
*/
/******************************************************************************/
#ifndef MOT_LIMITS_H
#define MOT_LIMITS_H

#include "Utility/LimitArray/LimitArray.h"

#define SPEED_LIMIT_ID_COUNT (2U)
#define I_LIMIT_ID_COUNT (4U)

typedef enum MotILimit_Id
{
    /* Heat Limits are polling, optionally share 1 Id */
    // MOT_I_LIMIT_HEAT_MOTOR,    /* MotorHeat */
    MOT_I_LIMIT_HEAT_MC,       /* From upper module */
    MOT_I_LIMIT_V_LOW,
    MOT_I_LIMIT_USER,
}
MotILimit_Id_T;

/* SpeedLimitId */
typedef enum MotSpeedLimit_Id
{
    MOT_SPEED_LIMIT_MC,    /* From upper module */
    MOT_SPEED_LIMIT_USER,
}
MotSpeedLimit_Id_T;

// typedef struct
// {
//     LimitArray_T SpeedLimit; /* Speed Limit */
//     LimitArray_T ILimit;     /* I Limit */
// }
// MotLimits_T;

#define MOT_LIMITS_INIT(p_this)                         \
{                                                       \
    .SpeedLimit = LIMIT_ALLOC(SPEED_LIMIT_ID_COUNT),    \
    .ILimit = LIMIT_ALLOC(I_LIMIT_ID_COUNT),            \
};

// #define MOT_SPEED_LIMITS_INIT(p_this) LIMIT_ALLOC(SPEED_LIMIT_ID_COUNT)
// #define MOT_I_LIMITS_INIT(p_this) LIMIT_ALLOC(I_LIMIT_ID_COUNT)


#endif