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

#include "Type/Array/LimitArray/LimitArray.h"

#define SPEED_LIMIT_ID_COUNT (3U)
#define I_LIMIT_ID_COUNT (4U)

typedef enum MotILimitId
{
    MOT_I_LIMIT_HEAT_MC,        /* thermal — MCU/PCB */
    MOT_I_LIMIT_HEAT_MOTOR,     /* thermal — per-motor winding */
    MOT_I_LIMIT_V_LOW,          /* power — VBus undervoltage droop */
    MOT_I_LIMIT_USER,           /* user / protocol */
    MOT_I_LIMIT_COUNT,
}
MotILimitId_T;

typedef enum MotIGenLimitId
{
    MOT_I_GEN_LIMIT_HEAT_MC,     /* thermal caps regen symmetrically */
    MOT_I_GEN_LIMIT_HEAT_MOTOR,
    MOT_I_GEN_LIMIT_V_HIGH,      /* power — VBus overvoltage regen chop */
    MOT_I_GEN_LIMIT_USER,        /* user regen strength */
    MOT_I_GEN_LIMIT_COUNT,
}
MotIGenLimitId_T;

typedef enum MotSpeedLimitId
{
    MOT_SPEED_LIMIT_MC,          /* generic system cap */
    MOT_SPEED_LIMIT_V_BUS,       /* power — back-EMF ceiling from VBus */
    MOT_SPEED_LIMIT_USER,
    MOT_SPEED_LIMIT_COUNT,
}
MotSpeedLimitId_T;


#endif