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

#include "Framework/LimitArray/LimitArray.h"

/*
    System-scope arbitration ids. Per-motor sources (winding heat, stall,
    field-weakening) live in Motor_Limits.h — never add them here or a single
    motor's derate would broadcast to all motors sharing this controller.
*/
typedef enum MotILimitId
{
    MOT_I_LIMIT_HEAT_MC,        /* thermal — MCU/PCB (shared) */
    MOT_I_LIMIT_V_LOW,          /* power — VBus undervoltage droop */
    MOT_I_LIMIT_USER,           /* user / protocol */
    MOT_I_LIMIT_COUNT,
}
MotILimitId_T;

typedef enum MotIGenLimitId
{
    MOT_I_GEN_LIMIT_HEAT_MC,     /* thermal caps regen symmetrically */
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


typedef struct
{
    LimitArray_Augments_T ILimitState;
    limit_t ILimitValues[MOT_I_LIMIT_COUNT];
    LimitArray_Augments_T IGenLimitState;
    limit_t IGenLimitValues[MOT_I_GEN_LIMIT_COUNT];
    LimitArray_Augments_T SpeedLimitState;
    limit_t SpeedLimitValues[MOT_SPEED_LIMIT_COUNT];
}
MotLimits_T;

static inline uint16_t MotLimits_IDerate(MotLimits_T * p_dev) { return _LimitArray_Upper(&p_dev->ILimitState); }
static inline uint16_t MotLimits_IGenDerate(MotLimits_T * p_dev) { return _LimitArray_Upper(&p_dev->IGenLimitState); }
static inline uint16_t MotLimits_SpeedDerate(MotLimits_T * p_dev) { return _LimitArray_Upper(&p_dev->SpeedLimitState); }


// for composition
// static inline LimitArray_T MotLimits_GetILimitArray(MotLimits_T * p_dev) { return (LimitArray_T) { .P_BUFFER = &p_dev->ILimitValues[0U], .LENGTH = MOT_I_LIMIT_COUNT, .P_AUGMENTS = &p_dev->ILimitState, }; }
// static inline LimitArray_T MotLimits_GetIGenLimitArray(MotLimits_T * p_dev) { return (LimitArray_T) { .P_BUFFER = &p_dev->IGenLimitValues[0U], .LENGTH = MOT_I_GEN_LIMIT_COUNT, .P_AUGMENTS = &p_dev->IGenLimitState, }; }
// static inline LimitArray_T MotLimits_GetSpeedLimitArray(MotLimits_T * p_dev) { return (LimitArray_T) { .P_BUFFER = &p_dev->SpeedLimitValues[0U], .LENGTH = MOT_SPEED_LIMIT_COUNT, .P_AUGMENTS = &p_dev->SpeedLimitState, }; }


// bool _MotorController_SetSpeedLimitAll(MotLimits_T * p_dev, Motor_Table_T * p_motors, MotSpeedLimitId_T id, limit_t speed_fract16)
// {
//     if (_LimitArray_TestSetUpper(&p_dev->SpeedLimitState, p_dev->SpeedLimitValues, id, speed_fract16) == true) { Motor_Table_ApplySpeedLimit(p_motors, &p_dev->SpeedLimitState); return true; }
//     return false;
// }




#endif