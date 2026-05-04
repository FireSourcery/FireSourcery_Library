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

    Slot values are unitless Q15 ufract16 derate factors in [0, FRACT16_MAX] ≡ [0, 1] of rated.
    Sources push their derate directly (no rated multiplication at the source).
    Active derate = LimitArray_Upper(...) = min of stored derates = most-restrictive source.
    Consumer multiplies by rated once: physical = fract16_mul(rated, active_derate).
*/
typedef enum MotILimitId
{
    MOT_I_LIMIT_HEAT_MC,        /* thermal — MCU/PCB (shared, both directions) */
    MOT_I_LIMIT_V_BUS,          /* power — VBus undervoltage droop OR overvoltage chop (mutually exclusive) */
    MOT_I_LIMIT_USER,           /* user / protocol (single derate ratio) */
    MOT_I_LIMIT_COUNT,
}
MotILimitId_T;

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
    LimitArray_Augments_T SpeedLimitState;
    limit_t SpeedLimitValues[MOT_SPEED_LIMIT_COUNT];
}
MotLimits_T;

static inline uint16_t MotLimits_IDerate(MotLimits_T * p_limits) { return _LimitArray_Upper(&p_limits->ILimitState); }
static inline uint16_t MotLimits_SpeedDerate(MotLimits_T * p_limits) { return _LimitArray_Upper(&p_limits->SpeedLimitState); }

static inline void MotLimits_ClearSpeedDerate(MotLimits_T * p_limits) { _LimitArray_ClearAll(&p_limits->SpeedLimitState, p_limits->SpeedLimitValues, MOT_SPEED_LIMIT_COUNT); }
static inline void MotLimits_ClearIDerate(MotLimits_T * p_limits) { _LimitArray_ClearAll(&p_limits->ILimitState, p_limits->ILimitValues, MOT_I_LIMIT_COUNT); }


// for composition
// static inline LimitArray_T MotLimits_GetILimitArray(MotLimits_T * p_limits) { return (LimitArray_T) { .P_BUFFER = &p_limits->ILimitValues[0U], .LENGTH = MOT_I_LIMIT_COUNT, .P_AUGMENTS = &p_limits->ILimitState, }; }
// static inline LimitArray_T MotLimits_GetIGenLimitArray(MotLimits_T * p_limits) { return (LimitArray_T) { .P_BUFFER = &p_limits->IGenLimitValues[0U], .LENGTH = MOT_I_GEN_LIMIT_COUNT, .P_AUGMENTS = &p_limits->IGenLimitState, }; }
// static inline LimitArray_T MotLimits_GetSpeedLimitArray(MotLimits_T * p_limits) { return (LimitArray_T) { .P_BUFFER = &p_limits->SpeedLimitValues[0U], .LENGTH = MOT_SPEED_LIMIT_COUNT, .P_AUGMENTS = &p_limits->SpeedLimitState, }; }

// bool MotLimits_SetSpeedLimitAll(MotLimits_T * p_limits, Motor_Table_T * p_motors, MotSpeedLimitId_T id, limit_t speed_fract16)
// {
//     if (_LimitArray_TestSetUpper(&p_limits->SpeedLimitState, p_limits->SpeedLimitValues, id, speed_fract16) == true) { Motor_Table_ApplySpeedLimit(p_motors, &p_limits->SpeedLimitState); return true; }
//     return false;
// }

// bool MotLimits_SetSpeedLimit(MotLimits_T * p_limits, MotSpeedLimitId_T id, limit_t speed_fract16)
// {
//     if (_LimitArray_TestSetUpper(&p_limits->SpeedLimitState, p_limits->SpeedLimitValues, id, speed_fract16) == true)
//     { Motor_Table_ApplySpeedLimit(&p_limits->MOTORS, &p_limits->SpeedLimitState); return true; }
//     return false;
// }

// bool MotLimits_ClearSpeedLimit(MotLimits_T * p_limits, MotSpeedLimitId_T id)
// {
//     if (_LimitArray_TestClearEntry(&p_limits->SpeedLimitState, p_limits->SpeedLimitValues, MOT_SPEED_LIMIT_COUNT, id) == true)
//     { Motor_Table_ApplySpeedLimit(&p_limits->MOTORS, &p_limits->SpeedLimitState); return true; }
//     return false;
// }

// bool MotLimits_SetILimit(MotLimits_T * p_limits, MotILimitId_T id, limit_t i_fract16)
// {
//     if (_LimitArray_TestSetUpper(&p_limits->ILimitState, p_limits->ILimitValues, id, i_fract16) == true)
//     { Motor_Table_ApplyILimit(&p_limits->MOTORS, &p_limits->ILimitState); return true; }
//     return false;
// }

// bool MotLimits_ClearILimit(MotLimits_T * p_limits, MotILimitId_T id)
// {
//     if (_LimitArray_TestClearEntry(&p_limits->ILimitState, p_limits->ILimitValues, id) == true)
//     { Motor_Table_ApplyILimit(&p_limits->MOTORS, &p_limits->ILimitState); return true; }
//     return false;
// }




#endif