// /******************************************************************************/
// /*!
//     @section LICENSE

//     Copyright (C) 2023 FireSourcery

//     This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).

//     This program is free software: you can redistribute it and/or modify
//     it under the terms of the GNU General Public License as published by
//     the Free Software Foundation, either version 3 of the License, or
//     (at your option) any later version.

//     This program is distributed in the hope that it will be useful,
//     but WITHOUT ANY WARRANTY; without even the implied warranty of
//     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//     GNU General Public License for more details.

//     You should have received a copy of the GNU General Public License
//     along with this program.  If not, see <https://www.gnu.org/licenses/>.
// */
// /******************************************************************************/
// /******************************************************************************/
// /*!
//     @file   .h
//     @author FireSourcery
//     @brief  Limits Input
// */
// /******************************************************************************/
// #ifndef MOT_LIMITS_H
// #define MOT_LIMITS_H

// #include "Framework/LimitArray/LimitArray.h"
// #include "Math/Fixed/fract16.h"

// /*
//     System-scope derate arbitration ids.
//     Slot values are unitless Q15 ufract16 derate factors in [0, FRACT16_MAX] ≡ [0, 1] of Config.
//     Sources push their derate directly (no rated multiplication at the source).
//     I arbitrates per quadrant, Motoring and Generating, indexed by the same id. A source writes the quadrants it constrains.
//     Active derate = LimitArray_Upper(...) = min of stored derates = most-restrictive source.
//     Pushed to each motor via Motor_Set*LimitDerate*; Motor compares against its physical value channel.
//     Physical value limits (user/OptDin/protocol) are per motor, not arbitrated here.
// */
// typedef enum MotILimitId
// {
//     MOT_I_LIMIT_HEAT_MC,        /* thermal — MCU/PCB (shared, both quadrants) */
//     MOT_I_LIMIT_V_BUS,          /* power — undervoltage droop on Motoring, overvoltage chop on Generating */
//     MOT_I_LIMIT_COUNT,
// }
// MotILimitId_T;

// typedef enum MotSpeedLimitId
// {
//     MOT_SPEED_LIMIT_MC,          /* generic system cap */
//     MOT_SPEED_LIMIT_V_BUS,       /* power — back-EMF ceiling from VBus */
//     MOT_SPEED_LIMIT_COUNT,
// }
// MotSpeedLimitId_T;

// typedef struct
// {
//     LimitArray_Augments_T ILimitMotoringState;
//     limit_t ILimitMotoringValues[MOT_I_LIMIT_COUNT];
//     LimitArray_Augments_T ILimitGeneratingState;
//     limit_t ILimitGeneratingValues[MOT_I_LIMIT_COUNT];
//     LimitArray_Augments_T SpeedLimitState;
//     limit_t SpeedLimitValues[MOT_SPEED_LIMIT_COUNT];
// }
// MotLimits_T;

// /* Derate = min(sources, 1.0). An empty array reads LIMIT_ARRAY_CLEAR, outside the Q15 slot domain */
// static inline uint16_t MotLimits_IDerateMotoring(MotLimits_T * p_limits) { return math_min(_LimitArray_Upper(&p_limits->ILimitMotoringState), FRACT16_MAX); }
// static inline uint16_t MotLimits_IDerateGenerating(MotLimits_T * p_limits) { return math_min(_LimitArray_Upper(&p_limits->ILimitGeneratingState), FRACT16_MAX); }
// static inline uint16_t MotLimits_SpeedDerate(MotLimits_T * p_limits) { return math_min(_LimitArray_Upper(&p_limits->SpeedLimitState), FRACT16_MAX); }

// static inline void MotLimits_ClearSpeedDerate(MotLimits_T * p_limits) { _LimitArray_ClearAll(&p_limits->SpeedLimitState, p_limits->SpeedLimitValues, MOT_SPEED_LIMIT_COUNT); }
// static inline void MotLimits_ClearIDerate(MotLimits_T * p_limits)
// {
//     _LimitArray_ClearAll(&p_limits->ILimitMotoringState, p_limits->ILimitMotoringValues, MOT_I_LIMIT_COUNT);
//     _LimitArray_ClearAll(&p_limits->ILimitGeneratingState, p_limits->ILimitGeneratingValues, MOT_I_LIMIT_COUNT);
// }


// // for composition
// // static inline LimitArray_T MotLimits_GetILimitMotoringArray(MotLimits_T * p_limits) { return (LimitArray_T) { .P_BUFFER = &p_limits->ILimitMotoringValues[0U], .LENGTH = MOT_I_LIMIT_COUNT, .P_AUGMENTS = &p_limits->ILimitMotoringState, }; }
// // static inline LimitArray_T MotLimits_GetILimitGeneratingArray(MotLimits_T * p_limits) { return (LimitArray_T) { .P_BUFFER = &p_limits->ILimitGeneratingValues[0U], .LENGTH = MOT_I_LIMIT_COUNT, .P_AUGMENTS = &p_limits->ILimitGeneratingState, }; }
// // static inline LimitArray_T MotLimits_GetSpeedLimitArray(MotLimits_T * p_limits) { return (LimitArray_T) { .P_BUFFER = &p_limits->SpeedLimitValues[0U], .LENGTH = MOT_SPEED_LIMIT_COUNT, .P_AUGMENTS = &p_limits->SpeedLimitState, }; }




// #endif