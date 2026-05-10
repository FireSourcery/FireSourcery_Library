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
    @file   Motor_OpenLoopStart.h
    @author FireSourcery
    @brief  Open-loop

*/
/******************************************************************************/
#include "Math/Fixed/fract16.h"
#include "Math/Angle/Angle.h"
#include "Math/Ramp/Ramp.h"

#include <stdint.h>
#include <stdbool.h>


typedef struct
{
    uint16_t SpeedFinal_Fract16;
    uint32_t SpeedTime_Cycles;      /* Time to reach OpenLoopSpeed */
    uint16_t IFinal_Fract16;
    uint32_t ITime_Cycles;          /* Time to reach OpenLoopI */
    angle16_t SpeedHandover;        /* min ω̂_e in angle16/poll for handover */
}
Motor_OpenLoopProfile_T;

typedef struct Motor_OpenLoopRun
{
    Angle_T Angle;          /* synthesised θ; .Angle high 16b is the angle16 */
    Ramp_T  SpeedRamp;      /* output: ω in angle16/poll */
    Ramp_T  IRamp;          /* output: I_ref in fract16 */
}
Motor_OpenLoopRun_T;

/******************************************************************************/
/*!
    Lifecycle
*/
/******************************************************************************/
extern void Motor_OpenLoopStart_Init(const Motor_OpenLoopRun_T * p_open);
extern void Motor_OpenLoopStart_Reset(const Motor_OpenLoopRun_T * p_open);

/* Phase configuration — switch ramp targets without resetting accumulated state. */
extern void Motor_OpenLoopStart_EnterAlign(const Motor_OpenLoopRun_T * p_open);
extern void Motor_OpenLoopStart_EnterRamp(const Motor_OpenLoopRun_T * p_open);


/******************************************************************************/
/*!
    Per-control-tick step. Advances the speed and current ramps and integrates
    speed into the synthesised angle. Returns the integrated angle.
*/
/******************************************************************************/
// angle16_t Motor_OpenLoopStart_Step(Motor_OpenLoopRun_T * p_open)
// {

//     Ramp_ProcNext(&p->SpeedRamp);
//     Ramp_ProcNext(&p->CurrentRamp);
//     return Angle_Integrate(&p->Angle, (angle16_t)Ramp_GetOutput(&p->SpeedRamp));
// }

// void Motor_OpenLoopStart_Reset(Motor_OpenLoopRun_T * p_open)
// {

//     Angle_ZeroCaptureState(&p->Angle);
//     Ramp_SetOutputState(&p->SpeedRamp, 0);
//     Ramp_SetOutputState(&p->CurrentRamp, 0);
//     Ramp_SetTarget(&p->SpeedRamp, 0);
//     Ramp_SetTarget(&p->CurrentRamp, 0);
// }

// void Motor_OpenLoopStart_Init(Motor_OpenLoopRun_T * p_open)
// {
//     const Motor_OpenLoopStart_Config_T * cfg = p_open->P_CONFIG;
//     p->SpeedRamp = make_ramp(cfg->SpeedRampCoef);
//     p->CurrentRamp = make_ramp(cfg->CurrentRampCoef);
//     Motor_OpenLoopStart_Reset(p_open);
// }

// /* Hold rotor at θ frozen, ramp current up to alignment level. */
// void Motor_OpenLoopStart_EnterAlign(Motor_OpenLoopRun_T * p_open)
// {

//     Ramp_SetTarget(&p->SpeedRamp, 0);
//     Ramp_SetTarget(&p->CurrentRamp, p_open->P_CONFIG->IAlign);
// }

// /* Begin spinning — ramp ω up to handover speed, current to open-loop level. */
// void Motor_OpenLoopStart_EnterRamp(Motor_OpenLoopRun_T * p_open)
// {

//     Ramp_SetTarget(&p->SpeedRamp, p_open->P_CONFIG->SpeedHandover);
//     Ramp_SetTarget(&p->CurrentRamp, p_open->P_CONFIG->IOpen);
// }


// /******************************************************************************/
// /*!
//     Outputs / state queries
// */
// /******************************************************************************/
// static inline angle16_t Motor_OpenLoopStart_GetAngle(const Motor_OpenLoopRun_T * p_open) { return Angle_Value(&p_open->Angle); }
// static inline angle16_t Motor_OpenLoopStart_GetSpeed(const Motor_OpenLoopRun_T * p_open) { return (angle16_t)Ramp_GetOutput(&p_open->SpeedRamp); }
// static inline fract16_t Motor_OpenLoopStart_GetCurrent(const Motor_OpenLoopRun_T * p_open) { return (fract16_t)Ramp_GetOutput(&p_open->IRamp); }

// /* True once both ramps have reached their current targets. */
// static inline bool Motor_OpenLoopStart_IsRampComplete(const Motor_OpenLoopRun_T * p_open)
// {
//     return (Ramp_GetOutput(&p_open->SpeedRamp) == Ramp_GetTarget(&p_open->SpeedRamp)) && (Ramp_GetOutput(&p_open->IRamp) == Ramp_GetTarget(&p_open->IRamp));
// }
