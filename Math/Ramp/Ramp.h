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
    @file   Ramp.h
    @author FireSourcery
    @version V0
    @brief
*/
/******************************************************************************/
#ifndef RAMP_H
#define RAMP_H

#include "../Accumulator/Accumulator.h"
#include <stdint.h>
#include <stdbool.h>

/* Aliases */
typedef Accumulator_T Ramp_T;

/******************************************************************************/
/*

*/
/******************************************************************************/
static inline int32_t Ramp_GetTarget(const Ramp_T * p_ramp) { return (p_ramp->Target >> p_ramp->Shift); }
static inline int32_t Ramp_GetOutput(const Ramp_T * p_ramp) { return (p_ramp->State >> p_ramp->Shift); }

static inline void Ramp_SetTarget(Ramp_T * p_ramp, int32_t target)
{
    p_ramp->Target = (target << p_ramp->Shift);
    if (p_ramp->Coefficient == 0) { p_ramp->State = p_ramp->Target; }
}

static inline void Ramp_SetOutputState(Ramp_T * p_ramp, int32_t match)
{
    p_ramp->Target = (match << p_ramp->Shift);
    p_ramp->State = p_ramp->Target;
}

static inline void Ramp_ZeroOutputState(Ramp_T * p_ramp)
{
    p_ramp->Target = 0;
    p_ramp->State = 0;
}

static inline bool Ramp_IsDisabled(const Ramp_T * p_ramp) { return (p_ramp->Coefficient == 0); }

static inline void Ramp_Disable(Ramp_T * p_ramp) { p_ramp->Coefficient = 0; }

/******************************************************************************/
/*
    Extern
*/
/******************************************************************************/
extern int32_t Ramp_ProcOutputN(Ramp_T * p_ramp, int32_t steps);
extern int32_t Ramp_ProcOutput(Ramp_T * p_ramp);

extern void Ramp_Init(Ramp_T * p_ramp, uint32_t duration_Ticks, int32_t range);
extern void Ramp_Init_Millis(Ramp_T * p_ramp, uint32_t updateFreq_Hz, uint16_t duration_Ms, int32_t range);
extern void Ramp_SetSlope(Ramp_T * p_ramp, uint32_t duration_Ticks, int32_t range);
extern void Ramp_SetSlope_Millis(Ramp_T * p_ramp, uint32_t updateFreq_Hz, uint16_t duration_Ms, int32_t range);

extern void Ramp_Set(Ramp_T * p_ramp, uint32_t duration_Ticks, int32_t initial, int32_t final);
extern void Ramp_Set_Millis(Ramp_T * p_ramp, uint32_t updateFreq_Hz, uint16_t duration_Ms, int32_t initial, int32_t final);

#endif


