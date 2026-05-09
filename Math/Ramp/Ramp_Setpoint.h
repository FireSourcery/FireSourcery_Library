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
    @file   Ramp_Setpoint.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Ramp.h"
#include "../PID/PID.h"

static inline int32_t Ramp_Setpoint_ProcControl(Ramp_T * p_ramp, PID_T * p_pid, int16_t feedback) { return PID_ProcPI(p_pid, feedback, Ramp_ProcNext(p_ramp)); }
static inline int32_t Ramp_Setpoint_ProcControlSource(Ramp_T * p_ramp, PID_T * p_pid, int16_t feedback, int16_t target) { return PID_ProcPI(p_pid, feedback, Ramp_ProcNextOnInputOf(p_ramp, target)); }

/* clamp internal state */
static inline void Ramp_Setpoint_SetOutputLimits(Ramp_T * p_ramp, PID_T * p_pid, int32_t min, int32_t max)
{
    Ramp_SetOutputLimit(p_ramp, min, max);
    PID_SetOutputLimits(p_pid, min, max);
}

// ramp contains setpoint state single surface for limits
// static inline int32_t Ramp_Setpoint_ProcControl_(Ramp_T * p_ramp, PID_T * p_pid, int16_t feedback)
// {
//     PID_CaptureOutputLimits(p_pid, Ramp_GetLimitLower(p_ramp), Ramp_GetLimitUpper(p_ramp));
//     return Ramp_Setpoint_ProcControl(p_ramp, p_pid, feedback);
// }