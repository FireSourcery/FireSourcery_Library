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
    @file   PulseTimer_Counter.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "PulseTimer.h"
#include "Math/Angle/AngleCounter.h"

/******************************************************************************/
/*
    Edge ISR — call on every pulse edge
    Captures DeltaT on the timer, accumulates the counter.
    @param[in] sign  { -1, 0, +1 } — direction of this edge (resolved by caller)
*/
/******************************************************************************/
/* Count only. Angle supplied externally (sector LUT, Hall snap, etc.) */
static inline void PulseTimer_Counter_CaptureEdge(const PulseTimer_T * p_timer, AngleCounter_T * p_counter, int sign)
{
    PulseTimer_CaptureEdge(p_timer);
    AngleCounter_CaptureCount(p_counter, sign);
}

/* Count + wrap angle. Angle derived from counter (Timer-style) */
static inline void PulseTimer_Counter_CaptureEdgeWrap(const PulseTimer_T * p_timer, AngleCounter_T * p_counter, int sign)
{
    PulseTimer_CaptureEdge(p_timer);
    AngleCounter_CaptureCountAngle(p_counter, sign);
}

static inline void PulseTimer_Counter_CaptureFreq(const PulseTimer_T * p_timer, AngleCounter_T * p_counter)
{
    AngleCounter_CaptureFreq(p_counter, PulseTimer_CaptureSampleTk_Freq(p_timer));
}