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
    @file   AnglePll.h
    @author FireSourcery
    @brief  Angle tracking loop. Companion to Angle_T — [Angle_T] is the integrator.

        phase error → PI → ω̂ → integrate → θ̂

    The caller's phase detector supplies the error, ≈ sin(θ̂ − θ). The PI regulates it to 0, so θ̂ lagging θ raises ω̂.
    Or the caller supplies a vector along θ, and ProcVector detects it against θ̂.
*/
/******************************************************************************/
#include "Angle.h"
#include "../PID/PID.h"

/******************************************************************************/
/*
    Phase detector. x points along the tracked angle θ, as a resolver's (cos θ, sin θ).
        |x|·sin(θ̂ − θ) = x·sin θ̂ − y·cos θ̂
*/
/******************************************************************************/
static inline accum32_t angle_pll_error(fract16_t x, fract16_t y, fract16_t sin, fract16_t cos) { return fract16_mul(x, sin) - fract16_mul(y, cos); }


static inline fract16_t angle_pll(ufract16_t mag_floor, fract16_t x, fract16_t y, fract16_t error)
{
    ufract16_t mag = fract16_vector_magnitude(x, y);
    return (mag <= mag_floor) ? 0 : fract16_sat(fract16_div(error, mag));
}

/* sin(θ̂ − θ), |x| stripped so the loop gain holds across amplitude. 0 at or below the floor, where θ is indeterminate */
static inline fract16_t angle_pll_error_normalized(ufract16_t mag_floor, fract16_t x, fract16_t y, fract16_t sin, fract16_t cos)
{
    // ufract16_t mag = fract16_vector_magnitude(x, y);
    // return (mag <= mag_floor) ? 0 : fract16_sat(fract16_div(angle_pll_error(x, y, sin, cos), mag));
    return angle_pll(mag_floor, x, y, angle_pll_error(x, y, sin, cos));
}

/* x already rotated into the frame of θ̂, so θ̂ = 0: sin(−∠x) */
static inline fract16_t angle_pll_error_normalized_relative(ufract16_t mag_floor, fract16_t x, fract16_t y)
{
    // ufract16_t mag = fract16_vector_magnitude(x, y);
    // return (mag <= mag_floor) ? 0 : fract16_sat(fract16_div(0 - (accum32_t)y, mag));
    return angle_pll(mag_floor, x, y, 0 - (accum32_t)y);
}

/*
    [AnglePll_T]
    Base.Delta is the PI output, overwritten each Proc. Seed sets ω̂ through the integrator.
*/
typedef struct AnglePll
{
    Angle_T Base;           /* Angle is θ̂. Delta is ω̂ [angle16/dt] */
    PID_T LoopFilter;       /* PI, phase error → ω̂ */
    fract16_t PhaseError;   /* last error */
}
AnglePll_T;


/******************************************************************************/
/*
    Per tick
*/
/******************************************************************************/
static inline angle16_t AnglePll_Proc(AnglePll_T * p_pll, fract16_t phaseError)
{
    p_pll->PhaseError = phaseError;
    return Angle_Integrate(&p_pll->Base, PID_ProcPI(&p_pll->LoopFilter, phaseError, 0));
}

/* x along θ, detected against θ̂ */
static inline angle16_t AnglePll_ProcVector(AnglePll_T * p_pll, ufract16_t magFloor, fract16_t x, fract16_t y)
{
    struct fract16_xy unit = Angle_UnitVector(&p_pll->Base);
    return AnglePll_Proc(p_pll, angle_pll_error_normalized(magFloor, x, y, unit.y, unit.x));
}

/******************************************************************************/
/*
    State
*/
/******************************************************************************/
/*
    Bumpless seed: θ̂ and ω̂ from a known reference, e.g. an open-loop ramp.
    The integrator holds ω̂, so the PI resumes from the seed rather than fighting it.
*/
static inline void AnglePll_Seed(AnglePll_T * p_pll, angle16_t theta, angle16_t delta)
{
    Angle_SetAngle(&p_pll->Base, theta);
    Angle_SetDelta(&p_pll->Base, delta);
    PID_Reset(&p_pll->LoopFilter);
    _PID_SetOutputState(&p_pll->LoopFilter, delta);
}

static inline void AnglePll_Reset(AnglePll_T * p_pll)
{
    Angle_ZeroCaptureState(&p_pll->Base);
    PID_Reset(&p_pll->LoopFilter);
    p_pll->PhaseError = 0;
}

/* Loop filter SampleFreq is the Proc rate */
static inline void AnglePll_InitFrom(AnglePll_T * p_pll, const PID_Config_T * p_loopFilter)
{
    PID_InitFrom(&p_pll->LoopFilter, p_loopFilter);
    AnglePll_Reset(p_pll);
}
