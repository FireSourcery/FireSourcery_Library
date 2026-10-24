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
    Or the caller supplies a vector along θ, and ProcVector detects it against θ̂, or ProcRelative in the frame of θ̂.
    The vector entries also hold its magnitude and the lock on it.
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

/* x already rotated into the frame of θ̂, so θ̂ = 0 */
static inline accum32_t angle_pll_error_relative(fract16_t y) { return 0 - (accum32_t)y; }

/* sin(θ̂ − θ), |x| stripped so the loop gain holds across amplitude. 0 at or below the floor, where θ is indeterminate */
static inline fract16_t angle_pll_normalize(ufract16_t mag_floor, ufract16_t mag, accum32_t error) { return (mag <= mag_floor) ? 0 : fract16_sat(fract16_div(error, mag)); }

static inline fract16_t angle_pll_error_normalized(ufract16_t mag_floor, fract16_t x, fract16_t y, fract16_t sin, fract16_t cos)
{
    return angle_pll_normalize(mag_floor, fract16_vector_magnitude(x, y), angle_pll_error(x, y, sin, cos));
}

static inline fract16_t angle_pll_error_normalized_relative(ufract16_t mag_floor, fract16_t x, fract16_t y)
{
    return angle_pll_normalize(mag_floor, fract16_vector_magnitude(x, y), angle_pll_error_relative(y));
}


typedef struct AnglePll_FilterTuning
{
    ufract16_t MagFloor;        /* |x| at or below which θ is indeterminate: the error reads 0 */
    ufract16_t LockErrTol;      /* |error| within which θ̂ tracks θ */
    uint16_t LockHoldCount;     /* consecutive ticks above the floor and within tolerance */
    uint16_t LossHoldCount;     /* consecutive ticks at or below the floor or out of tolerance that the lock holds through. 0 releases on the first */
}
AnglePll_FilterTuning_T;

/*
    [AnglePll_T]
    Base.Delta is the PI output, overwritten each Proc. Seed sets ω̂ through the integrator.
*/
typedef struct AnglePll
{
    Angle_T Base;           /* Angle is θ̂. Delta is ω̂ [angle16/dt] */

    PID_T LoopFilter;       /* PI, phase error → ω̂ */
    fract16_t PhaseError;   /* last error */

    /* Lock state */
    AnglePll_FilterTuning_T FilterTuning;
    ufract16_t Magnitude;       /* |x| of the last detected vector */
    uint16_t LockCount;
    uint16_t LossCount;
}
AnglePll_T;

// typedef struct
// {
//     AnglePll_FilterTuning_T FilterTuning;
//     ufract16_t Magnitude;       /* |x| of the last detected vector */
//     uint16_t LockCount;
// }
// AnglePll_Collab_T;

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

static inline bool AnglePll_IsLocked(const AnglePll_T * p_pll) { return (p_pll->LockCount >= p_pll->FilterTuning.LockHoldCount); }

// static inline void _AnglePll_ProcLock(AnglePll_T * p_pll, ufract16_t magnitude, fract16_t phaseError)
// {
//     bool stable = (magnitude > p_pll->FilterTuning.MagFloor) && (fract16_abs(phaseError) < p_pll->FilterTuning.LockErrTol);
//     p_pll->Magnitude = magnitude;
//     p_pll->LockCount = stable ? (uint16_t)math_min(p_pll->LockCount + 1, p_pll->FilterTuning.LockHoldCount) : 0;
// }

/*
    Lock: a strong signal and a tight error, sustained. Held through their loss for LossHoldCount ticks.
    Only the vector entries see the signal: below the floor the error reads 0, as it does at lock.
*/
static inline void _AnglePll_ProcLock(AnglePll_T * p_pll, ufract16_t magnitude, fract16_t phaseError)
{
    bool stable = (magnitude > p_pll->FilterTuning.MagFloor) && (fract16_abs(phaseError) < p_pll->FilterTuning.LockErrTol);
    p_pll->Magnitude = magnitude;
    if (AnglePll_IsLocked(p_pll) == false)
    {
        p_pll->LockCount = stable ? (p_pll->LockCount + 1U) : 0U;
    }
    else
    {
        p_pll->LossCount = stable ? 0U : (p_pll->LossCount + 1U);
        if (p_pll->LossCount > p_pll->FilterTuning.LossHoldCount) { p_pll->LockCount = 0U; p_pll->LossCount = 0U; }
    }
}

/* The detected vector: its error normalized against the floor, the lock on both, then the loop */
static inline angle16_t _AnglePll_ProcDetected(AnglePll_T * p_pll, ufract16_t magnitude, accum32_t error)
{
    fract16_t phaseError = angle_pll_normalize(p_pll->FilterTuning.MagFloor, magnitude, error);
    _AnglePll_ProcLock(p_pll, magnitude, phaseError);
    return AnglePll_Proc(p_pll, phaseError);
}

/* x along θ, detected against θ̂ */
static inline angle16_t AnglePll_ProcVector(AnglePll_T * p_pll, fract16_t x, fract16_t y)
{
    struct fract16_xy unit = Angle_UnitVector(&p_pll->Base);
    return _AnglePll_ProcDetected(p_pll, fract16_vector_magnitude(x, y), angle_pll_error(x, y, unit.y, unit.x));
}

/* x already in the frame of θ̂ */
static inline angle16_t AnglePll_ProcRelative(AnglePll_T * p_pll, fract16_t x, fract16_t y)
{
    return _AnglePll_ProcDetected(p_pll, fract16_vector_magnitude(x, y), angle_pll_error_relative(y));
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
    p_pll->Magnitude = 0;
    p_pll->LockCount = 0;
    p_pll->LossCount = 0;
}

/* Loop filter SampleFreq is the Proc rate */
static inline void AnglePll_InitFrom(AnglePll_T * p_pll, const PID_Config_T * p_loopFilter)
{
    PID_InitFrom(&p_pll->LoopFilter, p_loopFilter);
    AnglePll_Reset(p_pll);
}

static inline void AnglePll_InitPid(AnglePll_T * p_pll, const PID_Config_T * p_loopFilter)
{
    PID_InitFrom(&p_pll->LoopFilter, p_loopFilter);
    // AnglePll_Reset(p_pll);
}

static inline void AnglePll_InitFilterTuning(AnglePll_T * p_pll, const AnglePll_FilterTuning_T * p_loopFilter)
{
    p_pll->FilterTuning = *p_loopFilter;
}
