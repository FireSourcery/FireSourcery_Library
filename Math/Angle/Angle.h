#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery

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
    @file   Angle.h
    @author FireSourcery
    @brief  Angle IIntegrator in digitized form, conversion math
            Common Interface for Angle and Speed state
*/
/******************************************************************************/
#include "angle_speed_math.h"
#include "../Fixed/fract16.h"

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*!
    Angle State — 32-bit shifted tracker.

    Angle accumulator shifts 16 for wrap.

    Three integration modes share one struct:
        - wrap     : Angle_Integrate      (position tracker, free wrap)
        - clamp    : Angle_Interpolate         (bounded step; Limit is signed remaining-allowance)
        - none     : Angle_SetAngle        (direct sensor snapshot)
*/
/******************************************************************************/
#define ANGLE32_SHIFT (16U)

/*
    [Angle_T] Pure stateful tracker — 12 bytes, all fields used by all modes.
    Speed_Fract16 and SpeedFractRef are sibling types composed by the caller.
*/
typedef struct Angle
{
    int32_t Angle;  /* Shifted 16 position/accumulator. GetAngle16 projects via >> ANGLE32_SHIFT */
    int32_t Delta;  /* Shifted 16 DegPerPoll */
    int32_t LimitUpper;
    int32_t LimitLower;
}
Angle_T;

/******************************************************************************/
/*
    Accessors — project shifted state down to angle16_t
*/
/******************************************************************************/
static inline angle16_t Angle_Value(const Angle_T * p_angle) { return (angle16_t)(p_angle->Angle >> ANGLE32_SHIFT); }
static inline angle16_t Angle_Delta(const Angle_T * p_angle) { return (angle16_t)(p_angle->Delta >> ANGLE32_SHIFT); }

static inline struct fract16_xy Angle_UnitVector(const Angle_T * p_angle) { return fract16_vector(Angle_Value(p_angle)); }


/******************************************************************************/
/*
    Capture On Feedback
*/
/******************************************************************************/
/* Direct sensor snapshot — shifts into internal representation */
static inline void Angle_SetAngle(Angle_T * p_angle, angle16_t angle16) { p_angle->Angle = (int32_t)angle16 << ANGLE32_SHIFT; }

/* Capture per-cycle delta from angle16 source */
static inline void Angle_SetDelta(Angle_T * p_angle, angle16_t delta) { p_angle->Delta = (int32_t)delta << ANGLE32_SHIFT; }

/******************************************************************************/
/*
    Integrate from commanded input
*/
/******************************************************************************/
/*
    Wrap-mode integration. Angle += Delta, free wrap on overflow.
    Used for position trackers (sensorless angle, commanded open-loop).
*/
static inline angle16_t Angle_Integrate(Angle_T * p_angle, angle16_t delta)
{
    p_angle->Delta = (int32_t)delta << ANGLE32_SHIFT;
    p_angle->Angle += p_angle->Delta;
    return Angle_Value(p_angle);
}

/*
    Wrap-mode step using the already-resolved shifted Delta.
    For sensor paths that set Delta out-of-band (e.g. AngleCounter_ResolveAngleDelta).
*/
static inline angle16_t Angle_IntegrateStep(Angle_T * p_angle)
{
    p_angle->Angle += p_angle->Delta;
    return Angle_Value(p_angle);
}

static inline void Angle_ZeroCaptureState(Angle_T * p_angle)
{
    p_angle->Angle = 0;
    p_angle->Delta = 0;
}

/******************************************************************************/
/*
    Interpolation / bounded integration —
    estimate angle between sensor edges or drive a clamped ramp.

    Angle_Interpolate clamps Angle + Delta to [LimitLower, LimitUpper].
    Bounds are independent: direction-agnostic, re-targetable per edge.
*/
/******************************************************************************/
/*
    Step Delta: [down_room, up_room]
    which when both are negative (Angle above LimitUpper) forces any Delta back toward the window.
    Angle next = clamp(Angle + Delta, LimitLower, LimitUpper)
*/
static inline int32_t _Angle_DeltaSat(const Angle_T * p_angle)
{
    return math_clamp(p_angle->Delta, (uint32_t)p_angle->LimitLower - p_angle->Angle, (uint32_t)p_angle->LimitUpper - p_angle->Angle);
}

static inline angle16_t Angle_Interpolate(Angle_T * p_angle)
{
    p_angle->Angle += _Angle_DeltaSat(p_angle);
    return Angle_Value(p_angle);
}

/*
    Bounded step — wrapping-aware clamp.
    Unsigned subtraction recovers the correct span/offset even when limits
*/
// static inline angle16_t Angle_Interpolate(Angle_T * p_angle)
// {
//     /*  assert(math_abs(p_angle->Delta >> ANGLE32_SHIFT) < ANGLE16_PER_REVOLUTION / 2); */
//     /* (x − low) > (high − low) */
//     if ((uint32_t)(p_angle->Angle + p_angle->Delta - p_angle->LimitLower) > (uint32_t)(p_angle->LimitUpper - p_angle->LimitLower))
//     {
//         p_angle->Angle = (p_angle->Delta >= 0) ? p_angle->LimitUpper : p_angle->LimitLower;
//     }
//     else
//     {
//         p_angle->Angle += p_angle->Delta;
//     }

//     return Angle_Value(p_angle);
// }

/* Set bounds as an angle window [lower, upper] from angle16 inputs. */
static inline void Angle_SetLimits(Angle_T * p_angle, angle16_t lower, angle16_t upper)
{
    p_angle->LimitLower = (int32_t)lower << ANGLE32_SHIFT;
    p_angle->LimitUpper = (int32_t)upper << ANGLE32_SHIFT;
}

/*
    Set bounds as a one-sided sector window: width ahead in sign(Delta), 0 behind.
    Called after snapping Angle to a sensor boundary (e.g. Hall edge).
        Delta > 0 : [Angle, Angle + width]
        Delta < 0 : [Angle - width, Angle]
*/
static inline void Angle_SetLimitWindow(Angle_T * p_angle, uangle16_t width_angle16)
{
    int32_t width_shifted = (int32_t)width_angle16 << ANGLE32_SHIFT;
    if (p_angle->Delta >= 0)
    {
        p_angle->LimitUpper = p_angle->Angle + width_shifted;
        p_angle->LimitLower = p_angle->Angle;
    }
    else
    {
        p_angle->LimitUpper = p_angle->Angle;
        p_angle->LimitLower = p_angle->Angle - width_shifted;
    }
}


/*
    Init: zero state + set symmetric bounds ±limit_angle16 around zero.
*/
static inline void Angle_InitLimits(Angle_T * p_angle, uangle16_t limit_angle16)
{
    p_angle->LimitUpper = (int32_t)limit_angle16 << ANGLE32_SHIFT;
    p_angle->LimitLower = -((int32_t)limit_angle16 << ANGLE32_SHIFT);
    p_angle->Angle = 0;
    p_angle->Delta = 0;
}

/*
    Reset accumulator on sensor edge / direction change. Retains Delta and limits.
*/
static inline void Angle_ZeroAngle(Angle_T * p_angle) { p_angle->Angle = 0; }

/* Disable integration until next Delta update */
static inline void Angle_StopDelta(Angle_T * p_angle) { p_angle->Delta = 0; }



/*
    Step is pinned in [−room, 0] or [0, +room]
    no auto-correction, 0 on overshoot.
    e.g. Already above upper : Angle = 110, Upper = 100, Lower = 0, Delta = +5
    returns 0, stay at 110.
*/
// static inline angle16_t _Angle_GetDeltaSat(const Angle_T * p_angle)
// {
//     int32_t upper = math_max((uint32_t)p_angle->LimitUpper - (uint32_t)p_angle->Angle, 0);
//     int32_t lower = math_min((uint32_t)p_angle->LimitLower - (uint32_t)p_angle->Angle, 0);
//     return math_clamp(p_angle->Delta, lower, upper);
// }

/******************************************************************************/
/*
    as progress state
*/
/******************************************************************************/
// static inline angle16_t Angle_Abs_Interpolate(Angle_T * p_angle)
// {
//     assert(p_angle->Angle < ANGLE16_PER_REVOLUTION / 2);
//     p_angle->Angle = math_min(p_angle->Angle + p_angle->Delta, p_angle->LimitUpper);
//     return Angle_Value(p_angle);
// }

// static inline void Angle_Abs_SetLimit (Angle_T * p_angle, uangle16_t width_angle16)
// {
//     int32_t width_shifted = (int32_t)width_angle16 << ANGLE32_SHIFT;
//     p_angle->LimitUpper = p_angle->Angle + math_sign(p_angle->Delta) * width_shifted;
// }



