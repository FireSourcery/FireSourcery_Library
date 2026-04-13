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
    @brief  Common Interface for Angle and Speed state
*/
/******************************************************************************/
#include "angle_speed_fn.h"
#include "../Fixed/fract16.h"

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*!
    Angle State — 32-bit shifted tracker.

    Internal storage is Q16.16: the high 16 bits are the angle16 projection
    (directly usable for sin/cos LUT, Park/Clarke), the low 16 bits are
    sub-degree precision preserved for integration at low speeds.

    Three integration modes share one struct:
        - wrap     : Angle_IntegrateDelta      (position tracker, free wrap)
        - clamp    : Angle_Interpolate         (interpolator / bounded ramp, uses Limit)
        - none     : Angle_CaptureAngle        (direct sensor snapshot)
*/
/******************************************************************************/
#define ANGLE32_SHIFT (16U)

/*
    [Angle_T] Pure stateful tracker — 12 bytes, all fields used by all modes.
    Speed_Fract16 and SpeedFractRef are sibling types composed by the caller.
*/
typedef struct Angle
{
    int32_t Angle;  /* Shifted Q16.16 position/accumulator. GetAngle16 projects via >> ANGLE32_SHIFT */
    int32_t Delta;  /* Shifted Q16.16 DegPerCycle */
    int32_t Limit;  /* Symmetric bound for Angle_Interpolate, shifted. Unused in wrap mode. */
}
Angle_T;


/*
    Runtime Precomputed Ref
*/
typedef struct Angle_SpeedFractRef
{
    angle16_t SpeedMax_Angle16; /* DegPerCycle */
    uint32_t InvSpeedMax_Fract32; /* CyclesPerDeg */
    // uint32_t PollingFreq; keep for per second conversions if needed
}
Angle_SpeedFractRef_T;

/* Config Options */
// typedef struct Angle_SpeedFractRef_Rpm
typedef struct Angle_SpeedFractCalib
{
    uint32_t PollingFreq;
    uint32_t SpeedMax_Rpm;
}
Angle_SpeedFractCalib_T;

#define ANGLE_SPEED_FRACT_REF(maxAngle16) (Angle_SpeedFractRef_T) { .SpeedMax_Angle16 = (maxAngle16), .InvSpeedMax_Fract32 = INT32_MAX / (maxAngle16) }
#define ANGLE_SPEED_FRACT_REF_FROM_CALIB(pollingFreq, maxRpm) ANGLE_SPEED_FRACT_REF(ANGLE16_OF_RPM(pollingFreq, maxRpm))


/*
    Precomputed unit conversions — operate on shifted (Q16.16) delta.
    All hot-path consumers keep Delta in shifted form; the helpers below
    read/write from that representation directly.
*/
/* ANGLE_PER_REVOLUTION / FRACT16_MAX == 2 */
static inline int16_t speed_fract16_of_angle_direct(angle16_t angleOfRpmMax, int16_t angle16) { return ((int32_t)angle16 * FRACT16_MAX) / angleOfRpmMax; }

static inline int16_t speed_fract16_of_angle(uint32_t angleOfRpmMaxInv_fract32, int16_t angle16) { return ((int32_t)angle16 * angleOfRpmMaxInv_fract32) >> 16U; }
static inline int16_t angle_of_speed_fract16(angle16_t angleOfRpmMax, int16_t speed_fract16) { return fract16_mul(speed_fract16, angleOfRpmMax); }


/******************************************************************************/
/*
    Accessors — project shifted state down to angle16_t
*/
/******************************************************************************/
static inline angle16_t Angle_GetAngle16(const Angle_T * p_angle) { return (angle16_t)(p_angle->Angle >> ANGLE32_SHIFT); }
static inline angle16_t Angle_GetDelta16(const Angle_T * p_angle) { return (angle16_t)(p_angle->Delta >> ANGLE32_SHIFT); }


/******************************************************************************/
/*
    Capture On Feedback
*/
/******************************************************************************/
/* Direct sensor snapshot — shifts into internal representation */
static inline void Angle_CaptureAngle(Angle_T * p_angle, angle16_t angle16) { p_angle->Angle = (int32_t)angle16 << ANGLE32_SHIFT; }

/* Capture per-cycle delta from angle16 source */
static inline void Angle_CaptureDelta(Angle_T * p_angle, angle16_t delta_degPerCycle)
{
    assert(math_abs(delta_degPerCycle) < ANGLE16_PER_REVOLUTION / 2);
    p_angle->Delta = (int32_t)delta_degPerCycle << ANGLE32_SHIFT;
}

/******************************************************************************/
/*
    Integrate from commanded input
*/
/******************************************************************************/
/*
    Wrap-mode integration. Angle += Delta, free wrap on overflow.
    Used for position trackers (sensorless angle, commanded open-loop).
*/
static inline angle16_t Angle_IntegrateDelta(Angle_T * p_angle, angle16_t delta_degPerCycle)
{
    p_angle->Delta = (int32_t)delta_degPerCycle << ANGLE32_SHIFT;
    p_angle->Angle += p_angle->Delta;
    return Angle_GetAngle16(p_angle);
}

static inline void Angle_ZeroCaptureState(Angle_T * p_angle)
{
    p_angle->Angle = 0;
    p_angle->Delta = 0;
}

/******************************************************************************/
/*
    Interpolation / bounded integration — estimate angle between sensor edges
    or drive a clamped ramp.

    The interpolator reuses Angle_T: its Angle field is the bounded accumulator,
    Delta is the shifted step (signed), Limit is the symmetric bound.
*/
/******************************************************************************/
/* Init symmetric bound from an angle16 magnitude (e.g. 60° sector for Hall) */
static inline void Angle_InitLimit(Angle_T * p_angle, angle16_t limit_angle16)
{
    p_angle->Limit = (int32_t)limit_angle16 << ANGLE32_SHIFT;
    p_angle->Angle = 0;
    p_angle->Delta = 0;
}

/*
    Reset accumulator on sensor edge / direction change. Retains Delta and Limit.
*/
static inline void Angle_ZeroAngle(Angle_T * p_angle) { p_angle->Angle = 0; }

/* Disable integration until next Delta update */
static inline void Angle_StopDelta(Angle_T * p_angle) { p_angle->Delta = 0; }

/*
    Clamped integration. Angle = clamp(Angle + Delta, ±Limit).
    Call at POLLING_FREQ (e.g. 20kHz). Returns projected angle16.
*/
static inline angle16_t Angle_Interpolate(Angle_T * p_angle)
{
    p_angle->Angle = math_clamp(p_angle->Angle + p_angle->Delta, -p_angle->Limit, p_angle->Limit);
    return Angle_GetAngle16(p_angle);
}

/* One-shot bounded integration with an external limit — stateless variant for ramps */
// static inline angle16_t Angle_InterpolateUpTo(Angle_T * p_angle, int32_t limit_shifted)
// {
//     p_angle->Angle = math_clamp(p_angle->Angle + p_angle->Delta, -limit_shifted, limit_shifted);
//     return Angle_GetAngle16(p_angle);
// }


/******************************************************************************/
/*
    Angle_SpeedFractRef_T
*/
/******************************************************************************/
/* Caller pass Ref ~ 2x for overflow range */
static void Angle_SpeedRef_Init(Angle_SpeedFractRef_T * p_ref, angle16_t maxAngle16) { *p_ref = ANGLE_SPEED_FRACT_REF(maxAngle16); }
static void Angle_SpeedRef_Init_Rpm(Angle_SpeedFractRef_T * p_ref, uint32_t pollingFreq, uint32_t maxRpm) { *p_ref = ANGLE_SPEED_FRACT_REF_FROM_CALIB(pollingFreq, maxRpm); }


/* Directly set the speed if available; updates Delta in shifted form */
static inline void Angle_CaptureSpeed_Fract16(Angle_T * p_angle, const Angle_SpeedFractRef_T * p_ref, accum32_t speed_fract16)
{
    Angle_CaptureDelta(p_angle, angle_of_speed_fract16(p_ref->SpeedMax_Angle16, speed_fract16));
}

static inline fract16_t Angle_ResolveSpeed_Fract16(Angle_T * p_angle, const Angle_SpeedFractRef_T * p_ref)
{
    return speed_fract16_of_angle(p_ref->InvSpeedMax_Fract32, p_angle->Delta >> ANGLE32_SHIFT);
}

static inline angle16_t Angle_IntegrateSpeed_Fract16(Angle_T * p_angle, const Angle_SpeedFractRef_T * p_ref, fract16_t speed_fract16)
{
    return Angle_IntegrateDelta(p_angle, angle_of_speed_fract16(p_ref->SpeedMax_Angle16, speed_fract16));
}


// typedef struct Angle_Config_T
// {
//     Angle_SpeedFractRef_T SpeedFractRef;
// }
// Angle_Config_T;

// static void Angle_InitFrom(Angle_T * p_angle, const Angle_Config_T * p_config)
// {
//     p_angle->SpeedFractRef
// }


/*
    Query
*/
// static inline fract16_t Angle_GetSpeed_Angle16(const Angle_T * p_angle) { return p_angle->Delta; }
// static inline fract16_t Angle_GetSpeed_Fract16(const Angle_T * p_angle) { return p_angle->Speed_Fract16; }


/* as data interface */
// typedef struct angle_speed { angle16_t Angle; angle16_t Delta; } angle_speed_t;
// typedef struct Angle_Data
// {
//     angle16_t Angle; /* Position Angle */
//     angle16_t Delta; /* DegPerCycle, DigitalSpeed */
// }
// Angle_Data_T;