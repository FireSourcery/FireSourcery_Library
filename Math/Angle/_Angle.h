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
    Angle State for math with angle16_t
*/
/******************************************************************************/

/* Runtime Ref */
typedef struct Angle_SpeedFractRef
{
    angle16_t SpeedMax_Angle16; /* DegPerCycle */
    uint32_t InvSpeedMax_Fract32; /* CyclesPerDeg */
}
Angle_SpeedFractRef_T;

#define ANGLE32_SHIFT (16U)

/*
    Interpolation between sensor edges.
    Internally shifted (32-bit) for sub-degree precision.
    ANGLE32_SHIFT fractional bits.
*/
typedef struct Angle_InterpolationState
{
    uint32_t Sum;        /* Accumulated interpolation angle, shifted. */
    uint32_t Delta;      /* Angle increment per poll cycle, shifted. */
    uint32_t Limit;      /* Sector boundary clamp magnitude, shifted. e.g. 60 degrees for Hall */
}
Angle_InterpolationState_T;

/*
    [Angle_T] State Interface
*/
typedef struct Angle
{
    angle16_t Angle; /* Position Angle. Retain state for integration */
    angle16_t Delta; /* DegPerTimeCycle */
    accum32_t Speed_Fract16;  /* Store for Feedback. Sensor select direct capture or convert from DeltaAngle */

    /* Interpolation Capture State */
    Angle_InterpolationState_T Interpolation;

    /* results from config */
    Angle_SpeedFractRef_T SpeedFractRef; /* Params */
}
Angle_T;

/* Config Options */
// typedef struct Angle_SpeedFractRef_Rpm
typedef struct Angle_SpeedFractCalib
{
    uint32_t PollingFreq;
    uint32_t SpeedMax_Rpm;
}
Angle_SpeedFractCalib_T;


/*
    Precomputed unit for Angle_T
*/
/* ANGLE_PER_REVOLUTION / FRACT16_MAX == 2 */
static inline int16_t speed_fract16_of_angle_direct(angle16_t angle_speed_max, int16_t angle16) { return ((int32_t)angle16 * FRACT16_MAX) / angle_speed_max; }

/* speedRefInv_fract32 as cycles per degree */
/* overflow note: angle16 < speedRefInv_fract32 */
static inline int16_t speed_fract16_of_angle(uint32_t angle_speed_max_inv_fract32, int16_t angle16) { return ((int32_t)angle16 * angle_speed_max_inv_fract32) >> 16U; }
/* DegPerCycle */
static inline int16_t angle_of_speed_fract16(angle16_t angle_speed_max, int16_t speed_fract16) { return fract16_mul(speed_fract16, angle_speed_max); }


/******************************************************************************/
/*
    Capture On Feedback
*/
/******************************************************************************/
static inline void Angle_CaptureAngle(Angle_T * p_angle, angle16_t angle16)
{
    p_angle->Angle = angle16;
}

/* Capture speed by Delta */
// static inline void _Angle_CaptureDelta(Angle_T * p_angle, angle16_t delta_degPerCycle)
// {
//     p_angle->Delta = delta_degPerCycle;
//     p_angle->Speed_Fract16 = speed_fract16_of_angle(p_angle->SpeedFractRef.InvSpeedMax_Fract32, delta_degPerCycle);
// }

/* Directly set the speed if available */
/* Set Delta for interpolation */
static inline fract16_t Angle_CaptureSpeed_Fract16(Angle_T * p_angle, accum32_t speed_fract16)
{
    p_angle->Speed_Fract16 = (speed_fract16 + p_angle->Speed_Fract16) / 2;
    p_angle->Delta = angle_of_speed_fract16(p_angle->SpeedFractRef.SpeedMax_Angle16, speed_fract16);
    return p_angle->Speed_Fract16;
}

static inline fract16_t Angle_ResolveSpeed_Fract16(Angle_T * p_angle)
{
    p_angle->Speed_Fract16 = speed_fract16_of_angle(p_angle->SpeedFractRef.InvSpeedMax_Fract32, p_angle->Delta);
    return p_angle->Speed_Fract16;
}

/******************************************************************************/
/*
    Integrate from commanded input
*/
/******************************************************************************/
static inline angle16_t Angle_IntegrateDelta(Angle_T * p_angle, angle16_t delta_degPerCycle)
{
    p_angle->Angle += delta_degPerCycle;
    p_angle->Delta = delta_degPerCycle;
    Angle_ResolveSpeed_Fract16(p_angle);
    return p_angle->Angle;
}

static inline angle16_t Angle_IntegrateSpeed_Fract16(Angle_T * p_angle, fract16_t speed_fract16)
{
    Angle_CaptureSpeed_Fract16(p_angle, speed_fract16);
    p_angle->Angle += p_angle->Delta;
    return p_angle->Angle;
}

// static inline void Angle_IntegrateAngle(Angle_T * p_angle, angle16_t angle)
// {
//     Angle_IntegrateDelta(p_angle, angle - p_angle->Angle);
// }

static inline void Angle_ZeroCaptureState(Angle_T * p_angle)
{
    p_angle->Angle = 0;
    p_angle->Delta = 0;
    p_angle->Speed_Fract16 = 0;
}

/******************************************************************************/
/*
    Interpolation - Estimate angle between sensor edges
*/
/******************************************************************************/
/*
    Set interpolation step
    Call at SAMPLE_FREQ
*/
static inline void Angle_ResolveInterpolationDelta(Angle_T * p_angle)
{
    assert(math_abs(p_angle->Delta) < (int32_t)ANGLE16_PER_REVOLUTION / 2); /* sanity check speed is within range for interpolation */
    p_angle->Interpolation.Delta = math_abs(p_angle->Delta) << ANGLE32_SHIFT;
}

/* Disable until next edge */
static inline void Angle_ClearInterpolationDelta(Angle_T * p_angle)
{
    p_angle->Interpolation.Delta = 0;
}

/*
    Accumulate interpolation angle, clamped to sector boundary.
    Call at POLLING_FREQ (e.g. 20kHz) between edge events
    Returns interpolation displacement as angle16_t.
    caller hold start reference for now
*/
static inline angle16_t Angle_Interpolate(Angle_T * p_angle)
{
    p_angle->Interpolation.Sum = math_limit_upper(p_angle->Interpolation.Sum + p_angle->Interpolation.Delta, p_angle->Interpolation.Limit);
    return math_sign(p_angle->Delta) * (int32_t)(p_angle->Interpolation.Sum >> ANGLE32_SHIFT);

    // return p_angle->Angle + math_sign(p_angle->Delta) * (int32_t)(p_angle->Interpolation.Sum >> ANGLE32_SHIFT);
}

static inline angle16_t Angle_InterpolateIndex(Angle_T * p_angle, size_t index)
{
    p_angle->Interpolation.Sum = math_limit_upper(index * p_angle->Interpolation.Delta, p_angle->Interpolation.Limit);
    return math_sign(p_angle->Delta) * (p_angle->Interpolation.Sum >> ANGLE32_SHIFT);
}

/* Reset interpolation on sensor edge or direction change */
static inline void Angle_ZeroInterpolation(Angle_T * p_angle)
{
    p_angle->Interpolation.Sum = 0;
}


/*
    Init interpolation limit from sector angle.
    sectorAngle_shifted: angle per sector in shifted representation.
    e.g. for Hall with PolePairs: UnitAngleD * PolePairs
*/
static inline void Angle_InitInterpolation(Angle_T * p_angle, angle16_t limit_angle16)
{
    p_angle->Interpolation.Limit = (int32_t)limit_angle16 << ANGLE32_SHIFT; /* shift up for accumulation, positive magnitude */
    p_angle->Interpolation.Sum = 0;
    p_angle->Interpolation.Delta = 0;
}

/******************************************************************************/
/*
    Init Runtime
*/
/******************************************************************************/
#define ANGLE_SPEED_FRACT_REF(maxAngle16) (Angle_SpeedFractRef_T) { .SpeedMax_Angle16 = (maxAngle16), .InvSpeedMax_Fract32 = INT32_MAX / (maxAngle16) }
#define ANGLE_SPEED_FRACT_REF_FROM_CALIB(pollingFreq, maxRpm) ANGLE_SPEED_FRACT_REF(ANGLE16_OF_RPM(pollingFreq, maxRpm))
// static inline Angle_SpeedFractRef_T Angle_SpeedFractRef(angle16_t maxAngle16) { return ANGLE_SPEED_FRACT_REF(maxAngle16); }
// static inline Angle_SpeedFractRef_T Angle_SpeedFractRef_FromCalib(uint32_t pollingFreq, uint32_t maxRpm) { return Angle_SpeedFractRef(angle_of_rpm(pollingFreq, maxRpm)); }

/* Caller pass Ref ~ 2x for overflow range */
static void Angle_InitSpeedRef(Angle_T * p_angle, angle16_t maxAngle16)
{
    p_angle->SpeedFractRef = ANGLE_SPEED_FRACT_REF(maxAngle16);
}

static void Angle_InitSpeedRef_Rpm(Angle_T * p_angle, uint32_t pollingFreq, uint32_t maxRpm)
{
    p_angle->SpeedFractRef = ANGLE_SPEED_FRACT_REF_FROM_CALIB(pollingFreq, maxRpm);
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
static inline fract16_t Angle_GetSpeed_Digital(const Angle_T * p_angle) { return p_angle->Delta; }
static inline fract16_t Angle_GetSpeed_Fract16(const Angle_T * p_angle) { return p_angle->Speed_Fract16; }


/* as data interface */
// typedef struct angle_speed { angle16_t Angle; angle16_t Delta; } angle_speed_t;
// typedef struct Angle_Data
// {
//     angle16_t Angle; /* Position Angle */
//     angle16_t Delta; /* DegPerCycle, DigitalSpeed */
// }
// Angle_Data_T;
