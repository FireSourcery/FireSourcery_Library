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
#include "math_angle_speed.h"
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

typedef struct Angle_InterpolationState
{
    angle16_t Limit;
    angle16_t Part; // if seperate from feedback Angle
}
Angle_InterpolationState_T;

/* as data interface */
// typedef struct Angle_Data
// {
//     angle16_t Angle; /* Position Angle */
//     angle16_t Delta; /* DegPerCycle, DigitalSpeed */
//     accum32_t Speed_Fract16;  /* Store for Feedback. Let sensor select direct capture or convert from DeltaAngle */
// }
// Angle_Data_T;

/*
    [Angle_T] State Interface
*/
typedef struct Angle
{
    angle16_t Angle; /* Position Angle */
    angle16_t Delta; /* DegPerCycle, DigitalSpeed */
    accum32_t Speed_Fract16;  /* Store for Feedback. Let sensor select direct capture or convert from DeltaAngle */

    /* Interpolation Capture State */
    Angle_InterpolationState_T Interpolation;

    /* results from config */
    Angle_SpeedFractRef_T SpeedFractRef; /* Params */

    /// Angle_PulseRef_T
    // angle16_t AnglePerPulse;
    // uint32_t Angle32PerPulse;
    // uint32_t AngleUnitShifted; /* Shifted */
    // uint32_t Angle32PerPoll;
}
Angle_T;

/* Config Options */
// typedef struct Angle_Config
// {
//     uint32_t SpeedRef_Angle16;
//     //
// }
// Angle_Config_T;

typedef struct Angle_SpeedFractCalib
{
    uint32_t PollingFreq;
    uint32_t SpeedMax_Rpm;
}
Angle_SpeedFractCalib_T;

/*
    Capture On Feedback
*/
static inline void Angle_CaptureAngle(Angle_T * p_angle, angle16_t angle16)
{
    p_angle->Delta = angle16 - p_angle->Angle; /* check wrap and sign extension */
    p_angle->Angle = angle16;
}

/* Capture async to angle */
// static inline void Angle_CaptureDelta(Angle_T * p_angle, angle16_t delta_degPerCycle)
// {
//     p_angle->Delta = delta_degPerCycle;
//     // speed_fract16_of_angle(p_angle->InvSpeedRated_Fract32, delta_degPerCycle);
// }

static inline fract16_t _Angle_GetSpeed_Fract16(const Angle_T * p_angle)
{
    return speed_fract16_of_angle(p_angle->SpeedFractRef.InvSpeedMax_Fract32, p_angle->Delta);
    // speed_fract16_of_angle16_rpm(Config.PollingFreq, Config.SpeedMaxRpm, elSpeed_degPerCycle);
}

/* Capture speed by Delta */
static inline void Angle_CaptureSpeed_Delta(Angle_T * p_angle, angle16_t delta_degPerCycle)
{
    p_angle->Delta = delta_degPerCycle;
    p_angle->Speed_Fract16 = speed_fract16_of_angle(p_angle->SpeedFractRef.InvSpeedMax_Fract32, delta_degPerCycle);
}

/* Directly set the speed if available */
/* Set Delta for interpolation */
static inline void Angle_CaptureSpeed_Fract16(Angle_T * p_angle, accum32_t speed_fract16)
{
    p_angle->Speed_Fract16 = (speed_fract16 + p_angle->Speed_Fract16) / 2;
    p_angle->Delta = angle_of_speed_fract16(p_angle->SpeedFractRef.SpeedMax_Angle16, speed_fract16);
}

/*
    Feedforward
*/
static inline void Angle_SetFeedforwardDelta(Angle_T * p_angle, angle16_t delta_degPerCycle)
{
    p_angle->Angle += delta_degPerCycle;
    Angle_CaptureSpeed_Delta(p_angle, delta_degPerCycle);
}

static inline void Angle_SetFeedforwardAngle(Angle_T * p_angle, angle16_t angle)
{
    Angle_SetFeedforwardDelta(p_angle, angle - p_angle->Angle);
}

static inline void Angle_SetFeedforwardSpeed_Fract16(Angle_T * p_angle, fract16_t speed_fract16)
{
    Angle_CaptureSpeed_Fract16(p_angle, speed_fract16);
    p_angle->Angle += p_angle->Delta;
    // Angle_SetFeedforwardDelta(p_angle, angle_of_speed_fract16(p_angle->SpeedFractRef.SpeedMax_Angle16, speed_fract16)); /* convert [fract16] of [SpeedRef] into [angle16] */
}

static inline void Angle_ZeroCaptureState(Angle_T * p_angle)
{
    p_angle->Angle = 0;
    p_angle->Delta = 0;
    p_angle->Speed_Fract16 = 0;
}

/*
   Interpolation Capture
*/
//todo with bounds
// static inline void Angle_Interpolate(Angle_T * p_angle, ticks )
// {
//     p_angle->Angle +  p_angle->Delta;
// }

// static inline void Angle_CaptureInterpolate(Angle_T * p_angle)
// {
//     p_angle->Angle += p_angle->Delta;
// }

/*
    Init Runtime
*/
/* Caller pass Ref ~ 2x for overflow range */
static void Angle_InitSpeedRef(Angle_T * p_angle, uint32_t speedMax_Angle16)
{
    p_angle->SpeedFractRef.SpeedMax_Angle16 = speedMax_Angle16;
    p_angle->SpeedFractRef.InvSpeedMax_Fract32 = INT32_MAX / speedMax_Angle16;
}

static void Angle_InitSpeedRef_Rpm(Angle_T * p_angle, uint32_t pollingFreq, uint32_t speedRef_Rpm)
{
    Angle_InitSpeedRef(p_angle, angle_of_rpm(pollingFreq, speedRef_Rpm));
}

// static void Angle_InitFrom(Angle_T * p_angle, const Angle_Config_T * p_config)
// {
//     p_angle->SpeedFractRef.InvSpeedMax_Fract32 = INT32_MAX / p_config->SpeedRef_Angle16;
// }


/*
    Query
*/
static inline fract16_t Angle_GetSpeed_Digital(const Angle_T * p_angle) { return p_angle->Delta; }
static inline fract16_t Angle_GetSpeed_Fract16(const Angle_T * p_angle) { return p_angle->Speed_Fract16; }


