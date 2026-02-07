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

/*  */
typedef struct Angle_UnitRef
{
    angle16_t SpeedRated; /* DegPerCycle */
    uint32_t InvSpeedRated_Fract32;
}
Angle_UnitRef_T;

/* Config Options */
/* alternatively caller handle */
typedef struct Angle_Config
{
    uint32_t SpeedRated_Digital;
    //
}
Angle_Config_T;

// typedef struct Angle_MotorUnitRef
// typedef struct Angle_ConfigMotor
// {
//     uint32_t PollingFreq;
//     uint32_t SpeedRated_Rpm;
//     uint32_t PolePairs;
// }
// Angle_ConfigMotor_T;

/*
    [Angle_T] State Interface
*/
typedef struct Angle
{
    angle16_t Angle; /* Position Angle */
    angle16_t Speed; /* DeltaAngle_DegPerCycle / DigitalSpeed */

    accum32_t Speed_Fract16;  /* Store for Feedback. Let sensor select direct capture or convert from DeltaAngle */

    /* Interpolation Capture */
    // angle16_t InterpolateAngleLimit;
    // angle16_t InterpolateAnglePart; // if seperate from feedback Angle

    /* results from config */
    /* Fract16 Unit */
    Angle_UnitRef_T UnitRef;

    // angle16_t AnglePerPulse;
    // uint32_t Angle32PerPulse;
    // uint32_t AngleUnitShifted; /* Shifted */

    // uint32_t Angle32PerPoll;

    // angle16_t MechanicalAngle;
    // uint32_t OuterRatio;
    // Angle_Config_T Config;
}
Angle_T;


/*
    Capture On Feedback
*/
static inline void Angle_CaptureAngle(Angle_T * p_angle, angle16_t angle16)
{
    p_angle->Angle = angle16;
}

static inline void Angle_CaptureDelta(Angle_T * p_angle, angle16_t delta_degPerCycle)
{
    p_angle->Speed = delta_degPerCycle;
}

static inline void Angle_CaptureSpeed(Angle_T * p_angle, angle16_t delta_degPerCycle)
{
    // p_angle->Speed = delta_degPerCycle;
    p_angle->Speed = ((int32_t)delta_degPerCycle + (int32_t)p_angle->Speed) / 2; /* check sign extension */
}

static inline void Angle_CaptureSpeed_Fract16(Angle_T * p_angle, accum32_t speed_fract16)
{
    // p_angle->Speed = angle_of_speed_fract16(p_angle->UnitRef.SpeedRated, speed_fract16);
    p_angle->Speed_Fract16 = (speed_fract16 + p_angle->Speed_Fract16) / 2;
}


/*
   Interpolation Capture
*/

//todo with bounds
// static inline void Angle_InterpolateAngle(Angle_T * p_angle)
// {
//     p_angle->Angle += p_angle->Speed;
// }

/*
    Feedforward
*/
static inline void Angle_SetFeedforwardDelta(Angle_T * p_angle, angle16_t delta_degPerCycle)
{
    p_angle->Angle += delta_degPerCycle;
    p_angle->Speed = delta_degPerCycle;
    p_angle->Speed_Fract16 = speed_fract16_of_angle(p_angle->UnitRef.InvSpeedRated_Fract32, delta_degPerCycle);
}

static inline void Angle_SetFeedforwardAngle(Angle_T * p_angle, angle16_t angle)
{
    Angle_SetFeedforwardDelta(p_angle, angle - p_angle->Angle);
}

static inline void Angle_SetFeedforwardSpeed_Fract16(Angle_T * p_angle, fract16_t speed_fract16)
{
    Angle_SetFeedforwardDelta(p_angle, angle_of_speed_fract16(p_angle->UnitRef.SpeedRated, speed_fract16)); /* convert [fract16] of [RatedSpeed] into [angle16] */
}

static inline void Angle_ZeroCaptureState(Angle_T * p_angle)
{
    p_angle->Angle = 0;
    p_angle->Speed = 0;
    p_angle->Speed_Fract16 = 0;
}

/*
    Init Runtime
*/
// static void Angle_InitSpeedRated_Rpm(Angle_UnitRef * p_angle, uint32_t pollingFreq, ratio, uint32_t SpeedRated_Rpm)
// {
//     p_angle->SpeedRated = angle_of_rpm(pollingFreq, SpeedRated_Rpm);
//     p_angle->InvSpeedRated_Fract32 = INT32_MAX / p_angle->SpeedRated;
// }

static void Angle_InitFrom(Angle_T * p_angle, const Angle_Config_T * p_config)
{
    // p_angle->SpeedRated = el_angle_of_mech_rpm(p_config->PollingFreq, p_config->PollingFreq, p_config->SpeedRated_Rpm);
    p_angle->UnitRef.SpeedRated = p_config->SpeedRated_Digital;
    p_angle->UnitRef.InvSpeedRated_Fract32 = INT32_MAX / p_angle->UnitRef.SpeedRated;
    /* alternatively INT32_MAX / 2 / p_angle->UnitRef.SpeedRated for 2x ratedSpeed  */
}


/*
    Query
*/
static inline fract16_t Angle_GetSpeed_Digital(const Angle_T * p_angle) { return p_angle->Speed; }
static inline fract16_t Angle_GetSpeed_Fract16(const Angle_T * p_angle) { return p_angle->Speed_Fract16; }
// static inline fract16_t Angle_GetSpeed_Fract16(const Angle_T * p_angle) { return speed_fract16_of_angle(p_angle->InvSpeedRated_Fract32, p_angle->Speed); }


/* Optionally load from flash */
// typedef const struct AngleRef
// {
//     volatile uint16_t SPEED_RATED_MAX_RPM;
//     volatile uint16_t SPEED_RATED_MAX;
//     volatile uint32_t InvSpeedRated_Fract32;
// }
// AngleRef_T;
// extern AngleRef_T ANGLE_REF;
