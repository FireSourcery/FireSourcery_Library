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
    @file   AngleSpeed.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Angle.h"
#include "angle_speed_math.h"
#include "../Fixed/fract16.h"

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*
    Angle_SpeedUnitRef_T
    Digital Angle to Per Unit Reference
*/
/******************************************************************************/
/******************************************************************************/
/*
    Angle_SpeedUnitRef_T — precomputed (ω_base · Δt) anchor.

        θ̇      = ω_pu · ω_base                            [definition of ω_pu]
        θ_step = θ̇ · Δt = ω_pu · (ω_base / Fs)            [per-tick step]

    Precompute (ω_base · Δt) once in angle16/tick so the hot path is a single
    multiply with no division. Numerically:

        OmegaBase_dT = (ω_base / Fs) · (65536 / 2π)
                     = ω_base_RPM · 65536 / (60 · Fs)
                     = ANGLE16_OF_RPM(Fs, ω_base_RPM)

    Inverse stores Fs / ω_base (Q32) for the angle → ω_pu direction.
*/
/******************************************************************************/
// typedef struct Angle_SpeedScale
typedef struct Angle_SpeedFractRef
{
    angle16_t SpeedMax_Angle16;    /* (ω_base · Δt) in angle16/tick — angle step at ω_pu = 1.0 */
    uint32_t InvSpeedMax_Fract32;   /* Fs / ω_base (Q32) — inverse for angle → ω_pu */
    // uint32_t PollingFreq; /* keep for per second conversions if needed */
}
Angle_SpeedUnitRef_T;

/* Config Options in RPM */
// typedef struct Angle_SpeedFractRef_Rpm
// typedef struct Angle_SpeedFractCalib
// {
//     uint32_t PollingFreq;
//     uint32_t SpeedMax_Rpm;  /* ω_base as   RPM (~2x rated) */
// }
// Angle_SpeedFractCalib_T;

// typedef struct Angle_SpeedFractCalib
// {
//     uint32_t PollingFreq;
//     uint32_t K; from shift
// }
// Angle_SpeedFractCalib_T;

#define ANGLE_SPEED_FRACT_REF(maxAngle16) (Angle_SpeedUnitRef_T) { .SpeedMax_Angle16 = (maxAngle16), .InvSpeedMax_Fract32 = INT32_MAX / (maxAngle16) }
#define ANGLE_SPEED_FRACT_REF_FROM_CALIB(pollingFreq, maxRpm) ANGLE_SPEED_FRACT_REF(ANGLE16_OF_RPM(pollingFreq, maxRpm))

// #if defined(ANGLE_POLLING_FREQ) && defined(ANGLE_SPEED_MAX_RPM)
// static const Angle_SpeedUnitRef_T ANGLE_SPEED_CALIB = ANGLE_SPEED_FRACT_REF_FROM_CALIB(ANGLE_POLLING_FREQ, ANGLE_SPEED_MAX_RPM);
// #endif

static inline Angle_SpeedUnitRef_T Angle_SpeedFractRef(angle16_t maxAngle16) { return ANGLE_SPEED_FRACT_REF(maxAngle16); }
static inline Angle_SpeedUnitRef_T Angle_SpeedFractRef_FromRpm(uint32_t pollingFreq, uint32_t maxRpm) { return ANGLE_SPEED_FRACT_REF_FROM_CALIB(pollingFreq, maxRpm); }

/* Caller pass Ref ~ 2x for overflow range */
static void Angle_SpeedRef_Init(Angle_SpeedUnitRef_T * p_ref, angle16_t maxAngle16) { *p_ref = ANGLE_SPEED_FRACT_REF(maxAngle16); }
static void Angle_SpeedRef_Init_Rpm(Angle_SpeedUnitRef_T * p_ref, uint32_t pollingFreq, uint32_t maxRpm) { *p_ref = ANGLE_SPEED_FRACT_REF_FROM_CALIB(pollingFreq, maxRpm); }

/*
    Precomputed unit conversions — operate on shifted (Q16.16) delta.
    All hot-path consumers keep Delta in shifted form; the helpers below
    read/write from that representation directly.
*/
/* ANGLE_PER_REVOLUTION / FRACT16_MAX == 2 */
/*
    effectively θ_step = ω_pu · (ω_base / Fs)
*/
static inline angle16_t angle_of_speed_fract16(angle16_t angleSpeedMax, int16_t speed_fract16) { return fract16_mul(speed_fract16, angleSpeedMax); }
static inline angle32_t angle32_of_speed_fract16(angle16_t angleSpeedMax, int16_t speed_fract16) { return (int32_t)speed_fract16 * angleSpeedMax << 1; }
static inline int16_t speed_fract16_of_angle(uint32_t angleSpeedMaxInv_fract32, angle16_t angle16) { return ((int32_t)angle16 * angleSpeedMaxInv_fract32) >> 16U; }

/* ω_pu = el_delta · π · Fs / ω_base */
/* π · Fs / ω_base = 32768 / ω_base_angle */
static inline int16_t speed_fract16_of_angle_direct(angle16_t angleSpeedMax, angle16_t angle16) { return ((int32_t)angle16 * FRACT16_MAX) / angleSpeedMax; }


static inline void Angle_CaptureSpeed_Fract16(Angle_T * p_angle, const Angle_SpeedUnitRef_T * p_ref, accum32_t speed_fract16)
{
    p_angle->Delta = (int32_t)speed_fract16 * p_ref->SpeedMax_Angle16 << 1;
    // Angle_SetDelta(p_angle, angle_of_speed_fract16(p_ref->SpeedMax_Angle16, speed_fract16));
}

static inline fract16_t Angle_ResolveSpeed_Fract16(const Angle_T * p_angle, const Angle_SpeedUnitRef_T * p_ref)
{
    return speed_fract16_of_angle(p_ref->InvSpeedMax_Fract32, p_angle->Delta >> ANGLE32_SHIFT);
}

static inline angle16_t Angle_IntegrateSpeed_Fract16(Angle_T * p_angle, const Angle_SpeedUnitRef_T * p_ref, fract16_t speed_fract16)
{
    return Angle_Integrate(p_angle, angle_of_speed_fract16(p_ref->SpeedMax_Angle16, speed_fract16));
}


