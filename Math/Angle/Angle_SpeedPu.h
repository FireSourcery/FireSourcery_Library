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
    @file   Angle_SpeedPu.h
    @author FireSourcery
    @brief  ω_pu ↔ angle_dt unit reference
*/
/******************************************************************************/
#include "Angle.h"
#include "angle_speed_math.h"
#include "../Fixed/fract16.h"

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*
    Angle_SpeedPuRef_T
    Digital Angle to Per Unit Reference
*/
/******************************************************************************/
/******************************************************************************/
/*
    Angle_SpeedPuRef_T — precomputed Δθ_base = ω_base · Ts, the dθ/dt at ω_pu = 1.

        ω       = ω_pu · ω_base                         [rad/s, definition of ω_pu]
        dθ/dt   = ω · Ts = ω_pu · Δθ_base               [angle16/dt, dt = Ts]

    Precompute Δθ_base once so the hot path is a single multiply with no division. Numerically:

        Δθ_base = (ω_base / Fs) · (65536 / 2π) = ω_base · 32768 / (π · Fs)
                = ω_base_rpm · 65536 / (60 · Fs)

    Inverse stores INT32_MAX / Δθ_base (~2^31 / Δθ_base) for the dθ/dt → ω_pu direction:
        ω_pu_fract16 = dθ/dt · 32768 / Δθ_base = (dθ/dt · Inv) >> 16
    ANGLE16_PER_REVOLUTION / FRACT16_SCALE == 2
*/
/******************************************************************************/
typedef struct Angle_SpeedPuRef
{
    angle_dt_t AngleDtBase;             /* Δθ_base — dθ/dt at ω_pu = 1.0 */
    uint32_t InvAngleDtBase_Fract32;    /* INT32_MAX / Δθ_base — inverse for dθ/dt → ω_pu */
    // uint32_t PollingFreq; /* keep for per second conversions if needed */
}
Angle_SpeedPuRef_T;

/* Config Options in [angle16/s] or RPM */
#define ANGLE_SPEED_PU_REF(angleDtBase) (Angle_SpeedPuRef_T) { .AngleDtBase = (angleDtBase), .InvAngleDtBase_Fract32 = INT32_MAX / (angleDtBase) }
#define ANGLE_SPEED_PU_REF_FROM_FREQ(Fs, angleFreqBase) ANGLE_SPEED_PU_REF(ANGLE_DT_OF_FREQ(Fs, angleFreqBase))
#define ANGLE_SPEED_PU_REF_FROM_RPM(Fs, baseRpm) ANGLE_SPEED_PU_REF(ANGLE_DT_OF_RPM(Fs, baseRpm))


// #define _ANGLE_SPEED_PU_REF(angleDtBase) (Angle_SpeedPuRef_T) { .AngleDtBase = (angleDtBase), .InvAngleDtBase_Fract32 = INT32_MAX / (angleDtBase) }
// #define ANGLE_SPEED_PU_REF(Fs, angleFreqBase) _ANGLE_SPEED_PU_REF(ANGLE_DT_OF_FREQ(Fs, angleFreqBase))
// #define ANGLE_SPEED_PU_REF_FROM_RPM(Fs, baseRpm) ANGLE_SPEED_PU_REF(ANGLE_DT_OF_RPM(Fs, baseRpm))

static inline Angle_SpeedPuRef_T Angle_SpeedPuRef(angle_dt_t angleDtBase) { return (Angle_SpeedPuRef_T) { .AngleDtBase = (angleDtBase), .InvAngleDtBase_Fract32 = INT32_MAX / (angleDtBase) }; }
static inline Angle_SpeedPuRef_T Angle_SpeedPuRef_FromFreq(uint32_t fs, angle_freq_t angleFreqBase) { return ANGLE_SPEED_PU_REF_FROM_FREQ(fs, angleFreqBase); }
static inline Angle_SpeedPuRef_T Angle_SpeedPuRef_FromRpm(uint32_t fs, uint32_t baseRpm) { return ANGLE_SPEED_PU_REF_FROM_RPM(fs, baseRpm); }

// probably deprecate
static void Angle_SpeedPuRef_Init(Angle_SpeedPuRef_T * p_ref, angle_dt_t angleDtBase) { *p_ref = ANGLE_SPEED_PU_REF(angleDtBase); }
static void Angle_SpeedPuRef_Init_Freq(Angle_SpeedPuRef_T * p_ref, uint32_t fs, angle_freq_t angleFreqBase) { *p_ref = ANGLE_SPEED_PU_REF_FROM_FREQ(fs, angleFreqBase); }
static void Angle_SpeedPuRef_Init_Rpm(Angle_SpeedPuRef_T * p_ref, uint32_t fs, uint32_t baseRpm) { *p_ref = ANGLE_SPEED_PU_REF_FROM_RPM(fs, baseRpm); }

/*
    Hot-path consumers — operate on precomputed unit conversions
*/
static inline void Angle_CaptureSpeed_Pu(Angle_T * p_angle, const Angle_SpeedPuRef_T * p_ref, accum32_t speed_pu)
{
    p_angle->Delta = (int32_t)speed_pu * p_ref->AngleDtBase << 1;
}

static inline angle16_t Angle_IntegrateSpeed_Pu(Angle_T * p_angle, const Angle_SpeedPuRef_T * p_ref, fract16_t speed_pu)
{
    Angle_CaptureSpeed_Pu(p_angle, p_ref, speed_pu);
    return Angle_IntegrateStep(p_angle);
}

/* |dθ/dt| ≤ Δθ_base bounds Inv · dθ/dt to INT32_MAX: resolved speed saturates at ±1.0 pu */
static inline fract16_t Angle_ResolveSpeed_Pu(const Angle_T * p_angle, const Angle_SpeedPuRef_T * p_ref)
{
    // return speed_fract16_of_angle(p_ref->InvAngleDtBase_Fract32, p_angle->Delta >> ANGLE32_SHIFT);
    return ((int32_t)p_ref->InvAngleDtBase_Fract32 * math_clamp(Angle_Delta(p_angle), -p_ref->AngleDtBase, p_ref->AngleDtBase)) >> 16U;
}
