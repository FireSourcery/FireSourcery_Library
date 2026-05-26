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
#include "angle_speed_math.h"
#include "../Fixed/fract16.h"

#include <stdint.h>
#include <stdbool.h>

// typedef struct
// {
//     angle16_t SpeedMax_Angle16;    /* (ω_base · Δt) in angle16/tick — angle step at ω_pu = 1.0 */
//     uint32_t InvSpeedMax_Fract32;   /* Fs / ω_base (Q32) — inverse for angle → ω_pu */
//     // uint32_t PollingFreq; /* keep for per second conversions if needed */
// }
// Angle_SpeedUnit_T;

// /* Config Options in RPM */
// typedef struct
// {
//     uint32_t PollingFreq;
//     uint32_t SpeedMax_Rpm;  /* ω_base as mechanical RPM (~2x rated) */
// }
// Angle_SpeedUnit_Rpm_T;

// static inline Angle_SpeedUnit_T Angle_SpeedUnit(angle16_t maxAngle16) { return ANGLE_SPEED_FRACT_REF(maxAngle16); }
// static inline Angle_SpeedUnit_T Angle_SpeedUnit_OfRpm(Angle_SpeedUnit_Rpm_T rpmConfig) { return ANGLE_SPEED_FRACT_REF_FROM_CALIB(rpmConfig.PollingFreq, rpmConfig.SpeedMax_Rpm); }
