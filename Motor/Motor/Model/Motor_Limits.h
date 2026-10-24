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
    @file   Motor_Limits.h
    @author FireSourcery
    @brief
*/
/******************************************************************************/
#include "Math/Fixed/fract16.h"
#include "Math/math_general.h"
#include <stdint.h>
#include <stdbool.h>

/*
    Envelope. The system-frame fan-in: magnitudes per frame, PU of the board I base. INT16_MAX => no cap.
    Mot/Gen keyed by the motoring sign, Fwd/Rev by the forward sign. Config holds the same type.
*/
typedef struct Motor_ILimit { uint16_t Motoring; uint16_t Generating; uint16_t Forward; uint16_t Reverse; } Motor_ILimit_T;

static const Motor_ILimit_T MOTOR_I_LIMIT_NONE = { .Motoring = INT16_MAX, .Generating = INT16_MAX, .Forward = INT16_MAX, .Reverse = INT16_MAX, };

static inline Motor_ILimit_T Motor_ILimit_Min(Motor_ILimit_T a, Motor_ILimit_T b)
{
    return (Motor_ILimit_T) { .Motoring = math_min(a.Motoring, b.Motoring), .Generating = math_min(a.Generating, b.Generating), .Forward = math_min(a.Forward, b.Forward), .Reverse = math_min(a.Reverse, b.Reverse), };
}

/* Ratios of a base, the derate form. Mot/Gen only: the intersect caps both directions */
static inline Motor_ILimit_T Motor_ILimit_Derate(Motor_ILimit_T base, ufract16_t motoring, ufract16_t generating)
{
    return (Motor_ILimit_T) { .Motoring = fract16_mul(motoring, base.Motoring), .Generating = fract16_mul(generating, base.Generating), .Forward = base.Forward, .Reverse = base.Reverse, };
}

/* [Cw:Ccw] = Mot/Gen along motoring ∩ Fwd/Rev along forward */
static inline interval_t Motor_ILimit_Resolve(Motor_ILimit_T limit, sign_t forward, sign_t motoring)
{
    return interval_intersect(interval_of_sign(motoring, limit.Motoring, limit.Generating), interval_of_sign(forward, limit.Forward, limit.Reverse));
}
// static inline interval_t Motor_ILimit_Resolve(struct Motor_ILimit * p_ilimit, sign_t directionCalibration, sign_t direction)
// {
//     return interval_of_sign(direction, p_ilimit->Motoring, p_ilimit->Generating);
// }
// return interval_intersect(interval_intersect(
//     interval_of_sign((sign_t)Motor_Direction(p_motor), Motor_ILimitMotoring(p_motor), Motor_ILimitGenerating(p_motor)),
//     interval_of_sign((sign_t)p_motor->Config.DirectionForward, p_motor->ILimit.Forward, p_motor->ILimit.Reverse)),
//     p_motor->ISupplyLimits);