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
    @file   Phase_Svpwm.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/

#include "Phase.h"
#include "Math/Fixed/fract16.h"
#include "Math/Fixed/fract16.h"
#include "../Math/math_svpwm.h"

static inline void  Phase_WriteSvpwm(const Phase_T * p_phase, uint32_t vBusInv_fract32, uint16_t vA, uint16_t vB, uint16_t vC)
{
    fract16_t vNormA = svpwm_norm_vbus_inv(vBusInv_fract32, vA);
    fract16_t vNormB = svpwm_norm_vbus_inv(vBusInv_fract32, vB);
    fract16_t vNormC = svpwm_norm_vbus_inv(vBusInv_fract32, vC);
    fract16_t vDutyA;
    fract16_t vDutyB;
    fract16_t vDutyC;
    svpwm_midclamp_vbus(&vDutyA, &vDutyB, &vDutyC, vNormA, vNormB, vNormC);

    Phase_WriteDuty_Fract16(p_phase, vDutyA, vDutyB, vDutyC);
}


