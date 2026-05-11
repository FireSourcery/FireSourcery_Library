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
    @file   Phase_VBus.h
    @author FireSourcery
    @brief  Compatibility shim — Phase_VBus_* API delegates to the named
            VBus_Instance singleton (a VBus_T). Kept so existing Motor-layer
            callers (Motor_FOC, Motor_User, etc.) continue to compile while
            VBus_T owns the canonical live/config/monitor state.
*/
/******************************************************************************/
#include "../VBus/VBus.h"

#include "Math/Fixed/fract16.h"
#include <stdint.h>


// extern VBus_T VBus_Instance;
// static inline ufract16_t Phase_VBus_Fract16(const VBus_T * p_vbus)         { return VBus_Fract16(p_vbus); }
// static inline uint32_t   Phase_VBus_Inv_Fract32(const VBus_T * p_vbus)     { return VBus_Inv_Fract32(p_vbus); }
// static inline ufract16_t Phase_VBus_GetVNominal(const VBus_T * p_vbus)     { return VBus_VNominal_Fract16(&p_vbus->Config); }
// static inline ufract16_t Phase_VBus_GetVRef(const VBus_T * p_vbus)         { return VBus_GetVPhaseRef(p_vbus); }
// static inline ufract16_t Phase_VBus_GetVRefSvpwm(const VBus_T * p_vbus)   { return VBus_GetVPhaseRefSvpwm(p_vbus); }
