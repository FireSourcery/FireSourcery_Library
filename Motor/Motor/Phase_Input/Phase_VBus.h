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
    @file   Phase_Analog.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Phase_Calibration.h"
#include "../Phase/Phase_Types.h"

#include "Math/Fixed/fract16.h"

/******************************************************************************/
/*!
    VBus Runtime
*/
/******************************************************************************/
typedef struct Phase_VBus
{
   uint16_t VBus_Fract16;
   uint32_t PerV_Fract32;
   /* Config */
   uint16_t VNominal_Fract16; /* VSupply */
//    uint16_t SpeedRef; handle static instance for units
}
Phase_VBus_T;

/*
    static Global State
*/
extern Phase_VBus_T Phase_VBus;

static inline void _Phase_VBus_CaptureFract16(uint16_t fract16)
{
    Phase_VBus.VBus_Fract16 = fract16;
    Phase_VBus.PerV_Fract32 = (uint32_t)FRACT16_MAX * 65536U / Phase_VBus.VBus_Fract16; /* shift 16 as fract32 */
}

static void Phase_VBus_InitV(uint16_t volts)
{
    Phase_VBus.VNominal_Fract16 = fract16(volts, Phase_Calibration_GetVMaxVolts());
    _Phase_VBus_CaptureFract16(Phase_VBus.VNominal_Fract16); /* init to nominal for startup */
}

static inline void Phase_VBus_CaptureFract16(uint16_t fract16)
{
    _Phase_VBus_CaptureFract16((fract16 + Phase_VBus.VBus_Fract16) / 2);
}

static inline ufract16_t Phase_VBus_Fract16(void) { return Phase_VBus.VBus_Fract16; }
static inline uint32_t Phase_VBus_Inv_Fract32(void) { return Phase_VBus.PerV_Fract32; }

static inline uint16_t Phase_VBus_Volts(void) { return fract16_mul(Phase_VBus.VBus_Fract16, Phase_Calibration_GetVMaxVolts()); }

static inline ufract16_t Phase_VBus_GetVNominal(void) { return Phase_VBus.VNominal_Fract16; }
/* Runtime phase peak */
/* Alternatively use Nominal */
static inline ufract16_t Phase_VBus_GetVRef(void) { return Phase_VBus.VBus_Fract16 / 2; }
static inline ufract16_t Phase_VBus_GetVRefSvpwm(void) { return fract16_mul(Phase_VBus.VBus_Fract16, FRACT16_1_DIV_SQRT3); }

static inline ufract16_t Phase_VBus_NormOf(fract16_t phaseV) { return (int32_t)phaseV * Phase_VBus.PerV_Fract32 / 65536; }


