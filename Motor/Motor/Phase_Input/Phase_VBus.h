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
#include "Phase_Analog.h"
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
}
Phase_VBus_T;


/*
    static Global State
*/
extern Phase_VBus_T Phase_VBus;

static void Phase_VBus_InitV(uint16_t vSource_V)
{
    Phase_VBus.VBus_Fract16 = fract16(vSource_V, Phase_Calibration_GetVMaxVolts());
    Phase_VBus.VNominal_Fract16 = fract16(vSource_V, Phase_Calibration_GetVMaxVolts());
}

static inline void Phase_VBus_CaptureFract16(uint16_t vSource_Fract16)
{
    Phase_VBus.VBus_Fract16 = vSource_Fract16;
    Phase_VBus.PerV_Fract32 = (uint32_t)FRACT16_MAX * 65536U / Phase_VBus.VBus_Fract16; /* shift 16 as fract32 */
}

static inline void Phase_VBus_CaptureAdcu(uint16_t vSource_Adcu)
{
    Phase_VBus_CaptureFract16(Phase_Analog_VFract16Of(vSource_Adcu));
}


static inline ufract16_t Phase_VBus_Fract16(void) { return Phase_VBus.VBus_Fract16; }
static inline uint32_t Phase_VBusInv_Fract32(void) { return Phase_VBus.PerV_Fract32; }
static inline uint16_t Phase_VBus_Volts(void) { return fract16_mul(Phase_VBus.VBus_Fract16, Phase_Calibration_GetVMaxVolts()); }

/* Alternatively use Nominal */
static inline ufract16_t Phase_VBus_GetVRef(void) { return Phase_VBus.VBus_Fract16 / 2; }
static inline ufract16_t Phase_VBus_GetVRefSvpwm(void) { return fract16_mul(Phase_VBus.VBus_Fract16, FRACT16_1_DIV_SQRT3); }


// typedef const struct
// {
//   PhaseVBus_T * P_STATE;
//   VMonitor_T MONITOR;
// //   Analog_Conversion_T PHASE_ANALOG;
// }
// PhaseVBus_Context_T;

/* Singleton Const */
// extern VMonitor_Context_T PHASE_VBUS_MONITOR;

// extern PhaseVBus_Context_T PHASE_VBUS_CONTEXT;

