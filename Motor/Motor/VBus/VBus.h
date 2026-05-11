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
    @file   VBus.h
    @author FireSourcery
    @brief  DC bus voltage module.

            Owns:
              - Live filtered ratio (VBus_Fract16, 1/VBus) for FOC Vout normalization.
              - Battery / pack profile and derating thresholds.
              - I-limit and speed-limit derating curves.

            Does NOT own:
              - ADC channel binding / ADCU<->Volts unit conversion — held by
                the layer that binds the physical channel (MotorController).
*/
/******************************************************************************/
#include "VBus_Config.h"
#include "../Phase_Input/Phase_Calibration.h"
#include "Transducer/Monitor/Voltage/VMonitor.h"

#include "Math/Fixed/fract16.h"
#include "Math/Linear/linear_math.h"

#include <stdint.h>
#include <stdbool.h>


/*
    Unified VBus concept for cohesion across MotorController and Motor layers.
    This keeps all bus voltage concepts together in one place, instead of scattering them across multiple layers.
    - Motor's FOC/SVPWM code reads VBus_Fract16 and PerV_Fract32 for voltage normalization and derating calculations.
    - Controller layer calls VBus_CaptureFract16 / VBus_PollMonitor:
*/
/*
    PHASE_CALIBRATION.V_MAX -> ADC Saturation
    PHASE_CALIBRATION.V_RATED -> controller voltage max
    Config.Nominal -> user set nominal voltage
    VBus -> live voltage
*/
/******************************************************************************/
/*!
    VBus — live state + embedded config
*/
/******************************************************************************/
typedef struct VBus
{
    uint16_t VBus_Fract16;          /* Filtered live ratio — canonical FOC input */
    uint32_t PerV_Fract32;          /* 1 / VBus, fract32-shifted — cached for Vout normalization */

    /* Precomputed derate slopes */
    int32_t IDerateUnderVSlope;     /* ((FRACT16_MAX - floor) << 15) / (WarnLow - FaultUnder), positive, rising  */
    int32_t IDerateOverVSlope;      /* ((floor - FRACT16_MAX) << 15) / (FaultOver - WarnHigh), negative, falling */

    VMonitor_State_T MonitorState;
    VBus_Config_T Config;           /* Battery profile + derate shape. Loaded from NVM at init. */
    /* Caller holds Nvm Address */
}
VBus_T;

/******************************************************************************/
/*!
    Capture / Init
*/
/******************************************************************************/
/* Phase scaling without filter */
static inline void _VBus_Capture(VBus_T * p_vbus, uint16_t fract16)
{
    p_vbus->VBus_Fract16 = fract16;
    p_vbus->PerV_Fract32 = (uint32_t)FRACT16_MAX * 65536U / p_vbus->VBus_Fract16;
}

/*
    Rolling two-sample filter; matches the prior Phase_VBus single-pole behavior.
*/
static inline void VBus_CaptureFract16(VBus_T * p_vbus, uint16_t fract16)
{
    _VBus_Capture(p_vbus, (fract16 + p_vbus->VBus_Fract16) / 2U);
}


/*
    Seed live state to VSupplyNominal before the first ADC sample lands.
*/
static inline void VBus_InitLive(VBus_T * p_vbus)
{
    _VBus_Capture(p_vbus, Phase_V_Fract16OfVolts(p_vbus->Config.VSupplyNominal_V));
    // p_vbus->PerVNominal_Fract32 = (uint32_t)FRACT16_MAX * 65536U / Phase_V_Fract16OfVolts(p_vbus->Config.VSupplyNominal_V);
}

/* Precompute rising/falling slopes from config thresholds + floor values. Shift=15 is safe for fract16 range (max 32767<<15 = 1.07B < INT32_MAX). */
static inline void VBus_InitDerateSlopes(VBus_T * p_vbus)
{
    int32_t underSpan = (int32_t)p_vbus->Config.MonitorConfig.Warning.LimitLow - p_vbus->Config.MonitorConfig.Fault.LimitLow;
    int32_t overSpan  = (int32_t)p_vbus->Config.MonitorConfig.Fault.LimitHigh - p_vbus->Config.MonitorConfig.Warning.LimitHigh;
    p_vbus->IDerateUnderVSlope = (underSpan > 0) ? (((int32_t)FRACT16_MAX - p_vbus->Config.IDerateUnderVFloor_Fract16) << 15) / underSpan : 0;
    p_vbus->IDerateOverVSlope  = (overSpan  > 0) ? (((int32_t)p_vbus->Config.IDerateOverVFloor_Fract16 - FRACT16_MAX)  << 15) / overSpan  : 0;
}

static inline void VBus_InitFrom(VBus_T * p_vbus, const VBus_Config_T * p_config)
{
    if (p_config != NULL) { p_vbus->Config = *p_config; }
    p_vbus->Config.MonitorConfig.Nominal = Phase_V_Fract16OfVolts(p_vbus->Config.VSupplyNominal_V); /* keep in sync */
    RangeMonitor_InitFrom(&p_vbus->MonitorState, &p_vbus->Config.MonitorConfig);
    VBus_InitDerateSlopes(p_vbus);
    VBus_InitLive(p_vbus);
}


/*
    Optionally as Adc callback, per control cycle
*/
#include "../Phase_Input/Phase_Analog.h" /* for Phase_Analog_VFract16Of() */
static inline void VBus_Analog_Capture(VBus_T * p_vbus, adc_result_t adcu) { VBus_CaptureFract16(p_vbus, Phase_Analog_VFract16Of(adcu)); }
/* optionally  */
// static inline void VBus_Analog_Capture(VBus_T * p_vbus, adc_result_t adcu) { _VBus_Capture(p_vbus, Phase_Analog_VFract16Of(adcu)); }


/******************************************************************************/
/*!
    Accessors — live ratio (FOC / modulation)
    Motor Access
*/
/******************************************************************************/
static inline ufract16_t VBus_Fract16(const VBus_T * p_vbus) { return p_vbus->VBus_Fract16; }
static inline uint32_t VBus_Inv_Fract32(const VBus_T * p_vbus) { return p_vbus->PerV_Fract32; }
static inline uint16_t VBus_Volts(const VBus_T * p_vbus) { return fract16_mul(p_vbus->VBus_Fract16, Phase_Calibration_GetVMaxVolts()); }

/* Runtime phase peak references — VBus/2 (sine) and VBus/√3 (SVPWM) */
static inline ufract16_t VBus_GetVPhaseRef(const VBus_T * p_vbus) { return p_vbus->VBus_Fract16 / 2U; }
static inline ufract16_t VBus_GetVPhaseRefSvpwm(const VBus_T * p_vbus) { return fract16_mul(p_vbus->VBus_Fract16, FRACT16_1_DIV_SQRT3); }

/* Normalize a phase voltage to fraction-of-VBus (duty cycle) */
static inline ufract16_t VBus_NormOf(const VBus_T * p_vbus, fract16_t phaseV) { return (int32_t)phaseV * p_vbus->PerV_Fract32 / 65536; }
static inline interval_t VBus_AntiPluggingLimits(const VBus_T * p_vbus, sign_t direction) { return interval_of_sign(direction, VBus_GetVPhaseRefSvpwm(p_vbus)); }



/******************************************************************************/
/*!
    Derating — battery droop (UV ramp) and regen chopping (OV ramp).
    MotorController owns the arbitration array

    Battery VBus spans a wide range: peak (freshly charged, no load),
    nominal (VSupplyRef), through sag under load, down to undervoltage cutoff.
    Two independent derating windows compensate:

      Undervoltage window  [VCut_Under:VFull]
        Shrinks current limit and/or speed limit as VBus falls.
        Purpose: prevent load-induced sag from tripping the UV fault,
        preserve battery runtime, respect chemistry discharge curves.

      Overvoltage window    [VFull_Regen:VCut_Over]
        Shrinks regen current as VBus rises.
        Purpose: prevent regen-induced OV fault, protect caps and pack.

    Linear two-point ramp. A floor keeps motion possible inside the
    warning region instead of cutting to zero. All thresholds are in
    fract16 VBus units so derating shares the same scale as the monitor.
*/
/******************************************************************************/
/*
    Undervoltage I-limit derate — ramp rising with VBus. (battery droop).
      VBus >= vFull_f16 → FRACT16_MAX (full I)
      VBus <= vCut_f16  → floor       (minimum I)
    [VCut_Under:VFull] => [floor, 1]
*/
static inline ufract16_t _VBus_IDerateUnderVOf(const VBus_T * p_vbus, const VMonitor_Config_T * p_config)
{
    return (ufract16_t)linear_map_slope_sat(p_vbus->IDerateUnderVSlope, 15, p_config->Fault.LimitLow, p_config->Warning.LimitLow, p_vbus->Config.IDerateUnderVFloor_Fract16, FRACT16_MAX, p_vbus->VBus_Fract16);
}

/*
    Overvoltage regen-I derate — ramp falling with VBus. (regen chopping).
      VBus <= vFull_f16 → FRACT16_MAX (full regen)
      VBus >= vCut_f16  → floor       (minimum regen)
    [VFull_Regen:VCut_Over] => [1, floor]
*/
static inline ufract16_t _VBus_IDerateOverVOf(const VBus_T * p_vbus, const VMonitor_Config_T * p_config)
{
    return (ufract16_t)linear_map_slope_sat(p_vbus->IDerateOverVSlope, 15, p_config->Warning.LimitHigh, p_config->Fault.LimitHigh, FRACT16_MAX, p_vbus->Config.IDerateOverVFloor_Fract16, p_vbus->VBus_Fract16);
}

/* UV rising:  floor + (slope * (vBus - vCutoff) >> 15) */
static inline ufract16_t VBus_GetIDerateUnderV(const VBus_T * p_vbus) { return _VBus_IDerateUnderVOf(p_vbus, &p_vbus->Config.MonitorConfig); }

/* OV falling: FRACT16_MAX - (slope * (vBus - vWarnHigh) >> 15) */
static inline ufract16_t VBus_GetIDerateOverV(const VBus_T * p_vbus) { return _VBus_IDerateOverVOf(p_vbus, &p_vbus->Config.MonitorConfig); }

/*
    Speed-limit derate — back-EMF constraint.
      scale = clamp(VBus / VSupplyNominal, floor, 1.0)

    Max achievable speed at unity modulation ∝ VBus. Running above forces
    field weakening or saturates current; clamping keeps the drive linear.
    Floor prevents the speed ceiling from collapsing to zero when the pack sags deep.
*/
static inline ufract16_t VBus_GetSpeedDerate(const VBus_T * p_vbus)
{
    uint16_t vNominal = VBus_VNominal_Fract16(&p_vbus->Config);
    if (p_vbus->VBus_Fract16 >= vNominal) { return FRACT16_MAX; }
    return math_clamp(fract16_div(p_vbus->VBus_Fract16, vNominal), p_vbus->Config.SpeedDerateFloor_Fract16, FRACT16_MAX);
    // return math_clamp(fract16_mul(p_vbus->VBus_Fract16, p_vbus->PerVNominal_Fract32), p_vbus->Config.SpeedDerateFloor_Fract16, FRACT16_MAX);
}


/*
    Physical scale fract16
*/
// static inline ufract16_t VBus_GetILimitUnderV_Fract16(const VBus_T * p_vbus) { return fract16_mul(VBus_GetIDerateUnderV(p_vbus), Phase_Calibration_GetIRatedPeak_Fract16()); }
// static inline ufract16_t VBus_GetILimitOverV_Fract16(const VBus_T * p_vbus) { return fract16_mul(VBus_GetIDerateOverV(p_vbus), Phase_Calibration_GetIRatedPeak_Fract16()); }
/* May move to per motor calculation */
// static inline ufract16_t _VBus_GetSpeedLimit_Fract16(const VBus_T * p_vbus) { return VBus_GetSpeedDerate(p_vbus) / 2; } /* Speed rated = SpeedTypeMax / 2 = 32768/2 for all cases for now */

/******************************************************************************/
/*!
    Charge-level estimator — % of (Nominal - Fault.LimitLow) span. Rough gauge for UI.
*/
/******************************************************************************/
static inline uint32_t VBus_GetChargeLevel_Fract16(const VBus_T * p_vbus)
{
    return fract16_normalize_sat(p_vbus->Config.MonitorConfig.Fault.LimitLow, p_vbus->Config.MonitorConfig.Warning.LimitHigh, p_vbus->VBus_Fract16);
}



/******************************************************************************/
/*!
    VarId accessors
*/
/******************************************************************************/
typedef enum VBus_VarId
{
    VBUS_VAR_ID_VBUS_FRACT16,
    VBUS_VAR_ID_PER_V_FRACT32,
    VBUS_VAR_ID_CHARGE_LEVEL_FRACT16,
}
VBus_VarId_T;

static inline uint32_t VBus_VarId_Get(const VBus_T * p_vbus, VBus_VarId_T var_id)
{
    switch (var_id)
    {
        case VBUS_VAR_ID_VBUS_FRACT16: return p_vbus->VBus_Fract16;
        case VBUS_VAR_ID_PER_V_FRACT32: return p_vbus->PerV_Fract32;
        case VBUS_VAR_ID_CHARGE_LEVEL_FRACT16: return VBus_GetChargeLevel_Fract16(p_vbus);
        default: return 0U;
    }
}


/*  */
static inline uint32_t VBus_BoardId_Get(VDivider_ConfigId_T var_id)
{
    switch (var_id)
    {
        case VDIVIDER_BOARD_R1: return PHASE_ANALOG_CALIBRATION.V_PHASE_R1;
        case VDIVIDER_BOARD_R2: return PHASE_ANALOG_CALIBRATION.V_PHASE_R2;
        default: return 0U;
    }
}
