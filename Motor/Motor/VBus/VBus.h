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

            Subject_Collaborator_Verb pattern (e.g. VBusMonitor_VBus_Poll).
*/
/******************************************************************************/
#include "../Phase_Input/Phase_Calibration.h"
#include "Transducer/Monitor/Voltage/VMonitor.h"

#include "Math/Fixed/fract16.h"
#include "Math/Linear/linear_math.h"

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*!
    NVM Config
*/
/******************************************************************************/
typedef struct VBus_Config
{
    /* Pack profile */
    uint16_t VSupplyNominal_V;              /* Pack nominal [V]. Speed-derate divisor, startup value, config-wizard anchor. */
    // uint16_t VFullPower_V;               /* Full I at/above this. Typically same as VSupplyNominal, but may be raised for headroom. */

    /* Derate floor scales — fraction of the respective limit at the ramp's saturated end */
    uint16_t IDerateUnderVFloor_Fract16;   /* I-limit scale at VLowDerate  (e.g. 0.30). Floor for VBus_IDerate_UnderV ramp */
    uint16_t IDerateOverVFloor_Fract16;    /* Regen-I scale at VHighDerate (e.g. 0.10). Floor for VBus_IDerate_OverV  ramp (regen) */
    uint16_t SpeedDerateFloor_Fract16;     /* Speed-limit scale clamp (e.g. 0.70) — back-EMF headroom preserved even at low VBus. Floor for speed derate (back-EMF ceiling) */

    // Linear_T IDerateUnderVLinear; /* Precomputed from VLowDerate, VSupplyNominal, and IDerateUnderVFloor_Fract16. */
    // Linear_T IDerateOverVLinear;

    /* Speed - derate uses VSupplyNominal (battery datasheet nominal). Running above means flux weakening territory; clamp to preserve headroom. */
    VMonitor_Config_T MonitorConfig;    /* Nominal, Warn/Fault Low/High, hysteresis, IsEnabled */
}
VBus_Config_T;

static inline uint16_t VBus_VNominal_Fract16(const VBus_Config_T * p_config) { return p_config->MonitorConfig.Nominal; }

/*
    - Regen-derate uses [VFullPower:VHighDerate] (OV ramp). Full regen below VFullPower, floor regen above VHighDerate.
    - I-derate uses [VLowDerate:VFullPower] (UV ramp). Full I above VFullPower, floor I below VLowDerate.
*/
static inline ufract16_t VBus_VHighDerate_Fract16(const VBus_Config_T * p_config) { return p_config->MonitorConfig.Warning.LimitHigh; }
static inline ufract16_t VBus_VLowDerate_Fract16(const VBus_Config_T * p_config) { return p_config->MonitorConfig.Warning.LimitLow; }

static inline uint16_t VBus_GetVSupplyNominal_V(const VBus_Config_T * p_vbus) { return p_vbus->VSupplyNominal_V; }
static inline uint16_t VBus_GetVLowDerate_V(const VBus_Config_T * p_vbus) { return Phase_V_VoltsOfFract16(p_vbus->MonitorConfig.Warning.LimitLow); }
static inline uint16_t VBus_GetVHighDerate_V(const VBus_Config_T * p_vbus) { return Phase_V_VoltsOfFract16(p_vbus->MonitorConfig.Warning.LimitHigh); }
// static inline uint16_t VBus_GetVFullPower_V(const VBus_Config_T * p_vbus) { return p_vbus->Config.VFullPower_V; }


/*
    Li-ion default config — thresholds anchored to cell chemistry, scaled around VNominal.

    Per-cell voltages (3.7 V nominal / 3.0 V cutoff / 4.2 V fresh):
        Fresh charge   4.20 V → 114% of nominal  (Warning.LimitHigh) — regen derate starts here
        Nominal        3.70 V → 100%             (Nominal)
        BMS cutoff     3.00 V →  81% of nominal  (FaultUnderLimit)
    Hardware / pre-cutoff margins (tunable per platform):
        FET-OV margin  +7% above fresh    → 122%  (FaultOverLimit)
        Pre-UV margin  +10% above cutoff  →  89%  (Warning.LimitLow) — I-derate ramp start
        Warning hyst   3% of nominal       (~1 V at 37 V)
*/
#define _VBUS_VMONITOR_CONFIG_LIION(VNominal, VMax) (VMonitor_Config_T) \
{                                                                                                               \
    .Nominal                = FRACT16((VNominal) * 1.000F / (VMax)),  /* 100% — 37 V VSupplyRef */              \
    .FaultOverLimit.Limit   = FRACT16((VNominal) * 1.220F / (VMax)),  /* 122% — 45 V — FET/cap margin */        \
    .Warning.LimitHigh      = FRACT16((VNominal) * 1.135F / (VMax)),  /* 114% — 42 V — fresh, regen chop */     \
    .Warning.LimitLow       = FRACT16((VNominal) * 0.890F / (VMax)),  /*  89% — 33 V — I-derate start */        \
    .FaultUnderLimit.Limit  = FRACT16((VNominal) * 0.810F / (VMax)),  /*  81% — 30 V — BMS cutoff */            \
    .Warning.Hysteresis     = FRACT16((VNominal) * 0.030F / (VMax)),  /*   3% — ~1 V at 37 V */                 \
    .IsEnabled              = true,                                                                             \
}

#define VBUS_CONFIG_LIION(VNominal, VMax) (VBus_Config_T) \
{                                                                                                  \
    .VSupplyNominal_V           = (VNominal),                                                    \
    .IDerateUnderVFloor_Fract16 = FRACT16(0.30F),  /* 30% I-max at ramp bottom */                \
    .IDerateOverVFloor_Fract16  = FRACT16(0.10F),  /* 10% regen at ramp top */                   \
    .SpeedDerateFloor_Fract16   = FRACT16(0.70F),  /* speed clamp never below 70% */             \
    .MonitorConfig              = _VBUS_VMONITOR_CONFIG_LIION(VNominal, VMax)                    \
}

/******************************************************************************/
/*!
    VBus — live state + embedded config

    Embedded by value, not pointer. One VBus_T per controller. Caller binds
    the instance; all public functions take VBus_T *.
*/
/******************************************************************************/
typedef struct VBus
{
    uint16_t VBus_Fract16;          /* Filtered live ratio — canonical FOC input */
    uint32_t PerV_Fract32;          /* 1 / VBus, fract32-shifted — cached for Vout normalization */

    // uint16_t IScale;        /* async capture.  */
    // uint16_t SpeedScale;    /* async capture. */
    VBus_Config_T Config;           /* Battery profile + derate shape. Loaded from NVM at init. */
    VMonitor_State_T MonitorState;  /* Embedded monitor state — no dynamic allocation, and config is copied from Config.MonitorConfig */
}
VBus_T;

/******************************************************************************/
/*!
    Capture / Init
*/
/******************************************************************************/
static inline void _VBus_Capture(VBus_T * p_vbus, uint16_t fract16)
{
    p_vbus->VBus_Fract16 = fract16;
    p_vbus->PerV_Fract32 = (uint32_t)FRACT16_MAX * 65536U / p_vbus->VBus_Fract16;
}

/*
    Rolling two-sample filter; matches the prior Phase_VBus single-pole behavior.
    Callers (typically MotorController's VBus thread) pass fract16 derived from ADCU.
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
}

static inline void VBus_InitFrom(VBus_T * p_vbus, const VBus_Config_T * p_config)
{
    if (p_config != NULL) { p_vbus->Config = *p_config; }
    p_vbus->Config.MonitorConfig.Nominal = Phase_V_Fract16OfVolts(p_vbus->Config.VSupplyNominal_V); /* ensure monitor nominal matches the nominal we just installed */
    RangeMonitor_InitFrom(&p_vbus->MonitorState, &p_vbus->Config.MonitorConfig);
    VBus_InitLive(p_vbus);
}


/******************************************************************************/
/*!
    Accessors — live ratio (FOC / modulation)
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

/******************************************************************************/
/*!
    Accessors — config (readers only; writers go through VBus_Config_* setters in VBus.c)
*/
/******************************************************************************/
// static inline const VBus_Config_T * VBus_Config(const VBus_T * p_vbus) { return &p_vbus->Config; }


/******************************************************************************/
/*!
    Derating — battery droop (UV ramp) and regen chopping (OV ramp).

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
static inline ufract16_t _VBus_IDerateUnderV(uint16_t vBus_f16, ufract16_t vCut_f16, ufract16_t vFull_f16, ufract16_t floor)
{
    return (ufract16_t)linear_map_sat(vCut_f16, vFull_f16, floor, FRACT16_MAX, vBus_f16);
}

/*
    Overvoltage regen-I derate — ramp falling with VBus. (regen chopping).
      VBus <= vFull_f16 → FRACT16_MAX (full regen)
      VBus >= vCut_f16  → floor       (minimum regen)
    [VFull_Regen:VCut_Over] => [1, floor]
*/
static inline ufract16_t _VBus_IDerateOverV(uint16_t vBus_f16, ufract16_t vFull_f16, ufract16_t vCut_f16, ufract16_t floor)
{
    return (ufract16_t)linear_map_sat(vFull_f16, vCut_f16, FRACT16_MAX, floor, vBus_f16);
}

/******************************************************************************/
/*!
    Config-bound derate — threshold args pulled from Config.
*/
/******************************************************************************/
static inline ufract16_t VBus_GetIDerateUnderV(const VBus_T * p_vbus)
{
    return _VBus_IDerateUnderV(p_vbus->VBus_Fract16, p_vbus->Config.MonitorConfig.Warning.LimitLow, p_vbus->Config.MonitorConfig.Nominal, p_vbus->Config.IDerateUnderVFloor_Fract16);
}

static inline ufract16_t VBus_GetIDerateOverV(const VBus_T * p_vbus)
{
    return _VBus_IDerateOverV(p_vbus->VBus_Fract16, p_vbus->Config.MonitorConfig.Nominal, p_vbus->Config.MonitorConfig.Warning.LimitHigh, p_vbus->Config.IDerateOverVFloor_Fract16);
}

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
    if (vNominal == 0U) { return FRACT16_MAX; }   /* fail-open on blank NVM */
    return math_clamp(fract16_div(p_vbus->VBus_Fract16, vNominal), p_vbus->Config.SpeedDerateFloor_Fract16, FRACT16_MAX);
}

/*
    May need to push to arbitration source to allow lower limits
*/
// static inline void VBus_CaptureSpeedDerate(VBus_T * p_vbus) { p_vbus->SpeedScale = VBus_GetSpeedDerate(p_vbus); }
// static inline void VBus_CaptureOverV(VBus_T * p_vbus) { p_vbus->IScale = VBus_GetIDerateOverV(p_vbus); }
// static inline void VBus_CaptureUnderV(VBus_T * p_vbus) { p_vbus->IScale = VBus_GetIDerateUnderV(p_vbus); }


/******************************************************************************/
/*!
    Charge-level estimator — % of (Nominal - FaultUnderLimit) span. Rough gauge for UI.
*/
/******************************************************************************/
static inline uint32_t VBus_GetChargeLevel_Fract16(const VBus_T * p_vbus)
{
    return fract16_normalize_sat(p_vbus->Config.MonitorConfig.FaultUnderLimit.Limit, p_vbus->Config.MonitorConfig.Warning.LimitHigh, p_vbus->VBus_Fract16);
}

