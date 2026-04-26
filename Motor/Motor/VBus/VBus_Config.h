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
    @file   VBus_Config.h
    @author FireSourcery
    @brief  [Brief description of the file]
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

    VMonitor_Config_T MonitorConfig;    /* Nominal, Warn/Fault Low/High, Hysteresis. In runtime units */
}
VBus_Config_T;

static inline uint16_t VBus_VNominal_Fract16(const VBus_Config_T * p_config) { return p_config->MonitorConfig.Nominal; }
static inline uint16_t VBus_VFullPower_Fract16(const VBus_Config_T * p_config) { return p_config->MonitorConfig.Nominal; }

/*
    - Regen-derate uses [VFullPower:VHighDerate] (OV ramp). Full regen below VFullPower, floor regen above VHighDerate.
    - I-derate uses [VLowDerate:VFullPower] (UV ramp). Full I above VFullPower, floor I below VLowDerate.
*/
static inline ufract16_t VBus_VHighDerate_Fract16(const VBus_Config_T * p_config) { return p_config->MonitorConfig.Warning.LimitHigh; }
static inline ufract16_t VBus_VLowDerate_Fract16(const VBus_Config_T * p_config) { return p_config->MonitorConfig.Warning.LimitLow; }

/*  */
static inline uint16_t VBus_GetVLowDerate_V(const VBus_Config_T * p_vbus) { return Phase_V_VoltsOfFract16(p_vbus->MonitorConfig.Warning.LimitLow); }
static inline uint16_t VBus_GetVHighDerate_V(const VBus_Config_T * p_vbus) { return Phase_V_VoltsOfFract16(p_vbus->MonitorConfig.Warning.LimitHigh); }
static inline uint16_t VBus_GetVSupplyNominal_V(const VBus_Config_T * p_vbus) { return p_vbus->VSupplyNominal_V; }
static inline uint16_t VBus_GetVFullPower_V(const VBus_Config_T * p_vbus) { return p_vbus->VSupplyNominal_V; }

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


static void VBus_Config_Init_LiIon(VBus_Config_T * p_config, uint16_t vNominal_fract16)
{
    *p_config = VBUS_CONFIG_LIION(vNominal_fract16, Phase_Calibration_GetVMaxVolts());
}


/******************************************************************************/
/*!
    Validate / IsValid
*/
/******************************************************************************/
static inline void VBus_Config_Validate(VBus_Config_T * p_config)
{
    p_config->VSupplyNominal_V           = math_min(p_config->VSupplyNominal_V, Phase_Calibration_GetVRated_V());
    p_config->IDerateUnderVFloor_Fract16 = math_min(p_config->IDerateUnderVFloor_Fract16, INT16_MAX);
    p_config->IDerateOverVFloor_Fract16  = math_min(p_config->IDerateOverVFloor_Fract16,  INT16_MAX);
    p_config->SpeedDerateFloor_Fract16   = math_min(p_config->SpeedDerateFloor_Fract16,   INT16_MAX);
}

static inline bool VBus_Config_IsValid(const VBus_Config_T * p_config)
{
    return (p_config->VSupplyNominal_V != 0U)
        && (p_config->VSupplyNominal_V           <= Phase_Calibration_GetVRated_V())
        && (p_config->IDerateUnderVFloor_Fract16 <= INT16_MAX)
        && (p_config->IDerateOverVFloor_Fract16  <= INT16_MAX)
        && (p_config->SpeedDerateFloor_Fract16   <= INT16_MAX)
        && (p_config->MonitorConfig.Warning.LimitLow  < p_config->MonitorConfig.Nominal)
        && (p_config->MonitorConfig.Warning.LimitHigh > p_config->MonitorConfig.Nominal)
        && (p_config->MonitorConfig.FaultUnderLimit.Limit < p_config->MonitorConfig.Warning.LimitLow)
        && (p_config->MonitorConfig.FaultOverLimit.Limit  > p_config->MonitorConfig.Warning.LimitHigh);
}



/******************************************************************************/
/*!

*/
/******************************************************************************/
typedef enum VBus_ConfigId
{
    VBUS_CONFIG_ID_VSUPPLY_NOMINAL_V,
    VBUS_CONFIG_ID_IDERATE_UNDER_V_FLOOR,
    VBUS_CONFIG_ID_IDERATE_OVER_V_FLOOR,
    VBUS_CONFIG_ID_SPEED_DERATE_FLOOR,
}
VBus_ConfigId_T;

static int VBus_ConfigId_Get(const VBus_Config_T * p_config, VBus_ConfigId_T id)
{
    int value = 0;
    switch (id)
    {
        case VBUS_CONFIG_ID_VSUPPLY_NOMINAL_V:        value = p_config->VSupplyNominal_V; break;
        case VBUS_CONFIG_ID_IDERATE_UNDER_V_FLOOR:    value = p_config->IDerateUnderVFloor_Fract16; break;
        case VBUS_CONFIG_ID_IDERATE_OVER_V_FLOOR:     value = p_config->IDerateOverVFloor_Fract16; break;
        case VBUS_CONFIG_ID_SPEED_DERATE_FLOOR:       value = p_config->SpeedDerateFloor_Fract16; break;
        default: break;
    }
    return value;
}

static void VBus_ConfigId_Set(VBus_Config_T * p_config, VBus_ConfigId_T id, int value)
{
    switch (id)
    {
        case VBUS_CONFIG_ID_VSUPPLY_NOMINAL_V:        p_config->VSupplyNominal_V = value; break;
        case VBUS_CONFIG_ID_IDERATE_UNDER_V_FLOOR:    p_config->IDerateUnderVFloor_Fract16 = value; break;
        case VBUS_CONFIG_ID_IDERATE_OVER_V_FLOOR:     p_config->IDerateOverVFloor_Fract16 = value; break;
        case VBUS_CONFIG_ID_SPEED_DERATE_FLOOR:       p_config->SpeedDerateFloor_Fract16 = value; break;
        default: break;
    }
}