/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

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
    @file   VMonitor.h
    @author FireSourcery
    @brief     Voltage divider unit conversion, plus status monitor
    @version V0
*/
/******************************************************************************/
#ifndef VMONITOR_H
#define VMONITOR_H

#include "Config.h"
#include "Peripheral/Analog/Global_Analog.h"
#include "Math/Linear/Linear_Voltage.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef enum VMonitor_Status
{
    VMONITOR_STATUS_OK,
    VMONITOR_FAULT_UPPER,
    VMONITOR_FAULT_LOWER,
    VMONITOR_WARNING_UPPER,
    VMONITOR_WARNING_LOWER,
}
VMonitor_Status_T;

typedef struct VMonitor_Config
{
    uint16_t FaultUpper_Adcu;
    uint16_t FaultLower_Adcu;
    uint16_t WarningUpper_Adcu;
    uint16_t WarningLower_Adcu;
    uint16_t Nominal_Adcu; /* VIn 100%, for Defaults and Charge */
    bool IsMonitorEnable;
    // uint16_t VInRef;    /* VIn 100%, fract16 use only, alternatively as adcu */
    // Linear_T LinearChargeLevel; /* return value [VRef:FaultLower_Adcu] as [65535:0] */
}
VMonitor_Config_T;

typedef const struct VMonitor_Const
{
    const uint32_t UNITS_R1;
    const uint32_t UNITS_R2;
    const VMonitor_Config_T * const P_CONFIG;
}
VMonitor_Const_T;

typedef struct
{
    VMonitor_Const_T CONST;
    VMonitor_Config_T Config;
    Linear_T Units;
    VMonitor_Status_T Status; /* Store status for async polling */
}
VMonitor_T;

#define VMONITOR_INIT(R1, R2, p_Config) \
{                                       \
    .CONST =                            \
    {                                   \
        .P_CONFIG = p_Config,           \
        .UNITS_R1 = R1,                 \
        .UNITS_R2 = R2,                 \
    },                                  \
}

/*
    VMONITOR_INIT_NAMED(
        .P_CONFIG = p_Config,
        .UNITS_R1 = R1,
        .UNITS_R2 = R2,
    )
*/
// #define VMONITOR_INIT_NAMED(...) \
// {                                       \
//     .CONST =                            \
//     {                                   \
//         __VA_ARGS__                     \
//     },                                  \
// }


typedef enum MotVarId_Config_VMonitor
{
    MOT_VAR_VMONITOR_R1, // All instance Read-Only
    MOT_VAR_VMONITOR_R2, // All instance Read-Only
    MOT_VAR_VMONITOR_FAULT_UPPER_ADCU,
    MOT_VAR_VMONITOR_FAULT_LOWER_ADCU,
    MOT_VAR_VMONITOR_WARNING_UPPER_ADCU,
    MOT_VAR_VMONITOR_WARNING_LOWER_ADCU,
    MOT_VAR_VMONITOR_IS_ENABLE,
}
MotVarId_Config_VMonitor_T;


static inline VMonitor_Status_T VMonitor_GetStatus(const VMonitor_T * p_vMonitor)   { return (p_vMonitor->Status); }
static inline bool VMonitor_IsFault(const VMonitor_T * p_vMonitor)               { return ((p_vMonitor->Status == VMONITOR_FAULT_UPPER) || (p_vMonitor->Status == VMONITOR_FAULT_LOWER)); }
static inline bool VMonitor_IsWarning(const VMonitor_T * p_vMonitor)             { return ((p_vMonitor->Status == VMONITOR_WARNING_UPPER) || (p_vMonitor->Status == VMONITOR_WARNING_LOWER)); }

/******************************************************************************/
/*!
    Local Unit Conversion
*/
/******************************************************************************/
static inline int32_t VMonitor_ScalarVOf(const VMonitor_T * p_vMonitor, uint16_t adcu, uint16_t vScalar) { return Linear_Voltage_ScalarV(&p_vMonitor->Units, adcu, vScalar); }
static inline int32_t VMonitor_AdcuOfMilliV(const VMonitor_T * p_vMonitor, uint32_t milliV)              { return Linear_Voltage_AdcuOfMilliV(&p_vMonitor->Units, milliV); }
static inline int32_t VMonitor_AdcuOfV(VMonitor_T * p_vMonitor, uint16_t v)                              { return Linear_Voltage_AdcuOfV(&p_vMonitor->Units, v); }

/* Map [0:VRef] to [0:65535] */
// static inline int32_t VMonitor_ConvertToPercent16(const VMonitor_T * p_vMonitor, uint16_t adcu)       { return Linear_Voltage_CalcPercent16(&p_vMonitor->Units, adcu); }
/******************************************************************************/
/*!
    Map [VNominalRef:FaultLower] to [65535:0]
*/
/******************************************************************************/
static inline uint32_t VMonitor_ChargeLevelOfAdcu_Percent16(const VMonitor_T * p_vMonitor, uint16_t adcu)
{
   return ((uint32_t)(adcu - p_vMonitor->Config.FaultLower_Adcu) * UINT16_MAX) / (p_vMonitor->Config.Nominal_Adcu - p_vMonitor->Config.FaultLower_Adcu);
}

/******************************************************************************/
/*!
    Config
*/
/******************************************************************************/
static inline bool VMonitor_IsEnable(const VMonitor_T * p_vMonitor)             { return p_vMonitor->Config.IsMonitorEnable; }
static inline uint16_t VMonitor_GetNominal(const VMonitor_T * p_vMonitor)       { return p_vMonitor->Config.Nominal_Adcu; }
static inline uint16_t VMonitor_GetFaultUpper(const VMonitor_T * p_vMonitor)    { return p_vMonitor->Config.FaultUpper_Adcu; }
static inline uint16_t VMonitor_GetFaultLower(const VMonitor_T * p_vMonitor)    { return p_vMonitor->Config.FaultLower_Adcu; }
static inline uint16_t VMonitor_GetWarningUpper(const VMonitor_T * p_vMonitor)  { return p_vMonitor->Config.WarningUpper_Adcu; }
static inline uint16_t VMonitor_GetWarningLower(const VMonitor_T * p_vMonitor)  { return p_vMonitor->Config.WarningLower_Adcu; }
// static inline uint16_t VMonitor_GetVInRef(const VMonitor_T * p_vMonitor)        { return p_vMonitor->Config.VInRef; }

static inline void VMonitor_Enable(VMonitor_T * p_vMonitor)                                     { p_vMonitor->Config.IsMonitorEnable = true; }
static inline void VMonitor_Disable(VMonitor_T * p_vMonitor)                                    { p_vMonitor->Config.IsMonitorEnable = false; }
static inline void VMonitor_SetIsEnable(VMonitor_T * p_vMonitor, bool isEnable)                 { p_vMonitor->Config.IsMonitorEnable = isEnable; }

/* Caller handle validation */
static inline void VMonitor_SetFaultUpper(VMonitor_T * p_vMonitor, uint16_t limit_adcu)         { p_vMonitor->Config.FaultUpper_Adcu = limit_adcu; }
static inline void VMonitor_SetFaultLower(VMonitor_T * p_vMonitor, uint16_t limit_adcu)         { p_vMonitor->Config.FaultLower_Adcu = limit_adcu; }
static inline void VMonitor_SetWarningUpper(VMonitor_T * p_vMonitor, uint16_t limit_adcu)       { p_vMonitor->Config.WarningUpper_Adcu = limit_adcu; }
static inline void VMonitor_SetWarningLower(VMonitor_T * p_vMonitor, uint16_t limit_adcu)       { p_vMonitor->Config.WarningLower_Adcu = limit_adcu; }
static inline void VMonitor_SetNominal(VMonitor_T * p_vMonitor, uint16_t nominal_adcu)          { p_vMonitor->Config.Nominal_Adcu = nominal_adcu; }

static inline uint32_t VMonitor_GetFaultUpper_V(const VMonitor_T * p_vMonitor, uint16_t vScalar)      { return Linear_Voltage_ScalarV(&p_vMonitor->Units, p_vMonitor->Config.FaultUpper_Adcu, vScalar); }
static inline uint32_t VMonitor_GetFaultLower_V(const VMonitor_T * p_vMonitor, uint16_t vScalar)      { return Linear_Voltage_ScalarV(&p_vMonitor->Units, p_vMonitor->Config.FaultLower_Adcu, vScalar); }
static inline uint32_t VMonitor_GetWarningUpper_V(const VMonitor_T * p_vMonitor, uint16_t vScalar)    { return Linear_Voltage_ScalarV(&p_vMonitor->Units, p_vMonitor->Config.WarningUpper_Adcu, vScalar); }
static inline uint32_t VMonitor_GetWarningLower_V(const VMonitor_T * p_vMonitor, uint16_t vScalar)    { return Linear_Voltage_ScalarV(&p_vMonitor->Units, p_vMonitor->Config.WarningLower_Adcu, vScalar); }
static inline uint32_t VMonitor_GetFaultUpper_MilliV(const VMonitor_T * p_vMonitor)             { return Linear_Voltage_MilliV(&p_vMonitor->Units, p_vMonitor->Config.FaultUpper_Adcu); }
static inline uint32_t VMonitor_GetFaultLower_MilliV(const VMonitor_T * p_vMonitor)             { return Linear_Voltage_MilliV(&p_vMonitor->Units, p_vMonitor->Config.FaultLower_Adcu); }
static inline uint32_t VMonitor_GetWarningUpper_MilliV(const VMonitor_T * p_vMonitor)           { return Linear_Voltage_MilliV(&p_vMonitor->Units, p_vMonitor->Config.WarningUpper_Adcu); }
static inline uint32_t VMonitor_GetWarningLower_MilliV(const VMonitor_T * p_vMonitor)           { return Linear_Voltage_MilliV(&p_vMonitor->Units, p_vMonitor->Config.WarningLower_Adcu); }
static inline void VMonitor_SetFaultUpper_MilliV(VMonitor_T * p_vMonitor, uint32_t limit_mV)    { p_vMonitor->Config.FaultUpper_Adcu = Linear_Voltage_AdcuInputOfMilliV(&p_vMonitor->Units, limit_mV); }
static inline void VMonitor_SetFaultLower_MilliV(VMonitor_T * p_vMonitor, uint32_t limit_mV)    { p_vMonitor->Config.FaultLower_Adcu = Linear_Voltage_AdcuInputOfMilliV(&p_vMonitor->Units, limit_mV); }
static inline void VMonitor_SetWarningUpper_MilliV(VMonitor_T * p_vMonitor, uint32_t limit_mV)  { p_vMonitor->Config.WarningUpper_Adcu = Linear_Voltage_AdcuInputOfMilliV(&p_vMonitor->Units, limit_mV); }
static inline void VMonitor_SetWarningLower_MilliV(VMonitor_T * p_vMonitor, uint32_t limit_mV)  { p_vMonitor->Config.WarningLower_Adcu = Linear_Voltage_AdcuInputOfMilliV(&p_vMonitor->Units, limit_mV); }

static inline void VMonitor_SetNominal_MilliV(VMonitor_T * p_vMonitor, uint32_t mV)             { p_vMonitor->Config.Nominal_Adcu = Linear_Voltage_AdcuInputOfMilliV(&p_vMonitor->Units, mV); }

/******************************************************************************/
/*!
    Extern
*/
/******************************************************************************/
extern void VMonitor_Init(VMonitor_T * p_vMonitor);
extern VMonitor_Status_T VMonitor_PollStatus(VMonitor_T * p_vMonitor, uint16_t adcu);
extern void VMonitor_ResetLimitsDefault(VMonitor_T * p_vMonitor);
// extern void VMonitor_SetVInRef(VMonitor_T * p_vMonitor, uint32_t vInRef);
#endif
