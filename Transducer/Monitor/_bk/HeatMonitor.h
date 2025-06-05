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
    @file   HeatMonitor.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
// #include "Peripheral/Analog/AnalogReference.h"
#include "Math/Linear/Linear_ADC.h"

#include <string.h>
#include <math.h>

// typedef enum TemperatureMonitor_Status
// {
//     TEMPERATURE_STATUS_NORMAL = 0,
//     TEMPERATURE_STATUS_HIGH_WARNING,       /* High temperature warning */
//     TEMPERATURE_STATUS_OVERTEMPERATURE,    /* OTP - OverTemperature Protection */
// }
// TemperatureMonitor_Status_T;


/* Monitor Status Return */
typedef enum HeatMonitor_Status
{
    HEAT_MONITOR_STATUS_OK,
    HEAT_MONITOR_STATUS_WARNING,
    HEAT_MONITOR_STATUS_FAULT,
}
HeatMonitor_Status_T;

/*
    Set Vin to same decimal precision as ADC_VREF
*/
typedef struct HeatMonitor_Config
{
    /* Monitor Limits */
    uint16_t FaultTrigger_Adcu;
    uint16_t FaultClear_Adcu;
    uint16_t WarningTrigger_Adcu;
    uint16_t WarningClear_Adcu;

    bool IsMonitorEnable;
}
HeatMonitor_Config_T;

/*
    Run time state only
*/
typedef struct HeatMonitor
{
    HeatMonitor_Config_T Config;
    Linear_T HeatScalar;     /* Linear fit for warning region, return value [WarningTrigger_Adcu:FaultTrigger_Adcu] as [65535:0], Roughly linear 70-100C */
    /* State for polling compare */
    HeatMonitor_Status_T Status; /* Status is sufficient for brief State */
    uint16_t Adcu; /* Previous ADC sample */
    // uint16_t InputAdcu; /*   */
    // uint16_t OutputAdcu; /* Filtered */

    /* Cache compare values */
    int16_t FaultTriggerCompare;
    int16_t FaultClearCompare;
    int16_t WarningTriggerCompare;
    int16_t WarningClearCompare;
}
HeatMonitor_T;

/******************************************************************************/
/*

*/
/******************************************************************************/
typedef enum HeatMonitor_ConfigId
{
    HEAT_MONITOR_CONFIG_FAULT_TRIGGER_ADCU,
    HEAT_MONITOR_CONFIG_FAULT_THRESHOLD_ADCU,
    HEAT_MONITOR_CONFIG_WARNING_TRIGGER_ADCU,
    HEAT_MONITOR_CONFIG_WARNING_THRESHOLD_ADCU,
    HEAT_MONITOR_CONFIG_IS_MONITOR_ENABLE,
}
HeatMonitor_ConfigId_T;


/******************************************************************************/
/*
    HeatScalar scalar value - Fraction Inverse to heat
    [WarningTrigger_Adcu:FaultTrigger_Adcu] as [65535:0]
*/
/******************************************************************************/
static inline uint16_t HeatMonitor_ScalarLimitOfAdcu_Percent16(const HeatMonitor_T * p_heat, uint16_t adcu) { return Linear_ADC_Percent16(&p_heat->HeatScalar, adcu); }

/* Captured adcu on HeatMonitor_PollMonitor */
static inline uint16_t HeatMonitor_GetScalarLimit_Percent16(const HeatMonitor_T * p_heat) { return HeatMonitor_ScalarLimitOfAdcu_Percent16(p_heat, p_heat->Adcu); }

/******************************************************************************/
/*
    Monitor
*/
/******************************************************************************/
// static inline bool HeatMonitor_IsNtc(const HeatMonitor_T * p_heat)
// {
//     int16_t compareSign = (p_heat->Config.FaultTrigger_Adcu < p_heat->Config.WarningTrigger_Adcu) ? -1 : 1; // NTC or PTC
// }

static inline HeatMonitor_Status_T HeatMonitor_GetStatus(const HeatMonitor_T * p_heat)    { return (p_heat->Status); }
static inline bool HeatMonitor_IsFault(const HeatMonitor_T * p_heat)                      { return (p_heat->Status == HEAT_MONITOR_STATUS_FAULT); }
static inline bool HeatMonitor_IsWarning(const HeatMonitor_T * p_heat)                    { return (p_heat->Status == HEAT_MONITOR_STATUS_WARNING); }
// static inline bool HeatMonitor_IsFault(const HeatMonitor_T * p_heat)                  { return ((p_heat->Status == HEAT_MONITOR_STATUS_FAULT) || (p_heat->Status == HEAT_MONITOR_STATUS_FAULT_THRESHOLD)); }
// static inline bool HeatMonitor_IsWarning(const HeatMonitor_T * p_heat)                { return ((p_heat->Status == HEAT_MONITOR_STATUS_WARNING) || (p_heat->Status == HEAT_MONITOR_STATUS_WARNING_THRESHOLD)); }

/******************************************************************************/
/*
    Config
*/
/******************************************************************************/
static inline bool HeatMonitor_IsEnabled(const HeatMonitor_T * p_heat)                       { return p_heat->Config.IsMonitorEnable; }
static inline void HeatMonitor_EnableMonitor(HeatMonitor_T * p_heat)                         { p_heat->Config.IsMonitorEnable = true; }
static inline void HeatMonitor_DisableMonitor(HeatMonitor_T * p_heat)                        { p_heat->Config.IsMonitorEnable = false; }
static inline void HeatMonitor_SetOnOff(HeatMonitor_T * p_heat, bool isEnable)               { p_heat->Config.IsMonitorEnable = isEnable; }

static inline uint16_t HeatMonitor_GetFaultTrigger(const HeatMonitor_T * p_heat)        { return p_heat->Config.FaultTrigger_Adcu; }
static inline uint16_t HeatMonitor_GetFaultThreshold(const HeatMonitor_T * p_heat)      { return p_heat->Config.FaultClear_Adcu; }
static inline uint16_t HeatMonitor_GetWarningTrigger(const HeatMonitor_T * p_heat)      { return p_heat->Config.WarningTrigger_Adcu; }
static inline uint16_t HeatMonitor_GetWarningThreshold(const HeatMonitor_T * p_heat)    { return p_heat->Config.WarningClear_Adcu; }

/* Caller handle validation */
static inline void HeatMonitor_SetFaultTrigger(HeatMonitor_T * p_heat, uint16_t fault)                  { p_heat->Config.FaultTrigger_Adcu = fault; }
static inline void HeatMonitor_SetFaultThreshold(HeatMonitor_T * p_heat, uint16_t faultThreshold)       { p_heat->Config.FaultClear_Adcu = faultThreshold; }
static inline void HeatMonitor_SetWarningTrigger(HeatMonitor_T * p_heat, uint16_t warning)              { p_heat->Config.WarningTrigger_Adcu = warning; }
static inline void HeatMonitor_SetWarningThreshold(HeatMonitor_T * p_heat, uint16_t warningThreshold)   { p_heat->Config.WarningClear_Adcu = warningThreshold; }


/******************************************************************************/
/*
    Extern
*/
/******************************************************************************/
// extern void HeatMonitor_Init(HeatMonitor_T * p_heat);
extern void HeatMonitor_InitFrom(HeatMonitor_T * p_heat, const HeatMonitor_Config_T * p_config);

extern HeatMonitor_Status_T HeatMonitor_Poll(HeatMonitor_T * p_heat, uint16_t adcu);
extern bool HeatMonitor_PollEdge(HeatMonitor_T * p_heat, uint16_t adcu);

extern void HeatMonitor_Reset(HeatMonitor_T * p_heat);

// extern void HeatMonitor_SetFault_DegC(HeatMonitor_T * p_heat, thermal_t fault_degC, thermal_t faultThreshold_degC);
// extern void HeatMonitor_SetWarning_DegC(HeatMonitor_T * p_heat, thermal_t warning_degC, thermal_t warningThreshold_degC);
// extern void HeatMonitor_SetLimits_DegC(HeatMonitor_T * p_heat, thermal_t fault, thermal_t faultThreshold, thermal_t warning, thermal_t warningThreshold);

// extern thermal_t HeatMonitor_GetFault_DegC(const HeatMonitor_T * p_heat);
// extern thermal_t HeatMonitor_GetFaultThreshold_DegC(const HeatMonitor_T * p_heat);
// extern thermal_t HeatMonitor_GetWarning_DegC(const HeatMonitor_T * p_heat);
// extern thermal_t HeatMonitor_GetWarningThreshold_DegC(const HeatMonitor_T * p_heat);

extern int32_t _HeatMonitor_ConfigId_Get(const HeatMonitor_T * p_heat, HeatMonitor_ConfigId_T id);
extern void _HeatMonitor_ConfigId_Set(HeatMonitor_T * p_heat, HeatMonitor_ConfigId_T id, int32_t value);

extern int HeatMonitor_ConfigId_Get(const HeatMonitor_T * p_heat, int id);
extern void HeatMonitor_ConfigId_Set(HeatMonitor_T * p_heat, int id, int value);
