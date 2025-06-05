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
    @file   VMonitor.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Config.h"
#include "Transducer/Hysteresis/Hysteresis.h"
#include "Peripheral/Analog/AnalogReference.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

// typedef enum VMonitor_Status
// {
//     VMONITOR_STATUS_FAULT_UNDERVOLTAGE,     /* Below fault boundary */
//     VMONITOR_STATUS_WARNING_LOW,            /* In low warning region */
//     VMONITOR_STATUS_NORMAL = 0,             /* Within normal operating range */
//     VMONITOR_STATUS_WARNING_HIGH,           /* In high warning region */
//     VMONITOR_STATUS_FAULT_OVERVOLTAGE,      /* Above fault boundary */
// }
// VMonitor_Status_T;

typedef enum VMonitor_Status
{
    VMONITOR_STATUS_OK,
    VMONITOR_FAULT_UPPER,
    VMONITOR_FAULT_LOWER,
    VMONITOR_WARNING_UPPER,
    VMONITOR_WARNING_LOWER,
}
VMonitor_Status_T;

// typedef enum VMonitor_EdgeStatus
// {
//     VMONITOR_EDGE_NONE,
//     VMONITOR_EDGE_FAULT_CLEAR,
//     VMONITOR_EDGE_FAULT_TRIGGER,
//     VMONITOR_EDGE_WARNING_LOWER_CLEAR,
//     VMONITOR_EDGE_WARNING_LOWER_TRIGGER,
//     VMONITOR_EDGE_WARNING_UPPER_CLEAR,
//     VMONITOR_EDGE_WARNING_UPPER_TRIGGER,

//     // VMONITOR_EDGE_NONE,
//     // VMONITOR_EDGE_CLEAR_ALL,
//     // VMONITOR_WARNING_LOWER_TRIGGER,
//     // VMONITOR_WARNING_UPPER_TRIGGER,
//     // VMONITOR_FAULT_TRIGGER,
// }
// VMonitor_EdgeStatus_T;

typedef struct VMonitor_Config
{
    uint16_t FaultUpper_Adcu;
    uint16_t FaultLower_Adcu;
    uint16_t WarningUpper_Adcu;
    uint16_t WarningLower_Adcu;
    uint16_t WarningHysteresis_Adcu; /* Band value */
    uint16_t Nominal_Adcu; /* VIn 100%, for Defaults and Charge */
    bool IsMonitorEnable;
}
VMonitor_Config_T;

typedef struct
{
    VMonitor_Config_T Config;
    Hysteresis_T WarningLower; /* Hysteresis for Warning Lower */
    Hysteresis_T WarningUpper;
    // uint16_t Adcu; /* Previous ADC sample */
    // uint16_t Conditioned_Adcu;
    VMonitor_Status_T Status; /* Store status for async polling */
}
VMonitor_T;

/******************************************************************************/
/*!
*/
/******************************************************************************/
typedef enum VMonitor_ConfigId
{
    // VMONITOR_CONFIG_R1, // All instance Read-Only
    // VMONITOR_CONFIG_R2, // All instance Read-Only
    VMONITOR_CONFIG_FAULT_UPPER_ADCU,
    VMONITOR_CONFIG_FAULT_UNDER_LIMIT,
    VMONITOR_CONFIG_WARNING_LIMIT_HIGH,
    VMONITOR_CONFIG_WARNING_LIMIT_LOW,
    VMONITOR_CONFIG_IS_ENABLE,
}
VMonitor_ConfigId_T;

/******************************************************************************/
/*!
*/
/******************************************************************************/
static inline VMonitor_Status_T VMonitor_GetStatus(const VMonitor_T * p_vMonitor) { return (p_vMonitor->Status); }
static inline bool VMonitor_IsFault(const VMonitor_T * p_vMonitor) { return ((p_vMonitor->Status == VMONITOR_FAULT_UPPER) || (p_vMonitor->Status == VMONITOR_FAULT_LOWER)); }
static inline bool VMonitor_IsWarning(const VMonitor_T * p_vMonitor) { return ((p_vMonitor->Status == VMONITOR_WARNING_UPPER) || (p_vMonitor->Status == VMONITOR_WARNING_LOWER)); }


/******************************************************************************/
/*!
    Config
*/
/******************************************************************************/
static inline bool VMonitor_IsEnabled(const VMonitor_T * p_vMonitor)                { return p_vMonitor->Config.IsMonitorEnable; }
static inline void VMonitor_Enable(VMonitor_T * p_vMonitor)                         { p_vMonitor->Config.IsMonitorEnable = true; }
static inline void VMonitor_Disable(VMonitor_T * p_vMonitor)                        { p_vMonitor->Config.IsMonitorEnable = false; }
static inline void VMonitor_SetOnOff(VMonitor_T * p_vMonitor, bool isEnable)        { p_vMonitor->Config.IsMonitorEnable = isEnable; }

/* Caller handle validation */
static inline uint16_t VMonitor_GetFaultUpper(const VMonitor_T * p_vMonitor)                    { return p_vMonitor->Config.FaultUpper_Adcu; }
static inline uint16_t VMonitor_GetFaultLower(const VMonitor_T * p_vMonitor)                    { return p_vMonitor->Config.FaultLower_Adcu; }
static inline uint16_t VMonitor_GetWarningUpper(const VMonitor_T * p_vMonitor)                  { return p_vMonitor->Config.WarningUpper_Adcu; }
static inline uint16_t VMonitor_GetWarningLower(const VMonitor_T * p_vMonitor)                  { return p_vMonitor->Config.WarningLower_Adcu; }
static inline uint16_t VMonitor_GetNominal(const VMonitor_T * p_vMonitor)                       { return p_vMonitor->Config.Nominal_Adcu; }
static inline void VMonitor_SetFaultUpper(VMonitor_T * p_vMonitor, uint16_t limit_adcu)         { p_vMonitor->Config.FaultUpper_Adcu = limit_adcu; }
static inline void VMonitor_SetFaultLower(VMonitor_T * p_vMonitor, uint16_t limit_adcu)         { p_vMonitor->Config.FaultLower_Adcu = limit_adcu; }
static inline void VMonitor_SetWarningUpper(VMonitor_T * p_vMonitor, uint16_t limit_adcu)       { p_vMonitor->Config.WarningUpper_Adcu = limit_adcu; }
static inline void VMonitor_SetWarningLower(VMonitor_T * p_vMonitor, uint16_t limit_adcu)       { p_vMonitor->Config.WarningLower_Adcu = limit_adcu; }
static inline void VMonitor_SetNominal(VMonitor_T * p_vMonitor, uint16_t nominal_adcu)          { p_vMonitor->Config.Nominal_Adcu = nominal_adcu; }


/******************************************************************************/
/*!
    Local Unit Conversion
*/
/******************************************************************************/
/******************************************************************************/
/*!
    Map [VNominalRef:FaultLower] to [65535:0]
*/
/******************************************************************************/
static inline uint32_t VMonitor_ChargeLevelOfAdcu_Percent16(const VMonitor_T * p_voltage, uint16_t adcu)
{
    return ((uint32_t)(adcu - p_voltage->Config.FaultLower_Adcu) * UINT16_MAX) / (p_voltage->Config.Nominal_Adcu - p_voltage->Config.FaultLower_Adcu);
}

/******************************************************************************/
/*!
    Extern
*/
/******************************************************************************/
// extern void VMonitor_Init(VMonitor_T * p_vMonitor);
extern void VMonitor_InitFrom(VMonitor_T * p_vMonitor, const VMonitor_Config_T * p_config);

extern VMonitor_Status_T VMonitor_Poll(VMonitor_T * p_vMonitor, uint16_t adcu);
extern VMonitor_Status_T VMonitor_Poll_OnInput(VMonitor_T * p_vMonitor, uint16_t adcu);
// extern VMonitor_EdgeStatus_T VMonitor_PollEdge(VMonitor_T * p_vMonitor, uint16_t adcu);

extern void VMonitor_ResetLimitsDefault(VMonitor_T * p_vMonitor);

extern int32_t VMonitor_ConfigId_Get(const VMonitor_T * p_vMonitor, VMonitor_ConfigId_T configId);
extern void VMonitor_ConfigId_Set(VMonitor_T * p_vMonitor, VMonitor_ConfigId_T configId, uint16_t value);




// static inline uint32_t VMonitor_GetFaultUpper_V(const VMonitor_T * p_vMonitor)      { return Linear_Voltage_V(&p_vMonitor->Units, p_vMonitor->Config.FaultUpper_Adcu); }
// static inline uint32_t VMonitor_GetFaultLower_V(const VMonitor_T * p_vMonitor)      { return Linear_Voltage_V(&p_vMonitor->Units, p_vMonitor->Config.FaultLower_Adcu); }
// static inline uint32_t VMonitor_GetWarningUpper_V(const VMonitor_T * p_vMonitor)    { return Linear_Voltage_V(&p_vMonitor->Units, p_vMonitor->Config.WarningUpper_Adcu); }
// static inline uint32_t VMonitor_GetWarningLower_V(const VMonitor_T * p_vMonitor)    { return Linear_Voltage_V(&p_vMonitor->Units, p_vMonitor->Config.WarningLower_Adcu); }
// static inline uint32_t VMonitor_GetFaultUpper_MilliV(const VMonitor_T * p_vMonitor)             { return Linear_Voltage_MilliV(&p_vMonitor->Units, p_vMonitor->Config.FaultUpper_Adcu); }
// static inline uint32_t VMonitor_GetFaultLower_MilliV(const VMonitor_T * p_vMonitor)             { return Linear_Voltage_MilliV(&p_vMonitor->Units, p_vMonitor->Config.FaultLower_Adcu); }
// static inline uint32_t VMonitor_GetWarningUpper_MilliV(const VMonitor_T * p_vMonitor)           { return Linear_Voltage_MilliV(&p_vMonitor->Units, p_vMonitor->Config.WarningUpper_Adcu); }
// static inline uint32_t VMonitor_GetWarningLower_MilliV(const VMonitor_T * p_vMonitor)           { return Linear_Voltage_MilliV(&p_vMonitor->Units, p_vMonitor->Config.WarningLower_Adcu); }
// static inline void VMonitor_SetFaultUpper_MilliV(VMonitor_T * p_vMonitor, uint32_t limit_mV)    { p_vMonitor->Config.FaultUpper_Adcu = Linear_Voltage_AdcuOfMilliV_Input(&p_vMonitor->Units, limit_mV); }
// static inline void VMonitor_SetFaultLower_MilliV(VMonitor_T * p_vMonitor, uint32_t limit_mV)    { p_vMonitor->Config.FaultLower_Adcu = Linear_Voltage_AdcuOfMilliV_Input(&p_vMonitor->Units, limit_mV); }
// static inline void VMonitor_SetWarningUpper_MilliV(VMonitor_T * p_vMonitor, uint32_t limit_mV)  { p_vMonitor->Config.WarningUpper_Adcu = Linear_Voltage_AdcuOfMilliV_Input(&p_vMonitor->Units, limit_mV); }
// static inline void VMonitor_SetWarningLower_MilliV(VMonitor_T * p_vMonitor, uint32_t limit_mV)  { p_vMonitor->Config.WarningLower_Adcu = Linear_Voltage_AdcuOfMilliV_Input(&p_vMonitor->Units, limit_mV); }
// static inline void VMonitor_SetNominal_MilliV(VMonitor_T * p_vMonitor, uint32_t mV)             { p_vMonitor->Config.Nominal_Adcu = Linear_Voltage_AdcuOfMilliV_Input(&p_vMonitor->Units, mV); }

