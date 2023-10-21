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
#include "Peripheral/Analog/Analog/Global_Analog.h"
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

typedef struct __attribute__((aligned(4U))) VMonitor_Params
{
    uint16_t FaultUpper_Adcu;
    uint16_t FaultLower_Adcu;
    uint16_t WarningUpper_Adcu;
    uint16_t WarningLower_Adcu;
    uint16_t VInRef;    /* VIn 100%, frac16 use only */// todo change to adcu for default?
    bool IsMonitorEnable;
    // Linear_T LinearLimits; /* return value [Warning_Adcu:Fault_Adcu] as [65535:0] */
}
VMonitor_Params_T;

typedef const struct VMonitor_Config
{
    const uint16_t UNITS_R1;
    const uint16_t UNITS_R2;
    const VMonitor_Params_T * const P_PARAMS;
}
VMonitor_Config_T;

typedef struct
{
    VMonitor_Config_T CONFIG;
    VMonitor_Params_T Params;
    Linear_T Units;
    VMonitor_Status_T Status; /* Store status for async polling */
}
VMonitor_T;

#define VMONITOR_INIT(R1, R2, p_Params) \
{                                       \
    .CONFIG =                           \
    {                                   \
        .UNITS_R1 = R1,                 \
        .UNITS_R2 = R2,                 \
        .P_PARAMS = p_Params,           \
    },                                  \
}

/* Map [0:VRef] to [0:65535] */
static inline int32_t VMonitor_ConvertToScalar16(const VMonitor_T * p_vMonitor, uint16_t adcu)                  { return Linear_Voltage_CalcScalar16(&p_vMonitor->Units, adcu); }
static inline int32_t VMonitor_ConvertToScalarV(const VMonitor_T * p_vMonitor, uint16_t adcu, uint16_t vScalar) { return Linear_Voltage_CalcScalarV(&p_vMonitor->Units, adcu, vScalar); }
static inline int32_t VMonitor_ConvertMilliVToAdcu(const VMonitor_T * p_vMonitor, uint32_t milliV)              { return Linear_Voltage_CalcAdcu_MilliV(&p_vMonitor->Units, milliV); }
// static inline int32_t VMonitor_ConvertToAdcu(VMonitor_T * p_vMonitor, uint16_t v, uint16_t scalar) { return Linear_Voltage_CalcAdcu_ScalarV(&p_vMonitor->Units, v, scalar); }

/******************************************************************************/
/*
    Map [FaultLower:VRef] to [0:65535]
*/
/******************************************************************************/
// static inline uint16_t VMonitor_ConvertChargeLevel_FracU16(const VMonitor_T * p_vMonitor, uint16_t adcu)   { return Linear_ADC_CalcFracU16(&p_vMonitor->Linear , adcu); }
// static inline uint16_t VMonitor_GetChargeLevel_FracU16(const VMonitor_T * p_vMonitor)                      { return VMonitor_ConvertPercentage_FracU16(p_vMonitor, p_vMonitor->Adcu); }

static inline VMonitor_Status_T VMonitor_GetStatus(const VMonitor_T * p_vMonitor)   { return (p_vMonitor->Status); }
static inline bool VMonitor_GetIsFault(const VMonitor_T * p_vMonitor)               { return ((p_vMonitor->Status == VMONITOR_FAULT_UPPER) || (p_vMonitor->Status == VMONITOR_FAULT_LOWER)); }
static inline bool VMonitor_GetIsWarning(const VMonitor_T * p_vMonitor)             { return ((p_vMonitor->Status == VMONITOR_WARNING_UPPER) || (p_vMonitor->Status == VMONITOR_WARNING_LOWER)); }

/******************************************************************************/
/*!
    Params
*/
/******************************************************************************/

static inline bool VMonitor_GetIsEnable(const VMonitor_T * p_vMonitor)          { return p_vMonitor->Params.IsMonitorEnable; }
static inline uint16_t VMonitor_GetVInRef(const VMonitor_T * p_vMonitor)        { return p_vMonitor->Params.VInRef; }
static inline uint16_t VMonitor_GetFaultUpper(const VMonitor_T * p_vMonitor)    { return p_vMonitor->Params.FaultUpper_Adcu; }
static inline uint16_t VMonitor_GetFaultLower(const VMonitor_T * p_vMonitor)    { return p_vMonitor->Params.FaultLower_Adcu; }
static inline uint16_t VMonitor_GetWarningUpper(const VMonitor_T * p_vMonitor)  { return p_vMonitor->Params.WarningUpper_Adcu; }
static inline uint16_t VMonitor_GetWarningLower(const VMonitor_T * p_vMonitor)  { return p_vMonitor->Params.WarningLower_Adcu; }
static inline uint32_t VMonitor_GetFaultUpper_V(const VMonitor_T * p_vMonitor, uint16_t vScalar)      { return Linear_Voltage_CalcScalarV(&p_vMonitor->Units, p_vMonitor->Params.FaultUpper_Adcu, vScalar); }
static inline uint32_t VMonitor_GetFaultLower_V(const VMonitor_T * p_vMonitor, uint16_t vScalar)      { return Linear_Voltage_CalcScalarV(&p_vMonitor->Units, p_vMonitor->Params.FaultLower_Adcu, vScalar); }
static inline uint32_t VMonitor_GetWarningUpper_V(const VMonitor_T * p_vMonitor, uint16_t vScalar)    { return Linear_Voltage_CalcScalarV(&p_vMonitor->Units, p_vMonitor->Params.WarningUpper_Adcu, vScalar); }
static inline uint32_t VMonitor_GetWarningLower_V(const VMonitor_T * p_vMonitor, uint16_t vScalar)    { return Linear_Voltage_CalcScalarV(&p_vMonitor->Units, p_vMonitor->Params.WarningLower_Adcu, vScalar); }
static inline uint32_t VMonitor_GetFaultUpper_MilliV(const VMonitor_T * p_vMonitor)   { return Linear_Voltage_CalcMilliV(&p_vMonitor->Units, p_vMonitor->Params.FaultUpper_Adcu); }
static inline uint32_t VMonitor_GetFaultLower_MilliV(const VMonitor_T * p_vMonitor)   { return Linear_Voltage_CalcMilliV(&p_vMonitor->Units, p_vMonitor->Params.FaultLower_Adcu); }
static inline uint32_t VMonitor_GetWarningUpper_MilliV(const VMonitor_T * p_vMonitor) { return Linear_Voltage_CalcMilliV(&p_vMonitor->Units, p_vMonitor->Params.WarningUpper_Adcu); }
static inline uint32_t VMonitor_GetWarningLower_MilliV(const VMonitor_T * p_vMonitor) { return Linear_Voltage_CalcMilliV(&p_vMonitor->Units, p_vMonitor->Params.WarningLower_Adcu); }

static inline void VMonitor_Enable(VMonitor_T * p_vMonitor)                                     { p_vMonitor->Params.IsMonitorEnable = true; }
static inline void VMonitor_Disable(VMonitor_T * p_vMonitor)                                    { p_vMonitor->Params.IsMonitorEnable = false; }
static inline void VMonitor_SetIsEnable(VMonitor_T * p_vMonitor, bool isEnable)                 { p_vMonitor->Params.IsMonitorEnable = isEnable; }
static inline void VMonitor_SetFaultUpper(VMonitor_T * p_vMonitor, uint32_t limit_adcu)         { p_vMonitor->Params.FaultUpper_Adcu = limit_adcu; }
static inline void VMonitor_SetFaultLower(VMonitor_T * p_vMonitor, uint32_t limit_adcu)         { p_vMonitor->Params.FaultLower_Adcu = limit_adcu; }
static inline void VMonitor_SetWarningUpper(VMonitor_T * p_vMonitor, uint32_t limit_adcu)       { p_vMonitor->Params.WarningUpper_Adcu = limit_adcu; }
static inline void VMonitor_SetWarningLower(VMonitor_T * p_vMonitor, uint32_t limit_adcu)       { p_vMonitor->Params.WarningLower_Adcu = limit_adcu; }
static inline void VMonitor_SetFaultUpper_MilliV(VMonitor_T * p_vMonitor, uint32_t limit_mV)    { p_vMonitor->Params.FaultUpper_Adcu = Linear_Voltage_CalcAdcuInput_MilliV(&p_vMonitor->Units, limit_mV); }
static inline void VMonitor_SetFaultLower_MilliV(VMonitor_T * p_vMonitor, uint32_t limit_mV)    { p_vMonitor->Params.FaultLower_Adcu = Linear_Voltage_CalcAdcuInput_MilliV(&p_vMonitor->Units, limit_mV); }
static inline void VMonitor_SetWarningUpper_MilliV(VMonitor_T * p_vMonitor, uint32_t limit_mV)  { p_vMonitor->Params.WarningUpper_Adcu = Linear_Voltage_CalcAdcuInput_MilliV(&p_vMonitor->Units, limit_mV); }
static inline void VMonitor_SetWarningLower_MilliV(VMonitor_T * p_vMonitor, uint32_t limit_mV)  { p_vMonitor->Params.WarningLower_Adcu = Linear_Voltage_CalcAdcuInput_MilliV(&p_vMonitor->Units, limit_mV); }

/******************************************************************************/
/*!
    Extern
*/
/******************************************************************************/
extern void VMonitor_Init(VMonitor_T * p_vMonitor);
extern VMonitor_Status_T VMonitor_PollStatus(VMonitor_T * p_vMonitor, uint16_t adcu);
extern void VMonitor_SetVInRef(VMonitor_T * p_vMonitor, uint32_t vInRef);
extern void VMonitor_SetLimits_MilliV(VMonitor_T * p_vMonitor, uint32_t faultLower, uint32_t faultUpper, uint32_t warningLower, uint32_t warningUpper);
extern void VMonitor_SetLimitsDefault(VMonitor_T * p_vMonitor);
#endif
