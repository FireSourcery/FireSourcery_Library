/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2021 FireSourcery / The Firebrand Forge Inc

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
    @file     VMonitor.h
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

typedef enum VMonitor_Status_Tag
{
    VMONITOR_STATUS_OK,
    VMONITOR_LIMIT_UPPER,
    VMONITOR_LIMIT_LOWER,
    VMONITOR_WARNING_UPPER,
    VMONITOR_WARNING_LOWER,
}
VMonitor_Status_T;

typedef struct __attribute__((aligned(4U))) VMonitor_Params_Tag
{
    uint16_t LimitUpper_Adcu;
    uint16_t LimitLower_Adcu;
    uint16_t WarningUpper_Adcu;
    uint16_t WarningLower_Adcu;
    uint16_t VInRef;    /* VIn 100%, frac16 use only */// todo change to adcu for default??
    bool IsMonitorEnable;
}
VMonitor_Params_T;

typedef const struct VMonitor_Config_Tag
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

static inline int32_t VMonitor_ConvertToFrac16(const VMonitor_T * p_vMonitor, uint16_t adcu)                { return Linear_Voltage_CalcFrac16(&p_vMonitor->Units, adcu); }
static inline int32_t VMonitor_ConvertToV(const VMonitor_T * p_vMonitor, uint16_t adcu, uint16_t vScalar)   { return Linear_Voltage_CalcScalarV(&p_vMonitor->Units, adcu, vScalar); }
static inline int32_t VMonitor_ConvertMilliVToAdcu(const VMonitor_T * p_vMonitor, uint32_t milliV)          { return Linear_Voltage_CalcAdcu_MilliV(&p_vMonitor->Units, milliV); }
// static inline int32_t VMonitor_ConvertToAdcu(VMonitor_T * p_vMonitor, uint16_t v, uint16_t scalar) { return Linear_Voltage_CalcAdcu_ScalarV(&p_vMonitor->Units, v, scalar); }

static inline VMonitor_Status_T VMonitor_GetStatus(const VMonitor_T * p_vMonitor)   { return (p_vMonitor->Status); }
static inline bool VMonitor_GetIsStatusLimit(const VMonitor_T * p_vMonitor)         { return ((p_vMonitor->Status == VMONITOR_LIMIT_UPPER) || (p_vMonitor->Status == VMONITOR_LIMIT_LOWER)); }
static inline bool VMonitor_GetIsStatusWarning(const VMonitor_T * p_vMonitor)       { return ((p_vMonitor->Status == VMONITOR_WARNING_UPPER) || (p_vMonitor->Status == VMONITOR_WARNING_LOWER)); }

/******************************************************************************/
/*!
    Params
*/
/******************************************************************************/
static inline void VMonitor_Enable(VMonitor_T * p_vMonitor)     { p_vMonitor->Params.IsMonitorEnable = true; }
static inline void VMonitor_Disable(VMonitor_T * p_vMonitor)    { p_vMonitor->Params.IsMonitorEnable = false; }

static inline uint16_t VMonitor_GetVInRef(const VMonitor_T * p_vMonitor)        { return p_vMonitor->Params.VInRef; }
static inline uint16_t VMonitor_GetLimitUpper(const VMonitor_T * p_vMonitor)    { return p_vMonitor->Params.LimitUpper_Adcu; }
static inline uint16_t VMonitor_GetLimitLower(const VMonitor_T * p_vMonitor)    { return p_vMonitor->Params.LimitLower_Adcu; }
static inline uint16_t VMonitor_GetWarningUpper(const VMonitor_T * p_vMonitor)  { return p_vMonitor->Params.WarningUpper_Adcu; }
static inline uint16_t VMonitor_GetWarningLower(const VMonitor_T * p_vMonitor)  { return p_vMonitor->Params.WarningLower_Adcu; }
static inline uint32_t VMonitor_GetLimitUpper_V(const VMonitor_T * p_vMonitor, uint16_t vScalar)      { return Linear_Voltage_CalcScalarV(&p_vMonitor->Units, p_vMonitor->Params.LimitUpper_Adcu, vScalar); }
static inline uint32_t VMonitor_GetLimitLower_V(const VMonitor_T * p_vMonitor, uint16_t vScalar)      { return Linear_Voltage_CalcScalarV(&p_vMonitor->Units, p_vMonitor->Params.LimitLower_Adcu, vScalar); }
static inline uint32_t VMonitor_GetWarningUpper_V(const VMonitor_T * p_vMonitor, uint16_t vScalar)    { return Linear_Voltage_CalcScalarV(&p_vMonitor->Units, p_vMonitor->Params.WarningUpper_Adcu, vScalar); }
static inline uint32_t VMonitor_GetWarningLower_V(const VMonitor_T * p_vMonitor, uint16_t vScalar)    { return Linear_Voltage_CalcScalarV(&p_vMonitor->Units, p_vMonitor->Params.WarningLower_Adcu, vScalar); }
static inline uint32_t VMonitor_GetLimitUpper_MilliV(const VMonitor_T * p_vMonitor)   { return Linear_Voltage_CalcMilliV(&p_vMonitor->Units, p_vMonitor->Params.LimitUpper_Adcu); }
static inline uint32_t VMonitor_GetLimitLower_MilliV(const VMonitor_T * p_vMonitor)   { return Linear_Voltage_CalcMilliV(&p_vMonitor->Units, p_vMonitor->Params.LimitLower_Adcu); }
static inline uint32_t VMonitor_GetWarningUpper_MilliV(const VMonitor_T * p_vMonitor) { return Linear_Voltage_CalcMilliV(&p_vMonitor->Units, p_vMonitor->Params.WarningUpper_Adcu); }
static inline uint32_t VMonitor_GetWarningLower_MilliV(const VMonitor_T * p_vMonitor) { return Linear_Voltage_CalcMilliV(&p_vMonitor->Units, p_vMonitor->Params.WarningLower_Adcu); }

static inline void VMonitor_SetLimitUpper_MilliV(VMonitor_T * p_vMonitor, uint32_t limit_mV)    { p_vMonitor->Params.LimitUpper_Adcu = Linear_Voltage_CalcAdcuInput_MilliV(&p_vMonitor->Units, limit_mV); }
static inline void VMonitor_SetLimitLower_MilliV(VMonitor_T * p_vMonitor, uint32_t limit_mV)    { p_vMonitor->Params.LimitLower_Adcu = Linear_Voltage_CalcAdcuInput_MilliV(&p_vMonitor->Units, limit_mV); }
static inline void VMonitor_SetWarningUpper_MilliV(VMonitor_T * p_vMonitor, uint32_t limit_mV)  { p_vMonitor->Params.WarningUpper_Adcu = Linear_Voltage_CalcAdcuInput_MilliV(&p_vMonitor->Units, limit_mV); }
static inline void VMonitor_SetWarningLower_MilliV(VMonitor_T * p_vMonitor, uint32_t limit_mV)  { p_vMonitor->Params.WarningLower_Adcu = Linear_Voltage_CalcAdcuInput_MilliV(&p_vMonitor->Units, limit_mV); }

/******************************************************************************/
/*!
    extern
*/
/******************************************************************************/
extern void VMonitor_Init(VMonitor_T * p_vMonitor);
extern VMonitor_Status_T VMonitor_PollStatus(VMonitor_T * p_vMonitor, uint16_t adcu);
extern void VMonitor_SetVInRef(VMonitor_T * p_vMonitor, uint32_t vInRef);
extern void VMonitor_SetLimits_MilliV(VMonitor_T * p_vMonitor, uint32_t limitLower, uint32_t limitUpper, uint32_t warningLower, uint32_t warningUpper);
extern void VMonitor_SetLimitsDefault(VMonitor_T * p_vMonitor);
#endif
