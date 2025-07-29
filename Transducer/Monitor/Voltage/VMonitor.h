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
#include "VDivider.h"

#include "../Base/RangeMonitor.h"
#include "../Base/Monitor_Config.h"

#include "Peripheral/Analog/Analog.h"

typedef uint16_t vmonitor_value_t; /* ADCU */

typedef RangeMonitor_Config_T VMonitor_Config_T; /* Use RangeMonitor_Config_T for all logic */
typedef RangeMonitor_T VMonitor_T;

typedef enum VMonitor_Status
{
    VMONITOR_STATUS_FAULT_UNDERVOLTAGE  = MONITOR_STATUS_FAULT_UNDER_LIMIT,     /* Below fault boundary */
    VMONITOR_STATUS_WARNING_LOW         = MONITOR_STATUS_WARNING_LOW,           /* In low warning region */
    VMONITOR_STATUS_NORMAL              = MONITOR_STATUS_NORMAL,                /* Within normal operating range */
    VMONITOR_STATUS_WARNING_HIGH        = MONITOR_STATUS_WARNING_HIGH,          /* In high warning region */
    VMONITOR_STATUS_FAULT_OVERVOLTAGE   = MONITOR_STATUS_FAULT_OVER_LIMIT,      /* Above fault boundary */
}
VMonitor_Status_T;

typedef const struct VMonitor_Context
{
    VMonitor_T * P_STATE;
    // void (*ON_FAULT)(void * p_context); /* Optionally callback table */
    const VMonitor_Config_T * P_NVM_CONFIG; /* NVM Config */

    Analog_Conversion_T ANALOG_CONVERSION;
    VDivider_T VDIVIDER;
    // VDivider_T * P_VDIVIDER; // pointer for writable
    Linear_T * P_LINEAR; /* if defined local unit conversion */
}
VMonitor_Context_T;

#define VMONITOR_LINEAR_ALLOC() (&(Linear_T){0})
#define VMONITOR_STATE_ALLOC() (&(VMonitor_T){0})

// #define VMONITOR_CONTEXT_INIT(p_State, p_Nvm, ConversionStruct, VDividerStruct, p_Linear) \
// { \
//     .P_STATE = (p_State), \
//     .P_NVM_CONFIG = (p_Nvm), \
//     .ANALOG_CONVERSION = (ConversionStruct), \
//     .VDIVIDER = (VDividerStruct), \
//     .P_LINEAR = (p_Linear), \
// }

// #define VMONITOR_CONTEXT_ALLOC(p_Nvm, ConversionStruct, VDividerStruct, p_Linear) VMONITOR_CONTEXT_INIT(

// #define VMonitor_ConfigId_Get RangeMonitor_ConfigId_Get

/******************************************************************************/
/*
*/
/******************************************************************************/
static inline VMonitor_Status_T VMonitor_PollStatus(const VMonitor_Context_T * p_context)
{
    return (VMonitor_Status_T)RangeMonitor_Poll(p_context->P_STATE, Analog_Conversion_GetResult(&p_context->ANALOG_CONVERSION));
}

static inline void VMonitor_MarkConversion(const VMonitor_Context_T * p_context)
{
    Analog_Conversion_Mark(&p_context->ANALOG_CONVERSION);
}

/******************************************************************************/
/*
*/
/******************************************************************************/
/* Or move to init */
/* Voltage-specific calculations */
static inline uint32_t VMonitor_ChargeLevelOfInput_Percent16(const VMonitor_T * p_voltage, int32_t input)
{
    return ((uint32_t)(input - p_voltage->Config.FaultUnderLimit.Limit) * UINT16_MAX) / (p_voltage->Config.Nominal - p_voltage->Config.FaultUnderLimit.Limit);
}

/******************************************************************************/
/*
*/
/******************************************************************************/
static inline VDivider_T * VMonitor_GetVDivider(const VMonitor_Context_T * p_context) { return (p_context != NULL) ? &p_context->VDIVIDER : NULL; }
static inline VMonitor_T * VMonitor_GetState(const VMonitor_Context_T * p_context) { return (p_context != NULL) ? p_context->P_STATE : NULL; }

/******************************************************************************/
/*
*/
/******************************************************************************/
static inline int VMonitor_VarId_Get(const VMonitor_Context_T * p_context, RangeMonitor_VarId_T id) { return RangeMonitor_VarId_Get(p_context->P_STATE, id); }

static inline int VMonitor_ConfigId_Get(const VMonitor_Context_T * p_context, RangeMonitor_ConfigId_T id) { return RangeMonitor_ConfigId_Get(p_context->P_STATE, id); }
static inline void VMonitor_ConfigId_Set(const VMonitor_Context_T * p_context, RangeMonitor_ConfigId_T id, int value) { RangeMonitor_ConfigId_Set(p_context->P_STATE, id, value); }

static inline int VMonitor_VDivider_RefId_Get(const VMonitor_Context_T * p_context, VDivider_RefId_T id) { return VDivider_RefId_Get(&p_context->VDIVIDER, id); }


/******************************************************************************/
/*
*/
/******************************************************************************/
extern void VMonitor_InitLimitsDefault(VMonitor_T * p_vMonitor, int32_t nominal, uint8_t faultPercent, uint8_t warnPercent, uint8_t hystPercent);

extern void VMonitor_InitFrom(const VMonitor_Context_T * p_context, const VMonitor_Config_T * p_config);
extern void VMonitor_Init(const VMonitor_Context_T * p_context);