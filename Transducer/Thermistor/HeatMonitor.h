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
    @file   HeatMonitor_Context.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Transducer/Monitor/Monitor.h"
#include "Transducer/Thermistor/Thermistor.h"
#include "Peripheral/Analog/Analog.h"
#include "Math/Linear/Linear_ADC.h"

#include <stdint.h>
#include <stdbool.h>

/* ADCU by default */
typedef uint16_t heatmonitor_value_t; /* Use heatmonitor_value_t for all logic */

/* Use Monitor module for all logic */
typedef Monitor_T HeatMonitor_T;

/*!
    @brief  Monitor configuration for temperature monitoring
    @note   Monitor_T auto handles invert
            For NTC: Higher temperature = Lower ADC value (inverse relationship)
            For PTC: Higher temperature = Higher ADC value (direct relationship)
*/
typedef Monitor_Config_T HeatMonitor_Config_T;

/* Status alias */
typedef enum HeatMonitor_Status
{
    HEAT_MONITOR_STATUS_NORMAL = MONITOR_STATUS_NORMAL,
    HEAT_MONITOR_STATUS_WARNING_HIGH = MONITOR_STATUS_WARNING,
    HEAT_MONITOR_STATUS_FAULT_OVERHEAT = MONITOR_STATUS_FAULT,
}
HeatMonitor_Status_T;

// typedef struct HeatMonitor_Config
// {
//      HeatMonitor_Config_T Config;
//      Thermistor_Coeffs_T
// }
// HeatMonitor_Config_T;

/******************************************************************************/
/*!
    @brief  HeatMonitor Context - Contains all necessary data for per-thermistor monitoring
    @note   This is a context structure for a single heat monitor instance.
    @note   It is used to encapsulate the state and configuration of the heat monitor.
    @note   It can be used in a group context for multiple heat monitors.
*/
/******************************************************************************/
// typedef const struct HeatMonitor_Base
// {
//     HeatMonitor_T * P_STATE;
//     Linear_T * P_LIMIT_SCALAR;
//     const HeatMonitor_Config_T * P_NVM_CONFIG; /* NVM Config */
// }
// HeatMonitor_Base_T;

/*
    HeatMonitor Per Thermistor source
*/
typedef const struct HeatMonitor_Context
{
    /* HeatMonitor_Base_T */
    HeatMonitor_T * P_STATE;

    /* Overwrite for GroupContext */
    Linear_T * P_LIMIT_SCALAR;
    const HeatMonitor_Config_T * P_NVM_CONFIG;

    /* Thermistor Context */
    Analog_Conversion_T ANALOG_CONVERSION;
    Thermistor_T THERMISTOR;
    Linear_T * P_LINEAR; /* Optional for local unit conversion */
    // Linear_T * P_LINEAR_R_OHMS;  /* R per Adcu */
    // Linear_T * P_LINEAR_T_CELCIUS;
}
HeatMonitor_Context_T;

#define HEAT_MONITOR_LINEAR_ALLOC() (&(Linear_T){0})
#define HEAT_MONITOR_STATE_ALLOC() (&(HeatMonitor_T){0})
// #define HEAT_MONITOR_CONTEXT_INIT(HeatMonitorState, AnalogConversion, Thermistor, Linear) \

/******************************************************************************/
/*
    Helpers
    Monitor_T extentions
*/
/******************************************************************************/
static inline void HeatMonitor_ToLimitScalar(const HeatMonitor_T * p_heat, Linear_T * p_limitScalar)
{
    Linear_Q16_Init(p_limitScalar, p_heat->Config.Fault.Limit, p_heat->Config.Warning.Setpoint);
}

/******************************************************************************/
/*
    Single Context
*/
/******************************************************************************/
/* Heat limit calculation */
/* assert(p_heat->P_LIMIT_SCALAR != NULL) */
// static inline uint16_t HeatMonitor_ScalarLimitOfInput_Percent16(const HeatMonitor_Context_T * p_heat, int32_t input) { return Linear_Q16_Percent(p_heat->P_LIMIT_SCALAR, input); }
static inline uint16_t HeatMonitor_GetScalarLimit_Percent16(const HeatMonitor_Context_T * p_heat) { return Linear_Q16_Percent(p_heat->P_LIMIT_SCALAR, p_heat->P_STATE->LastInput); }

static inline HeatMonitor_Status_T HeatMonitor_Poll(const HeatMonitor_Context_T * p_context)
{
    Monitor_Poll(p_context->P_STATE, Analog_Conversion_GetResult(&p_context->ANALOG_CONVERSION));
    // Analog_Channel_MarkConversion(&p_context->ANALOG_CONVERSION);
    return (HeatMonitor_Status_T)Monitor_GetStatus(p_context->P_STATE);
}

static inline void HeatMonitor_MarkConversion(const HeatMonitor_Context_T * p_context) { Analog_Conversion_MarkConversion(&p_context->ANALOG_CONVERSION); }

/******************************************************************************/
/*
    VarId
*/
/******************************************************************************/
/* Optionally combine thermistor and monitor ids here */
static inline int32_t HeatMonitor_ConfigId_Get(const HeatMonitor_Context_T * p_context, Monitor_ConfigId_T id) { return Monitor_ConfigId_Get(p_context->P_STATE, id); }
static inline void HeatMonitor_ConfigId_Set(const HeatMonitor_Context_T * p_context, Monitor_ConfigId_T id, int32_t value) { Monitor_ConfigId_Set(p_context->P_STATE, id, value); }

static inline int32_t HeatMonitor_ConfigId_Thermistor_Get(const HeatMonitor_Context_T * p_context, Thermistor_ConfigId_T id) { return Thermistor_ConfigId_Get(&p_context->THERMISTOR, id); }
static inline void HeatMonitor_ConfigId_Thermistor_Set(const HeatMonitor_Context_T * p_context, Thermistor_ConfigId_T id, int32_t value) { Thermistor_ConfigId_Set(&p_context->THERMISTOR, id, value); }


/******************************************************************************/
/*
    Group Context
*/
/******************************************************************************/
// typedef struct HeatMonitor_GroupState
// {
//     /* Group Management */
//     uint8_t ActiveSensorIndex;          /* Currently active/monitored sensor */
//     uint8_t LastProcessedIndex;         /* Last sensor that was processed */
//     uint32_t GroupPollCounter;          /* Group polling cycle counter */
//     /* Group State Tracking */
//     HeatMonitor_Status_T GroupStatus;   /* Overall group status */
//     uint8_t FaultCount;                 /* Number of sensors in fault */
//     uint8_t WarningCount;               /* Number of sensors in warning */

// }
// HeatMonitor_GroupState_T;

typedef const struct HeatMonitor_GroupContext
{
    /* Array of HeatMonitor_Context_T */
    /* Include HeatMonitor_T per sensor, for individual status */
    HeatMonitor_Context_T * P_CONTEXTS;
    uint8_t COUNT;

    /* Collective State */
    HeatMonitor_T * P_STATE;
    Linear_T * P_LIMIT_SCALAR;
    const HeatMonitor_Config_T * P_NVM_CONFIG; /* NVM Config */
}
HeatMonitor_GroupContext_T;


/* Monitor_GetLastInputComparable returns value for > compare */
/* Find hottest sensor */
static inline uint8_t HeatMonitor_Group_FindHottest(const HeatMonitor_GroupContext_T * p_group)
{
    uint8_t index = 0U;
    int32_t max = 0;
    int32_t compare;

    for (uint8_t i = 0U; i < p_group->COUNT; i++)
    {
        compare = Monitor_GetLastInputComparable(p_group->P_CONTEXTS[i].P_STATE);
        if (compare > max) { max = compare; index = i; }
    }

    return index;
}

/* This function polls all sensors and returns the index of the hottest */
static inline uint8_t _HeatMonitor_Group_PollEach_Index(const HeatMonitor_GroupContext_T * p_group)
{
    uint8_t index = 0U;
    int32_t max = 0; /* Monitor_GetLastInputComparable returns value for > direction compare. sensor values > 0 */
    int32_t compare;

    for (uint8_t i = 0U; i < p_group->COUNT; i++)
    {
        HeatMonitor_Poll(&p_group->P_CONTEXTS[i]);
        compare = Monitor_GetLastInputComparable(p_group->P_CONTEXTS[i].P_STATE);
        if (compare > max) { max = compare; index = i; }
    }

    return index;
}

// static inline HeatMonitor_Context_T * _HeatMonitor_Group_PollEach_Context(const HeatMonitor_GroupContext_T * p_group)
// {
//     HeatMonitor_Context_T * p_element = NULL;
//     int32_t max = 0; /* Monitor_GetLastInputComparable returns value for > direction compare. sensor values > 0 */
//     int32_t compare;

//     for (uint8_t i = 0U; i < p_group->COUNT; i++)
//     {
//         HeatMonitor_Poll(&p_group->P_CONTEXTS[i]);
//         compare = Monitor_GetLastInputComparable(p_group->P_CONTEXTS[i].P_STATE);
//         if (compare > max)
//         {
//             max = compare;
//             p_element = &p_group->P_CONTEXTS[i];
//         }
//     }

//     return p_element;
// }

/* This function polls all sensors and returns the most severe status */
// static inline HeatMonitor_Status_T _HeatMonitor_Group_PollEach_Status(const HeatMonitor_GroupContext_T * p_group)
// {
//     HeatMonitor_Status_T worstStatus = HEAT_MONITOR_STATUS_NORMAL;
//     HeatMonitor_Status_T status;
//     // uint8_t faultCount = 0;
//     // uint8_t warningCount = 0;

//     for (uint8_t i = 0U; i < p_group->COUNT; i++)
//     {
//         status = HeatMonitor_Poll(&p_group->P_CONTEXTS[i]);
//         /* Track worst status */
//         if (status > worstStatus) { worstStatus = status; }

//         /* Track group statistics */
//         // if (status == HEAT_MONITOR_STATUS_FAULT_OVERHEAT) { faultCount++; }
//         // else if (status == HEAT_MONITOR_STATUS_WARNING_HIGH) { warningCount++; }
//     }

//     return worstStatus;
// }

/* Poll group using round-robin strategy */
// static inline HeatMonitor_Status_T HeatMonitor_Group_PollRoundRobin(HeatMonitor_GroupContext_T * p_group)
// {
//     HeatMonitor_Status_T worstStatus = HEAT_MONITOR_STATUS_NORMAL;

//     /* Poll next sensor in sequence */
//     uint8_t sensorIndex = p_group->ActiveSensorIndex;
//     HeatMonitor_Context_T * p_context = &p_group->P_CONTEXTS[sensorIndex];

//     HeatMonitor_Status_T status = HeatMonitor_Context_PollSensor(p_context);

//     /* Update group tracking */
//     if (status > worstStatus) worstStatus = status;

//     /* Advance to next sensor */
//     p_group->ActiveSensorIndex = (sensorIndex + 1) % p_group->COUNT;
//     p_group->LastProcessedIndex = sensorIndex;
//     p_group->GroupPollCounter++;

//     return worstStatus;
// }

/*
    Poll and store results into collective state,
    collective state using hottest sensor
*/
static inline HeatMonitor_Status_T HeatMonitor_Group_PollAll(const HeatMonitor_GroupContext_T * p_group)
{
    return (HeatMonitor_Status_T)Monitor_Poll(p_group->P_STATE, p_group->P_CONTEXTS[_HeatMonitor_Group_PollEach_Index(p_group)].P_STATE->LastInput);
}


/******************************************************************************/
/*
    Group Query Functions - Polling results
*/
/******************************************************************************/
static inline uint16_t HeatMonitor_Group_GetScalarLimit_Percent16(const HeatMonitor_GroupContext_T * p_group)
{
    return Linear_Q16_Percent(p_group->P_LIMIT_SCALAR, p_group->P_STATE->LastInput);
}

static inline HeatMonitor_Status_T HeatMonitor_Group_GetStatus(const HeatMonitor_GroupContext_T * p_group)
{
    return (HeatMonitor_Status_T)Monitor_GetStatus(p_group->P_STATE);
}


/******************************************************************************/
/*
    For VarId interface

    Monitor_ConfigId_Get(HeatMonitor_Group_GetMonitor(p_group, instance), id)
*/
/******************************************************************************/
static inline uint8_t HeatMonitor_Group_GetInstanceCount(const HeatMonitor_GroupContext_T * p_group) { return p_group->COUNT; }

static inline HeatMonitor_Context_T * HeatMonitor_Group_GetInstance(const HeatMonitor_GroupContext_T * p_group, uint8_t index)
{
    return (index < p_group->COUNT) ? &p_group->P_CONTEXTS[index] : NULL;
}

static inline Thermistor_T * HeatMonitor_Group_GetThermistor(const HeatMonitor_GroupContext_T * p_group, uint8_t index)
{
    HeatMonitor_Context_T * p_context = HeatMonitor_Group_GetInstance(p_group, index);
    return (p_context != NULL) ? &p_context->THERMISTOR : NULL;
}

static inline HeatMonitor_T * HeatMonitor_Group_GetMonitor(const HeatMonitor_GroupContext_T * p_group, uint8_t index)
{
    HeatMonitor_Context_T * p_context = HeatMonitor_Group_GetInstance(p_group, index);
    return (p_context != NULL) ? p_context->P_STATE : NULL;
}

// static inline int HeatMonitor_Group_VarId_Get(const HeatMonitor_GroupContext_T * p_group, Monitor_VarId_T id)
static inline int HeatMonitor_Group_MonitorVar_Get(const HeatMonitor_GroupContext_T * p_group, Monitor_VarId_T id)
{
    return Monitor_VarId_Get(p_group->P_STATE, id);
}

/* Shared Monitor Config */
static inline int HeatMonitor_Group_MonitorConfig_Get(const HeatMonitor_GroupContext_T * p_group, Monitor_ConfigId_T id)
{
    return Monitor_ConfigId_Get(p_group->P_STATE, id);
}

static inline void HeatMonitor_Group_MonitorConfig_Set(const HeatMonitor_GroupContext_T * p_group, Monitor_ConfigId_T id, int value)
{
    Monitor_ConfigId_Set(p_group->P_STATE, id, value);
}

/* instanced */
static inline int HeatMonitor_Group_ThermistorConfig_Get(const HeatMonitor_GroupContext_T * p_group, uint8_t instance, Thermistor_ConfigId_T id)
{
    return Thermistor_ConfigId_Get(HeatMonitor_Group_GetThermistor(p_group, instance), id);
}

static inline void HeatMonitor_Group_ThermistorConfig_Set(const HeatMonitor_GroupContext_T * p_group, uint8_t instance, Thermistor_ConfigId_T id, int value)
{
    Thermistor_ConfigId_Set(HeatMonitor_Group_GetThermistor(p_group, instance), id, value);
}



/******************************************************************************/
/*
    Extern
*/
/******************************************************************************/
extern void HeatMonitor_InitFrom(const HeatMonitor_Context_T * p_context, const HeatMonitor_Config_T * p_config);
extern void HeatMonitor_Init(const HeatMonitor_Context_T * p_context);
/* Initialize group with shared resources */
extern void HeatMonitor_Group_Init(const HeatMonitor_GroupContext_T * p_group);