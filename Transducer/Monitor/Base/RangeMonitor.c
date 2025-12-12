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
    @file   RangeMonitor.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "RangeMonitor.h"


/*
    Intermediate expand RangeMonitor_Config_T to two Monitor_Config_T for high and low sides
*/
/*
    For High-Side Monitoring: Nominal < Warning < Fault
    For Low-Side Monitoring: Fault < Warning < Nominal
*/
static inline Monitor_Config_T HighConfigOf(const RangeMonitor_Config_T * p_config)
{
    /* Validate high-side thresholds are properly configured */
    bool isHighSide = (p_config->FaultOverLimit.Limit > p_config->Warning.LimitHigh) && (p_config->Warning.LimitHigh > p_config->Nominal);

    return (Monitor_Config_T)
    {
        .Warning = { .Setpoint = p_config->Warning.LimitHigh, .Resetpoint = p_config->Warning.LimitHigh - p_config->Warning.Hysteresis, },
        .Fault = p_config->FaultOverLimit,
        .Nominal = p_config->Nominal, /* or disable */
        .IsEnabled = isHighSide
    };
}

static inline Monitor_Config_T LowConfigOf(const RangeMonitor_Config_T * p_config)
{
    /* Validate low-side thresholds are properly configured */
    bool isLowSide = (p_config->FaultUnderLimit.Limit < p_config->Warning.LimitLow) && (p_config->Warning.LimitLow < p_config->Nominal);

    return (Monitor_Config_T)
    {
        .Warning = { .Setpoint = p_config->Warning.LimitLow, .Resetpoint = p_config->Warning.LimitLow + p_config->Warning.Hysteresis, },
        .Fault = p_config->FaultUnderLimit,
        .Nominal = p_config->Nominal,
        .IsEnabled = isLowSide
    };
}

bool RangeMonitor_IsConfigValid(const RangeMonitor_Config_T * p_config)
{
    // remap to two Monitor_Config_T, alternatively implement full check here
    Monitor_Config_T high = HighConfigOf(p_config);
    Monitor_Config_T low = LowConfigOf(p_config);
    return Monitor_IsConfigValid(&high) && Monitor_IsConfigValid(&low);
}

/*
    Always init both monitors. Single Direction use case directly use Monitor_T instead.
*/
void RangeMonitor_InitFrom(RangeMonitor_T * p_monitor, const RangeMonitor_Config_T * p_config)
{
    if (p_config != NULL) { p_monitor->Config = *p_config; }

    Monitor_Config_T high = HighConfigOf(&p_monitor->Config);
    Monitor_Config_T low = LowConfigOf(&p_monitor->Config);
    _Monitor_InitFrom(&p_monitor->MonitorHigh, &high);
    _Monitor_InitFrom(&p_monitor->MonitorLow, &low);

    if (Monitor_IsConfigValid(&high) == false) { p_monitor->Config.IsEnabled = false; }
    if (Monitor_IsConfigValid(&low) == false) { p_monitor->Config.IsEnabled = false; }
    /* always with both sides. single sided operation use Monitor_T */
    if (high.IsEnabled == false || low.IsEnabled == false) { p_monitor->Config.IsEnabled = false; }

    /* Reset monitor state */
    RangeMonitor_Reset(p_monitor);
}

/*!
    Monitor Threshold Layout:

    FaultOverLimit                  ←── Immediate fault protection
    WarningLimitHigh                ←── Warning region entry (trigger)
    WarningLimitHigh - Band         ←── Warning region exit (release)
    ┌─────────────────────────────── Normal Operating Range
    │ Nominal                       ←── Reference (100%)
    └─────────────────────────────── Normal Operating Range
    WarningLimitLow + Band          ←── Warning region exit (release)
    WarningLimitLow                 ←── Warning region entry (trigger)
    FaultUnderLimit                 ←── Immediate fault protection

    Hysteresis Behavior:
    - Entry: Signal crosses Entry threshold → Enter warning
    - Exit:  Signal crosses Exit threshold → Leave warning
    - Band:  Difference between Entry and Exit thresholds
*/
RangeMonitor_Status_T RangeMonitor_Evaluate(RangeMonitor_T * p_monitor, int32_t input)
{
    /* Poll individual monitors and combine results */
    /* Check the most inner limits first, for normal state */
    Monitor_Status_T highStatus = _Monitor_EvaluateAsHigh(&p_monitor->MonitorHigh, input);
    Monitor_Status_T lowStatus = _Monitor_EvaluateAsLow(&p_monitor->MonitorLow, input);
    RangeMonitor_Status_T status;

    if (highStatus > lowStatus) { status = (RangeMonitor_Status_T)highStatus; }
    else if (lowStatus > highStatus) { status = (RangeMonitor_Status_T)(lowStatus * -1); }
    else { status = RANGE_MONITOR_STATUS_NORMAL; } // Both are equal, so normal

    return status;
}

/*
    Caller latch fault if needed.
*/
RangeMonitor_Status_T RangeMonitor_Poll(RangeMonitor_T * p_monitor, int32_t input)
{
    if (p_monitor->Config.IsEnabled == true)
    {
        p_monitor->LastInput = input;
        p_monitor->StatusPrev = p_monitor->Status;
        p_monitor->Status = RangeMonitor_Evaluate(p_monitor, input);
        return p_monitor->Status;
    }

    return RANGE_MONITOR_STATUS_NORMAL;
}

void RangeMonitor_Reset(RangeMonitor_T * p_monitor)
{
    p_monitor->Status = RANGE_MONITOR_STATUS_NORMAL;
    p_monitor->StatusPrev = RANGE_MONITOR_STATUS_NORMAL;
    p_monitor->LastInput = p_monitor->Config.Nominal;

    Hysteresis_Reset(&p_monitor->MonitorHigh.Warning);
    Hysteresis_Reset(&p_monitor->MonitorLow.Warning);
}

void RangeMonitor_SetFaultOverLimit(RangeMonitor_T * p_monitor, int32_t limit) { p_monitor->Config.FaultOverLimit.Limit = limit; RangeMonitor_InitFrom(p_monitor, &p_monitor->Config); }
void RangeMonitor_SetFaultUnderLimit(RangeMonitor_T * p_monitor, int32_t limit) { p_monitor->Config.FaultUnderLimit.Limit = limit; RangeMonitor_InitFrom(p_monitor, &p_monitor->Config); }
void RangeMonitor_SetWarningLimitHigh(RangeMonitor_T * p_monitor, int32_t limit) { p_monitor->Config.Warning.LimitHigh = limit; RangeMonitor_InitFrom(p_monitor, &p_monitor->Config); }
void RangeMonitor_SetWarningLimitLow(RangeMonitor_T * p_monitor, int32_t limit) { p_monitor->Config.Warning.LimitLow = limit; RangeMonitor_InitFrom(p_monitor, &p_monitor->Config); }
void RangeMonitor_SetWarningDeadband(RangeMonitor_T * p_monitor, int32_t deadband) { p_monitor->Config.Warning.Hysteresis = deadband; RangeMonitor_InitFrom(p_monitor, &p_monitor->Config); }
void RangeMonitor_SetNominal(RangeMonitor_T * p_monitor, int32_t nominal) { p_monitor->Config.Nominal = nominal; RangeMonitor_InitFrom(p_monitor, &p_monitor->Config); }


/******************************************************************************/
/*
    By Id
*/
/******************************************************************************/
int32_t _RangeMonitor_VarId_Get(const RangeMonitor_T * p_monitor, RangeMonitor_VarId_T varId)
{
    switch (varId)
    {
        case RANGE_MONITOR_VAR_STATUS:  return (int32_t)p_monitor->Status;
        case RANGE_MONITOR_VAR_VALUE:   return p_monitor->LastInput;
        default: return 0;
    }
}

// int32_t _RangeMonitor_ConfigId_Get(const RangeMonitor_Config_T * p_monitor, RangeMonitor_ConfigId_T configId)
int32_t _RangeMonitor_ConfigId_Get(const RangeMonitor_T * p_monitor, RangeMonitor_ConfigId_T configId)
{
    switch (configId)
    {
        case RANGE_MONITOR_CONFIG_FAULT_OVER_LIMIT:         return RangeMonitor_GetFaultOverLimit(p_monitor);
        case RANGE_MONITOR_CONFIG_FAULT_UNDER_LIMIT:        return RangeMonitor_GetFaultUnderLimit(p_monitor);
        case RANGE_MONITOR_CONFIG_WARNING_LIMIT_HIGH:       return RangeMonitor_GetWarningLimitHigh(p_monitor);
        case RANGE_MONITOR_CONFIG_WARNING_LIMIT_LOW:        return RangeMonitor_GetWarningLimitLow(p_monitor);
        case RANGE_MONITOR_CONFIG_WARNING_HYSTERESIS_BAND:  return RangeMonitor_GetWarningDeadband(p_monitor);
        case RANGE_MONITOR_CONFIG_NOMINAL:                  return RangeMonitor_GetNominal(p_monitor);
        case RANGE_MONITOR_CONFIG_IS_ENABLED:               return RangeMonitor_IsEnabled(p_monitor);
        default: return 0;
    }
}

void _RangeMonitor_ConfigId_Set(RangeMonitor_T * p_monitor, RangeMonitor_ConfigId_T configId, int32_t value)
{
    switch (configId)
    {
        case RANGE_MONITOR_CONFIG_FAULT_OVER_LIMIT:         RangeMonitor_SetFaultOverLimit(p_monitor, value);      break;
        case RANGE_MONITOR_CONFIG_FAULT_UNDER_LIMIT:        RangeMonitor_SetFaultUnderLimit(p_monitor, value);     break;
        case RANGE_MONITOR_CONFIG_WARNING_LIMIT_HIGH:       RangeMonitor_SetWarningLimitHigh(p_monitor, value);    break;
        case RANGE_MONITOR_CONFIG_WARNING_LIMIT_LOW:        RangeMonitor_SetWarningLimitLow(p_monitor, value);     break;
        case RANGE_MONITOR_CONFIG_WARNING_HYSTERESIS_BAND:  RangeMonitor_SetWarningDeadband(p_monitor, value);     break;
        case RANGE_MONITOR_CONFIG_NOMINAL:                  RangeMonitor_SetNominal(p_monitor, value);             break;
        case RANGE_MONITOR_CONFIG_IS_ENABLED:               RangeMonitor_SetEnabled(p_monitor, value != 0);        break;
        default: break;
    }
}

int RangeMonitor_VarId_Get(const RangeMonitor_T * p_monitor, int varId) { return (p_monitor != NULL) ? _RangeMonitor_VarId_Get(p_monitor, (RangeMonitor_VarId_T)varId) : 0; }

int RangeMonitor_ConfigId_Get(const RangeMonitor_T * p_monitor, int configId) { return (p_monitor != NULL) ? _RangeMonitor_ConfigId_Get(p_monitor, (RangeMonitor_ConfigId_T)configId) : 0; }
void RangeMonitor_ConfigId_Set(RangeMonitor_T * p_monitor, int configId, int value) { if (p_monitor != NULL) { _RangeMonitor_ConfigId_Set(p_monitor, (RangeMonitor_ConfigId_T)configId, value); } }

// void RangeMonitor_ConfigId_SetWithReinit(RangeMonitor_T * p_monitor, int configId, int value)
// {
//     if (p_monitor != NULL)
//     {
//         _RangeMonitor_ConfigId_Set(p_monitor, (RangeMonitor_ConfigId_T)configId, value);
//         RangeMonitor_InitFrom(p_monitor, &p_monitor->Config);
//     }
// }