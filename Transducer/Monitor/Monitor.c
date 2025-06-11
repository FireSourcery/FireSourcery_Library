/******************************************************************************/
/*!
    @file   Monitor.c
    @author FireSourcery
    @brief  Generic monitor implementation
*/
/******************************************************************************/
#include "Monitor.h"

/******************************************************************************/
/*
   Base
*/
/******************************************************************************/
void _Monitor_InitFrom(Monitor_Base_T * p_monitor, const Monitor_Config_T * p_config)
{
    Hysteresis_InitThresholds(&p_monitor->Warning, p_config->Warning.Setpoint, p_config->Warning.Resetpoint);
    p_monitor->FaultLimit = p_config->Fault.Limit;
}

/******************************************************************************/
/*
    Core Monitoring Logic - Simplified Industry Standard
    Common with RangeMonitor
*/
/******************************************************************************/
/*!
    @brief  Check fault condition (hard limit, no hysteresis)
*/
static bool _Monitor_CheckFaultAsHigh(const Monitor_Base_T * p_monitor, int32_t input) { return (input >= p_monitor->FaultLimit); }

Monitor_Status_T _Monitor_EvaluateAsHigh(Monitor_Base_T * p_monitor, int32_t input)
{
    Monitor_Status_T status;

    /*
        PERFORMANCE OPTIMIZATION: Check if input is in "safe" region first

        For high-side monitoring: safe when (input <= resetpoint)
        For low-side monitoring:  safe when (input >= resetpoint)

        If in safe region, guaranteed no warning or fault conditions exist
        This avoids expensive hysteresis state machine updates for normal operation

        The most common case can skip all other checks.

        Caller must retain latching fault state after trigger
    */
    if (_Hysteresis_RegionOf(&p_monitor->Warning, input) == HYSTERESIS_REGION_OFF) { status = MONITOR_STATUS_NORMAL; }

    /* Check fault condition next (highest priority, no hysteresis) */
    else if (_Monitor_CheckFaultAsHigh(p_monitor, input) == true)    { status = MONITOR_STATUS_FAULT; }
    else if (_Hysteresis_Poll(&p_monitor->Warning, input) == true)   { status = MONITOR_STATUS_WARNING; }
    else                                                             { status = MONITOR_STATUS_NORMAL; }

    return status;
}


static bool _Monitor_CheckFaultAsLow(const Monitor_Base_T * p_monitor, int32_t input) { return (input <= p_monitor->FaultLimit); }

Monitor_Status_T _Monitor_EvaluateAsLow(Monitor_Base_T * p_monitor, int32_t input)
{
    Monitor_Status_T status;

    if (_Hysteresis_RegionOf_Inverted(&p_monitor->Warning, input) == HYSTERESIS_REGION_OFF) { status = MONITOR_STATUS_NORMAL; }
    else if (_Monitor_CheckFaultAsLow(p_monitor, input) == true)                            { status = MONITOR_STATUS_FAULT; }
    else if (_Hysteresis_Poll_Inverted(&p_monitor->Warning, input) == true)                 { status = MONITOR_STATUS_WARNING; }
    else                                                                                    { status = MONITOR_STATUS_NORMAL; }

    return status;
}


// Monitor_Status_T _Monitor_EvaluateWindow(Monitor_Base_T * p_low, Monitor_Base_T * p_high, int32_t input)
// {
//     /* Poll individual monitors and combine results */
//     /* Check the most inner limits first, for normal state */
//     Monitor_Status_T highStatus = _Monitor_EvaluateAsHigh(p_high, input);
//     Monitor_Status_T lowStatus = _Monitor_EvaluateAsLow(p_low, input);
//     Monitor_Status_T status;

//     if      (highStatus > lowStatus) { status = highStatus; }
//     else if (lowStatus > highStatus) { status = (lowStatus * -1); }
//     else { status = MONITOR_STATUS_NORMAL; } // Both are equal, so normal

//     return status;
// }

/******************************************************************************/
/*
    Single Direction Monitor Implementation
*/
/******************************************************************************/
void Monitor_InitFrom(Monitor_T * p_monitor, const Monitor_Config_T * p_config)
{
    if (p_config != NULL) { p_monitor->Config = *p_config; }

    _Monitor_InitFrom(&p_monitor->Base, &p_monitor->Config);

    /* Disable if invalid */
    if (Monitor_IsValidConfig(&p_monitor->Config) == false) { p_monitor->Config.IsEnabled = false; }

    if (p_monitor->Config.IsEnabled == false) { p_monitor->Direction = MONITOR_DISABLED; }
    else
    {
        p_monitor->Direction = (p_monitor->Config.Fault.Limit > p_monitor->Config.Warning.Setpoint) ? MONITOR_THRESHOLD_HIGH : MONITOR_THRESHOLD_LOW;
    }

    Monitor_Reset(p_monitor);
}

/******************************************************************************/
/*
    Monitor returns a status for user handling, function mapping.
    A "status" rather than a "state" with transition barriers.
    Call handle latching states.
    Effectively a simple state machine, single input, synchronous processing.
*/
/******************************************************************************/
Monitor_Status_T Monitor_Poll(Monitor_T * p_monitor, int32_t input)
{
    switch (p_monitor->Direction)
    {
        case MONITOR_DISABLED:
            break;
        // default:
        //     p_monitor->LastInput = input;
        //     p_monitor->StatusPrev = p_monitor->Status;
        case MONITOR_THRESHOLD_LOW:
            p_monitor->LastInput = input;
            p_monitor->StatusPrev = p_monitor->Status;
            p_monitor->Status = _Monitor_EvaluateAsLow(&p_monitor->Base, input);
            break;
        case MONITOR_THRESHOLD_HIGH:
            p_monitor->LastInput = input;
            p_monitor->StatusPrev = p_monitor->Status;
            p_monitor->Status = _Monitor_EvaluateAsHigh(&p_monitor->Base, input);
            break;
    }

    return p_monitor->Status;
}

/*
    Alternative to mapping edge statuses
    // MONITOR_STATUS_NULL = INT32_MIN,
*/
// Monitor_Status_T * Monitor_PollEdge(Monitor_T * p_monitor, int32_t input)
// {
//     Monitor_Poll(p_monitor, input);
//     return Monitor_IsStatusEdge(p_monitor) ? &p_monitor->Status : NULL;
// }

/******************************************************************************/
/*

*/
/******************************************************************************/
void Monitor_Reset(Monitor_T * p_monitor)
{
    p_monitor->Status = MONITOR_STATUS_NORMAL;
    p_monitor->StatusPrev = MONITOR_STATUS_NORMAL;
    p_monitor->LastInput = p_monitor->Config.Nominal;

    Hysteresis_Reset(&p_monitor->Base.Warning);
}


/******************************************************************************/
/*
    Config
*/
/******************************************************************************/
bool Monitor_IsValidConfig(const Monitor_Config_T * p_config)
{
    bool isValid = false;
    if (p_config != NULL)
    {
        bool isHighSide = (p_config->Warning.Setpoint >= p_config->Warning.Resetpoint);

        if (isHighSide)
        {
            /* High-side: Fault >= Warning.Setpoint > Warning.Resetpoint >= Nominal */
            isValid = (p_config->Fault.Limit >= p_config->Warning.Setpoint) &&
                      (p_config->Warning.Setpoint > p_config->Warning.Resetpoint) &&
                      (p_config->Warning.Resetpoint >= p_config->Nominal);
        }
        else
        {
            /* Low-side: Fault <= Warning.Setpoint < Warning.Resetpoint <= Nominal */
            isValid = (p_config->Fault.Limit <= p_config->Warning.Setpoint) &&
                      (p_config->Warning.Setpoint < p_config->Warning.Resetpoint) &&
                      (p_config->Warning.Resetpoint <= p_config->Nominal);
        }
    }
    return isValid;
}

void Monitor_SetFaultLimit(Monitor_T * p_monitor, int32_t limit) { p_monitor->Config.Fault.Limit = limit; Monitor_InitFrom(p_monitor, &p_monitor->Config); }
void Monitor_SetWarningSetpoint(Monitor_T * p_monitor, int32_t setpoint) { p_monitor->Config.Warning.Setpoint = setpoint; Monitor_InitFrom(p_monitor, &p_monitor->Config); }
void Monitor_SetWarningResetpoint(Monitor_T * p_monitor, int32_t resetpoint) { p_monitor->Config.Warning.Resetpoint = resetpoint;  Monitor_InitFrom(p_monitor, &p_monitor->Config); }
void Monitor_SetNominal(Monitor_T * p_monitor, int32_t nominal) { p_monitor->Config.Nominal = nominal; Monitor_InitFrom(p_monitor, &p_monitor->Config); }


/******************************************************************************/
/*
    By Id
*/
/******************************************************************************/
int32_t _Monitor_ConfigId_Get(const Monitor_T * p_monitor, Monitor_ConfigId_T id)
{
    switch ((Monitor_ConfigId_T)id)
    {
        case MONITOR_CONFIG_FAULT_LIMIT:        return Monitor_GetFaultLimit(p_monitor);
        case MONITOR_CONFIG_WARNING_SETPOINT:   return Monitor_GetWarningSetpoint(p_monitor);
        case MONITOR_CONFIG_WARNING_RESETPOINT: return Monitor_GetWarningResetpoint(p_monitor);
        case MONITOR_CONFIG_NOMINAL:            return Monitor_GetNominal(p_monitor);
        case MONITOR_CONFIG_IS_ENABLED:         return Monitor_IsConfigEnabled(p_monitor);
        default: return 0;
    }
}

void _Monitor_ConfigId_Set(Monitor_T * p_monitor, Monitor_ConfigId_T id, int32_t value)
{
    switch ((Monitor_ConfigId_T)id)
    {
        case MONITOR_CONFIG_FAULT_LIMIT:         Monitor_SetFaultLimit(p_monitor, value);           break;
        case MONITOR_CONFIG_WARNING_SETPOINT:    Monitor_SetWarningSetpoint(p_monitor, value);      break;
        case MONITOR_CONFIG_WARNING_RESETPOINT:  Monitor_SetWarningResetpoint(p_monitor, value);    break;
        case MONITOR_CONFIG_NOMINAL:             Monitor_SetNominal(p_monitor, value);              break;
        case MONITOR_CONFIG_IS_ENABLED:          Monitor_SetEnabled(p_monitor, value != 0);         break;
        default: break;
    }
}

int Monitor_ConfigId_Get(const Monitor_T * p_monitor, int id)
{
    return (p_monitor != NULL) ? _Monitor_ConfigId_Get(p_monitor, id) : 0;
}

void Monitor_ConfigId_Set(Monitor_T * p_monitor, int id, int value)
{
    if (p_monitor != NULL) { _Monitor_ConfigId_Set(p_monitor, id, value); }
}
