
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
    @file   HeatMonitor.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "HeatMonitor.h"


/******************************************************************************/
/*

*/
/******************************************************************************/
void HeatMonitor_InitFrom(HeatMonitor_T * p_heat, const HeatMonitor_Config_T * p_config)
{
    if (p_config != NULL) { memcpy(&p_heat->Config, p_config, sizeof(HeatMonitor_Config_T)); }

    if (p_heat->Config.FaultTrigger_Adcu == 0U)    { p_heat->Config.IsMonitorEnable = false; }
    if (p_heat->Config.FaultClear_Adcu == 0U)      { p_heat->Config.IsMonitorEnable = false; }
    if (p_heat->Config.WarningTrigger_Adcu == 0U)  { p_heat->Config.IsMonitorEnable = false; }
    if (p_heat->Config.WarningClear_Adcu == 0U)    { p_heat->Config.IsMonitorEnable = false; }

    // Linear_Q16_Init(&p_heat->HeatScalar, p_heat->Config.FaultTrigger_Adcu, p_heat->Config.WarningTrigger_Adcu); /* for Percent16 only */

    HeatMonitor_Reset(p_heat); /* Reset the compare values. */

    p_heat->Adcu = p_heat->Config.WarningClear_Adcu; /* Set to a nominal value, so poll on init does not fault. */
    p_heat->Status = HEAT_MONITOR_STATUS_OK;
}

void HeatMonitor_Reset(HeatMonitor_T * p_heat)
{
    int16_t compareSign = (p_heat->Config.FaultTrigger_Adcu < p_heat->Config.WarningTrigger_Adcu) ? -1 : 1; // NTC or PTC

    p_heat->WarningTriggerCompare = compareSign * p_heat->Config.WarningTrigger_Adcu;
    p_heat->WarningClearCompare = compareSign * p_heat->Config.WarningClear_Adcu;
    p_heat->FaultTriggerCompare = compareSign * p_heat->Config.FaultTrigger_Adcu;
    p_heat->FaultClearCompare = compareSign * p_heat->Config.FaultClear_Adcu;

    Linear_Q16_Init(&p_heat->HeatScalar, p_heat->Config.FaultTrigger_Adcu, p_heat->Config.WarningTrigger_Adcu);
}

/******************************************************************************/
/*!
    Limits Monitor
*/
/******************************************************************************/
/*!
    Monitor: heat_nominal < WarningThreshold_DegC < Warning_DegC < FaultThreshold_DegC < Fault_DegC
    As pulldown
        NTC lesser adcu is higher heat
        PTC lesser adcu is lower heat
*/
static inline void PollMonitor(HeatMonitor_T * p_heat, uint16_t adcu)
{
    int16_t compareSign = (p_heat->Config.FaultTrigger_Adcu < p_heat->Config.WarningTrigger_Adcu) ? -1 : 1; // NTC or PTC
    int16_t compare = compareSign * adcu;

    if      (compare < p_heat->WarningClearCompare)                                     { p_heat->Status = HEAT_MONITOR_STATUS_OK; }
    else if (compare > p_heat->FaultTriggerCompare)                                     { p_heat->Status = HEAT_MONITOR_STATUS_FAULT; }
    else if ((compare > p_heat->FaultClearCompare) && HeatMonitor_IsFault(p_heat))      { p_heat->Status = HEAT_MONITOR_STATUS_FAULT; }
    else if (compare > p_heat->WarningTriggerCompare)                                   { p_heat->Status = HEAT_MONITOR_STATUS_WARNING; }
    else if ((compare > p_heat->WarningClearCompare) && HeatMonitor_IsWarning(p_heat))  { p_heat->Status = HEAT_MONITOR_STATUS_WARNING; }
    else                                                                                { p_heat->Status = HEAT_MONITOR_STATUS_OK; }

    // if      (adcu > warningLower) { p_heat->Status = HEAT_MONITOR_STATUS_OK; }
    // else if (is_on_with(faultUpper, faultLower, HeatMonitor_IsFault(p_heat), adcu)) { p_heat->Status = HEAT_MONITOR_STATUS_FAULT; }
    // else if (is_over_threshold_feedback(faultUpper, faultLower, p_heat->Output_Adcu, adcu)) { p_heat->Status = HEAT_MONITOR_STATUS_FAULT; }
}

HeatMonitor_Status_T HeatMonitor_PollMonitor(HeatMonitor_T * p_heat, uint16_t adcu)
{
    if (p_heat->Config.IsMonitorEnable == true) { PollMonitor(p_heat, adcu); }
    p_heat->Adcu = adcu;
    return p_heat->Status;
}

bool HeatMonitor_PollMonitorEdge(HeatMonitor_T * p_heat, uint16_t adcu)
{
    return (p_heat->Status != HeatMonitor_PollMonitor(p_heat, adcu));
}

/******************************************************************************/
/*!
    Set Limits Config
*/
/******************************************************************************/
/* Caller handle range validation */
// void HeatMonitor_SetFaultTrigger(HeatMonitor_T * p_heat, uint16_t fault)                   { p_heat->Config.FaultTrigger_Adcu = fault; HeatMonitor_ResetConfig(p_heat); }
// void HeatMonitor_SetFaultThreshold(HeatMonitor_T * p_heat, uint16_t faultThreshold)        { p_heat->Config.FaultClear_Adcu = faultThreshold; HeatMonitor_ResetConfig(p_heat); }
// void HeatMonitor_SetWarningTrigger(HeatMonitor_T * p_heat, uint16_t warning)               { p_heat->Config.WarningTrigger_Adcu = warning; HeatMonitor_ResetConfig(p_heat); }
// void HeatMonitor_SetWarningThreshold(HeatMonitor_T * p_heat, uint16_t warningThreshold)    { p_heat->Config.WarningClear_Adcu = warningThreshold; HeatMonitor_ResetConfig(p_heat); }

/******************************************************************************/
/*!
    Local Units
*/
/******************************************************************************/
// void HeatMonitor_SetFault_DegC(Thermistor_T * p_heat, thermal_t fault_degC, thermal_t faultThreshold_degC)
// {
//     p_heat->Config.FaultTrigger_Adcu = Thermistor_AdcuOfCelsius(p_heat, fault_degC);
//     p_heat->Config.FaultClear_Adcu = Thermistor_AdcuOfCelsius(p_heat, faultThreshold_degC);
// }

// void HeatMonitor_SetWarning_DegC(Thermistor_T * p_heat, thermal_t warning_degC, thermal_t warningThreshold_degC)
// {
//     p_heat->Config.WarningTrigger_Adcu = Thermistor_AdcuOfCelsius(p_heat, warning_degC);
//     p_heat->Config.WarningClear_Adcu = Thermistor_AdcuOfCelsius(p_heat, warningThreshold_degC);
// }

// void HeatMonitor_SetLimits_DegC(Thermistor_T * p_heat, thermal_t fault, thermal_t faultThreshold, thermal_t warning, thermal_t warningThreshold)
// {
//     HeatMonitor_SetFault_DegC(p_heat, fault, faultThreshold);
//     HeatMonitor_SetWarning_DegC(p_heat, warning, warningThreshold);
//     ResetUnitsLinear(p_heat);
// }

// thermal_t HeatMonitor_GetFault_DegC(const Thermistor_T * p_heat)              { return CelsiusOfAdcu(p_heat, p_heat->Config.FaultTrigger_Adcu); }
// thermal_t HeatMonitor_GetFaultThreshold_DegC(const Thermistor_T * p_heat)     { return CelsiusOfAdcu(p_heat, p_heat->Config.FaultClear_Adcu); }
// thermal_t HeatMonitor_GetWarning_DegC(const Thermistor_T * p_heat)            { return CelsiusOfAdcu(p_heat, p_heat->Config.WarningTrigger_Adcu); }
// thermal_t HeatMonitor_GetWarningThreshold_DegC(const Thermistor_T * p_heat)   { return CelsiusOfAdcu(p_heat, p_heat->Config.WarningClear_Adcu); }


/******************************************************************************/
/*
    Id
*/
/******************************************************************************/
// int32_t Thermistor_VarId_Get(const Thermistor_T * p_heat, Thermistor_VarId id)
// {
//     int32_t value = 0;
//     switch (id)
//     {
//         case HEAT_MONITOR_CONFIG_VALUE_ADCU:     value = p_heat->Adcu;                         break;
//         case HEAT_MONITOR_CONFIG_STATUS:         value = Thermistor_GetStatus(p_heat);         break;
//         default: value = 0; break;
//     }
//     return value;
// }

int32_t _HeatMonitor_ConfigId_Get(const HeatMonitor_T * p_heat, HeatMonitor_ConfigId_T id)
{
    int32_t value = 0;
    switch (id)
    {
        case HEAT_MONITOR_CONFIG_FAULT_TRIGGER_ADCU:         value = HeatMonitor_GetFaultTrigger(p_heat);      break;
        case HEAT_MONITOR_CONFIG_FAULT_THRESHOLD_ADCU:       value = HeatMonitor_GetFaultThreshold(p_heat);    break;
        case HEAT_MONITOR_CONFIG_WARNING_TRIGGER_ADCU:       value = HeatMonitor_GetWarningTrigger(p_heat);    break;
        case HEAT_MONITOR_CONFIG_WARNING_THRESHOLD_ADCU:     value = HeatMonitor_GetWarningThreshold(p_heat);  break;
        case HEAT_MONITOR_CONFIG_IS_MONITOR_ENABLE:          value = HeatMonitor_IsEnabled(p_heat);            break;
        default: break;
    }
    return value;
}

void _HeatMonitor_ConfigId_Set(HeatMonitor_T * p_heat, HeatMonitor_ConfigId_T id, int32_t value)
{
    switch (id)
    {
        case HEAT_MONITOR_CONFIG_FAULT_TRIGGER_ADCU:         HeatMonitor_SetFaultTrigger(p_heat, value);       break;
        case HEAT_MONITOR_CONFIG_FAULT_THRESHOLD_ADCU:       HeatMonitor_SetFaultThreshold(p_heat, value);     break;
        case HEAT_MONITOR_CONFIG_WARNING_TRIGGER_ADCU:       HeatMonitor_SetWarningTrigger(p_heat, value);     break;
        case HEAT_MONITOR_CONFIG_WARNING_THRESHOLD_ADCU:     HeatMonitor_SetWarningThreshold(p_heat, value);   break;
        case HEAT_MONITOR_CONFIG_IS_MONITOR_ENABLE:          HeatMonitor_SetOnOff(p_heat, value);              break;
    }
}

int HeatMonitor_ConfigId_Get(const HeatMonitor_T * p_heat, int id)
{
    return (p_heat != NULL) ? _HeatMonitor_ConfigId_Get(p_heat, id) : 0;
}

void HeatMonitor_ConfigId_Set(HeatMonitor_T * p_heat, int id, int value)
{
    if (p_heat != NULL) { _HeatMonitor_ConfigId_Set(p_heat, id, value); }
}


// static inline void PollMonitor(HeatMonitor_T * p_heat, uint16_t adcu)
// {
//     int compareSign = (p_heat->Config.Type == HEAT_MONITOR_TYPE_NTC) ? 1 : -1;
//     int16_t adcuCompare = compareSign * adcu;
//     int16_t warningLower = compareSign * p_heat->Config.WarningClear_Adcu;
//     int16_t warningUpper = compareSign * p_heat->Config.WarningTrigger_Adcu;
//     int16_t faultUpper = compareSign * p_heat->Config.FaultTrigger_Adcu;
//     int16_t faultLower = compareSign * p_heat->Config.FaultClear_Adcu;

//     if      (adcuCompare > warningLower)                                       { p_heat->Status = HEAT_MONITOR_STATUS_OK; }
//     else if (adcuCompare < faultUpper)                                         { p_heat->Status = HEAT_MONITOR_STATUS_FAULT; }
//     else if ((adcuCompare < faultLower) && HeatMonitor_IsFault(p_heat))       { p_heat->Status = HEAT_MONITOR_STATUS_FAULT; }
//     else if (adcuCompare < warningUpper)                                       { p_heat->Status = HEAT_MONITOR_STATUS_WARNING; }
//     else if ((adcuCompare < warningLower) && HeatMonitor_IsWarning(p_heat))   { p_heat->Status = HEAT_MONITOR_STATUS_WARNING; }
//     else                                                                        { p_heat->Status = HEAT_MONITOR_STATUS_OK; }
// }