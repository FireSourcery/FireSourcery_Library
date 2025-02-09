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
    @file   VMonitor.c
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#include "VMonitor.h"

#include <string.h>
#include <stdio.h>


static void ResetUnitConversion(VMonitor_T * p_vMonitor)
{
    Linear_Voltage_Init(&p_vMonitor->Units, p_vMonitor->CONST.UNITS_R1, p_vMonitor->CONST.UNITS_R2, GLOBAL_ANALOG.ADC_VREF_MILLIV, GLOBAL_ANALOG.ADC_BITS, 0);
}

void VMonitor_Init(VMonitor_T * p_vMonitor)
{
    if(p_vMonitor->CONST.P_CONFIG != NULL) { memcpy(&p_vMonitor->Config, p_vMonitor->CONST.P_CONFIG, sizeof(VMonitor_Config_T)); }

    ResetUnitConversion(p_vMonitor);
    p_vMonitor->Status = VMONITOR_STATUS_OK;
    if (p_vMonitor->Config.FaultUpper_Adcu == 0U)   { p_vMonitor->Config.IsMonitorEnable = false; }
    if (p_vMonitor->Config.FaultLower_Adcu == 0U)   { p_vMonitor->Config.IsMonitorEnable = false; }
    if (p_vMonitor->Config.WarningUpper_Adcu == 0U) { p_vMonitor->Config.WarningUpper_Adcu = p_vMonitor->Config.FaultUpper_Adcu; }
    if (p_vMonitor->Config.WarningLower_Adcu == 0U) { p_vMonitor->Config.WarningLower_Adcu = p_vMonitor->Config.FaultLower_Adcu; }
    // Linear_ADC_Init(&p_vMonitor->LinearLimits, p_vMonitor->Config.FaultLower_Adcu, p_vMonitor->Config.WarningLower_Adcu, 0, 0);
}

/* select from compound literal or flash */
// void _VMonitor_Init(VMonitor_T * p_vMonitor, const VMonitor_Config_T * p_config)
// {
//     const VMonitor_Config_T * p_effectiveConfig = (p_config == NULL) ? &p_vMonitor->CONST.P_CONFIG : p_config;
//     if (p_effectiveConfig != NULL) { memcpy(&p_vMonitor->Config, p_vMonitor->CONST.P_CONFIG, sizeof(VMonitor_Config_T)); }
// }

/*
    No previous state
    Monitor: FaultLower_Adcu < WarningLower_Adcu <  adcu < WarningUpper_Adcu < FaultUpper_Adcu
*/
VMonitor_Status_T VMonitor_PollStatus(VMonitor_T * p_vMonitor, uint16_t adcu)
{
    if (p_vMonitor->Config.IsMonitorEnable == true)
    {
        if      (adcu > p_vMonitor->Config.WarningUpper_Adcu)   { p_vMonitor->Status = (adcu > p_vMonitor->Config.FaultUpper_Adcu) ? VMONITOR_FAULT_UPPER : VMONITOR_WARNING_UPPER; }
        else if (adcu < p_vMonitor->Config.WarningLower_Adcu)   { p_vMonitor->Status = (adcu < p_vMonitor->Config.FaultLower_Adcu) ? VMONITOR_FAULT_LOWER : VMONITOR_WARNING_LOWER; }
        else                                                    { p_vMonitor->Status = VMONITOR_STATUS_OK; }
    }

    return p_vMonitor->Status;
}

// VMonitor_Status_T VMonitor_PollStatusWithEdge(VMonitor_T * p_vMonitor, uint16_t adcu)

/******************************************************************************/
/*!
    Config
*/
/******************************************************************************/
void VMonitor_ResetLimitsDefault(VMonitor_T * p_vMonitor)
{
    uint32_t vRef = p_vMonitor->Config.Nominal_Adcu;
    VMonitor_SetFaultLower(p_vMonitor, vRef * 5 / 8);
    VMonitor_SetFaultUpper(p_vMonitor, vRef * 10 / 8);
    VMonitor_SetWarningLower(p_vMonitor, vRef * 6 / 8);
    VMonitor_SetWarningUpper(p_vMonitor, vRef * 9 / 8);
}


/******************************************************************************/
/*!
    ConfigId
*/
/******************************************************************************/
void VMonitor_ConfigId_Set(VMonitor_T * p_vMonitor, VMonitor_ConfigId_T configId, uint16_t value)
{
    switch(configId)
    {
        case VMONITOR_CONFIG_R1:                                                                    break;
        case VMONITOR_CONFIG_R2:                                                                    break;
        case VMONITOR_CONFIG_FAULT_UPPER_ADCU:     VMonitor_SetFaultUpper(p_vMonitor, value);       break;
        case VMONITOR_CONFIG_FAULT_LOWER_ADCU:     VMonitor_SetFaultLower(p_vMonitor, value);       break;
        case VMONITOR_CONFIG_WARNING_UPPER_ADCU:   VMonitor_SetWarningUpper(p_vMonitor, value);     break;
        case VMONITOR_CONFIG_WARNING_LOWER_ADCU:   VMonitor_SetWarningLower(p_vMonitor, value);     break;
        case VMONITOR_CONFIG_IS_ENABLE:            VMonitor_SetIsEnable(p_vMonitor, value);         break;
        default: break;
    }
}

int32_t VMonitor_ConfigId_Get(const VMonitor_T * p_vMonitor, VMonitor_ConfigId_T configId)
{
    int32_t value = 0;
    switch(configId)
    {
        case VMONITOR_CONFIG_R1:                   value = p_vMonitor->CONST.UNITS_R1 / 10U;       break;
        case VMONITOR_CONFIG_R2:                   value = p_vMonitor->CONST.UNITS_R2 / 10U;       break;
        case VMONITOR_CONFIG_FAULT_UPPER_ADCU:     value = VMonitor_GetFaultUpper(p_vMonitor);     break;
        case VMONITOR_CONFIG_FAULT_LOWER_ADCU:     value = VMonitor_GetFaultLower(p_vMonitor);     break;
        case VMONITOR_CONFIG_WARNING_UPPER_ADCU:   value = VMonitor_GetWarningUpper(p_vMonitor);   break;
        case VMONITOR_CONFIG_WARNING_LOWER_ADCU:   value = VMonitor_GetWarningLower(p_vMonitor);   break;
        case VMONITOR_CONFIG_IS_ENABLE:            value = VMonitor_IsEnable(p_vMonitor);          break;
        default: break;
    }
    return value;
}