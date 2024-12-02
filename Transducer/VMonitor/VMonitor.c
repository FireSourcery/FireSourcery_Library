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
    Linear_Voltage_Init(&p_vMonitor->Units, p_vMonitor->CONST.UNITS_R1, p_vMonitor->CONST.UNITS_R2, GLOBAL_ANALOG.ADC_BITS, GLOBAL_ANALOG.ADC_VREF_MILLIV, 0);
}

void VMonitor_Init(VMonitor_T * p_vMonitor)
{
    if(p_vMonitor->CONST.P_CONFIG != 0U) { memcpy(&p_vMonitor->Config, p_vMonitor->CONST.P_CONFIG, sizeof(VMonitor_Config_T)); }

    ResetUnitConversion(p_vMonitor);
    p_vMonitor->Status = VMONITOR_STATUS_OK;
    if(p_vMonitor->Config.FaultUpper_Adcu == 0U)        { p_vMonitor->Config.IsMonitorEnable = false; }
    if(p_vMonitor->Config.FaultLower_Adcu == 0U)        { p_vMonitor->Config.IsMonitorEnable = false; }
    if(p_vMonitor->Config.WarningUpper_Adcu == 0U)      { p_vMonitor->Config.WarningUpper_Adcu = p_vMonitor->Config.FaultUpper_Adcu; }
    if(p_vMonitor->Config.WarningLower_Adcu == 0U)      { p_vMonitor->Config.WarningLower_Adcu = p_vMonitor->Config.FaultLower_Adcu; }
    // Linear_ADC_Init(&p_vMonitor->LinearLimits, p_vMonitor->Config.FaultLower_Adcu, p_vMonitor->Config.WarningLower_Adcu, 0, 0);
}

/*
    No previous state
    Monitor: FaultLower_Adcu < WarningLower_Adcu <  adcu < WarningUpper_Adcu < FaultUpper_Adcu
*/
VMonitor_Status_T VMonitor_PollStatus(VMonitor_T * p_vMonitor, uint16_t adcu)
{
    if(p_vMonitor->Config.IsMonitorEnable == true)
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
    VMonitor_SetFaultLower(p_vMonitor, vRef * 3U / 4U);
    VMonitor_SetFaultUpper(p_vMonitor, vRef * 5U / 4U);
    VMonitor_SetWarningLower(p_vMonitor, vRef * 7U / 8U);
    VMonitor_SetWarningUpper(p_vMonitor, vRef * 9U / 8U);
}


