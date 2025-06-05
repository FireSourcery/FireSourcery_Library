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
    @file   VMonitor.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
/******************************************************************************/
#include "VMonitor.h"


/* select from compound literal or flash */
void VMonitor_InitFrom(VMonitor_T * p_vMonitor, const VMonitor_Config_T * p_config)
{
    if (p_config != NULL) { p_vMonitor->Config = *p_config; }

    p_vMonitor->Status = VMONITOR_STATUS_OK;
    if (p_vMonitor->Config.FaultUpper_Adcu == 0U) { p_vMonitor->Config.IsMonitorEnable = false; }
    if (p_vMonitor->Config.FaultLower_Adcu == 0U) { p_vMonitor->Config.IsMonitorEnable = false; }
    if (p_vMonitor->Config.WarningUpper_Adcu == 0U) { p_vMonitor->Config.WarningUpper_Adcu = p_vMonitor->Config.FaultUpper_Adcu; }
    if (p_vMonitor->Config.WarningLower_Adcu == 0U) { p_vMonitor->Config.WarningLower_Adcu = p_vMonitor->Config.FaultLower_Adcu; }

    Hysteresis_InitThresholds(&p_vMonitor->WarningUpper, p_vMonitor->Config.WarningUpper_Adcu, p_vMonitor->Config.WarningUpper_Adcu - p_vMonitor->Config.WarningHysteresis_Adcu);
    Hysteresis_InitThresholds(&p_vMonitor->WarningLower, p_vMonitor->Config.WarningLower_Adcu, p_vMonitor->Config.WarningLower_Adcu + p_vMonitor->Config.WarningHysteresis_Adcu);
}



/*
    No previous state
    Monitor: FaultLower_Adcu < WarningLower_Adcu <  adcu_nominal < WarningUpper_Adcu < FaultUpper_Adcu

    Monitor:
        FaultUpper_Adcu
        WarningUpper_Adcu == WarningUpper.ThresholdTrigger
        WarningUpper.ThresholdClear
        adcu_nominal
        WarningLower.ThresholdClear
        WarningLower_Adcu == WarningLower.ThresholdTrigger
        FaultLower_Adcu
*/
VMonitor_Status_T VMonitor_Poll(VMonitor_T * p_vMonitor, uint16_t adcu)
{
    if (p_vMonitor->Config.IsMonitorEnable == true)
    {
        if (adcu > p_vMonitor->WarningUpper.ThresholdClear)
        {
            if (adcu > p_vMonitor->Config.FaultUpper_Adcu) { p_vMonitor->Status = VMONITOR_FAULT_UPPER; }
            else
            {
                switch (Hysteresis_Poll(&p_vMonitor->WarningUpper, adcu))
                {
                    case HYSTERESIS_REGION_ABOVE_TRIGGER: p_vMonitor->Status = VMONITOR_WARNING_UPPER; break;
                    case HYSTERESIS_REGION_BELOW_CLEAR: p_vMonitor->Status = VMONITOR_STATUS_OK; break;
                    case HYSTERESIS_REGION_HYSTERESIS_BAND: break; /* No change */
                }
            }
        }
        else if (adcu < p_vMonitor->WarningLower.ThresholdClear)
        {
            if (adcu < p_vMonitor->Config.FaultLower_Adcu) { p_vMonitor->Status = VMONITOR_FAULT_LOWER; }
            else
            {
                switch (Hysteresis_Poll(&p_vMonitor->WarningLower, adcu))
                {
                    /* todo invert or use symetric */
                    case HYSTERESIS_REGION_ABOVE_TRIGGER: p_vMonitor->Status = VMONITOR_WARNING_UPPER; break;
                    case HYSTERESIS_REGION_BELOW_CLEAR: p_vMonitor->Status = VMONITOR_STATUS_OK; break;
                    case HYSTERESIS_REGION_HYSTERESIS_BAND: break; /* No change */
                }
            }
        }
        else
        {
            p_vMonitor->Status = VMONITOR_STATUS_OK;
        }
    }

    return p_vMonitor->Status;
}


/* Poll without hysteresis */
VMonitor_Status_T VMonitor_Poll_OnInput(VMonitor_T * p_vMonitor, uint16_t adcu)
{
    if (p_vMonitor->Config.IsMonitorEnable == true)
    {
        if      (adcu > p_vMonitor->Config.WarningUpper_Adcu)   { p_vMonitor->Status = (adcu > p_vMonitor->Config.FaultUpper_Adcu) ? VMONITOR_FAULT_UPPER : VMONITOR_WARNING_UPPER; }
        else if (adcu < p_vMonitor->Config.WarningLower_Adcu)   { p_vMonitor->Status = (adcu < p_vMonitor->Config.FaultLower_Adcu) ? VMONITOR_FAULT_LOWER : VMONITOR_WARNING_LOWER; }
        else                                                    { p_vMonitor->Status = VMONITOR_STATUS_OK; }
    }
    return p_vMonitor->Status;
}

// VMonitor_EdgeStatus_T VMonitor_PollEdge(VMonitor_T * p_vMonitor, uint16_t adcu)
// {
//     VMonitor_Status_T prev = p_vMonitor->Status;
//     VMonitor_Status_T new = VMonitor_Poll(p_vMonitor, adcu);

//     /* Return edge status based on status transition */
//     if (prev != new)
//     {
//         /* Determine edge type based on new status */
//         switch (new)
//         {
//             case VMONITOR_FAULT_UPPER: return VMONITOR_EDGE_FAULT_TRIGGER;
//             case VMONITOR_FAULT_LOWER: return VMONITOR_EDGE_FAULT_TRIGGER;
//             case VMONITOR_WARNING_UPPER: return (prev == VMONITOR_FAULT_UPPER) ? VMONITOR_EDGE_FAULT_CLEAR : VMONITOR_EDGE_WARNING_UPPER_TRIGGER;
//             case VMONITOR_WARNING_LOWER: return (prev == VMONITOR_FAULT_LOWER) ? VMONITOR_EDGE_FAULT_CLEAR : VMONITOR_EDGE_WARNING_LOWER_TRIGGER;
//             case VMONITOR_STATUS_OK:
//                 /* Determine what we're clearing from */
//                 switch (prev)
//                 {
//                     case VMONITOR_FAULT_UPPER: return VMONITOR_EDGE_FAULT_CLEAR;
//                     case VMONITOR_FAULT_LOWER: return VMONITOR_EDGE_FAULT_CLEAR;
//                     case VMONITOR_WARNING_UPPER: return VMONITOR_EDGE_WARNING_UPPER_CLEAR;
//                     case VMONITOR_WARNING_LOWER: return VMONITOR_EDGE_WARNING_LOWER_CLEAR;
//                     default: return VMONITOR_EDGE_NONE;
//                 }

//             default: return VMONITOR_EDGE_NONE;
//         }
//     }

//     return VMONITOR_EDGE_NONE;
// }

// bool VMonitor_PollEdge(VMonitor_T * p_vMonitor, uint16_t adcu)
// {
//     return (p_vMonitor->Status != VMonitor_Poll(p_vMonitor, adcu));
// }

/******************************************************************************/
/*!
    Config
*/
/******************************************************************************/
bool VMonitor_IsValidConfig(const VMonitor_Config_T * p_config)
{
    return (p_config->FaultLower_Adcu < p_config->WarningLower_Adcu) &&
        (p_config->WarningLower_Adcu < p_config->WarningUpper_Adcu) &&
        (p_config->WarningUpper_Adcu < p_config->FaultUpper_Adcu);
}

void VMonitor_ResetLimitsDefault(VMonitor_T * p_vMonitor)
{
    uint32_t vRef = p_vMonitor->Config.Nominal_Adcu;
    VMonitor_SetFaultUpper(p_vMonitor, vRef * 125 / 100);
    VMonitor_SetFaultLower(p_vMonitor, vRef * 75 / 100);
    VMonitor_SetWarningUpper(p_vMonitor, vRef * 115 / 100);
    VMonitor_SetWarningLower(p_vMonitor, vRef * 85 / 100);
}



//  void VMonitor_SetNominal(VMonitor_T * p_vMonitor, uint16_t nominal_adcu)
// {
//     p_vMonitor->Config.Nominal_Adcu = nominal_adcu;
//     VMonitor_ResetLimitsDefault(p_vMonitor);
// }

/******************************************************************************/
/*!
    ConfigId
*/
/******************************************************************************/
int32_t _VMonitor_ConfigId_Get(const VMonitor_T * p_vMonitor, VMonitor_ConfigId_T configId)
{
    int32_t value = 0;
    switch (configId)
    {
        // case VMONITOR_CONFIG_R1:                   value = p_vMonitor->CONST.UNITS_R1 / 10U;       break;
        // case VMONITOR_CONFIG_R2:                   value = p_vMonitor->CONST.UNITS_R2 / 10U;       break;
        case VMONITOR_CONFIG_FAULT_OVER_LIMIT:     value = VMonitor_GetFaultUpper(p_vMonitor);     break;
        case VMONITOR_CONFIG_FAULT_UNDER_LIMIT:     value = VMonitor_GetFaultLower(p_vMonitor);     break;
        case VMONITOR_CONFIG_WARNING_LIMIT_HIGH:   value = VMonitor_GetWarningUpper(p_vMonitor);   break;
        case VMONITOR_CONFIG_WARNING_LIMIT_LOW:   value = VMonitor_GetWarningLower(p_vMonitor);   break;
        case VMONITOR_CONFIG_IS_ENABLE:            value = VMonitor_IsEnabled(p_vMonitor);          break;
        default: break;
    }
    return value;
}

void _VMonitor_ConfigId_Set(VMonitor_T * p_vMonitor, VMonitor_ConfigId_T configId, uint16_t value)
{
    switch(configId)
    {
        // case VMONITOR_CONFIG_R1:                                                                    break;
        // case VMONITOR_CONFIG_R2:                                                                    break;
        case VMONITOR_CONFIG_FAULT_OVER_LIMIT:     VMonitor_SetFaultUpper(p_vMonitor, value);       break;
        case VMONITOR_CONFIG_FAULT_UNDER_LIMIT:     VMonitor_SetFaultLower(p_vMonitor, value);       break;
        case VMONITOR_CONFIG_WARNING_LIMIT_HIGH:   VMonitor_SetWarningUpper(p_vMonitor, value);     break;
        case VMONITOR_CONFIG_WARNING_LIMIT_LOW:   VMonitor_SetWarningLower(p_vMonitor, value);     break;
        case VMONITOR_CONFIG_IS_ENABLE:            VMonitor_SetOnOff(p_vMonitor, value);         break;
        default: break;
    }
}


int32_t VMonitor_ConfigId_Get(const VMonitor_T * p_vMonitor, VMonitor_ConfigId_T configId) { return (p_vMonitor != NULL) ? _VMonitor_ConfigId_Get(p_vMonitor, configId) : 0; }

void VMonitor_ConfigId_Set(VMonitor_T * p_vMonitor, VMonitor_ConfigId_T configId, uint16_t value) { if (p_vMonitor != NULL) { _VMonitor_ConfigId_Set(p_vMonitor, configId, value); } }
