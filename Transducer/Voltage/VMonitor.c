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
#include "VMonitor.h"

/******************************************************************************/
/*
    Helper
*/
/******************************************************************************/
/*
    using +/- 25%, +/- 15% for warning limits
*/
void VMonitor_InitLimitsDefault(VMonitor_T * p_vMonitor, int32_t nominal, uint8_t faultPercent, uint8_t warnPercent, uint8_t hystPercent)
{
    // uint32_t vRef = p_vMonitor->Monitor.Config.Nominal;
    // VMonitor_SetFaultOverLimit(p_vMonitor, vRef * 125 / 100);
    // VMonitor_SetFaultUnderLimit(p_vMonitor, vRef * 75 / 100);
    // VMonitor_SetWarningLimitHigh(p_vMonitor, vRef * 115 / 100);
    // VMonitor_SetWarningLimitLow(p_vMonitor, vRef * 85 / 100);

    RangeMonitor_Config_InitSymmetricPercent(&p_vMonitor->Config, nominal, faultPercent, warnPercent, hystPercent);
    /* Reinitialize with new limits */
    RangeMonitor_InitFrom(p_vMonitor, &p_vMonitor->Config);
}


/******************************************************************************/
/*

*/
/******************************************************************************/
void VMonitor_InitFrom(const VMonitor_Context_T * p_context, const VMonitor_Config_T * p_config)
{
    RangeMonitor_InitFrom(p_context->P_STATE, p_config);
    if (p_context->P_LINEAR != NULL) { VDivider_ToLinear(&p_context->VDIVIDER, p_context->P_LINEAR); }
}

void VMonitor_Init(const VMonitor_Context_T * p_context)
{
    VMonitor_InitFrom(p_context, p_context->P_NVM_CONFIG);
}


