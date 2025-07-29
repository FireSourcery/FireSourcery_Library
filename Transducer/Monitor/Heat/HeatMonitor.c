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
/******************************************************************************/
#include "HeatMonitor.h"

/******************************************************************************/
/*
    Context
*/
/******************************************************************************/
void HeatMonitor_InitFrom(const HeatMonitor_Context_T * p_context, const HeatMonitor_Config_T * p_config)
{
    assert(p_context->P_STATE != NULL); /* Ensure state is provided */

    Monitor_InitFrom(p_context->P_STATE, p_config); /* Monitor_T auto handle invert */
    if (p_context->P_LIMIT_SCALAR != NULL) { HeatMonitor_ToLimitScalar(p_context->P_STATE, p_context->P_LIMIT_SCALAR); }

    Thermistor_Init(&p_context->THERMISTOR); /* Configurable/Wired Thermistor load Coeffcients */
    if (p_context->P_LINEAR != NULL) { Thermistor_ToLinear_CelsiusPerAdcu(&p_context->THERMISTOR, p_context->P_LINEAR); }
}

void HeatMonitor_Init(const HeatMonitor_Context_T * p_context)
{
    HeatMonitor_InitFrom(p_context, p_context->P_NVM_CONFIG);
}


/******************************************************************************/
/*
    Group Context
*/
/******************************************************************************/
/* Initialize group with shared resources */
void HeatMonitor_Group_Init(const HeatMonitor_GroupContext_T * p_group)
{
    /* Initialize shared state with group configuration */
    Monitor_InitFrom(p_group->P_STATE, p_group->P_NVM_CONFIG);
    /* Initialize shared limit scalar */
    if (p_group->P_LIMIT_SCALAR != NULL) { HeatMonitor_ToLimitScalar(p_group->P_STATE, p_group->P_LIMIT_SCALAR); }

    /* Pass the same detection parameters to each */
    for (uint8_t i = 0; i < p_group->COUNT; i++)
    {
        if (p_group->P_CONTEXTS[i].P_NVM_CONFIG != NULL) { HeatMonitor_Init(&p_group->P_CONTEXTS[i]); }
        else { HeatMonitor_InitFrom(&p_group->P_CONTEXTS[i], p_group->P_NVM_CONFIG); }
    }
}

// static inline int HeatMonitor_Group_ConfigId_Get(const HeatMonitor_T * p_heat, int id) { return Monitor_ConfigId_Get(p_heat, id); }
// static inline void HeatMonitor_Group_ConfigId_Set(HeatMonitor_T * p_heat, int id, int value) { Monitor_ConfigId_Set(p_heat, id, value); }