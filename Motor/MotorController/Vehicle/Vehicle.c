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
    @file   Vehicle.c
    @author FireSourcery

    @brief
*/
/******************************************************************************/
#include "Vehicle.h"


void _Vehicle_InitFrom(Vehicle_State_T * p_fields, const Vehicle_Config_T * p_config)
{
    if (p_config != NULL) { p_fields->Config = *p_config; }
}

void Vehicle_Init(const Vehicle_T * p_handle)
{
    _Vehicle_InitFrom(p_handle->P_VEHICLE_STATE, p_handle->P_NVM_CONFIG);
    StateMachine_Init(&p_handle->STATE_MACHINE);
}



