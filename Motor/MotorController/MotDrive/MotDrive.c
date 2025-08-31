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
    @file   MotDrive.c
    @author FireSourcery

    @brief
*/
/******************************************************************************/
#include "MotDrive.h"


void _MotDrive_InitFrom(MotDrive_State_T * p_fields, const MotDrive_Config_T * p_config)
{
    if (p_config != NULL) { p_fields->Config = *p_config; }
}

void MotDrive_Init(const MotDrive_T * p_handle)
{
    _MotDrive_InitFrom(p_handle->P_MOT_DRIVE_STATE, p_handle->P_NVM_CONFIG);
    StateMachine_Init(&p_handle->STATE_MACHINE);
}



