/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2021 FireSourcery / The Firebrand Forge Inc

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
    @file     Global_Motor.c
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#include "Global_Motor.h"

/* Global Static, for all Motor instances */
static uint16_t VSourceRef_V;         /* Battery/Supply voltage. Sync with upper layer */

void Global_Motor_InitVSourceRef_V(uint16_t vSource) { VSourceRef_V = (vSource > GLOBAL_MOTOR.V_MAX_VOLTS) ? GLOBAL_MOTOR.V_MAX_VOLTS : vSource; }
uint16_t Global_Motor_GetVSource_V(void) { return VSourceRef_V; }

