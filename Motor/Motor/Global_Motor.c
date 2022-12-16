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
	@file 	Global_Motor.c
	@author FireSourcery
	@brief
	@version V0
*/
/******************************************************************************/
#include "Global_Motor.h"

/* Global Static, for all Motor instances */
static uint16_t AdcVRef_MilliV;  	/* Sync with upper layer */
static uint16_t VSourceRef_V; 		/* Battery/Supply voltage. Sync with upper layer */

uint16_t Global_Motor_GetAdcVRef(void) 		{ return AdcVRef_MilliV; }
uint16_t Global_Motor_GetVSourceRef(void) 	{ return VSourceRef_V; }

void Global_Motor_InitAdcVRef_MilliV(uint16_t adcVRef_MilliV) 	{ AdcVRef_MilliV = adcVRef_MilliV; }
void Global_Motor_InitVSourceRef_V(uint16_t vSourceRef_V) 		{ VSourceRef_V = vSourceRef_V; }