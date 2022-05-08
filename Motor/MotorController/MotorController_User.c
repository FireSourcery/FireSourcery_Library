/******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSoucery / The Firebrand Forge Inc

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
	@file 	MotorController_User.c
	@author FireSoucery
	@brief
	@version V0
*/
/******************************************************************************/
#include "MotorController_User.h"

void MotorController_User_SetAdcVRef(MotorController_T * p_mc, uint16_t adcVRef_MilliV)
{
	if		(adcVRef_MilliV > p_mc->CONFIG.ADC_VREF_MAX_MILLIV) { p_mc->Parameters.AdcVRef_MilliV = p_mc->CONFIG.ADC_VREF_MIN_MILLIV; }
	else if	(adcVRef_MilliV < p_mc->CONFIG.ADC_VREF_MIN_MILLIV) { p_mc->Parameters.AdcVRef_MilliV = p_mc->CONFIG.ADC_VREF_MIN_MILLIV; }
	else 														{ p_mc->Parameters.AdcVRef_MilliV = adcVRef_MilliV; }

	VMonitor_InitAdcVRef_MilliV(p_mc->Parameters.AdcVRef_MilliV); 
	Motor_InitAdcVRef_MilliV(p_mc->Parameters.AdcVRef_MilliV); 
	Thermistor_InitAdcVRef_Scalar(p_mc->Parameters.AdcVRef_MilliV); 

	//must reset to propagate set
}