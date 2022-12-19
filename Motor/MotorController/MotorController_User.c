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
	@file 	MotorController_User.c
	@author FireSourcery
	@brief
	@version V0
*/
/******************************************************************************/
#include "MotorController_User.h"

void MotorController_User_SetVSource(MotorController_T * p_mc, uint16_t volts)
{
	Global_Motor_SetVSource_V(volts);
	p_mc->Parameters.VSourceRef = Global_Motor_GetVSource_V();
}

void MotorController_User_SetBatteryLife_MilliV(MotorController_T * p_mc, uint32_t zero_mV, uint32_t max_mV)
{
	p_mc->Parameters.BatteryZero_Adcu = VMonitor_ConvertMilliVToAdcu(&p_mc->VMonitorSource, zero_mV);
	p_mc->Parameters.BatteryFull_Adcu = VMonitor_ConvertMilliVToAdcu(&p_mc->VMonitorSource, max_mV);
	Linear_ADC_Init(&p_mc->BatteryLife, p_mc->Parameters.BatteryZero_Adcu, p_mc->Parameters.BatteryFull_Adcu, 1000U);
}

// void MotorController_User_SetILimitDc(MotorController_T * p_mc, uint16_t dc)
// {
// 	uint16_t iPeakAc = dc;
// 	uint16_t motoring = iPeakAc;

// 	for(uint8_t iMotor = 0U; iMotor < p_mc->CONFIG.MOTOR_COUNT; iMotor++)
// 	{
// 		Motor_User_SetILimitMotoringParam_Amp(&p_mc->CONFIG.P_MOTORS[iMotor], motoring);
// 	}
// }