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
	@file 	VMonitor.c
	@author FireSourcery
	@brief
	@version V0
*/
/******************************************************************************/
#include "VMonitor.h"

#include <string.h>
#include <stdio.h>

static inline void SetUnitConversion(VMonitor_T * p_vMonitor) //public reset?
{
	Linear_Voltage_Init(&p_vMonitor->Units, p_vMonitor->CONFIG.UNITS_R1, p_vMonitor->CONFIG.UNITS_R2, GLOBAL_ANALOG.ADC_BITS, GLOBAL_ANALOG.ADC_VREF_MILLIV, p_vMonitor->Params.VInRefMax);
}

void VMonitor_Init(VMonitor_T * p_vMonitor)
{
	if(p_vMonitor->CONFIG.P_PARAMS != 0U)
	{
		memcpy(&p_vMonitor->Params, p_vMonitor->CONFIG.P_PARAMS, sizeof(VMonitor_Params_T));
	}

	SetUnitConversion(p_vMonitor);
	p_vMonitor->Status = VMONITOR_STATUS_OK;
}

VMonitor_Status_T VMonitor_PollStatus(VMonitor_T * p_vMonitor, uint16_t adcu)
{
	VMonitor_Status_T status = VMONITOR_STATUS_OK;

	if(p_vMonitor->Params.IsMonitorEnable == true)
	{
		if		(adcu > p_vMonitor->Params.LimitUpper_Adcu) { status = VMONITOR_LIMIT_UPPER; }
		else if	(adcu < p_vMonitor->Params.LimitLower_Adcu) { status = VMONITOR_LIMIT_LOWER; }
		else
		{
			if		(adcu > p_vMonitor->Params.WarningUpper_Adcu) { status = VMONITOR_WARNING_UPPER; }
			else if	(adcu < p_vMonitor->Params.WarningLower_Adcu) { status = VMONITOR_WARNING_LOWER; }
		}
		p_vMonitor->Status = status;
	}

	return status;
}

// Frac16 only
// void VMonitor_SetVInRefMax(VMonitor_T * p_vMonitor, uint32_t vInRefMax)
// {
// 	if(p_vMonitor->Params.VInRefMax != vInRefMax)
// 	{
// 		p_vMonitor->Params.VInRefMax = vInRefMax;
// 		SetUnitConversion(p_vMonitor);
// 	}
// }

// uint16_t VMonitor_GetVInRefMax(VMonitor_T * p_vMonitor) { return p_vMonitor->Params.VInRefMax; }

void VMonitor_SetLimitUpper_MilliV(VMonitor_T * p_vMonitor, uint32_t limit_mV) { p_vMonitor->Params.LimitUpper_Adcu = Linear_Voltage_CalcAdcu_UserMilliV(&p_vMonitor->Units, limit_mV); }
void VMonitor_SetLimitLower_MilliV(VMonitor_T * p_vMonitor, uint32_t limit_mV) { p_vMonitor->Params.LimitLower_Adcu = Linear_Voltage_CalcAdcu_UserMilliV(&p_vMonitor->Units, limit_mV); }
void VMonitor_SetWarningUpper_MilliV(VMonitor_T * p_vMonitor, uint32_t limit_mV) { p_vMonitor->Params.WarningUpper_Adcu = Linear_Voltage_CalcAdcu_UserMilliV(&p_vMonitor->Units, limit_mV); }
void VMonitor_SetWarningLower_MilliV(VMonitor_T * p_vMonitor, uint32_t limit_mV) { p_vMonitor->Params.WarningLower_Adcu = Linear_Voltage_CalcAdcu_UserMilliV(&p_vMonitor->Units, limit_mV); }

void VMonitor_SetLimits_MilliV(VMonitor_T * p_vMonitor, uint32_t limitLower, uint32_t limitUpper, uint32_t warningLower, uint32_t warningUpper)
{
	VMonitor_SetLimitLower_MilliV(p_vMonitor, limitLower);
	VMonitor_SetLimitUpper_MilliV(p_vMonitor, limitUpper);
	VMonitor_SetWarningLower_MilliV(p_vMonitor, warningLower);
	VMonitor_SetWarningUpper_MilliV(p_vMonitor, warningUpper);
}

uint32_t VMonitor_GetLimitUpper_V(VMonitor_T * p_vMonitor, uint16_t vScalar) 	{ return Linear_Voltage_CalcScalarV(&p_vMonitor->Units, p_vMonitor->Params.LimitUpper_Adcu, vScalar); }
uint32_t VMonitor_GetLimitLower_V(VMonitor_T * p_vMonitor, uint16_t vScalar) 	{ return Linear_Voltage_CalcScalarV(&p_vMonitor->Units, p_vMonitor->Params.LimitLower_Adcu, vScalar); }
uint32_t VMonitor_GetWarningUpper_V(VMonitor_T * p_vMonitor, uint16_t vScalar) 	{ return Linear_Voltage_CalcScalarV(&p_vMonitor->Units, p_vMonitor->Params.WarningUpper_Adcu, vScalar); }
uint32_t VMonitor_GetWarningLower_V(VMonitor_T * p_vMonitor, uint16_t vScalar) 	{ return Linear_Voltage_CalcScalarV(&p_vMonitor->Units, p_vMonitor->Params.WarningLower_Adcu, vScalar); }
uint32_t VMonitor_GetLimitUpper_MilliV(VMonitor_T * p_vMonitor) 	{ return Linear_Voltage_CalcMilliV(&p_vMonitor->Units, p_vMonitor->Params.LimitUpper_Adcu); }
uint32_t VMonitor_GetLimitLower_MilliV(VMonitor_T * p_vMonitor) 	{ return Linear_Voltage_CalcMilliV(&p_vMonitor->Units, p_vMonitor->Params.LimitLower_Adcu); }
uint32_t VMonitor_GetWarningUpper_MilliV(VMonitor_T * p_vMonitor) 	{ return Linear_Voltage_CalcMilliV(&p_vMonitor->Units, p_vMonitor->Params.WarningUpper_Adcu); }
uint32_t VMonitor_GetWarningLower_MilliV(VMonitor_T * p_vMonitor) 	{ return Linear_Voltage_CalcMilliV(&p_vMonitor->Units, p_vMonitor->Params.WarningLower_Adcu); }



