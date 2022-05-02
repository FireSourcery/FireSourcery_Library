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
	@file 	.h
	@author FireSoucery
	@brief
	@version V0
*/
/******************************************************************************/
#include "VMonitor.h"
#include <string.h>

void SetUnitConversion(VMonitor_T * p_vMonitor)
{ 
	Linear_Voltage_Init(&p_vMonitor->Units, p_vMonitor->CONFIG.UNITS_R1, p_vMonitor->CONFIG.UNITS_R2, p_vMonitor->CONFIG.UNITS_ADC_BITS, p_vMonitor->Params.AdcVRef_MilliV, p_vMonitor->Params.VInRefMax);
}

void VMonitor_Init(VMonitor_T * p_vMonitor)
{
	if(p_vMonitor->CONFIG.P_PARAMS != 0U)
	{
		memcpy(&p_vMonitor->Params, p_vMonitor->CONFIG.P_PARAMS, sizeof(VMonitor_Params_T));
	} 

	SetUnitConversion(p_vMonitor);
}

VMonitor_Status_T VMonitor_CheckLimits(VMonitor_T * p_monitor, uint16_t adcu)
{
	VMonitor_Status_T status;

	if		(adcu > p_monitor->Params.LimitUpper_ADCU) 	{ status = VMONITOR_ERROR_UPPER; }
	else if	(adcu < p_monitor->Params.LimitLower_ADCU) 	{ status = VMONITOR_ERROR_LOWER; }
	else 												{ status = VMONITOR_STATUS_OK; }

	return status;
}

VMonitor_Status_T VMonitor_Check(VMonitor_T * p_monitor, uint16_t adcu)
{
	VMonitor_Status_T status = VMonitor_CheckLimits(p_monitor, adcu);

	if	(status == VMONITOR_STATUS_OK) 	
	{ 
		if		(adcu > p_monitor->Params.WarningUpper_ADCU) 	{ status = VMONITOR_WARNING_UPPER; }
		else if	(adcu < p_monitor->Params.WarningLower_ADCU) 	{ status = VMONITOR_WARNING_LOWER; }
		else 													{ status = VMONITOR_STATUS_OK; }
	}

	return status;
}
 
void VMonitor_SetAdcVRef_MilliV(VMonitor_T * p_vMonitor, uint32_t adcVRef_MilliV)
{
	if(p_vMonitor->Params.AdcVRef_MilliV != adcVRef_MilliV)
	{
		p_vMonitor->Params.AdcVRef_MilliV = adcVRef_MilliV;
		SetUnitConversion(p_vMonitor);
	} 
}

void VMonitor_SetVInRefMax(VMonitor_T * p_vMonitor, uint32_t vInRefMax)
{
	if(p_vMonitor->Params.VInRefMax != vInRefMax)
	{
		p_vMonitor->Params.VInRefMax = vInRefMax;
		SetUnitConversion(p_vMonitor);
	}
}

void VMonitor_SetLimitUpper_MilliV(VMonitor_T * p_vMonitor, uint32_t limit_mV)
{
	p_vMonitor->Params.LimitUpper_ADCU = Linear_Voltage_CalcAdcu_UserMilliV(&p_vMonitor->Units, limit_mV);
}

void VMonitor_SetLimitLower_MilliV(VMonitor_T * p_vMonitor, uint32_t limit_mV)
{
	p_vMonitor->Params.LimitLower_ADCU = Linear_Voltage_CalcAdcu_UserMilliV(&p_vMonitor->Units, limit_mV);
} 

void VMonitor_SetWarningUpper_MilliV(VMonitor_T * p_vMonitor, uint32_t limit_mV)
{
	p_vMonitor->Params.WarningUpper_ADCU = Linear_Voltage_CalcAdcu_UserMilliV(&p_vMonitor->Units, limit_mV);
}

void VMonitor_SetWarningLower_MilliV(VMonitor_T * p_vMonitor, uint32_t limit_mV)
{
	p_vMonitor->Params.WarningLower_ADCU = Linear_Voltage_CalcAdcu_UserMilliV(&p_vMonitor->Units, limit_mV);
} 
 
int32_t VMonitor_GetLimitUpper_MilliV(VMonitor_T * p_vMonitor)
{
	return Linear_Voltage_CalcMilliV(&p_vMonitor->Units, p_vMonitor->Params.LimitUpper_ADCU);
}

int32_t VMonitor_GetLimitLower_MilliV(VMonitor_T * p_vMonitor)
{
	return Linear_Voltage_CalcMilliV(&p_vMonitor->Units, p_vMonitor->Params.LimitLower_ADCU);
}

int32_t VMonitor_GetWarningUpper_MilliV(VMonitor_T * p_vMonitor)
{
	return Linear_Voltage_CalcMilliV(&p_vMonitor->Units, p_vMonitor->Params.WarningUpper_ADCU);
}

int32_t VMonitor_GetWarningLower_MilliV(VMonitor_T * p_vMonitor)
{
	return Linear_Voltage_CalcMilliV(&p_vMonitor->Units, p_vMonitor->Params.WarningLower_ADCU);
}
