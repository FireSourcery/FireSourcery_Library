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
	@file 	 	.h
	@author 	FireSoucery
	@brief		Analog Board Sensors, 1 instance per for all motors
	@version 	V0
*/
/******************************************************************************/
#ifndef VMONITOR_H
#define VMONITOR_H

#include "Math/Linear/Linear_Voltage.h"  
#include <stdint.h> 

typedef enum VMonitor_Status_Tag
{
	VMONITOR_STATUS_OK,
	VMONITOR_ERROR_UPPER,
	VMONITOR_ERROR_LOWER,
	VMONITOR_WARNING_UPPER,
	VMONITOR_WARNING_LOWER,
}
VMonitor_Status_T;

typedef struct __attribute__((aligned(4U))) VMonitor_Params_Tag
{
	uint16_t LimitUpper_ADCU;
	uint16_t LimitLower_ADCU;
	uint16_t WarningUpper_ADCU;
	uint16_t WarningLower_ADCU;

	uint16_t AdcVRef_MilliV; /* static */
	uint16_t VInRefMax; /* Max as Frac16 */
}
VMonitor_Params_T;

typedef const struct VMonitor_Config_Tag
{
	const uint16_t UNITS_R1;
	const uint16_t UNITS_R2;
	const uint16_t UNITS_ADC_BITS;
	const VMonitor_Params_T * const P_PARAMS;
}
VMonitor_Config_T;

typedef struct
{
	VMonitor_Config_T CONFIG;
	VMonitor_Params_T Params;
	Linear_T Units;
}
VMonitor_T;

#define VMONITOR_CONFIG(r1, r2, adcBits, p_Params)		\
{														\
	.CONFIG =											\
	{													\
	 	.UNITS_R1 = r1, 								\
		.UNITS_R2 = r2,									\
		.UNITS_ADC_BITS = adcBits,						\
		.P_PARAMS = p_Params,							\
	},													\
}

static inline int32_t VMonitor_ConvertToMilliV(VMonitor_T * p_vMonitor, uint16_t adcu)
{
	return Linear_Voltage_CalcMilliV(&p_vMonitor->Units, adcu);
}

static inline int32_t VMonitor_ConvertToFrac16(VMonitor_T * p_vMonitor, uint16_t adcu)
{
	return Linear_Voltage_CalcFraction16(&p_vMonitor->Units, adcu);
}

static inline int32_t VMonitor_ConvertMilliVToAdcu(VMonitor_T * p_vMonitor, uint32_t milliV)
{
	return Linear_Voltage_CalcAdcu_MilliV(&p_vMonitor->Units, milliV);
}

extern void VMonitor_Init(VMonitor_T * p_monitor);
extern VMonitor_Status_T VMonitor_CheckLimits(VMonitor_T * p_monitor, uint16_t adcu);
extern VMonitor_Status_T VMonitor_Check(VMonitor_T * p_monitor, uint16_t adcu);
extern void VMonitor_SetAdcVRef_MilliV(VMonitor_T * p_vMonitor, uint32_t adcVRef_MilliV);
extern void VMonitor_SetVInRefMax(VMonitor_T * p_vMonitor, uint32_t vInRefMax);
extern void VMonitor_SetLimitUpper_MilliV(VMonitor_T * p_vMonitor, uint32_t limit_mV);
extern void VMonitor_SetLimitLower_MilliV(VMonitor_T * p_vMonitor, uint32_t limit_mV);
extern void VMonitor_SetWarningUpper_MilliV(VMonitor_T * p_vMonitor, uint32_t limit_mV);
extern void VMonitor_SetWarningLower_MilliV(VMonitor_T * p_vMonitor, uint32_t limit_mV);
extern int32_t VMonitor_GetLimitUpper_MilliV(VMonitor_T * p_vMonitor);
extern int32_t VMonitor_GetLimitLower_MilliV(VMonitor_T * p_vMonitor);
extern int32_t VMonitor_GetWarningUpper_MilliV(VMonitor_T * p_vMonitor);
extern int32_t VMonitor_GetWarningLower_MilliV(VMonitor_T * p_vMonitor);

#endif
