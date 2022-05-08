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
	@file 	VMonitor.h
	@author FireSoucery
	@brief 	Voltage divider unit conversion, plus status monitor
	@version V0
*/
/******************************************************************************/
#ifndef VMONITOR_H
#define VMONITOR_H

#include "Math/Linear/Linear_Voltage.h"  
#include <stdint.h> 
#include <stdbool.h>
 
typedef enum VMonitor_Status_Tag
{
	VMONITOR_STATUS_OK,
	VMONITOR_LIMIT_UPPER,
	VMONITOR_LIMIT_LOWER,
	VMONITOR_WARNING_UPPER,
	VMONITOR_WARNING_LOWER,
}
VMonitor_Status_T;

typedef struct __attribute__((aligned(4U))) VMonitor_Params_Tag
{
	uint16_t LimitUpper_Adcu;
	uint16_t LimitLower_Adcu;
	uint16_t WarningUpper_Adcu;
	uint16_t WarningLower_Adcu; 
	uint16_t VInRefMax; /* VIn 100% */
	bool IsEnableOnInit;
}
VMonitor_Params_T;

typedef const struct VMonitor_Config_Tag
{
	const uint16_t UNITS_R1;
	const uint16_t UNITS_R2; 
	const VMonitor_Params_T * const P_PARAMS;
}
VMonitor_Config_T;

typedef struct
{
	VMonitor_Config_T CONFIG;
	VMonitor_Params_T Params;
	Linear_T Units;
	VMonitor_Status_T Status; /* Store status for low freq polling */
}
VMonitor_T;

#define VMONITOR_CONFIG(r1, r2, p_Params)	\
{											\
	.CONFIG =								\
	{										\
	 	.UNITS_R1 = r1, 					\
		.UNITS_R2 = r2,						\
		.P_PARAMS = p_Params,				\
	},										\
}

static inline int32_t VMonitor_ConvertToV(VMonitor_T * p_vMonitor, uint16_t adcu, uint16_t vScalar) { return Linear_Voltage_CalcScalarV(&p_vMonitor->Units, adcu, vScalar); }
static inline int32_t VMonitor_ConvertToFrac16(VMonitor_T * p_vMonitor, uint16_t adcu) { return Linear_Voltage_CalcFraction16(&p_vMonitor->Units, adcu); }

// static inline int32_t VMonitor_ConvertToMilliV(VMonitor_T * p_vMonitor, uint16_t adcu) { return Linear_Voltage_CalcMilliV(&p_vMonitor->Units, adcu); }
static inline int32_t VMonitor_ConvertMilliVToAdcu(VMonitor_T * p_vMonitor, uint32_t milliV) { return Linear_Voltage_CalcAdcu_MilliV(&p_vMonitor->Units, milliV); }

static inline bool VMonitor_GetIsStatusLimit(VMonitor_T * p_vMonitor) { return ((p_vMonitor->Status == VMONITOR_LIMIT_UPPER) || (p_vMonitor->Status == VMONITOR_LIMIT_LOWER)); }
static inline bool VMonitor_GetIsStatusWarning(VMonitor_T * p_vMonitor) { return ((p_vMonitor->Status == VMONITOR_WARNING_UPPER) || (p_vMonitor->Status == VMONITOR_WARNING_LOWER)); }
static inline VMonitor_Status_T VMonitor_GetStatus(VMonitor_T * p_vMonitor) { return  (p_vMonitor->Status); }

static inline void VMonitor_Enable(VMonitor_T * p_vMonitor) { p_vMonitor->Params.IsEnableOnInit = true; }
static inline void VMonitor_Disable(VMonitor_T * p_vMonitor) { p_vMonitor->Params.IsEnableOnInit = false; }

extern void VMonitor_InitAdcVRef_MilliV(uint16_t adcVRef_MilliV);
extern void VMonitor_Init(VMonitor_T * p_vMonitor);

extern VMonitor_Status_T VMonitor_PollStatus(VMonitor_T * p_vMonitor, uint16_t adcu);  

extern void VMonitor_SetVInRefMax(VMonitor_T * p_vMonitor, uint32_t vInRefMax);
extern void VMonitor_SetLimitUpper_MilliV(VMonitor_T * p_vMonitor, uint32_t limit_mV);
extern void VMonitor_SetLimitLower_MilliV(VMonitor_T * p_vMonitor, uint32_t limit_mV);
extern void VMonitor_SetWarningUpper_MilliV(VMonitor_T * p_vMonitor, uint32_t limit_mV);
extern void VMonitor_SetWarningLower_MilliV(VMonitor_T * p_vMonitor, uint32_t limit_mV);
extern void VMonitor_SetLimits_MilliV(VMonitor_T * p_vMonitor, uint32_t limitLower, uint32_t limitUpper, uint32_t warningLower, uint32_t warningUpper);

extern uint16_t VMonitor_GetVInRefMax(VMonitor_T * p_vMonitor);
extern uint32_t VMonitor_GetLimitUpper_V(VMonitor_T * p_vMonitor, uint16_t vScalar);
extern uint32_t VMonitor_GetLimitLower_V(VMonitor_T * p_vMonitor, uint16_t vScalar);
extern uint32_t VMonitor_GetWarningUpper_V(VMonitor_T * p_vMonitor, uint16_t vScalar); 
extern uint32_t VMonitor_GetWarningLower_V(VMonitor_T * p_vMonitor, uint16_t vScalar); 
extern uint32_t VMonitor_GetLimitUpper_MilliV(VMonitor_T * p_vMonitor);
extern uint32_t VMonitor_GetLimitLower_MilliV(VMonitor_T * p_vMonitor);
extern uint32_t VMonitor_GetWarningUpper_MilliV(VMonitor_T * p_vMonitor);
extern uint32_t VMonitor_GetWarningLower_MilliV(VMonitor_T * p_vMonitor);

#ifdef VMONITOR_STRING_FUNCTIONS 
extern size_t VMonitor_ToString_Verbose(VMonitor_T * p_vMonitor, char * p_stringBuffer, uint16_t unitVScalar);
extern  bool VMonitor_ToString_Data(VMonitor_T * p_vMonitor, char * p_stringDest); 
#endif

#endif
