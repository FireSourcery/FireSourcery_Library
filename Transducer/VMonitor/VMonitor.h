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
//#include <stdbool.h>

typedef enum
{
	VMONITOR_LIMITS_OK,
	VMONITOR_ERROR_UPPER,
	VMONITOR_ERROR_LOWER,
}
VMonitor_Status_T;

typedef struct __attribute__ ((aligned (4U)))
{
	uint16_t LimitUpper_ADCU;
	uint16_t LimitLower_ADCU;
	//Vmax
}
VMonitor_Params_T;

typedef const struct
{
	const VMonitor_Params_T * const P_PARAMS;
	//R1, R2
	const Linear_T UNITS;
}
VMonitor_Config_T;

typedef struct
{
	VMonitor_Config_T CONFIG;
	VMonitor_Params_T Params;
}
VMonitor_T;

#define VMONITOR_CONFIG(r1, r2, adcVRef10, adcBits, vInMax, p_Params)		\
{																			\
	.CONFIG =																\
	{																		\
		.UNITS = LINEAR_VOLTAGE_CONFIG(r1, r2, adcVRef10, adcBits, vInMax),	\
		.P_PARAMS = p_Params,												\
	},																		\
}

static inline int32_t VMonitor_ConvertToMilliV(VMonitor_T * p_vMonitor, uint16_t adcu)
{
	return Linear_Voltage_CalcMilliV(&p_vMonitor->CONFIG.UNITS, adcu);
}

static inline int32_t VMonitor_ConvertToFrac16(VMonitor_T * p_vMonitor, uint16_t adcu)
{
	return Linear_Voltage_CalcFraction16(&p_vMonitor->CONFIG.UNITS, adcu);
}

static inline int32_t VMonitor_ConvertMilliVToAdcu(VMonitor_T * p_vMonitor, uint32_t mv)
{
	return Linear_Voltage_CalcAdcu_MilliV(&p_vMonitor->CONFIG.UNITS, mv);
}

extern void VMonitor_Init(VMonitor_T * p_monitor);

#endif
