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
#ifndef MOT_ANALOG_MONITOR_H
#define MOT_ANALOG_MONITOR_H

//#include "Motor/Utility/MotAnalog/MotAnalog.h"

#include "Transducer/Thermistor/Thermistor.h"

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
	MOT_ANALOG_MONITOR_LIMITS_OK  ,
	MOT_ANALOG_MONITOR_ERROR_UPPER,
	MOT_ANALOG_MONITOR_ERROR_LOWER,
}
MotAnalogMonitor_LimitsStatus_T;


typedef struct __attribute__ ((aligned (4U)))
{
//	uint16_t VPosLimitUpper_V;
//	uint16_t VPosLimitLower_V;

	uint16_t VPosLimitUpper_ADCU;
	uint16_t VPosLimitLower_ADCU;
	uint16_t VSenseLimitUpper_ADCU;
	uint16_t VSenseLimitLower_ADCU;
	uint16_t VAccLimitUpper_ADCU;
	uint16_t VAccLimitLower_ADCU;
}
MotAnalogMonitor_Params_T;

//typedef struct
//{
//	uint16_t  VBus_ADCU;
//	uint16_t  VAcc_ADCU;
//	uint16_t  VSense_ADCU;
//}
//MotAnalogMonitor_LimitsCheckValues_T;
//


typedef const struct
{
	const MotAnalogMonitor_Params_T * const P_PARAMS;
}
MotAnalogMonitor_Config_T;

typedef struct
{
	MotAnalogMonitor_Config_T CONFIG;
	MotAnalogMonitor_Params_T Params;


}
MotAnalogMonitor_T;

#define MOT_ANALOG_MONITOR_CONFIG(p_Params)			\
{													\
	.CONFIG =										\
	{												\
		.P_PARAMS =		p_Params,					\
	},												\
}

static inline MotAnalogMonitor_LimitsStatus_T CheckMotAnalogMonitorLimits(int32_t upperLimit, int32_t lowerLimit, int32_t value)
{
	MotAnalogMonitor_LimitsStatus_T status;

	if (value > upperLimit)
	{
		status = MOT_ANALOG_MONITOR_ERROR_UPPER;
	}
	else if (value < lowerLimit)
	{
		status = MOT_ANALOG_MONITOR_ERROR_LOWER;
	}
	else
	{
		status = MOT_ANALOG_MONITOR_LIMITS_OK;
	}

	return status;
}

static inline MotAnalogMonitor_LimitsStatus_T MotAnalogMonitor_CheckVPos_ADCU(MotAnalogMonitor_T * p_monitor, uint16_t vPos_ADCU)
{
	return CheckMotAnalogMonitorLimits(p_monitor->Params.VPosLimitUpper_ADCU, p_monitor->Params.VPosLimitLower_ADCU, vPos_ADCU);
}

static inline MotAnalogMonitor_LimitsStatus_T MotAnalogMonitor_GetVPosLimitUpper_MilliV(MotAnalogMonitor_T * p_monitor, uint16_t vPos_ADCU)
{

}


static inline MotAnalogMonitor_LimitsStatus_T MotAnalogMonitor_CheckVAcc_ADCU(MotAnalogMonitor_T * p_monitor, uint16_t vPos_ADCU)
{
//	return CheckMotAnalogMonitorLimits(p_monitor->Params. , p_monitor->Params. , vPos_ADCU);
}

static inline MotAnalogMonitor_LimitsStatus_T MotAnalogMonitor_CheckVSense_ADCU(MotAnalogMonitor_T * p_monitor, uint16_t vPos_ADCU)
{
//	return CheckMotAnalogMonitorLimits(p_monitor->Params. , p_monitor->Params. , vPos_ADCU);
}


extern void MotAnalogMonitor_Init(MotAnalogMonitor_T * p_motorUser);

#endif
