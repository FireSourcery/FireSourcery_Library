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
	MOT_ANALOG_MONITOR_OK = 0U,
	MOT_ANALOG_MONITOR_ERROR,
}
MotAnalogMonitor_Status_T;

typedef enum
{
	MOT_ANALOG_MONITOR_LIMITS_OK = MOT_ANALOG_MONITOR_OK,
	MOT_ANALOG_MONITOR_ERROR_UPPER,
	MOT_ANALOG_MONITOR_ERROR_LOWER,
}
MotAnalogMonitor_LimitsStatus_T;

typedef enum
{
	MOT_ANALOG_MONITOR_THRESHOLD_OK = MOT_ANALOG_MONITOR_OK,
	MOT_ANALOG_MONITOR_ERROR_SHUTDOWN,
	MOT_ANALOG_MONITOR_ERROR_THRESHOLD,
}
MotAnalogMonitor_ThesholdStatus_T;

typedef struct __attribute__ ((aligned (4U)))
{
//	uint16_t VPosLimitUpper_V;
//	uint16_t VPosLimitLower_V;
//	uint8_t HeatPcbShutdown_C;
//	uint8_t HeatPcbThreshold_C;

	uint16_t VPosLimitUpper_ADCU;
	uint16_t VPosLimitLower_ADCU;
	uint16_t VSenseLimitUpper_ADCU;
	uint16_t VSenseLimitLower_ADCU;
	uint16_t VAccLimitUpper_ADCU;
	uint16_t VAccLimitLower_ADCU;

	uint16_t HeatPcbShutdown_ADCU;
	uint16_t HeatPcbThreshold_ADCU;
	uint16_t HeatMosfetsTopShutdown_ADCU;
	uint16_t HeatMosfetsTopThreshold_ADCU;
	uint16_t HeatMosfetsBotShutdown_ADCU;
	uint16_t HeatMosfetsBotThreshold_ADCU;

//	uint16_t HeatMotorShutdownArray_ADCU[CONFIG_MOT_ANALOG_MONITOR_MOTOR_COUNT];
//	uint16_t HeatMotorThresholdArray_ADCU[CONFIG_MOT_ANALOG_MONITOR_MOTOR_COUNT];
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
//typedef struct
//{
//	uint16_t HeatPcb_ADCU;
//	uint16_t HeatMosfetsTop_ADCU;
//	uint16_t HeatMosfetsBot_ADCU;
//
////	uint16_t HeatMotor_ADCU[CONFIG_MOT_ANALOG_MONITOR_MOTOR_COUNT];
////	uint16_t HeatMotor_ADCU[CONFIG_MOT_ANALOG_MONITOR_MOTOR_COUNT];
//}
//MotAnalogMonitor_ThresholdCheckValues_T;

typedef const struct
{
	const MotAnalogMonitor_Params_T * const P_PARAMS;
}
MotAnalogMonitor_Config_T;

typedef struct
{
	MotAnalogMonitor_Config_T CONFIG;
	MotAnalogMonitor_Params_T Params;

//	Thermistor_T ThermistorPcb;
//	Linear_T Unit;

	/*
	 * Save state for threshold type
	 */
	MotAnalogMonitor_ThesholdStatus_T HeatPcbShutdown;
	MotAnalogMonitor_ThesholdStatus_T HeatMosfetTopShutdown;
	MotAnalogMonitor_ThesholdStatus_T HeatMosfetBotShutdown;
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

//static inline MotAnalogMonitor_LimitsStatus_T MotAnalogMonitor_CheckVAcc_ADCU(MotAnalogMonitor_T * p_monitor, uint16_t vPos_ADCU)
//{
//	return CheckMotAnalogMonitorLimits(p_monitor->Params. , p_monitor->Params. , vPos_ADCU);
//}
//
//static inline MotAnalogMonitor_LimitsStatus_T MotAnalogMonitor_CheckVSense_ADCU(MotAnalogMonitor_T * p_monitor, uint16_t vPos_ADCU)
//{
//	return CheckMotAnalogMonitorLimits(p_monitor->Params. , p_monitor->Params. , vPos_ADCU);
//}

static inline MotAnalogMonitor_ThesholdStatus_T CheckMotAnalogMonitorShutdown(MotAnalogMonitor_ThesholdStatus_T * p_state, int32_t shutdown, int32_t threshold, int32_t value)
{
	if(*p_state == MOT_ANALOG_MONITOR_THRESHOLD_OK)
	{
		if(value > shutdown)
		{
			*p_state = MOT_ANALOG_MONITOR_ERROR_SHUTDOWN; //crossing shutdown
		}
	}
	else
	{
		if(value > shutdown)
		{
			*p_state = MOT_ANALOG_MONITOR_ERROR_SHUTDOWN; //crossing shutdown
		}
		else if(value > threshold)
		{
			*p_state = MOT_ANALOG_MONITOR_ERROR_THRESHOLD; //still over threshold
		}
		else
		{
			*p_state = MOT_ANALOG_MONITOR_THRESHOLD_OK;
		}
	}

	return *p_state;
}

static inline MotAnalogMonitor_ThesholdStatus_T MotAnalogMonitor_CheckHeatPcb_ADCU(MotAnalogMonitor_T * p_monitor, uint16_t heatPcb_ADCU)
{
	return CheckMotAnalogMonitorShutdown(&p_monitor->HeatPcbShutdown, p_monitor->Params.HeatPcbShutdown_ADCU, p_monitor->Params.HeatPcbThreshold_ADCU, heatPcb_ADCU);
}

static inline MotAnalogMonitor_ThesholdStatus_T MotAnalogMonitor_CheckHeatMosfetsTop_ADCU(MotAnalogMonitor_T * p_monitor, uint16_t heat_ADCU)
{
	return CheckMotAnalogMonitorShutdown(&p_monitor->HeatMosfetTopShutdown, p_monitor->Params.HeatMosfetsTopShutdown_ADCU, p_monitor->Params.HeatMosfetsTopThreshold_ADCU, heat_ADCU);
}

static inline MotAnalogMonitor_ThesholdStatus_T MotAnalogMonitor_CheckHeatMosfetsBot_ADCU(MotAnalogMonitor_T * p_monitor, uint16_t heat_ADCU)
{
	return CheckMotAnalogMonitorShutdown(&p_monitor->HeatMosfetBotShutdown, p_monitor->Params.HeatMosfetsBotShutdown_ADCU, p_monitor->Params.HeatMosfetsBotThreshold_ADCU, heat_ADCU);
}

static inline MotAnalogMonitor_Status_T MotAnalogMonitor_CheckHeat_ADCU(MotAnalogMonitor_T * p_monitor, uint16_t heatPcb_ADCU, uint16_t heatMosTop_ADCU, uint16_t heatMosBot_ADCU)
{
	MotAnalogMonitor_Status_T status;

	if
	(
		(MotAnalogMonitor_CheckHeatPcb_ADCU(p_monitor, heatPcb_ADCU) 			!= MOT_ANALOG_MONITOR_THRESHOLD_OK) ||
		(MotAnalogMonitor_CheckHeatMosfetsTop_ADCU(p_monitor, heatMosTop_ADCU) 	!= MOT_ANALOG_MONITOR_THRESHOLD_OK) ||
		(MotAnalogMonitor_CheckHeatMosfetsBot_ADCU(p_monitor, heatMosBot_ADCU) 	!= MOT_ANALOG_MONITOR_THRESHOLD_OK)
	)
	{
		status = MOT_ANALOG_MONITOR_ERROR;
	}
	else
	{
		status = MOT_ANALOG_MONITOR_OK;
	}

	return status;
}

//static inline MotAnalogMonitor_ThesholdStatus_T MotAnalogMonitor_CheckHeatMotor_ADCU(MotAnalogMonitor_T * p_monitor, uint8_t motorIndex, uint16_t heat_ADCU)
//{
//	return CheckMotAnalogMonitorShutdown(&p_monitor->HeatMotor, p_monitor->Params.HeatMotorsShutdown_ADCU[motorIndex], p_monitor->Params.HeatMotorsThreshold_ADCU[motorIndex], heat_ADCU);
//}

extern void MotAnalogMonitor_Init(MotAnalogMonitor_T * p_motorUser);

#endif
