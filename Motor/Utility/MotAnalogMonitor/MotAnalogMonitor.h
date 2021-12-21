/**************************************************************************/
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
/**************************************************************************/
/**************************************************************************/
/*!
    @file 	MotAnalogUser_Motor.h
    @author FireSoucery
    @brief  1 instance for all motor, input output control
    @version V0
*/
/**************************************************************************/
#ifndef MOT_ANALOG_MONITOR_H
#define MOT_ANALOG_MONITOR_H

//Analog Board Sensors

//per controller monitor

//typedef const struct
//{
//	volatile const uint16_t * const P_VBUS_ADCU;
//	volatile const uint16_t * const P_VACC_ADCU;
//	volatile const uint16_t * const P_VSENSE_ADCU;
//	volatile const uint16_t * const P_HEAT_PCB_ADCU;
//	volatile const uint16_t * const P_HEAT_MOSFETS_H_ADCU;
//	volatile const uint16_t * const P_HEAT_MOSFETS_L_ADCU;
////	volatile const uint16_t * const P_THROTTLE_ADCU;
////	volatile const uint16_t * const P_BRAKE_ADCU;
//}
//MotAnalogMonitor_AdcMap_T;

//alarm status
typedef enum
{
	MOT_ANALOG_MONITOR_OK,

}
MotAnalogMonitor_Status_T;


typedef struct __attribute__ ((aligned (4U)))
{
	uint16_t VPosLimitUpperV;
	uint16_t VPosLimitLowerV;
//	uint16_t VPosLimitHigh_Adcu;
//	uint16_t VPosLimitLow_ADCU;

	uint8_t HeatPcbShutdownC;
	uint8_t HeatPcbThresholdC;

//	uint8_t HeatPcbShutdownHigh_Adcu;
//	uint8_t HeatPcbThresholdHigh_Adcu;

	uint16_t HeatMosfetsTopShutdown_Adcu;
	uint16_t HeatMosfetsTopThreshold_Adcu;
	uint16_t HeatMosfetsBotShutdown_Adcu;
	uint16_t HeatMosfetsBotThreshold_Adcu;
}
MotAnalogMonitor_Params_T;

typedef const struct
{
//	const MotAnalogMonitor_AdcMap_T ADC_MAP;
	const uint16_t * const P_VBUS_ADCU;
	const uint16_t * const P_VACC_ADCU;
	const uint16_t * const P_VSENSE_ADCU;

	//included in thermistor
	const uint16_t * const P_HEAT_PCB_ADCU;
	const uint16_t * const P_HEAT_MOSFETS_H_ADCU;
	const uint16_t * const P_HEAT_MOSFETS_L_ADCU;

	const MotAnalogMonitor_Params_T * const P_PARAMS;
}
MotAnalogMonitor_Config_T;

typedef struct
{
	MotAnalogMonitor_Config_T CONFIG;
//	const MotAnalogMonitor_AdcMap_T ADC_MAP;
	MotAnalogMonitor_Params_T Params;

//	Thermistor_T ThermistorPcb;

	Linear_T Unit;
	Linear_T UnitBrake;




	uint16_t SampleValue;
}
MotAnalogMonitor_T;

#endif
