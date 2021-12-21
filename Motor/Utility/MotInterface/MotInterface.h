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
    @file 	Motor.h
    @author FireSoucery
    @brief  common registers for all input output devices
    		+ error checking
    @version V0
*/
/**************************************************************************/
#ifndef MOT_INTERFACE_H
#define MOT_INTERFACE_H

#include "Motor/Motor/Motor.h"

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
	volatile bool InputSwitchDirection;
}
MotInterface_InputMotor_T;

typedef struct
{

}
MotInterface_Input_T;

//
////known at compile time
////typedef const struct
////{
////	const Motor_T * const  P_MOTOR; //temp read access to all motor data
////	const Motor_AdcMap_T * const P_ADC_MAP;
////	const Motor_Parameters_T * const P_PARAMETERS;
////	volatile const uint16_t * const P_SPEED_RPM;
////}
////MotInterface_OutputMap_T;

typedef struct
{
//	const MotInterface_Output_MotorMap_T * p_MotorMap;
	//can be auto set during init if not const
//	const Motor_T * p_Motor; //temp read access to all motor data
//	const Motor_AdcMap_T * p_AdcMap;
//	const Motor_Parameters_T * p_Parameters;
//	volatile const uint16_t * p_SpeedRpm;

//	const Motor_T * p_Motor; // interface functions limit access, read-only

	//processed outputs;
	/* UI Unit report */
	uint16_t VBus_mV;
	uint16_t VBemfPeak_mV;
	uint16_t VBemfA_mV;
	uint16_t IBus_Amp;

	//	bool SettingEnableOutputUnitsX;
}
MotInterface_OutputMotor_T;

typedef const struct
{
//	const 	MotInterface_Output_MotorMap_T 	* const P_MOTOR_MAPS;
//			MotInterface_Output_MotorVars_T 	* const P_MOTOR_VARS; //pointer to array, out module  allocate
//	const uint8_t OUTPUT_MOTORS_COUNT
//	MotInterface_OutputMotor_T * const P_OUTPUT_MOTORS;
//	const AnalogMonitor_AdcMap_T * const P_ADC_MAP_BOARD;
}
MotInterface_Output_T;

typedef struct
{
//	const Motor_T * const  P_MOTORS;
//	const uint8_t MOTORS_COUNT;
//	const MotInterface_OutputFull_T OUTPUT_MAP;
//	const MotInterface_Output_MotorMap_T * const P_OUTPUT_MOTOR_MAPS;
}
MotInterface_Config_T;

typedef struct
{
	MotInterface_Input_T Input;
	MotInterface_Output_T Output;
}
MotInterface_T;


#endif
