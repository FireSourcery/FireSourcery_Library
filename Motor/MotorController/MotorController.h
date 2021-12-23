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
    @file 	MotorController.h
    @author FireSoucery
    @brief	Facade Wrapper
    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "Config.h"

#include "Motor/Utility/MotAnalogUser/MotAnalogUser.h"
#include "Motor/Utility/MotAnalogMonitor/MotAnalogMonitor.h"
//#include "Motor/Utility/MotProtocol/MotProtocol.h"
//#include "Motor/Utility/MotShell/MotShell.h"
#include "Motor/Utility/MotMemMap/MotMemMap.h"


#include "Motor/Motor/Motor.h"

#include "Transducer/Blinky/Blinky.h"
#include "Transducer/Thermistor/Thermistor.h"

#include "Protocol/Protocol/Protocol.h"

#include "Peripheral/Serial/Serial.h"
#include "Peripheral/Analog/AnalogN/AnalogN.h"
#include "Peripheral/NvMemory/Flash/Flash.h"
#include "Peripheral/NvMemory/EEPROM/EEPROM.h"

//#include "Utility/Shell/Shell.h"
#include "Utility/Thread/Thread.h"

#include <stdint.h>

typedef enum
{
	MOTOR_CONTROLLER_INPUT_MODE_ANALOG,
	MOTOR_CONTROLLER_INPUT_MODE_SERIAL,
	MOTOR_CONTROLLER_INPUT_MODE_CAN,
}
MotorController_InputMode_T;


typedef  __attribute__ ((aligned (4U))) const volatile struct
{
//	uint8_t ShellConnectId;
//	uint8_t ProtocolDataLinkId[1]; //per protocol
//	uint8_t AuxProtocolSpecsId[CONFIG_MOTOR_CONTROLLER_AUX_PROTOCOL_COUNT];
//	uint8_t MotProtocolSpecsId;
	bool IsAnalogUserEnable;
	bool IsBuzzerOnReverseEnable;
}
MotorController_Parameters_T;

/*
 * allocated memory outside for less CONFIG define redundancy
 */
typedef const struct
{

	const MotorController_Parameters_T * const P_PARAMETERS;

	Motor_T * const P_MOTORS;
	const uint8_t MOTOR_COUNT;

	Serial_T * const P_SERIALS; //simultaneous active serial
	const uint8_t SERIAL_COUNT;

	AnalogN_T * const P_ANALOG_N;
	const AnalogN_Conversion_T CONVERSION_ANALOG_USER;
	const AnalogN_Conversion_T CONVERSION_ANALOG_MONITOR;

	//	Protocol_T * const P_AUX_PROTOCOLS; 	//Simultaneously active protocols
	//	const uint8_t AUX_PROTOCOL_COUNT;

	//	const Protocol_Specs_T * const P_PROTOCOL_SPECS_TABLE;
	//	const uint8_t PROTOCOL_SPECS_COUNT;
}
MotorController_Config_T;

/*   */
typedef struct
{
	const MotorController_Config_T CONFIG;

	MotorController_Parameters_T Parameters; //ram copy

	MotAnalogUser_T AnalogUser;
	MotAnalogMonitor_T AnalogMonitor;

	Flash_T Flash;
	EEPROM_T Eeprom;
	Blinky_T Buzzer;

	Thermistor_T ThermistorPcb;
	Thermistor_T ThermistorMosfets;

	Timer_T TimerMillis;
	Timer_T TimerMillis10;
	Timer_T TimerSeconds;

	/*
		MOTOR_ANALOG_CHANNEL_HEAT_MOSFETS,
		MOTOR_ANALOG_CHANNEL_HEAT_MOSFETS_HIGH,
		MOTOR_ANALOG_CHANNEL_HEAT_MOSFETS_LOW,
		MOTOR_ANALOG_CHANNEL_HEAT_PCB,
		MOTOR_ANALOG_CHANNEL_VACC,				 V accessories ~12V
		MOTOR_ANALOG_CHANNEL_VSENSE,			 V analog sensors ~5V
		MOTOR_ANALOG_CHANNEL_THROTTLE,
		MOTOR_ANALOG_CHANNEL_BRAKE,
	*/
	uint16_t AnalogResults[MOTOR_ANALOG_CHANNEL_COMMON_COUNT];
 	AnalogN_AdcFlags_T SignalBufferAnalogMonitor;
 	AnalogN_AdcFlags_T SignalBufferAnalogUser;

//	Protocol_T				MotProtocol;
//	MotProtocol_Output_T 	MotProtocolOutput;
//	MotProtocol_Input_T 	MotProtocolInput;

//	MotorController_InputMode_T InputMode;
//	Shell_T MotShell;

 	//Allocate outside
//	Motor_T Motors[CONFIG_MOTOR_CONTROLLER_MOTOR_COUNT];
//	Serial_T Serials[CONFIG_MOTOR_CONTROLLER_SERIAL_COUNT]; //simultaneous active serial
//	Protocol_T	 AuxProtocols[CONFIG_MOTOR_CONTROLLER_AUX_PROTOCOL_COUNT]; 	//Simultaneously active protocols
}
MotorController_T;

//priority brake thread
//static inline void MotorUser_PollBrake(MotorUser_T * p_motorUser, Motor_T * p_motor)
//{
//	Debounce_CaptureState_IO(p_motorUser->MotorUserAnalog.PinBrake);
//	p_motorUser->InputSwitchBrake 		= Debounce_GetState(p_motorUser->MotorUserAnalog.PinBrake);
//
//	if (p_motorUser->InputSwitchBrake == true)
//	{
//			Motor_SetUserCmd(p_motor, (p_motorUser->InputValueBrake + p_motorUser->InputValueBrake) / 2U);
//			Motor_SetRampDecelerate(p_motor, 0);
//	}
//}

static inline void MotorUser_PollBrake(MotorController_T * p_motorController)
{
	p_motorController->CONFIG.P_MOTORS->Direction;
	p_motorController->CONFIG.P_MOTORS[0].Direction;



	p_motorController->CONFIG.P_MOTORS[1].Direction;

	(*(p_motorController->CONFIG.P_MOTORS + 1U)).Direction;

	(p_motorController->CONFIG.P_MOTORS + 1U)->Direction;
}

static inline Motor_T * MotorController_GetPtrMotor(MotorController_T * p_motorController, uint8_t motorIndex) {return &(p_motorController->CONFIG.P_MOTORS[motorIndex]);}


extern void MotorController_Init(MotorController_T * p_controller);

#endif
