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
#include "MotorController.h"
#include "Config.h"

#include "Motor/Utility/MotAnalogUser/MotAnalogUser.h"
#include "Motor/Utility/MotAnalogMonitor/MotAnalogMonitor.h"
//#include "Motor/Utility/MotProtocol/MotProtocol.h"

#include "Motor/Motor/Motor.h"
#include "Motor/Motor/Motor_User.h"

#include "Transducer/Blinky/Blinky.h"
#include "Transducer/Thermistor/Thermistor.h"

#include "Protocol/Protocol/Protocol.h"

#include "Peripheral/Serial/Serial.h"
#include "Peripheral/NvMemory/Flash/Flash.h"
#include "Peripheral/NvMemory/EEPROM/EEPROM.h"

//#include "Utility/Shell/Shell.h"
#include "Utility/Timer/Timer.h"
#include "Utility/StateMachine/StateMachine.h"

#include <stdint.h>
#include <string.h>

/*

 */
void MotorController_Init(MotorController_T * p_controller)
{
//	if (p_controller->CONFIG.P_PARAMETERS != 0U)
//	{
	memcpy(&p_controller->Parameters, p_controller->CONFIG.P_PARAMETERS, sizeof(MotorController_Params_T));
//	}

	StateMachine_Init(&p_controller->StateMachine);

	for (uint8_t iMotor = 0U; iMotor < p_controller->CONFIG.MOTOR_COUNT; iMotor++)
	{
		Motor_Init(&p_controller->CONFIG.P_MOTORS[iMotor]);
	}

	for (uint8_t iSerial = 0U; iSerial < p_controller->CONFIG.SERIAL_COUNT; iSerial++)
	{
//		Serial_Init(&p_controller->CONFIG.P_SERIALS[iSerial]);
	}

	//	Flash_Init(&p_controller->Flash);
	//	EEPROM_Init(&p_controller->Eeprom);
	MotAnalogUser_Init(&p_controller->AnalogUser);
	MotAnalogMonitor_Init(&p_controller->AnalogMonitor);
	Blinky_Init(&p_controller->Buzzer);
	Debounce_Init(&p_controller->DIn, 	5U);	//5millis
	Pin_Output_Init(&p_controller->CONFIG.PIN_COIL);
	Pin_Output_Init(&p_controller->CONFIG.PIN_METER);

	Timer_InitPeriodic(&p_controller->TimerSeconds,		1000U);
	Timer_InitPeriodic(&p_controller->TimerMillis, 		1U);
	Timer_InitPeriodic(&p_controller->TimerMillis10, 	10U);

	p_controller->SignalBufferAnalogMonitor.AdcFlags 	= 0U;
	p_controller->SignalBufferAnalogUser.AdcFlags 		= 0U;

	//	Protocol_Init(&p_controller->MotProtocol);

	//	for (uint8_t iProtocol = 0U; iProtocol < p_controller->CONFIG.AUX_PROTOCOL_COUNT; iProtocol++)
	//	{
	//		Protocol_Init(&p_controller->CONFIG.P_AUX_PROTOCOLS[iProtocol]);
	//		ProtocolG_SetSpecs(&p_motorController->Protocols[iProtocol], p_motorController->p_Constants->P_PROTOCOL_SPECS[p_controller->Parameters.ProtocolSpecsId[iProtocol]]);
	//	}

	//	MotShell_Init(&p_controller->MotShell);
	p_controller->MainDirection = MOTOR_CONTROLLER_DIRECTION_FORWARD;
}
