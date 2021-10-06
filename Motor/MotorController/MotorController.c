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
    @brief Motor Utilities Instance, 1 instance per all motor
    @version V0
*/
/**************************************************************************/
#include "MotorController.h"
#include "Config.h"

#include "Motor/Utility/MotShell/MotShell.h"
#include "Motor/Utility/MotProtocol/MotProtocol.h"
#include "Motor/Utility/MotAnalogUser/MotAnalogUser.h"

#include "Motor/Motor/Motor.h"
#include "Motor/Motor/HAL_Motor.h"

#include <stdint.h>

/*

 */
void MotorController_Init(MotorController_T * p_controller, const MotorController_Config_T * p_consts)
{
	p_controller->p_Config = p_consts;

	HAL_Motor_Init(); //HAL_MotorController_Init init hal analog instances

	for(uint8_t iMotor = 0U; iMotor < CONFIG_MOTOR_CONTROLLER_MOTOR_COUNT; iMotor++)
	{
		//if default
		Motor_Init_Default(&p_controller->Motors[iMotor], &p_consts->MOTOR_INITS[iMotor]);

		//if flash
		// must set eeprom pointer first
		// Motor_Init(&p_controller->Motors[iMotor], &p_consts->P_MOTORS_CONSTANTS[iMotor]);
	}

	for(uint8_t iSerial = 0U; iSerial < CONFIG_MOTOR_CONTROLLER_SERIAL_COUNT; iSerial++)
	{
		Serial_Init(&p_controller->Serials[iSerial]);
	}

//	for(uint8_t iProtocol = 0U; iProtocol < 1; iProtocol++)
//	{
//		ProtocolG_Init(&p_controller->AuxProtocols[iProtocol], &p_consts->AUX_PROTOCOL_CONFIG[iProtocol]);
//
//		//if flash set, com port and protocol can be fixed
////		ProtocolG_SetSpecs(&p_motorController->Protocols[iProtocol], p_motorController->p_Constants->P_PROTOCOL_SPECS[p_controller->Parameters.ProtocolSpecsId[iProtocol]]);
//	}

	ProtocolG_Init(&p_controller->MotProtocol, &p_consts->MOT_PROTOCOL_CONFIG);
	MotProtocol_Output_Init(&p_controller->MotProtocolOutput, &p_controller->p_Config->MOT_PROTOCOL_CONFIG_REGISTERS);
	MotProtocol_Input_Init(&p_controller->MotProtocolInput,  &p_controller->p_Config->MOT_PROTOCOL_CONFIG_REGISTERS);

	MotAnalogUser_Init(&p_controller->AnalogUser, &p_consts->MOT_ANALOG_USER_CONFIG);

	MotShell_Init(&p_controller->MotShell, &p_consts->MOT_SHELL_CONFIG);
//		&p_motorController->Serials[p_motorController->Parameters.ShellConnectId]
//		&p_motorController->Motors[0U],
//		CONFIG_MOTOR_CONTROLLER_MOTOR_COUNT,
//		&p_controller->MotorUser


	Thread_InitThreadPeriodic_Period(&p_controller->TimerSeconds, 	p_consts->P_MILLIS_TIMER, 1000U, 1000U,	0U, 0U);
	Thread_InitThreadPeriodic_Period(&p_controller->TimerMillis, 	p_consts->P_MILLIS_TIMER, 1000U, 1U,	0U, 0U);
	Thread_InitThreadPeriodic_Period(&p_controller->TimerMillis10, 	p_consts->P_MILLIS_TIMER, 1000U, 10U,	0U, 0U);

	Blinky_Init(&p_controller->BlinkyAlarm, &p_consts->HAL_PIN_ALARM);

	//	MotorFlash_Init(&p_controller->MotorFlash, &p_consts->MOTOR_FLASH_CONSTS);
}


//void MotorUser_CaptureInput_IO(MotorUser_T * p_motorUser)
//{
//	switch (p_motorUser->InputMode)
//	{
//	case MOTOR_INPUT_MODE_ANALOG:
//		break;
//
//	case MOTOR_INPUT_MODE_SERIAL:
//		//select protocol
//
////		Protocol_Slave_Proc(p_motorUser->Protocol);
//		break;
//
//
////	case MOTOR_INPUT_MODE_DISABLED:
////		break;
//
//	default:
//		break;
//	}
//
//}


