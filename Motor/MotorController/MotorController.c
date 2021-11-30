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
    @file 	Motor.h
    @author FireSoucery
    @brief Motor Utilities Instance, 1 instance per all motor
    @version V0
*/
/******************************************************************************/
#include "MotorController.h"
#include "Config.h"

//#include "Motor/Utility/MotShell/MotShell.h"
//#include "Motor/Utility/MotProtocol/MotProtocol.h"
#include "Motor/Utility/MotAnalogUser/MotAnalogUser.h"

#include "Motor/Motor/Motor.h"

#include <stdint.h>

/*

 */
void MotorController_Init(MotorController_T * p_controller)
{
	for (uint8_t iMotor = 0U; iMotor < p_controller->CONFIG.MOTOR_COUNT; iMotor++)
	{
		Motor_Init(&p_controller->CONFIG.P_MOTORS[iMotor]);
	}

	for (uint8_t iSerial = 0U; iSerial < p_controller->CONFIG.SERIAL_COUNT; iSerial++)
	{
//		Serial_Init(&p_controller->CONFIG.P_SERIALS[iSerial]);
	}

	MotAnalogUser_Init(&p_controller->AnalogUser);

	Blinky_Init(&p_controller->Buzzer);
//	Flash_Init(&p_controller->Flash);
//	EEPROM_Init(&p_controller->Eeprom);

	Timer_InitPeriodic(&p_controller->TimerSeconds,		1000U);
	Timer_InitPeriodic(&p_controller->TimerMillis, 		1U);
	Timer_InitPeriodic(&p_controller->TimerMillis10, 	10U);

	p_controller->SignalBufferAnalogMonitor.AdcFlags = 0U;
	p_controller->SignalBufferAnalogUser.AdcFlags = 0U;

	//	for (uint8_t iProtocol = 0U; iProtocol < p_controller->CONFIG.AUX_PROTOCOL_COUNT; iProtocol++)
	//	{
	//		Protocol_Init(&p_controller->CONFIG.P_AUX_PROTOCOLS[iProtocol]);
	////		ProtocolG_SetSpecs(&p_motorController->Protocols[iProtocol], p_motorController->p_Constants->P_PROTOCOL_SPECS[p_controller->Parameters.ProtocolSpecsId[iProtocol]]);
	//	}
	//
	//	Protocol_Init(&p_controller->MotProtocol);
	//	MotProtocol_Output_Init(&p_controller->MotProtocolOutput);
	//	MotProtocol_Input_Init(&p_controller->MotProtocolInput);

	//	MotShell_Init(&p_controller->MotShell);
	//	HAL_Motor_Init(); //HAL_MotorController_Init init hal analog instances
}


//void MotorUser_CaptureInput (MotorUser_T * p_motorUser)
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


