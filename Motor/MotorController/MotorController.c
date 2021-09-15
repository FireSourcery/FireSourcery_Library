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

#include "Motor/Motor.h"
#include "Motor/HAL_Motor.h"

#include "Motor/Utility/MotorUser.h"
//#include "Motor/Utility/MotorFlash.h"
#include "Motor/Utility/MotorShell.h"

#include <stdint.h>

/*

 */
void MotorController_Init(MotorController_T * p_motorController, const MotorController_Constants_T * p_consts)
{
	p_motorController->p_Constants = p_consts;

	p_motorController->Parameters.ShellConnectId = 1U;

	HAL_Motor_Init(); //HAL_MotorController_Init init hal analog instances

	for(uint8_t iMotor = 0U; iMotor < CONFIG_MOTOR_CONTROLLER_MOTOR_COUNT; iMotor++)
	{
		//if default
		Motor_Init_Default(&p_motorController->Motors[iMotor], &p_consts->MOTOR_INITS[iMotor]);

		//if flash
		// must set eeprom pointer first
		// Motor_Init(&p_motorController->Motors[iMotor], &p_consts->P_MOTORS_CONSTANTS[iMotor]);
	}

	for(uint8_t iSerial = 0U; iSerial < CONFIG_MOTOR_CONTROLLER_SERIAL_COUNT; iSerial++)
	{
		Serial_Init(&p_motorController->Serials[iSerial], &p_consts->SERIAL_INITS[iSerial]);
	}

	MotorUser_Init(&p_motorController->MotorUser, &p_consts->MOTOR_USER_INIT);
//	MotorFlash_Init(&p_motorController->MotorFlash, &p_consts->MOTOR_FLASH_CONSTS);
//	MotorShell_Init(p_motorController);
	MotorShell_Init
	(
		&p_motorController->MotorShell,
		&p_motorController->Serials[p_motorController->Parameters.ShellConnectId],
		&p_motorController->Motors[0U],
		CONFIG_MOTOR_CONTROLLER_MOTOR_COUNT,
		&p_motorController->MotorUser
	);

	Thread_InitThreadPeriodic_Period(&p_motorController->TimerSeconds, 	p_consts->P_MILLIS_TIMER, 1000U, 1000U, 	0U, 0U);
	Thread_InitThreadPeriodic_Period(&p_motorController->TimerMillis, 	p_consts->P_MILLIS_TIMER, 1000U, 1U, 		0U, 0U);
	Thread_InitThreadPeriodic_Period(&p_motorController->TimerMillis10, p_consts->P_MILLIS_TIMER, 1000U, 10U, 		0U, 0U);


	Blinky_Init(&p_motorController->BlinkyAlarm, &p_consts->HAL_PIN_ALARM);
}


