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

#include "Motor/HAL_Motor.h"
#include "Motor/Utility/MotorUser.h"

//#include "Motor/Utility/MotorFlash.h"

#include <stdint.h>

/*

 */
//MotorController_T MotorControllerMain
//void MotorController_Init(const MotorController_Constants_T * p_consts)
void MotorController_Init(MotorController_T * p_motorController, const MotorController_Constants_T * p_consts)
{
	MotorUser_Init(&p_motorController->MotorUser, &p_consts->MOTOR_USER_INIT);
	MotorControllerShell_Init(p_motorController);

	HAL_Motor_Init(); //HAL_MotorController_Init init hal analog instances

	for(uint8_t iMotor = 0U; iMotor < CONFIG_MOTOR_CONTROLLER_MOTOR_COUNT; iMotor++)
	{
//		Motor_Init(&p_motorController->Motors[iMotor], &MOTOR_CONSTANTS_1, &MOTOR_PARAMETERS_DEFAULT);
	}


//	Serial_Init();
	//MotorFlash_Init(&p_motorController->MotorFlash, p_init->P_HAL_FLASH);
}


