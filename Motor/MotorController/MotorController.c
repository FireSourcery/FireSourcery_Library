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
    @brief
    @version V0
*/
/**************************************************************************/
#include "MotorController.h"
#include "MotorUser.h"
#include "MotorFlash.h"

//#include "Config.h"

//#include "Peripheral/Pin/Debounce.h"
//#include "Peripheral/Pin/Pin.h"
//
//#include "Math/Linear/Linear_ADC.h"
//#include "Math/Linear/Linear.h"

#include <stdint.h>
#include <stdbool.h>

/*
 * Motor Utility Instance Init, 1 for all motor
 */
void MotorController_Init(MotorController_Init_T * p_init)
{

	MotorUser_Init(&p_init->MOTOR_USER_INIT);

	MotorFlash_Init(p_init->P_HAL_FLASH);


//	MotorShell_Init()
}

//writeflash state
