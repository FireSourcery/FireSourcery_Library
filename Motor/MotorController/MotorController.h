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
    @file 	MotorController.h
    @author FireSoucery
    @brief
    @version V0
*/
/**************************************************************************/
#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "Config.h"

#include "Motor/Utility/MotorUser.h"
//#include "MotorFlash.h"

#include "Motor/Motor.h"
#include "Motor/HAL_Motor.h"

#include "System/Shell/Shell.h"

#ifdef CONFIG_MOTOR_ADC_8
	typedef uint16_t adc_t;
#elif defined(CONFIG_MOTOR_ADC_16)
	typedef uint16_t adc_t;
#endif

typedef const struct
{
//	volatile Motor_T * const P_MOTORS;
//	const uint8_t MOTORS_COUNT;

	//	volatile const adc_t * P_VBUS_ADCU;
	volatile const adc_t * const P_VACC_ADCU;
	volatile const adc_t * const P_VSENSE_ADCU;
	volatile const adc_t * const P_HEAT_PCB_ADCU;

	const MotorUser_Consts_T MOTOR_USER_INIT;

	const HAL_Serial_T * const P_HAL_SERIALS[CONFIG_MOTOR_CONTROLLER_HW_HAL_SERIAL_COUNT];

//	const HAL_Flash_T * const P_HAL_FLASH;

	//		.P_TX_BUFFER
	//		.P_RX_BUFFER
	//		.TX_BUFFER_SIZE
	//		.RX_BUFFER_SIZE

	//todo alarm thermistor
}
MotorController_Constants_T;

typedef struct
{
	uint8_t ShellConnectId;
}
MotorController_Parameters_T;

typedef struct
{
	const MotorController_Constants_T * p_Constants;
	MotorController_Parameters_T Parameters;

	/* compile time const for all instancesl, only 1 instance expected. */

	Motor_T Motors[CONFIG_MOTOR_CONTROLLER_MOTOR_COUNT];

	uint8_t SerialTxBuffers[CONFIG_MOTOR_CONTROLLER_SERIAL_COUNT][100U];
	uint8_t SerialRxBuffers[CONFIG_MOTOR_CONTROLLER_SERIAL_COUNT][100U];
	Serial_T Serials[CONFIG_MOTOR_CONTROLLER_SERIAL_COUNT]; //simutanous active serial


	Shell_T MotorControllerShell;
	MotorUser_T MotorUser;

	//Flash_T MotorFlash;
}
MotorController_T;






#endif
