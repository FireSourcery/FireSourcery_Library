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
    @file 	Motor_Controller.h
    @author FireSoucery
    @brief  Motor module functions must be placed into corresponding user app threads
    		Most outer functions to call from MCU app
    @version V0
*/
/**************************************************************************/
#include "../Motor.h"
//#include "../Motor_FOC.h"
//#include "../Motor_SixStep.h"

#include "MotorShell.h"
#include "MotorUser.h"
#include "MotorFlash.h"

//#include "HAL_Motor.h"


typedef const struct
{
	Motor_T * const P_MOTORS;
	uint8_t MOTORS_COUNT;

	const HAL_Flash_T * const P_HAL_FLASH;
	const MotorUser_Init_T MOTOR_USER_INIT;


	//todo alarm

//	volatile const adc_t * p_VBus_ADCU;
//	volatile const adc_t * p_VAcc_ADCU;
//	volatile const adc_t * p_VSense_ADCU;
//	volatile const adc_t * p_HeatPcb_ADCU;
} MotorController_Init_T;

typedef const struct
{
	 Motor_T * p_Motors;
	 uint8_t MotorCount;

	 //mode all motor
} MotorController_T;


static inline void MotorController_Main_Thread()
{
	MotorUser_Analog_CaptureInput_IO();

	MotorUser_WriteOutput();

	HAL_Motor_EnqueueConversionUser( );

//	MotorUser_SetInput(MotorControllerMain.p_Init.P_MOTORS); //write input to motor

//		if (Thread_PollTimerCompletePeriodic(&MotorUserMain.MotorUserAnalog.MillisTimerThread) == true)
//		{
//		}
}
