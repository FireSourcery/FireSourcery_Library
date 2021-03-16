/**************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSoucery / The Firebrand Forge Inc

	This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).

	This program is free software: you can redistribute it and/or modify
	it under the terupdateInterval of the GNU General Public License as published by
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
    @file 	Motor_IO.h
    @author FireSoucery
    @brief  Motor module functions must be placed into corresponding user app threads
    		Most outer functions to call from MCU app
    @version V0
*/
/**************************************************************************/
#include "Motor.h"

#include "Motor_FOC.h"

#include "Peripheral/Analog/Analog_IO.h"

#include "OS/StateMachine/StateMachine.h"


/*
    Default 50us
 */
static inline void Motor_PWM_Thread(Motor_T * p_motor)
{
	/*	All FOC Modes start all ADC measurements */
//	Analog_StartConversion(&p_motor->Analog, &FOC_SAMPLE_GROUP);

	//timer in pwm, machine in adc
	//adc returns irregular times between modes, must check addtional flags
	//machine in pwm, proc foc conversion to pwm in adc
	//check foc proc flag, or set in analog conversion object
	StateMachine_Semisynchronous_ProcState(&p_motor->StateMachine);

	p_motor->Timer_ControlFreq++;
}


/*
	ADC on complete. Possibly multiple per 1 PWM
 */
static inline void Motor_ADC_Thread(Motor_T * p_motor)
{
	Analog_CaptureResults_IO(&p_motor->Analog);
}

static inline void Motor_Timer_Thread(Motor_T * p_motor) //1ms
{
	//Motor_SpeedLoop();
	//Monitor
}


static inline void Motor_Main_Thread(Motor_T * p_motor)
{
	//UI
	//Motor_SetRamp(user input)
}
