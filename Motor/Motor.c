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
    @file 	Motor.c
    @author FireSoucery
    @brief  Motor module conventional function definitions.
    @version V0
*/
/**************************************************************************/
#include "Motor.h"

//#include "Motor/Instance/MotorAnalog.h"

#include "Motor/Instance/MotorStateMachine.h"
//#include "OS/StateMachine/StateMachine.h"

#include "Transducer/Encoder/Encoder_Motor.h"
#include "Transducer/Encoder/Encoder.h"

#include "Transducer/Phase/Phase.h"

//#include "Math/Q/Q.h"

void Motor_Init(Motor_T * p_motor, const Motor_Init_T * p_motorInitStruct)
{
	/*HW Wrappers Init */
//	Analog_Init
//	(
//		&p_motor->Analog,
//		p_motorInitStruct->p_AdcRegMap,
//		p_motorInitStruct->N_Adc,
//		p_motorInitStruct->M_HwBuffer,
//		MOTOR_ANALOG_ADC_CHANNEL_COUNT,
//		p_motorInitStruct->p_AdcChannelPinMap,
//		p_motor->AnalogChannelMapResults,
//		p_motor->AnalogIndexesBuffer,
//		p_motor
//	);

	//FOC Init
	Encoder_Motor_Init
	(
		&p_motor->Encoder,
		p_motorInitStruct->p_EncoderTimerCounter,
		p_motorInitStruct->EncoderTimerCounterMax,
		p_motorInitStruct->ControlFreq_Hz,
		p_motorInitStruct->PolePairs,
		p_motorInitStruct->EncoderCountsPerRevolution,
		p_motorInitStruct->EncoderDistancePerCount
	);

	Phase_Init
	(
		&p_motor->Phase,
		p_motorInitStruct->p_PhasePwmA,		/* load from flash as defined const */
		p_motorInitStruct->p_PhasePwmB,
		p_motorInitStruct->p_PhasePwmC,
		p_motorInitStruct->PhasePwmPeroid, /* possibly load via parameters extension */
		p_motorInitStruct->PhaseOnAB,
		p_motorInitStruct->PhaseOnAC,
		p_motorInitStruct->PhaseOnBC,
		p_motorInitStruct->PhaseOnBA,
		p_motorInitStruct->PhaseOnCA,
		p_motorInitStruct->PhaseOnCB
	);
	//MotorStateMachine_Init(p_Motor);
	//Linear_Init(&(p_Motor->VFMap), vPerRPM, 1, vOffset); //f(freq) = voltage
	//	Linear_Voltage_Init(&DividerBatteryVoltage, R1, R2, 5, 12);
}





