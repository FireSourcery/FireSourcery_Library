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
    @file 	Motor.c
    @author FireSoucery
    @brief  Motor module conventional function definitions.
    @version V0
*/
/******************************************************************************/
#include "Motor.h"

#include <string.h>

void Motor_Init(Motor_T * p_motor)
{
	if (p_motor->CONFIG.P_PARAMS_NVM != 0U)
	{
		memcpy(&p_motor->Parameters, p_motor->CONFIG.P_PARAMS_NVM, sizeof(Motor_Params_T));
	}

	StateMachine_Init(&p_motor->StateMachine);
	Motor_InitReboot(p_motor);
}

void Motor_InitReboot(Motor_T * p_motor)
{
	/*
	 * HW Wrappers Init
	 */
	Phase_Init(&p_motor->Phase);
	Phase_Polar_ActivateMode(&p_motor->Phase, p_motor->Parameters.PhasePwmMode);

	if ((p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP) || (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_HALL))
	{
		/*
		 * all sixstep modes and hall foc mode use CaptureTime
		 */
		Encoder_Motor_InitCaptureTime(&p_motor->Encoder);
	}
	else
	{
		Encoder_Motor_InitCaptureCount(&p_motor->Encoder);
		AnalogN_EnqueueConversionOptions(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_OPTION_RESTORE);
	}

	if(p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_HALL)
	{
		Hall_Init(&p_motor->Hall);
	}

	Thermistor_Init(&p_motor->Thermistor);

	/*
	 * SW Structs
	 */
	FOC_Init(&p_motor->Foc);
//	BEMF_Init(&p_motor->Bemf);

	PID_Init(&p_motor->PidSpeed);
	PID_Init(&p_motor->PidIq);
	PID_Init(&p_motor->PidId);
	PID_Init(&p_motor->PidIBus);

	Timer_InitPeriodic(&p_motor->ControlTimer, 	1U);
	Timer_InitPeriodic(&p_motor->MillisTimer, 	1U);
	Timer_InitPeriodic(&p_motor->SpeedTimer, 	1U);

	/*
	 * Initial runtime config settings
	 */
	/*
	 * Run calibration later, default zero to middle adc
	 */
	//scales 4095 to physical units. alternatively use opamp equation
#ifdef CONFIG_MOTOR_CURRENT_SAMPLE_INVERT
	Linear_ADC_Init_Inverted(&p_motor->UnitIa, p_motor->Parameters.IaRefZero_ADCU, p_motor->Parameters.IaRefMax_ADCU, p_motor->Parameters.IRefMax_Amp);
	Linear_ADC_Init_Inverted(&p_motor->UnitIb, p_motor->Parameters.IbRefZero_ADCU, p_motor->Parameters.IbRefMax_ADCU, p_motor->Parameters.IRefMax_Amp);
	Linear_ADC_Init_Inverted(&p_motor->UnitIc, p_motor->Parameters.IcRefZero_ADCU, p_motor->Parameters.IcRefMax_ADCU, p_motor->Parameters.IRefMax_Amp);
#elif defined(CONFIG_MOTOR_CURRENT_SAMPLE_NONINVERT)
	Linear_ADC_Init(&p_motor->UnitIa, p_motor->Parameters.IaRefZero_ADCU, p_motor->Parameters.IaRefMax_ADCU,  p_motor->Parameters.IRefMax_Amp);
	Linear_ADC_Init(&p_motor->UnitIb, p_motor->Parameters.IbRefZero_ADCU, p_motor->Parameters.IbRefMax_ADCU,  p_motor->Parameters.IRefMax_Amp);
	Linear_ADC_Init(&p_motor->UnitIc, p_motor->Parameters.IcRefZero_ADCU, p_motor->Parameters.IcRefMax_ADCU,  p_motor->Parameters.IRefMax_Amp);
#endif

	/*
	 * Index max is 13,107
	 */
	Linear_Ramp_InitMillis(&p_motor->Ramp, 10U, 20000U, 0U, 1000U); /* final value is overwritten, slope is persistent */
	p_motor->RampCmd = 0;
	p_motor->RampIndex = 0;

	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP)
	{

	}
	else
	{
		//can start at 0 speed in foc mode for continuous angle displacements
		Linear_Ramp_InitMillis(&p_motor->OpenLoopRamp, 2000U, 20000U, 0U, 300U);
	}
	p_motor->OpenLoopRampIndex = 0U;

	Motor_SetDirectionForward(p_motor);
	p_motor->UserDirection = p_motor->Direction;

	p_motor->Speed_RPM 				= 0U;
	p_motor->VPwm 					= 0U;
	p_motor->ControlTimerBase 		= 0U;
}

// void Motor_SetControlMode(Motor_T * p_motor, Motor_ControlMode_T mode)
//{
//	switch(mode)
//	{
//		case MOTOR_CONTROL_MODE_OPEN_LOOP:
//			p_motor->ControlMode.OpenLoop = 1U;
//			p_motor->ControlMode.Current = 0U;
//			break;
//
//		case MOTOR_CONTROL_MODE_CONSTANT_VOLTAGE:
//			p_motor->ControlMode.Speed = 0U;
//			p_motor->ControlMode.Current = 0U;
//			break;
//
//	//	MOTOR_CONTROL_MODE_SCALAR_VOLTAGE_FREQ: break;
//
//		case MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE:
//			p_motor->ControlMode.Speed = 1U;
//			p_motor->ControlMode.Current = 0U;
//			break;
//
//		case MOTOR_CONTROL_MODE_CONSTANT_CURRENT:
//			p_motor->ControlMode.Speed = 0U;
//			p_motor->ControlMode.Current = 1U;
//			break;
//
//		case MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT:
//			p_motor->ControlMode.Speed = 1U;
//			p_motor->ControlMode.Current = 1U;
//			break;
//	}
//}






