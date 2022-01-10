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
#include "Config.h"
#include "MotorAnalog.h"

#include "Transducer/Phase/Phase.h"
#include "Transducer/Hall/Hall.h"
#include "Transducer/BEMF/BEMF.h"
#include "Math/FOC.h"

#include "Transducer/Encoder/Encoder_Motor.h"

#include "Utility/StateMachine/StateMachine.h"
#include "Utility/Timer/Timer.h"

#include "Math/Linear/Linear_ADC.h"
#include "Math/Linear/Linear_Voltage.h"
#include "Math/Linear/Linear.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

void Motor_Init(Motor_T * p_motor)
{
	if (p_motor->CONFIG.P_PARAMETERS != 0U)
	{
		memcpy(&p_motor->Parameters, p_motor->CONFIG.P_PARAMETERS, sizeof(p_motor->Parameters));
	}

	StateMachine_Init(&p_motor->StateMachine);
	Motor_InitReboot(p_motor);
}

//void Motor_Init_Default(Motor_T * p_motor)
//{
//	if (p_motor->CONFIG.P_PARAMS_DEFAULT != 0U)
//	{
//		memcpy(&p_motor->Parameters, p_motor->CONFIG.P_PARAMS_DEFAULT, sizeof(p_motor->Parameters));
//	}
//
//	StateMachine_Init(&p_motor->StateMachine);
//	Motor_InitReboot(p_motor);
//}

//todo state machine init-state run
void Motor_InitReboot(Motor_T * p_motor)
{
	/*
	 * HW Wrappers Init
	 */
	Phase_Init(&p_motor->Phase);
	Phase_Polar_ActivateMode(&p_motor->Phase, p_motor->Parameters.PhasePwmMode);

	PID_Init(&p_motor->PidSpeed);
	PID_Init(&p_motor->PidIq);
	PID_Init(&p_motor->PidId);
	PID_Init(&p_motor->PidIBus);

	if (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP || p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_HALL)
	{
		/*
		 * all sixstep modes and hall foc mode use CaptureTime
		 * use PolePairs * 6 for count per commutation or PolePairs for count per erotation
		 */
		Encoder_Motor_InitCaptureTime(&p_motor->Encoder, p_motor->Parameters.PolePairs * 6U, p_motor->Parameters.EncoderDistancePerCount, p_motor->Parameters.PolePairs);
		Hall_Init(&p_motor->Hall);
	}
	else if (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_ENCODER)
	{
		Encoder_Motor_InitCaptureCount(&p_motor->Encoder, p_motor->Parameters.EncoderCountsPerRevolution, p_motor->Parameters.EncoderDistancePerCount, p_motor->Parameters.PolePairs);
		Encoder_SetQuadratureMode(&p_motor->Encoder, p_motor->Parameters.EncoderIsQuadratureModeEnabled);
		Encoder_SetQuadratureDirectionCalibration(&p_motor->Encoder, p_motor->Parameters.EncoderIsALeadBPositive);
		FOC_Init(&p_motor->Foc);
	}
	else
	{
		//FOC sensorless
	}

	/*
	 * SW Structs
	 */
	BEMF_Init(&p_motor->Bemf);
//	p_motor->p_BemfConversionActive = &p_motor->CONFIG.CONVERSION_BEMF_A;


	/*
	 * Timer only mode, no function attached
	 */
	Timer_InitPeriodic(&p_motor->ControlTimer, 	1U);
	Timer_InitPeriodic(&p_motor->MillisTimer, 	1U);
//	Timer_InitPeriodic(&p_motor->SecondsTimer, 	1000U);

	/*
	 * Initial runtime config settings
	 */
	/*
	 * Run calibration later, default zero to middle adc
	 */
	Linear_ADC_Init(&p_motor->UnitIa, p_motor->Parameters.IaZero_ADCU, 4095U, p_motor->Parameters.Imax_Amp); //scales 4095 to physical units. alternatively use opamp equation
	Linear_ADC_Init(&p_motor->UnitIb, p_motor->Parameters.IbZero_ADCU, 4095U, p_motor->Parameters.Imax_Amp);
	Linear_ADC_Init(&p_motor->UnitIc, p_motor->Parameters.IcZero_ADCU, 4095U, p_motor->Parameters.Imax_Amp);

	//Linear_Init(&(p_Motor->VFMap), vPerRPM, 1, vOffset); //f(freq) = voltage

	p_motor->Direction 				= MOTOR_DIRECTION_CCW;
//	p_motor->DirectionInput 		= MOTOR_DIRECTION_CCW;
	p_motor->Speed_RPM 				= 0U;
	p_motor->VPwm 					= 0U;
	p_motor->ControlTimerBase 		= 0U;

// 	p_motor->SignalBufferBemfA.AdcFlags 		= 0U;
//	p_motor->SignalBufferBemfB.AdcFlags 		= 0U;
//	p_motor->SignalBufferBemfC.AdcFlags 		= 0U;
//	p_motor->SignalBufferRemainder.AdcFlags 	= 0U;
//	p_motor->SignalBufferFocIabc.AdcFlags 		= 0U;
//	p_motor->SignalBufferFocRemainder.AdcFlags 	= 0U;
//	p_motor->SignalBufferIdle.AdcFlags 			= 0U;

}



/******************************************************************************/
/*
 *	Align State
 */
/******************************************************************************/
Motor_AlignMode_T Motor_GetAlignMode(Motor_T *p_motor)
{
	Motor_AlignMode_T alignMode;

	if (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_HALL)
	{
		alignMode = MOTOR_ALIGN_MODE_DISABLE;
	}
	else
	{
		//		if useHFI alignMode= MOTOR_ALIGN_MODE_HFI;
		//		else
		alignMode = MOTOR_ALIGN_MODE_ALIGN;
	}

	return alignMode;
}

void Motor_StartAlign(Motor_T * p_motor)
{
	uint32_t alignVoltage = (65536U/10U/4U); // + (p_motor->UserCmd / 2U); //= (65536U/10U/4U) + (p_motor->VPwm / 2U);
	Timer_StartPeriod(&p_motor->ControlTimer, 20000U); //Parameter.AlignTime
	Phase_Ground(&p_motor->Phase);
	Phase_ActuateDuty(&p_motor->Phase, alignVoltage, 0, 0);
}

bool Motor_ProcAlign(Motor_T * p_motor)
{
	bool status = Timer_Poll(&p_motor->ControlTimer);

	if(status == true)
	{
		p_motor->ElectricalAngle = 0U;
		Encoder_Reset(&p_motor->Encoder); //zero angularD
//		Motor_Float(&p_motor->Foc);
	}

	return status;
}

/******************************************************************************/
/*
 * 	Calibration State Functions
 * 	@{
 */
/******************************************************************************/
/*
 * Nonblocking Calibration State Functions
 */
void Motor_SetCalibrationStateAdc(Motor_T * p_motor)		{p_motor->CalibrationState = MOTOR_CALIBRATION_STATE_ADC;}
void Motor_SetCalibrationStateHall(Motor_T * p_motor)		{p_motor->CalibrationState = MOTOR_CALIBRATION_STATE_HALL;}
void Motor_SetCalibrationStateEncoder(Motor_T * p_motor)	{p_motor->CalibrationState = MOTOR_CALIBRATION_STATE_ENCODER;}

static void StartMotorCalibrateCommon(Motor_T * p_motor)
{
	p_motor->ControlTimerBase = 0U;
	Phase_Ground(&p_motor->Phase);
}

//void Motor_StartCalibrateAdc(Motor_T * p_motor)
//{
//	StartMotorCalibrateCommon(p_motor);
//	Timer_StartPeriod(&p_motor->ControlTimer, 20000U);//Motor.Parameters.AdcCalibrationTime
//
//}
//
//bool Motor_CalibrateAdc(Motor_T *p_motor)
//{
//
//}

void Motor_StartCalibrateEncoder(Motor_T * p_motor)
{
	StartMotorCalibrateCommon(p_motor);
	Timer_StartPeriod(&p_motor->ControlTimer, 20000U);
	Phase_ActuateDuty(&p_motor->Phase, 6553U/4U, 0, 0); /* Align Phase A 10% pwm */
}

bool Motor_CalibrateEncoder(Motor_T * p_motor)
{
	static uint8_t state = 0; //limits calibration to 1 at a time;
	const uint16_t duty = 65536/10/4;
	bool isComplete = false;

	if (Timer_Poll(&p_motor->ControlTimer) == true)
	{
		switch (state)
		{
			case 0U:
	//			Encoder_DeltaD_CalibrateQuadratureReference(&p_motor->Encoder);
				Phase_ActuateDuty(&p_motor->Phase, 0U, duty, 0U);
				state++;
				break;

			case 1U:
	//			Encoder_DeltaD_CalibrateQuadratureDirectionPositive(&p_motor->Encoder);
				Phase_Float(&p_motor->Phase);
				state = 0;
				isComplete = true;
				break;

			default:
				break;

		}
	}

	return isComplete;
}

void Motor_StartCalibrateHall(Motor_T * p_motor)
{
	StartMotorCalibrateCommon(p_motor);
	Timer_StartPeriod(&p_motor->ControlTimer, 20000U); //Parameter.HallCalibrationTime
}

//120 degree hall aligned with phase
bool Motor_CalibrateHall(Motor_T * p_motor)
{
	static uint8_t state = 0U; //limits calibration to 1 at a time; todo reuse hall sate
	const uint16_t duty = 65536U/10U/4U;
	bool isComplete = false;

	if (Timer_Poll(&p_motor->ControlTimer) == true)
	{
		switch (state)
		{
		case 0U:
			Phase_ActuateDuty(&p_motor->Phase, duty, 0U, 0U);
			state++;
			break;

		case 1U:
			Hall_CalibratePhaseA(&p_motor->Hall);
			Phase_ActuateDuty(&p_motor->Phase, duty, duty, 0U);
			state++;
			break;

		case 2U:
			Hall_CalibratePhaseInvC(&p_motor->Hall);
			Phase_ActuateDuty(&p_motor->Phase, 0U, duty, 0);
			state++;
			break;

		case 3U:
			Hall_CalibratePhaseB(&p_motor->Hall);
			Phase_ActuateDuty(&p_motor->Phase, 0U, duty, duty);
			state++;
			break;

		case 4U:
			Hall_CalibratePhaseInvA(&p_motor->Hall);
			Phase_ActuateDuty(&p_motor->Phase, 0U, 0U, duty);
			state++;
			break;

		case 5U:
			Hall_CalibratePhaseC(&p_motor->Hall);
			Phase_ActuateDuty(&p_motor->Phase, duty, 0U, duty);
			state++;
			break;

		case 6U:
			Hall_CalibratePhaseInvB(&p_motor->Hall);
			Phase_Float(&p_motor->Phase);
			state = 0U;
			isComplete = true;
			break;

		default:
			break;
		}
	}

	return isComplete;
}

/******************************************************************************/
/*! @} */
/******************************************************************************/

