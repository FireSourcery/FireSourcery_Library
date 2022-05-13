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

uint16_t _Motor_AdcVRef_MilliV;  /* Sync with upper layer */
uint16_t _Motor_VRefSupply_V;	/* Battery/Supply voltage. Sync with upper layer */

uint16_t _Motor_GetAdcVRef(void) { return _Motor_AdcVRef_MilliV; }
uint16_t _Motor_GetVRefSupply(void) { return _Motor_VRefSupply_V; }

void Motor_InitAdcVRef_MilliV(uint16_t adcVRef_MilliV) 	{ _Motor_AdcVRef_MilliV = adcVRef_MilliV; }
void Motor_InitVRefSupply_V(uint16_t vRefSupply) 		{ _Motor_VRefSupply_V = vRefSupply; }

void Motor_Init(Motor_T * p_motor)
{
	if(p_motor->CONFIG.P_PARAMS_NVM != 0U)
	{
		memcpy(&p_motor->Parameters, p_motor->CONFIG.P_PARAMS_NVM, sizeof(Motor_Params_T));
	}

	Motor_InitReboot(p_motor);

	StateMachine_Init(&p_motor->StateMachine);
}

void Motor_InitReboot(Motor_T * p_motor)
{
	/*
		HW Wrappers Init
	*/
	Phase_Init(&p_motor->Phase);
	Phase_Polar_ActivateMode(&p_motor->Phase, p_motor->Parameters.PhasePwmMode);

	Motor_ResetSensorMode(p_motor);

	Thermistor_Init(&p_motor->Thermistor);

	/*
		SW Structs
	*/
	Timer_InitPeriodic(&p_motor->ControlTimer, 1U);
	Timer_InitPeriodic(&p_motor->SpeedTimer, 1U);

	FOC_Init(&p_motor->Foc);
	//	BEMF_Init(&p_motor->Bemf);

	PID_Init(&p_motor->PidSpeed);
	PID_Init(&p_motor->PidIq);
	PID_Init(&p_motor->PidId);
	PID_Init(&p_motor->PidIBus);

	//temporary //set assuming forward direction
	// PID_SetOutputLimits(&p_motor->PidSpeed, 0 - p_motor->Parameters.SpeedLimitCw_Frac16, p_motor->Parameters.SpeedLimitCcw_Frac16);
	// PID_SetOutputLimits(&p_motor->PidIq, 0 - iqOutCw, iqOutCcw); /
	// PID_SetOutputLimits(&p_motor->PidId, 0 - speedIOutCcw / 2U, speedIOutCcw / 2U);

	/*
		Ramp 0 to 32767 max in ~500ms
	*/
	Linear_Ramp_InitMillis(&p_motor->Ramp, 500U, 20000U, 0U, 32767U); /* final value is overwritten, slope is persistent */
	Motor_SetRamp(p_motor, 0U);
	p_motor->RampCmd = 0U;
	p_motor->RampIndex = 0U;

	Motor_ResetUnitsVabc(p_motor);
	Motor_ResetUnitsIabc(p_motor);

	Motor_ResetSpeedVMatchRatio(p_motor);
	Motor_ResetSpeedLimits(p_motor);
	Motor_ResetILimits(p_motor);

	Motor_SetDirectionForward(p_motor);

	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
	{
		Linear_Ramp_InitMillis(&p_motor->OpenLoopRamp, 2000U, 20000U, 0U, 300U);	//can start at 0 speed in foc mode for continuous angle displacements
	}
	else
	{

	}
	p_motor->OpenLoopRampIndex = 0U;

	Motor_SetFeedbackModeFlags(p_motor, p_motor->Parameters.FeedbackMode); //set user control mode so pids set to initial state.

	p_motor->ControlTimerBase = 0U;
	p_motor->UserDirection = p_motor->Direction;
	// p_motor->SpeedLimitActiveId = MOTOR_SPEED_LIMIT_ACTIVE_DISABLE;
	p_motor->ILimitActiveId = MOTOR_I_LIMIT_ACTIVE_DISABLE;
}

/******************************************************************************/
/*
	Direction functions - state machine protected
*/
/******************************************************************************/
void Motor_SetDirectionCcw(Motor_T * p_motor)
{
	p_motor->Direction = MOTOR_DIRECTION_CCW;
	p_motor->SpeedLimit_Frac16 = p_motor->SpeedLimitCcw_Frac16;

	switch(p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_OPEN_LOOP: break;
		case MOTOR_SENSOR_MODE_SENSORLESS: break;
		case MOTOR_SENSOR_MODE_ENCODER: break;
		case MOTOR_SENSOR_MODE_HALL: Hall_SetDirection(&p_motor->Hall, HALL_DIRECTION_CCW); break;
		default: break;
	}
}

void Motor_SetDirectionCw(Motor_T * p_motor)
{
	p_motor->Direction = MOTOR_DIRECTION_CW;
	p_motor->SpeedLimit_Frac16 = p_motor->SpeedLimitCw_Frac16;

	switch(p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_OPEN_LOOP: break;
		case MOTOR_SENSOR_MODE_SENSORLESS: break;
		case MOTOR_SENSOR_MODE_ENCODER: break;
		case MOTOR_SENSOR_MODE_HALL: Hall_SetDirection(&p_motor->Hall, HALL_DIRECTION_CW); break;
		default: break;
	}
}

/*
	CCW or CW
*/
void Motor_SetDirection(Motor_T * p_motor, Motor_Direction_T direction)
{
	(direction == MOTOR_DIRECTION_CCW) ? Motor_SetDirectionCcw(p_motor) : Motor_SetDirectionCw(p_motor);
}

/*
	Forward/Reverse using calibration param
*/
void Motor_SetDirectionForward(Motor_T * p_motor)
{
	(p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW) ? Motor_SetDirectionCcw(p_motor) : Motor_SetDirectionCw(p_motor);
}

void Motor_SetDirectionReverse(Motor_T * p_motor)
{
	(p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW) ? Motor_SetDirectionCw(p_motor) : Motor_SetDirectionCcw(p_motor);
}

/******************************************************************************/
/*
	Propage param values
*/
/******************************************************************************/
void Motor_ResetSensorMode(Motor_T * p_motor)
{
	switch(p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_SENSORLESS:
			if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP)
			{
				Encoder_Motor_InitCaptureTime(&p_motor->Encoder);
			}
			else
			{

			}
			break;

		case MOTOR_SENSOR_MODE_HALL:
			Hall_Init(&p_motor->Hall);
			Encoder_Motor_InitCaptureTime(&p_motor->Encoder);
			/* Set Encoder module individual param consistent to main motor module setting */
			Motor_ResetUnitsHall(p_motor);
			break;

		case MOTOR_SENSOR_MODE_ENCODER:
			Encoder_Motor_InitCaptureCount(&p_motor->Encoder);
			Encoder_SetSpeedRef(&p_motor->Encoder, p_motor->Parameters.SpeedFeedbackRef_Rpm);
			break;

		case MOTOR_SENSOR_MODE_SIN_COS:
			SinCos_Init(&p_motor->SinCos);
			// SinCos_SetERotationsPerCycle(&p_motor->SinCos, p_motor->Parameters.PolePairs, 1U);
			Linear_Speed_InitAngleRpm(&p_motor->UnitAngleRpm, 1000U, 16U, p_motor->Parameters.SpeedFeedbackRef_Rpm);
			break;

		default:
			break;
	}
}

void Motor_ResetUnitsVabc(Motor_T * p_motor)
{
#if !defined(CONFIG_MOTOR_V_SENSORS_ISOLATED) && defined(CONFIG_MOTOR_V_SENSORS_ADC)
	Linear_Voltage_Init(&p_motor->UnitVabc, p_motor->CONFIG.UNIT_VABC_R1, p_motor->CONFIG.UNIT_VABC_R2, ADC_BITS, _Motor_AdcVRef_MilliV, _Motor_VRefSupply_V);
#endif
}

void Motor_ResetUnitsIabc(Motor_T * p_motor)
{
#ifdef CONFIG_MOTOR_I_SENSORS_INVERT
	Linear_ADC_Init_Inverted(&p_motor->UnitIa, p_motor->Parameters.IaRefZero_Adcu, p_motor->Parameters.IaRefZero_Adcu + p_motor->Parameters.IRefPeak_Adcu, p_motor->CONFIG.I_MAX_AMP);
	Linear_ADC_Init_Inverted(&p_motor->UnitIb, p_motor->Parameters.IbRefZero_Adcu, p_motor->Parameters.IbRefZero_Adcu + p_motor->Parameters.IRefPeak_Adcu, p_motor->CONFIG.I_MAX_AMP);
	Linear_ADC_Init_Inverted(&p_motor->UnitIc, p_motor->Parameters.IcRefZero_Adcu, p_motor->Parameters.IcRefZero_Adcu + p_motor->Parameters.IRefPeak_Adcu, p_motor->CONFIG.I_MAX_AMP);
#elif defined(CONFIG_MOTOR_I_SENSORS_NONINVERT)
	Linear_ADC_Init(&p_motor->UnitIa, p_motor->Parameters.IaRefZero_Adcu, p_motor->Parameters.IaRefZero_Adcu + p_motor->Parameters.IRefPeak_Adcu, p_motor->CONFIG.I_MAX_AMP);
	Linear_ADC_Init(&p_motor->UnitIb, p_motor->Parameters.IbRefZero_Adcu, p_motor->Parameters.IbRefZero_Adcu + p_motor->Parameters.IRefPeak_Adcu, p_motor->CONFIG.I_MAX_AMP);
	Linear_ADC_Init(&p_motor->UnitIc, p_motor->Parameters.IcRefZero_Adcu, p_motor->Parameters.IcRefZero_Adcu + p_motor->Parameters.IRefPeak_Adcu, p_motor->CONFIG.I_MAX_AMP);
#endif
}

void Motor_ResetUnitsHall(Motor_T * p_motor)
{
	Encoder_Motor_SetHallCountsPerRevolution(&p_motor->Encoder, p_motor->Parameters.PolePairs); //check repeat set here or lower layer?
	Encoder_SetSpeedRef(&p_motor->Encoder, p_motor->Parameters.SpeedFeedbackRef_Rpm);
	Linear_Speed_InitElectricalAngleRpm(&p_motor->UnitAngleRpm, 20000U, 16U, p_motor->Parameters.PolePairs, p_motor->Parameters.SpeedFeedbackRef_Rpm); /* Alt speed calc */
}

void Motor_ResetSpeedVMatchRatio(Motor_T * p_motor)
{
	//check overflow SpeedFeedback_Frac16 = 65536+,  SpeedVMatchRef_Rpm = min
	Linear_Init(&p_motor->SpeedVMatchRatio, p_motor->Parameters.SpeedFeedbackRef_Rpm, p_motor->Parameters.SpeedVMatchRef_Rpm, 0, 65535*2);
}

/******************************************************************************/
/*

*/
/******************************************************************************/
//todo check state machine
void Motor_Jog12Step(Motor_T * p_motor, uint8_t step)
{
	const uint16_t duty = p_motor->Parameters.AlignVoltage_Frac16;
	uint16_t index = step % 12U;
	switch(index)
	{
		case 0U: Phase_Polar_ActivateA(&p_motor->Phase, duty); break;
		case 1U: Phase_Polar_ActivateAC(&p_motor->Phase, duty); break;
		case 2U: Phase_Polar_ActivateInvC(&p_motor->Phase, duty); break;
		case 3U: Phase_Polar_ActivateBC(&p_motor->Phase, duty); break;
		case 4U: Phase_Polar_ActivateB(&p_motor->Phase, duty); break;
		case 5U: Phase_Polar_ActivateBA(&p_motor->Phase, duty); break;
		case 6U: Phase_Polar_ActivateInvA(&p_motor->Phase, duty); break;
		case 7U: Phase_Polar_ActivateCA(&p_motor->Phase, duty); break;
		case 8U: Phase_Polar_ActivateC(&p_motor->Phase, duty); break;
		case 9U: Phase_Polar_ActivateCB(&p_motor->Phase, duty); break;
		case 10U: Phase_Polar_ActivateInvB(&p_motor->Phase, duty); break;
		case 11U: Phase_Polar_ActivateAB(&p_motor->Phase, duty); break;
		default: break;
	}
}

void Motor_Jog6PhaseStep(Motor_T * p_motor, uint8_t step)
{
	const uint16_t duty = p_motor->Parameters.AlignVoltage_Frac16;
	uint16_t index = step % 6U;
	switch(index)
	{
		case 0U: Phase_Polar_ActivateAC(&p_motor->Phase, duty); break;
		case 1U: Phase_Polar_ActivateBC(&p_motor->Phase, duty); break;
		case 2U: Phase_Polar_ActivateBA(&p_motor->Phase, duty); break;
		case 3U: Phase_Polar_ActivateCA(&p_motor->Phase, duty); break;
		case 4U: Phase_Polar_ActivateCB(&p_motor->Phase, duty); break;
		case 5U: Phase_Polar_ActivateAB(&p_motor->Phase, duty); break;
		default: break;
	}
}

/*
	Perfered for stability
*/
void Motor_Jog6Step(Motor_T * p_motor, uint8_t step)
{
	const uint16_t duty = p_motor->Parameters.AlignVoltage_Frac16;
	uint16_t index = step % 6U;
	switch(index)
	{
		case 0U: Phase_Polar_ActivateA(&p_motor->Phase, duty); break;
		case 1U: Phase_Polar_ActivateInvC(&p_motor->Phase, duty); break;
		case 2U: Phase_Polar_ActivateB(&p_motor->Phase, duty); break;
		case 3U: Phase_Polar_ActivateInvA(&p_motor->Phase, duty); break;
		case 4U: Phase_Polar_ActivateC(&p_motor->Phase, duty); break;
		case 5U: Phase_Polar_ActivateInvB(&p_motor->Phase, duty); break;
		default: break;
	}
}

void Motor_Jog6(Motor_T * p_motor)
{
	Motor_Jog6Step(p_motor, p_motor->JogIndex);
	p_motor->JogIndex++;
}

void Motor_Jog12(Motor_T * p_motor)
{
	Motor_Jog12Step(p_motor, p_motor->JogIndex);
	p_motor->JogIndex++;
}