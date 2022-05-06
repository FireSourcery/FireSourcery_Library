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

uint16_t _Motor_GetAdcVRef(void) { return _Motor_AdcVRef_MilliV; }

void Motor_InitReference(uint16_t adcVRef_MilliV) { _Motor_AdcVRef_MilliV = adcVRef_MilliV; }

void Motor_Init(Motor_T * p_motor)
{
	if(p_motor->CONFIG.P_PARAMS_NVM != 0U)
	{
		memcpy(&p_motor->Parameters, p_motor->CONFIG.P_PARAMS_NVM, sizeof(Motor_Params_T));
	}

	StateMachine_Init(&p_motor->StateMachine);
	Motor_InitReboot(p_motor);
}

void Motor_InitReboot(Motor_T * p_motor)
{
	/*
		HW Wrappers Init
	*/
	Phase_Init(&p_motor->Phase);
	Phase_Polar_ActivateMode(&p_motor->Phase, p_motor->Parameters.PhasePwmMode);

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
			Encoder_Motor_SetHallCountsPerRevolution(&p_motor->Encoder, p_motor->Parameters.PolePairs);
			Encoder_SetSpeedRef(&p_motor->Encoder, p_motor->Parameters.SpeedRefMax_RPM);
			Linear_Speed_InitElectricalAngleRpm(&p_motor->UnitAngleRpm, 20000U, 16U, p_motor->Parameters.PolePairs, p_motor->Parameters.SpeedRefMax_RPM);
			break;

		case MOTOR_SENSOR_MODE_ENCODER:
			Encoder_Motor_InitCaptureCount(&p_motor->Encoder);
			Encoder_SetSpeedRef(&p_motor->Encoder, p_motor->Parameters.SpeedRefMax_RPM);
			break;

		case MOTOR_SENSOR_MODE_SIN_COS:
			SinCos_Init(&p_motor->SinCos);
			Linear_Speed_InitAngleRpm(&p_motor->UnitAngleRpm, 1000U, 16U, p_motor->Parameters.SpeedRefMax_RPM);
			break;

		default:
			break;
	}

	Thermistor_Init(&p_motor->Thermistor);
	p_motor->AnalogResults.Heat_ADCU = p_motor->Thermistor.Params.Threshold_ADCU;

	/*
		SW Structs
	*/
	Timer_InitPeriodic(&p_motor->ControlTimer, 1U);
	Timer_InitPeriodic(&p_motor->MillisTimer, 1U);
	Timer_InitPeriodic(&p_motor->SpeedTimer, 1U);

	FOC_Init(&p_motor->Foc);
	//	BEMF_Init(&p_motor->Bemf);

	PID_Init(&p_motor->PidSpeed);
	PID_Init(&p_motor->PidIq);
	PID_Init(&p_motor->PidId);
	PID_Init(&p_motor->PidIBus);
 
#if !defined(CONFIG_MOTOR_V_SENSORS_ISOLATED) && defined(CONFIG_MOTOR_V_SENSORS_ADC)
	Linear_Voltage_Init(&p_motor->UnitVabc, p_motor->CONFIG.UNIT_VABC_R1, p_motor->CONFIG.UNIT_VABC_R2, ADC_BITS, _Motor_AdcVRef_MilliV, p_motor->Parameters.VSupply);
#endif

#ifdef CONFIG_MOTOR_I_SENSORS_INVERT
	Linear_ADC_Init_Inverted(&p_motor->UnitIa, p_motor->Parameters.IaRefZero_ADCU, p_motor->Parameters.IaRefMax_ADCU, p_motor->Parameters.IRefMax_Amp);
	Linear_ADC_Init_Inverted(&p_motor->UnitIb, p_motor->Parameters.IbRefZero_ADCU, p_motor->Parameters.IbRefMax_ADCU, p_motor->Parameters.IRefMax_Amp);
	Linear_ADC_Init_Inverted(&p_motor->UnitIc, p_motor->Parameters.IcRefZero_ADCU, p_motor->Parameters.IcRefMax_ADCU, p_motor->Parameters.IRefMax_Amp);
#elif defined(CONFIG_MOTOR_I_SENSORS_NONINVERT)
	Linear_ADC_Init(&p_motor->UnitIa, p_motor->Parameters.IaRefZero_ADCU, p_motor->Parameters.IaRefMax_ADCU, p_motor->Parameters.IRefMax_Amp);
	Linear_ADC_Init(&p_motor->UnitIb, p_motor->Parameters.IbRefZero_ADCU, p_motor->Parameters.IbRefMax_ADCU, p_motor->Parameters.IRefMax_Amp);
	Linear_ADC_Init(&p_motor->UnitIc, p_motor->Parameters.IcRefZero_ADCU, p_motor->Parameters.IcRefMax_ADCU, p_motor->Parameters.IRefMax_Amp);
#endif

	/*
		Ramp 0 to 65535 max in ~500ms
	*/
	Linear_Ramp_InitMillis(&p_motor->Ramp, 500U, 20000U, 0U, 65535U); /* final value is overwritten, slope is persistent */
	Motor_SetRamp(p_motor, 0U);
	p_motor->RampCmd = 0U;
	p_motor->RampIndex = 0U;

	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
	{
		Linear_Ramp_InitMillis(&p_motor->OpenLoopRamp, 2000U, 20000U, 0U, 300U);	//can start at 0 speed in foc mode for continuous angle displacements
	}
	else
	{

	} 
	p_motor->OpenLoopRampIndex = 0U;

	Motor_ResetSpeedLimits(p_motor); 
	Motor_SetDirectionForward(p_motor); 

	p_motor->UserDirection = p_motor->Direction; 
	p_motor->ControlTimerBase = 0U;
} 

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
 

