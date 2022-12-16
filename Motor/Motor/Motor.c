/******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSourcery / The Firebrand Forge Inc

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
	@author FireSourcery
	@brief  Motor module conventional function definitions.
	@version V0
*/
/******************************************************************************/
#include "Motor.h"
#include <string.h>

void Motor_Init(Motor_T * p_motor)
{
	if(p_motor->CONFIG.P_PARAMS_NVM != 0U) { memcpy(&p_motor->Parameters, p_motor->CONFIG.P_PARAMS_NVM, sizeof(Motor_Params_T)); }
	Motor_InitReboot(p_motor);
	StateMachine_Init(&p_motor->StateMachine);
}

void Motor_InitReboot(Motor_T * p_motor)
{
	/*
		HW Wrappers Init
	*/
	Motor_ResetSensorMode(p_motor);
	Phase_Init(&p_motor->Phase);
	Phase_Polar_ActivateMode(&p_motor->Phase, p_motor->Parameters.PhasePwmMode);
	Thermistor_Init(&p_motor->Thermistor);

	/*
		SW Structs
	*/
	Timer_InitPeriodic(&p_motor->ControlTimer, 1U);
	Timer_InitPeriodic(&p_motor->SpeedTimer, 1U);

	FOC_Init(&p_motor->Foc);
	//	BEMF_Init(&p_motor->Bemf);

	/* Output Limits Set later depending on commutation mode, feedback mode, direction */
	PID_Init(&p_motor->PidSpeed);
	PID_Init(&p_motor->PidIq);
	PID_Init(&p_motor->PidId);
#if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
	PID_Init(&p_motor->PidIBus);
#endif
	Motor_SetDirectionForward(p_motor);

	/*
		Ramp 0 to 32767 max in ~500ms, 3.2767 per control cycle
		Final value is overwritten, slope is persistent
	*/
	Linear_Ramp_Init_Millis(&p_motor->Ramp, GLOBAL_MOTOR.CONTROL_FREQ, 500U, 0U, 32767U);
	Motor_ResetRamp(p_motor);

	Motor_ResetUnitsVabc(p_motor);
	Motor_ResetUnitsIabc(p_motor);

	Motor_ResetSpeedVMatchRatio(p_motor);
	Motor_ResetSpeedLimits(p_motor);
	Motor_ResetILimits(p_motor);
	p_motor->ILimitActiveId = MOTOR_I_LIMIT_ACTIVE_DISABLE;

	Linear_Frac16_Init_Map
	(
		&p_motor->ILimitHeatRate,
		p_motor->Thermistor.Params.Shutdown_Adcu, 	p_motor->Thermistor.Params.Warning_Adcu,
		p_motor->Parameters.ILimitHeat_Frac16, 		0U /* Param not used */
	);

#if defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
	{
		/* todo frac16 */ /* Start at 0 speed in foc mode for continuous angle displacements */
		Linear_Ramp_Init_Millis(&p_motor->OpenLoopRamp, GLOBAL_MOTOR.CONTROL_FREQ, 3000U, 0U, p_motor->Parameters.OpenLoopSpeed_RPM);
	}
	else
	{

	}

	p_motor->OpenLoopRampIndex = 0U;
#endif

	Motor_SetFeedbackModeFlags(p_motor, p_motor->Parameters.UserFeedbackMode); // set user control mode so pids set to initial state.
	p_motor->ControlTimerBase = 0U;
}

/******************************************************************************/
/*
	Reset Sensors/Align
*/
/******************************************************************************/
/* From Stop and after Align */
void Motor_ZeroSensor(Motor_T * p_motor)
{
	p_motor->ElectricalAngle = 0U;
	switch(p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_HALL:
			Motor_SetPositionFeedback(p_motor, 1U); /* move to init? */
			Encoder_DeltaT_SetInitial(&p_motor->Encoder); /* Set first capture DeltaT = 0xffff */
			Hall_ResetCapture(&p_motor->Hall);
			break;
		case MOTOR_SENSOR_MODE_ENCODER:
			Motor_SetPositionFeedback(p_motor, 1U);
			Encoder_DeltaD_SetInitial(&p_motor->Encoder);
			break;
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
		case MOTOR_SENSOR_MODE_SIN_COS:		break;
#endif
#if defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
		case MOTOR_SENSOR_MODE_OPEN_LOOP:
			Motor_SetPositionFeedback(p_motor, 0U);
		break;
#endif
#if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
		case MOTOR_SENSOR_MODE_SENSORLESS:
			Motor_SetPositionFeedback(p_motor, 0U);
		break;
#endif
		default:
			break;
	}
}

/******************************************************************************/
/*
	Direction functions - Include in StateMachine protected calling function
*/
/******************************************************************************/
void Motor_SetDirectionCcw(Motor_T * p_motor)
{
	p_motor->Direction = MOTOR_DIRECTION_CCW;
	p_motor->SpeedLimit_Frac16 = p_motor->SpeedLimitCcw_Frac16;

	switch(p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_HALL: 		Hall_SetDirection(&p_motor->Hall, HALL_DIRECTION_CCW); break;
		case MOTOR_SENSOR_MODE_ENCODER: 	break;
		case MOTOR_SENSOR_MODE_OPEN_LOOP: 	break;
		case MOTOR_SENSOR_MODE_SENSORLESS: 	break;
		default: break;
	}
}

void Motor_SetDirectionCw(Motor_T * p_motor)
{
	p_motor->Direction = MOTOR_DIRECTION_CW;
	p_motor->SpeedLimit_Frac16 = p_motor->SpeedLimitCw_Frac16;

	switch(p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_HALL: 		Hall_SetDirection(&p_motor->Hall, HALL_DIRECTION_CW); break;
		case MOTOR_SENSOR_MODE_ENCODER: 	break;
		case MOTOR_SENSOR_MODE_SENSORLESS: 	break;
		case MOTOR_SENSOR_MODE_OPEN_LOOP: 	break;
		default: break;
	}
}

/*
	CCW or CW
*/
void Motor_SetDirection(Motor_T * p_motor, Motor_Direction_T direction)
{
	if(direction == MOTOR_DIRECTION_CCW) 	{ Motor_SetDirectionCcw(p_motor); }
	else 									{ Motor_SetDirectionCw(p_motor); }
}

/*
	Forward/Reverse using calibration param
*/
void Motor_SetDirectionForward(Motor_T * p_motor)
{
	if(p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW) 	{ Motor_SetDirectionCcw(p_motor); }
	else 																	{ Motor_SetDirectionCw(p_motor); }
}

void Motor_SetDirectionReverse(Motor_T * p_motor)
{
	if(p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW) 	{ Motor_SetDirectionCw(p_motor); }
	else 																	{ Motor_SetDirectionCcw(p_motor); }
}

/******************************************************************************/
/*
	Propagate param values
*/
/******************************************************************************/
void Motor_ResetSensorMode(Motor_T * p_motor)
{
	switch(p_motor->Parameters.SensorMode)
	{

		case MOTOR_SENSOR_MODE_HALL:
			p_motor->CONFIG.INIT_SENSOR_HALL();
			Hall_Init(&p_motor->Hall);
			Encoder_Motor_InitModeT(&p_motor->Encoder);
			Motor_ResetUnitsHallPolePairs(p_motor);
			Motor_ResetUnitsEncoder(p_motor); //testing
			Motor_ResetUnitsAngleSpeed_ElecControl(p_motor);
			break;
		case MOTOR_SENSOR_MODE_ENCODER:
			p_motor->CONFIG.INIT_SENSOR_ENCODER();
			Encoder_Motor_InitModeD(&p_motor->Encoder);
			Motor_ResetUnitsEncoder(p_motor);
			break;
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
		case MOTOR_SENSOR_MODE_SIN_COS:
			SinCos_Init(&p_motor->SinCos);
			Motor_ResetUnitsSinCos(&p_motor->SinCos);
			Motor_ResetUnitsAngleSpeed_Mech(&p_motor);
			break;
#endif
#if defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
		case MOTOR_SENSOR_MODE_OPEN_LOOP:
			Motor_ResetUnitsAngleSpeed_ElecControl(p_motor); /* fix */
			p_motor->Parameters.UserFeedbackMode = MOTOR_FEEDBACK_MODE_OPEN_LOOP;
			break;
#endif
#if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
		case MOTOR_SENSOR_MODE_SENSORLESS:
			Motor_ResetUnitsAngleSpeed_ElecControl(p_motor);
			break;
#endif
		default:
			break;
	}
}

void Motor_ResetUnitsVabc(Motor_T * p_motor)
{
#if defined(CONFIG_MOTOR_V_SENSORS_ANALOG)
	Linear_Voltage_Init(&p_motor->UnitVabc, GLOBAL_MOTOR.VABC_R1, GLOBAL_MOTOR.VABC_R2, ADC_BITS, Global_Motor_GetAdcVRef(), Global_Motor_GetVSourceRef());
#else
	(void)p_motor;
#endif
}

void Motor_ResetUnitsIabc(Motor_T * p_motor)
{
#if defined(CONFIG_MOTOR_DEBUG_ENABLE)
	uint16_t iPeakRef_Adcu = p_motor->Parameters.IPeakRef_Adcu;
#elif defined(CONFIG_MOTOR_DEBUG_DISABLE)
	uint16_t iPeakRef_Adcu =  GLOBAL_MOTOR.I_ZERO_TO_PEAK_ADCU;
#endif

#ifdef CONFIG_MOTOR_I_SENSORS_INVERT
	Linear_ADC_Init_Inverted(&p_motor->UnitIa, p_motor->Parameters.IaZeroRef_Adcu, p_motor->Parameters.IaZeroRef_Adcu + iPeakRef_Adcu,  GLOBAL_MOTOR.I_MAX_AMP);
	Linear_ADC_Init_Inverted(&p_motor->UnitIb, p_motor->Parameters.IbZeroRef_Adcu, p_motor->Parameters.IbZeroRef_Adcu + iPeakRef_Adcu,  GLOBAL_MOTOR.I_MAX_AMP);
	Linear_ADC_Init_Inverted(&p_motor->UnitIc, p_motor->Parameters.IcZeroRef_Adcu, p_motor->Parameters.IcZeroRef_Adcu + iPeakRef_Adcu,  GLOBAL_MOTOR.I_MAX_AMP);
#elif defined(CONFIG_MOTOR_I_SENSORS_NONINVERT)
	Linear_ADC_Init(&p_motor->UnitIa, p_motor->Parameters.IaZeroRef_Adcu, p_motor->Parameters.IaZeroRef_Adcu + iPeakRef_Adcu, p_motor->CONFIG.I_MAX_AMP);
	Linear_ADC_Init(&p_motor->UnitIb, p_motor->Parameters.IbZeroRef_Adcu, p_motor->Parameters.IbZeroRef_Adcu + iPeakRef_Adcu, p_motor->CONFIG.I_MAX_AMP);
	Linear_ADC_Init(&p_motor->UnitIc, p_motor->Parameters.IcZeroRef_Adcu, p_motor->Parameters.IcZeroRef_Adcu + iPeakRef_Adcu, p_motor->CONFIG.I_MAX_AMP);
#endif
}

void Motor_ResetSpeedVMatchRatio(Motor_T * p_motor)
{
	//todo xref
	// Linear_Init(&p_motor->SpeedVMatchRatio, p_motor->Parameters.SpeedFeedbackRef_Rpm, p_motor->Parameters.SpeedVMatchRef_Rpm, 0, 65535*2);
}

/* Sensorless */
void Motor_ResetUnitsAngleSpeed_ElecControl(Motor_T * p_motor)
{
	Linear_Speed_InitElectricalAngleRpm(&p_motor->UnitAngleRpm, GLOBAL_MOTOR.CONTROL_FREQ, 16U, p_motor->Parameters.PolePairs, p_motor->Parameters.SpeedFeedbackRef_Rpm); /* Alt speed calc */
}

/* SinCos, Mechanical Rotation Sensor */
void Motor_ResetUnitsAngleSpeed_Mech(Motor_T * p_motor)
{
	Linear_Speed_InitAngleRpm(&p_motor->UnitAngleRpm, 1000U, 16U, p_motor->Parameters.SpeedFeedbackRef_Rpm);
}

/*
	Reset Sensors, propagate Motor Params to sensor module independent params,
	Sync to main motor module setting
*/
void Motor_ResetUnitsHallPolePairs(Motor_T * p_motor)
{
	if((p_motor->Parameters.PolePairs != p_motor->Encoder.Params.MotorPolePairs) || (p_motor->Parameters.PolePairs * 6U != p_motor->Encoder.Params.CountsPerRevolution))
	{
		Encoder_Motor_SetHallCountsPerRevolution(&p_motor->Encoder, p_motor->Parameters.PolePairs);
	}
}

// void Motor_ResetUnitsEncoderPolePairs(Motor_T * p_motor)
// {
// 	if(p_motor->Parameters.PolePairs != p_motor->Encoder.Params.MotorPolePairs)
// 	{
// 		Encoder_Motor_SetPolePairs(&p_motor->Encoder, p_motor->Parameters.PolePairs);
// 	}
// }

/* Common, Set after PolePairs */
void Motor_ResetUnitsEncoder(Motor_T * p_motor)
{
	if(p_motor->Parameters.PolePairs != p_motor->Encoder.Params.MotorPolePairs)
	{
		Encoder_Motor_SetPolePairs(&p_motor->Encoder, p_motor->Parameters.PolePairs);
	}
	if(p_motor->Parameters.SpeedFeedbackRef_Rpm != p_motor->Encoder.Params.ScalarSpeedRef_Rpm)
	{
		Encoder_SetScalarSpeedRef(&p_motor->Encoder, p_motor->Parameters.SpeedFeedbackRef_Rpm);
	}
	// if(p_motor->Parameters.GearRatio_Factor != p_motor->Encoder.Params.GearRatio_Factor)
	// {
	// 	// Encoder_Motor_SetSurfaceRatio(&p_motor->Encoder, p_motor->Parameters.GearRatio);
	// }
}

#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
void Motor_ResetUnitsSinCos(Motor_T * p_motor)
{
	if(p_motor->Parameters.PolePairs != p_motor->SinCos.Params.ElectricalRotationsRatio)
	{
		SinCos_SetAngleRatio(&p_motor->SinCos, p_motor->Parameters.PolePairs);
	}
}
#endif


/******************************************************************************/
/*

*/
/******************************************************************************/
//todo check state machine
void Motor_Jog12Step(Motor_T * p_motor, uint8_t step)
{
	const uint16_t duty = p_motor->Parameters.AlignVPwm_Frac16;
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
	const uint16_t duty = p_motor->Parameters.AlignVPwm_Frac16;
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
	Preferred for stability
*/
void Motor_Jog6Step(Motor_T * p_motor, uint8_t step)
{
	const uint16_t duty = p_motor->Parameters.AlignVPwm_Frac16;
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

void Motor_Jog6Phase(Motor_T * p_motor)
{
	Motor_Jog6PhaseStep(p_motor, p_motor->JogIndex);
	p_motor->JogIndex++;
}

void Motor_Jog12(Motor_T * p_motor)
{
	Motor_Jog12Step(p_motor, p_motor->JogIndex);
	p_motor->JogIndex++;
}