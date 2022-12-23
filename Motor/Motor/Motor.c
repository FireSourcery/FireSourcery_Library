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

/*
	Init Global_Motor first
*/
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
	Motor_InitSensor(p_motor);
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
		Ramp 0 to 32767 max in ~500ms, 3.2767 per ControlCycle
		Final value is overwritten, Slope is persistent
	*/
	Linear_Ramp_Init_Ticks(&p_motor->Ramp, _Motor_ConvertToControlCycles(p_motor, p_motor->Parameters.RampAccel_Ms), 0U, INT16_MAX);

	Motor_ResetUnitsVabc(p_motor);
	Motor_ResetUnitsIabc(p_motor);

	Motor_ResetSpeedVMatchRatio(p_motor);
	Motor_ResetSpeedLimits(p_motor);
	Motor_ResetILimits(p_motor);
	p_motor->ILimitActiveId = MOTOR_I_LIMIT_ACTIVE_DISABLE;

#if defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
	{
		/* todo frac16 */ /* Start at 0 speed in FOC mode for continuous angle displacements */
		Linear_Ramp_Init_Ticks(&p_motor->OpenLoopRamp, _Motor_ConvertToControlCycles(p_motor, p_motor->Parameters.OpenLoopAccel_Ms), 0U, p_motor->Parameters.OpenLoopSpeed_RPM);
	}
	else
	{

	}
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
			Encoder_ModeDT_SetInitial(&p_motor->Encoder);
			Hall_SetInitial(&p_motor->Hall);
			break;
		case MOTOR_SENSOR_MODE_ENCODER:
			Motor_SetPositionFeedback(p_motor, 1U);
			Encoder_ModeDT_SetInitial(&p_motor->Encoder);
			break;
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
		case MOTOR_SENSOR_MODE_SIN_COS:
		break;
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

qangle16_t Motor_GetMechanicalAngle(Motor_T * p_motor)
{
	qangle16_t angle;
	switch(p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_SENSORLESS: 	angle = 0; 	break;
		// case MOTOR_SENSOR_MODE_HALL: 		angle = Encoder_Motor_GetMechanicalTheta(&p_motor->Encoder);	break;
		// case MOTOR_SENSOR_MODE_ENCODER: 	angle = Encoder_Motor_GetMechanicalTheta(&p_motor->Encoder);	break;
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
		case MOTOR_SENSOR_MODE_SIN_COS: 	angle = SinCos_GetMechanicalAngle(&p_motor->SinCos); 			break;
#endif
		default: 							angle = 0; 	break;
	}
	return angle;
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
void Motor_InitSensor(Motor_T * p_motor)
{
	switch(p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_HALL:
			p_motor->CONFIG.INIT_SENSOR_HALL();
			Hall_Init(&p_motor->Hall);
			// Encoder_Motor_InitModeT(&p_motor->Encoder);
			Encoder_ModeDT_Init(&p_motor->Encoder);
			Motor_ResetUnitsHallEncoder(p_motor);
			Motor_ResetUnitsEncoder(p_motor); //testing
			Motor_ResetUnitsAngleSpeed_ElecControl(p_motor);
			break;
		case MOTOR_SENSOR_MODE_ENCODER:
			p_motor->CONFIG.INIT_SENSOR_ENCODER();
			// Encoder_Motor_InitModeD(&p_motor->Encoder);
			Encoder_ModeDT_Init(&p_motor->Encoder);
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
	Linear_Voltage_Init(&p_motor->UnitVabc, GLOBAL_MOTOR.V_ABC_R1, GLOBAL_MOTOR.V_ABC_R2, GLOBAL_ANALOG.ADC_BITS, GLOBAL_ANALOG.ADC_VREF_MILLIV, Global_Motor_GetVSource_V());
#else
	(void)p_motor;
#endif
}

void Motor_ResetUnitsIabc(Motor_T * p_motor)
{
#if defined(CONFIG_MOTOR_DEBUG_ENABLE)
	uint16_t iPeakRef_Adcu = p_motor->Parameters.IPeakRef_Adcu;
#elif defined(CONFIG_MOTOR_DEBUG_DISABLE)
	uint16_t iPeakRef_Adcu =  GLOBAL_MOTOR.I_MAX_ZTP_ADCU;
#endif
	Linear_ADC_Init_ZeroToPeak(&p_motor->UnitIa, p_motor->Parameters.IaZeroRef_Adcu, iPeakRef_Adcu, 0, GLOBAL_MOTOR.I_UNITS_AMPS);
	Linear_ADC_Init_ZeroToPeak(&p_motor->UnitIb, p_motor->Parameters.IbZeroRef_Adcu, iPeakRef_Adcu, 0, GLOBAL_MOTOR.I_UNITS_AMPS);
	Linear_ADC_Init_ZeroToPeak(&p_motor->UnitIc, p_motor->Parameters.IcZeroRef_Adcu, iPeakRef_Adcu, 0, GLOBAL_MOTOR.I_UNITS_AMPS);
#ifdef CONFIG_MOTOR_I_SENSORS_INVERT
	Linear_ADC_SetInverted(&p_motor->UnitIa);
	Linear_ADC_SetInverted(&p_motor->UnitIb);
	Linear_ADC_SetInverted(&p_motor->UnitIc);
#endif
}

/* resume from freewheel */
void Motor_ResetSpeedVMatchRatio(Motor_T * p_motor)
{
	//todo xref
	Linear_Init(&p_motor->SpeedVMatchRatio, p_motor->Parameters.SpeedFeedbackRef_Rpm, p_motor->Parameters.SpeedVMatchRef_Rpm, 0, 65535*2);
}

/* Sensorless */
void Motor_ResetUnitsAngleSpeed_ElecControl(Motor_T * p_motor)
{
	Linear_Speed_InitElectricalAngleRpm(&p_motor->UnitAngleRpm, GLOBAL_MOTOR.CONTROL_FREQ, 16U, p_motor->Parameters.PolePairs, p_motor->Parameters.SpeedFeedbackRef_Rpm);
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
void Motor_ResetUnitsHallEncoder(Motor_T * p_motor)
{
	if((p_motor->Parameters.PolePairs * 6U != p_motor->Encoder.Params.CountsPerRevolution) || (p_motor->Parameters.PolePairs != p_motor->Encoder.Params.InterpolateAngleScalar))
	{
		Encoder_ModeDT_SetCountsPerRevolution(&p_motor->Encoder, p_motor->Parameters.PolePairs * 6U);
		Encoder_DeltaT_SetInterpolateAngleScalar(&p_motor->Encoder, p_motor->Parameters.PolePairs);
	}
	p_motor->Encoder.Params.IsQuadratureCaptureEnabled = false;
}


/* Common, Set after PolePairs */
void Motor_ResetUnitsEncoder(Motor_T * p_motor)
{
	if(p_motor->Parameters.SpeedFeedbackRef_Rpm != p_motor->Encoder.Params.ScalarSpeedRef_Rpm)
	{
		Encoder_ModeDT_SetScalarSpeedRef(&p_motor->Encoder, p_motor->Parameters.SpeedFeedbackRef_Rpm);
	}
	// if(p_motor->Parameters.GearRatio_Factor != p_motor->Encoder.Params.GearRatio_Factor) ||
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