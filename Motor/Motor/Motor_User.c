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
	@file 	Motor_User.c
	@author FireSoucery
	@brief
	@version V0
*/
/******************************************************************************/
#include "Motor_User.h"

qangle16_t Motor_User_GetMechanicalAngle(Motor_T * p_motor)
{
	qangle16_t angle;

	switch(p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_SENSORLESS: 	angle = 0; 	break;
		case MOTOR_SENSOR_MODE_HALL: 		angle = Encoder_Motor_GetMechanicalTheta(&p_motor->Encoder);	break;
		case MOTOR_SENSOR_MODE_ENCODER: 	angle = Encoder_Motor_GetMechanicalTheta(&p_motor->Encoder);	break;
		case MOTOR_SENSOR_MODE_SIN_COS: 	angle = SinCos_GetMechanicalAngle(&p_motor->SinCos); 			break;
		default: 							angle = 0; 	break;
	}

	return angle;
}

/******************************************************************************/
/*!
	State Machine Error checked inputs
*/
/******************************************************************************/
/*
	Motor State Machine Thread Safety

	SemiSync Mode -
	State Proc in PWM thread. User Input in Main Thread. Need critical section during input.

	Set async to proc, sync issue can recover?,
	control proc may overwrite pid state set, but last to complete is always user input?

	Sync Mode
	Must check input flags every pwm cycle
*/

/*
	Feedback mode update, match Ramp and PID state to output
	Transition to Run/Control State

	Motor_User_Set[Control]ModeCmd must alway check feedback flags,
	addtionall include update/active flag optimize state transition check
*/
void _Motor_User_SetControlMode(Motor_T * p_motor, Motor_FeedbackMode_T mode)
{
	if(Motor_CheckControlUpdate(p_motor, mode) == true)
	{
		Critical_Enter();
		Motor_SetFeedbackModeFlags(p_motor, mode);
		StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_CONTROL);
		Critical_Exit();
	}
}


/*
	Disable control, motor may remain spinning
*/
void Motor_User_DisableControl(Motor_T * p_motor)
{
	Phase_Float(&p_motor->Phase); //move to state machine for field weakening
	StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_FLOAT); /* no critical for transition, only 1 transiition in run state? cannot conflict? */
}

void Motor_User_Ground(Motor_T * p_motor)
{
	Phase_Ground(&p_motor->Phase);
//	StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_GROUND);	//alternatively only ground in stop state
}

/*
	set buffered direction, check on state machine
*/
bool Motor_User_SetDirection(Motor_T * p_motor, Motor_Direction_T direction)
{
	p_motor->UserDirection = direction;
	if(p_motor->Direction != direction) { StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_DIRECTION); }
	//StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_DIRECTION, direction);
	return (p_motor->UserDirection == p_motor->Direction);
}

bool Motor_User_SetDirectionForward(Motor_T * p_motor)
{
	Motor_Direction_T direction = (p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW) ? MOTOR_DIRECTION_CCW : MOTOR_DIRECTION_CW;
	return Motor_User_SetDirection(p_motor, direction);
}

bool Motor_User_SetDirectionReverse(Motor_T * p_motor)
{
	Motor_Direction_T direction = (p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW) ? MOTOR_DIRECTION_CW : MOTOR_DIRECTION_CCW;
	return Motor_User_SetDirection(p_motor, direction);
}

/*
	Calibration State - Run Calibration functions
*/
void Motor_User_ActivateCalibrationHall(Motor_T * p_motor)
{
	p_motor->CalibrationState = MOTOR_CALIBRATION_STATE_HALL;
	StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION);
}

void Motor_User_ActivateCalibrationEncoder(Motor_T * p_motor)
{
	p_motor->CalibrationState = MOTOR_CALIBRATION_STATE_ENCODER;
	StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION);
}

void Motor_User_ActivateCalibrationAdc(Motor_T * p_motor)
{
	p_motor->CalibrationState = MOTOR_CALIBRATION_STATE_ADC;
	StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION);
}

void Motor_User_ActivateCalibrationSinCos(Motor_T * p_motor)
{
	if(p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_SIN_COS)
	{
		//check null SinCos config
		p_motor->CalibrationState = MOTOR_CALIBRATION_STATE_SIN_COS;
		StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION);
	}
}

/******************************************************************************/
/*!
	Limits

	RefMax -> Param -> [Direction] -> Scalar

	RefMax => 5000 RPM
	Parameters.Limit_Frac16 == 32767 => 2500 RPM
	scalar_frac16 == 52428 => Limit[Active]_Frac16 == 26213 => 2000 RPM

	Limit[Active]_Frac16 is applied every SetUserCmd
*/
/******************************************************************************/
/*
	Speed limit always sets most recent input
*/
void Motor_User_SetSpeedLimitActive(Motor_T * p_motor, uint16_t scalar_frac16)
{
	p_motor->SpeedLimitCcw_Frac16 = (uint32_t)scalar_frac16 * p_motor->Parameters.SpeedLimitCcw_Frac16 / 65536U;
	p_motor->SpeedLimitCw_Frac16 = (uint32_t)scalar_frac16 * p_motor->Parameters.SpeedLimitCw_Frac16 / 65536U;
	p_motor->SpeedLimit_Frac16 = (p_motor->Direction == MOTOR_DIRECTION_CCW) ? p_motor->SpeedLimitCcw_Frac16 : p_motor->SpeedLimitCw_Frac16;
}

void Motor_User_ClearSpeedLimitActive(Motor_T * p_motor)
{
	Motor_ResetSpeedLimits(p_motor);
	// p_motor->SpeedLimitActiveScalar = 0xFFFF;
	p_motor->SpeedLimit_Frac16 = (p_motor->Direction == MOTOR_DIRECTION_CCW) ? p_motor->SpeedLimitCcw_Frac16 : p_motor->SpeedLimitCw_Frac16;
}

// bool Motor_User_SetSpeedLimitActive_Id(Motor_T * p_motor, uint16_t scalar_frac16, Motor_SpeedLimitActiveId_T id)
// {
// 	bool isSet = (scalar_frac16 < p_motor->SpeedLimitActiveScalar);
// 	if(isSet == true)
// 	{
// 		_Motor_User_SetSpeedLimitActive(p_motor, scalar_frac16);
// 		p_motor->SpeedLimitActiveId = id;
// 	}
// 	return isSet;
// }

// void Motor_User_ClearSpeedLimitActive_Id(Motor_T * p_motor, Motor_SpeedLimitActiveId_T id)
// {
// 	if(p_motor->SpeedLimitActiveId == id)
// 	{
// 		_Motor_User_ClearSpeedLimitActive(p_motor);
// 		p_motor->SpeedLimitActiveId = MOTOR_SPEED_LIMIT_ACTIVE_DISABLE;
// 	}
// }

/*
	ILimit check lower
	E.g.
	LimitParam = 32768 => LimitActive = 32768
	Scalar1 = 32768 => LimitActive = 16384
	Scalar2 = 6553 => LimitActive = 3276
	Scalar3 = 32768 => LimitActive = 3276
*/
void _Motor_User_SetILimitActive(Motor_T * p_motor, uint16_t scalar_frac16)
{
	p_motor->ILimitActiveScalar = scalar_frac16;
	p_motor->ILimitMotoring_Frac16 = (uint32_t)scalar_frac16 * p_motor->Parameters.ILimitMotoring_Frac16 / 65536U;
	p_motor->ILimitGenerating_Frac16 = (uint32_t)scalar_frac16 * p_motor->Parameters.ILimitGenerating_Frac16 / 65536U;
	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC) { Motor_FOC_ResetSpeedPidILimits(p_motor); }
}

void _Motor_User_ClearILimitActive(Motor_T * p_motor)
{
	p_motor->ILimitActiveScalar = 0xFFFFU;
	p_motor->ILimitMotoring_Frac16 = p_motor->Parameters.ILimitMotoring_Frac16;
	p_motor->ILimitGenerating_Frac16 = p_motor->Parameters.ILimitGenerating_Frac16;
	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC) { Motor_FOC_ResetSpeedPidILimits(p_motor); }
}

//todo may need list
/*! @return true if set */
bool Motor_User_SetILimitActive(Motor_T * p_motor, uint16_t scalar_frac16, Motor_ILimitActiveId_T id)
{
	bool isSet = (scalar_frac16 < p_motor->ILimitActiveScalar);
	if(isSet == true)
	{
		_Motor_User_SetILimitActive(p_motor, scalar_frac16);
		p_motor->ILimitActiveId = id;
	}
	return isSet;
}

/*! @return true if cleared. ILimit of input id */
bool Motor_User_ClearILimitActive(Motor_T * p_motor, Motor_ILimitActiveId_T id)
{
	bool isClear = (p_motor->ILimitActiveId == id);
	if(isClear == true)
	{
		_Motor_User_ClearILimitActive(p_motor);
		p_motor->ILimitActiveId = MOTOR_I_LIMIT_ACTIVE_DISABLE;
	}
	return isClear;
}

Motor_ILimitActiveId_T Motor_User_GetILimitActive(Motor_T * p_motor)
{
	return p_motor->ILimitActiveId;
}

/******************************************************************************/
/*
	Nvm Param Peristent Limits
*/
/******************************************************************************/
/* Persistent SpeedLimit - effective Speed Feedback Mode only */
void Motor_User_SetSpeedLimitParam_Frac16(Motor_T * p_motor, uint16_t forward_Frac16, uint16_t reverse_Frac16)
{
	if(p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW)
	{
		p_motor->Parameters.SpeedLimitCcw_Frac16 = forward_Frac16;
		p_motor->Parameters.SpeedLimitCw_Frac16 = reverse_Frac16;
	}
	else
	{
		p_motor->Parameters.SpeedLimitCcw_Frac16 = reverse_Frac16;
		p_motor->Parameters.SpeedLimitCw_Frac16 = forward_Frac16;
	}
#ifndef CONFIG_MOTOR_PROPOGATE_SET_PARAM_DISABLE
	Motor_User_ClearSpeedLimitActive(p_motor);
#endif
}

/* Persistent ILimit */
void Motor_User_SetILimitParam_Frac16(Motor_T * p_motor, uint16_t motoring_Frac16, uint16_t generating_Frac16)
{
	p_motor->Parameters.ILimitMotoring_Frac16 = motoring_Frac16;
	p_motor->Parameters.ILimitGenerating_Frac16 = generating_Frac16;
#ifndef CONFIG_MOTOR_PROPOGATE_SET_PARAM_DISABLE
	_Motor_User_ClearILimitActive(p_motor);
#endif
}

// #ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL

/* Persistent SpeedLimit - effective speed control mode only */
void Motor_User_SetSpeedLimitParam_Rpm(Motor_T * p_motor, uint16_t forward_Rpm, uint16_t reverse_Rpm)
{
	int32_t forward_Frac16 = Motor_ConvertToSpeedFrac16(p_motor, forward_Rpm);
	int32_t reverse_Frac16 = Motor_ConvertToSpeedFrac16(p_motor, reverse_Rpm);

	forward_Frac16 = (forward_Frac16 > UINT16_MAX) ? UINT16_MAX : forward_Frac16;
	reverse_Frac16 = (reverse_Frac16 > UINT16_MAX) ? UINT16_MAX : reverse_Frac16;

	Motor_User_SetSpeedLimitParam_Frac16(p_motor, forward_Frac16, reverse_Frac16);
}

/* Persistent ILimit */
void Motor_User_SetILimitParam_Amp(Motor_T * p_motor, uint16_t motoring_Amp, uint16_t generating_Amp)
{
	int32_t motoring_Frac16 	= Motor_ConvertToIFrac16(p_motor, motoring_Amp);
	int32_t generating_Frac16 	= Motor_ConvertToIFrac16(p_motor, generating_Amp);

	motoring_Frac16 	= (motoring_Frac16 > UINT16_MAX) ? UINT16_MAX : motoring_Frac16;
	generating_Frac16 	= (generating_Frac16 > UINT16_MAX) ? UINT16_MAX : generating_Frac16;

	Motor_User_SetILimitParam_Frac16(p_motor, motoring_Frac16, generating_Frac16);
}

/******************************************************************************/
/*
	Nvm Reference/Calibration
	Optionally propagate values during set, or wait for reboot
*/
/******************************************************************************/

/* SpeedFeedbackRef_Rpm => speed at VRefSupply */
void Motor_User_SetSpeedFeedbackRef_Rpm(Motor_T * p_motor, uint16_t rpm)
{
	p_motor->Parameters.SpeedFeedbackRef_Rpm = rpm;
#ifndef CONFIG_MOTOR_PROPOGATE_SET_PARAM_DISABLE
	p_motor->Parameters.SpeedLimitCcw_Frac16 = (uint32_t)p_motor->Parameters.SpeedLimitCcw_Frac16 * rpm / p_motor->Parameters.SpeedFeedbackRef_Rpm;
	p_motor->Parameters.SpeedLimitCw_Frac16 = (uint32_t)p_motor->Parameters.SpeedLimitCw_Frac16 * rpm / p_motor->Parameters.SpeedFeedbackRef_Rpm;
	Motor_ResetSpeedVMatchRatio(p_motor);
#endif
}

void Motor_User_SetSpeedFeedbackRef_VRpm(Motor_T * p_motor, uint16_t vMotor_V, uint16_t vMotorSpeed_Rpm)
{
	Motor_User_SetSpeedFeedbackRef_Rpm(p_motor, vMotorSpeed_Rpm * _Motor_GetVSourceRef() / vMotor_V);
}

void Motor_User_SetSpeedVMatchRef_Rpm(Motor_T * p_motor, uint16_t rpm)
{
	uint32_t min = p_motor->Parameters.SpeedFeedbackRef_Rpm * 3 / 4;
	uint32_t max = p_motor->Parameters.SpeedFeedbackRef_Rpm * 5 / 4;

	if		(rpm > max) { p_motor->Parameters.SpeedVMatchRef_Rpm = max; }
	else if	(rpm < min) { p_motor->Parameters.SpeedVMatchRef_Rpm = min; }
	else 				{ p_motor->Parameters.SpeedVMatchRef_Rpm = rpm; };

#ifndef CONFIG_MOTOR_PROPOGATE_SET_PARAM_DISABLE
	Motor_ResetSpeedVMatchRatio(p_motor);
#endif
}

void Motor_User_SetSpeedVMatchRef_VRpm(Motor_T * p_motor, uint16_t vMotor_V, uint16_t vMotorSpeed_Rpm)
{
	Motor_User_SetSpeedVMatchRef_Rpm(p_motor, vMotorSpeed_Rpm * _Motor_GetVSourceRef() / vMotor_V);
}

void Motor_User_SetIaZero_Adcu(Motor_T * p_motor, uint16_t adcu)
{
	p_motor->Parameters.IaZeroRef_Adcu = adcu;
// #ifndef CONFIG_MOTOR_PROPOGATE_SET_PARAM_DISABLE
// 	Motor_ResetUnitsIa(p_motor);
// #endif
}
void Motor_User_SetIbZero_Adcu(Motor_T * p_motor, uint16_t adcu)
{
	p_motor->Parameters.IbZeroRef_Adcu = adcu;
// #ifndef CONFIG_MOTOR_PROPOGATE_SET_PARAM_DISABLE
// 	Motor_ResetUnitsIb(p_motor);
// #endif
}
void Motor_User_SetIcZero_Adcu(Motor_T * p_motor, uint16_t adcu)
{
	p_motor->Parameters.IcZeroRef_Adcu = adcu;
// #ifndef CONFIG_MOTOR_PROPOGATE_SET_PARAM_DISABLE
// 	Motor_ResetUnitsIc(p_motor);
// #endif
}

void Motor_User_SetIaIbIcZero_Adcu(Motor_T * p_motor, uint16_t ia_adcu, uint16_t ib_adcu, uint16_t ic_adcu)
{
	p_motor->Parameters.IaZeroRef_Adcu = ia_adcu;
	p_motor->Parameters.IbZeroRef_Adcu = ib_adcu;
	p_motor->Parameters.IcZeroRef_Adcu = ic_adcu;
#ifndef CONFIG_MOTOR_PROPOGATE_SET_PARAM_DISABLE
	Motor_ResetUnitsIabc(p_motor);
#endif
}

void Motor_User_SetDirectionCalibration(Motor_T * p_motor, Motor_DirectionCalibration_T mode)
{
#ifndef CONFIG_MOTOR_PROPOGATE_SET_PARAM_DISABLE
	uint32_t ccw, cw;
	if(p_motor->Parameters.DirectionCalibration != mode)
	{
		ccw = p_motor->Parameters.SpeedLimitCcw_Frac16;
		cw = p_motor->Parameters.SpeedLimitCw_Frac16;
		p_motor->Parameters.SpeedLimitCcw_Frac16 = cw;
		p_motor->Parameters.SpeedLimitCw_Frac16 = ccw;
	}
#endif
	p_motor->Parameters.DirectionCalibration = mode;
}

void Motor_User_SetPolePairs(Motor_T * p_motor, uint8_t polePairs)
{
	p_motor->Parameters.PolePairs = polePairs;
#ifndef CONFIG_MOTOR_PROPOGATE_SET_PARAM_DISABLE
	if(p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_HALL)
	{
		Motor_ResetUnitsHall(p_motor);
	}
#endif
}

void Motor_User_SetSensorMode(Motor_T * p_motor, Motor_SensorMode_T mode)
{
	p_motor->Parameters.SensorMode = mode;
#ifndef CONFIG_MOTOR_PROPOGATE_SET_PARAM_DISABLE
	Motor_ResetSensorMode(p_motor);
#endif
}

/******************************************************************************/
/*   */
/******************************************************************************/

void Motor_User_SetIPeakRef_Adcu(Motor_T * p_motor, uint16_t adcu)
{
	p_motor->Parameters.IPeakRef_Adcu = (adcu > p_motor->CONFIG.I_ZERO_TO_PEAK_ADCU) ? p_motor->CONFIG.I_ZERO_TO_PEAK_ADCU : adcu;
#ifndef CONFIG_MOTOR_PROPOGATE_SET_PARAM_DISABLE
	Motor_ResetUnitsIabc(p_motor);
#endif
}

void Motor_User_SetIPeakRef_MilliV(Motor_T * p_motor, uint16_t min_MilliV, uint16_t max_MilliV)
{
	uint16_t adcuZero = (uint32_t)(max_MilliV + min_MilliV) * ADC_MAX / 2U / _Motor_GetAdcVRef();
	uint16_t adcuMax = (uint32_t)max_MilliV * ADC_MAX / _Motor_GetAdcVRef();
	Motor_User_SetIPeakRef_Adcu(p_motor, adcuMax - adcuZero);
}