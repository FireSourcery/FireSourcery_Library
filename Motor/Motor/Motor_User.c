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
	@file 	Motor_User.c
	@author FireSourcery
	@brief
	@version V0
*/
/******************************************************************************/
#include "Motor_User.h"
#include "System/Critical/Critical.h"

/******************************************************************************/
/*!
	State Machine Error checked inputs
*/
/******************************************************************************/
/*
	Motor State Machine Thread Safety

	SemiSync Mode -
	State Proc in PWM thread. User Input in Main Thread. Need critical section during input.
	No Critical during transistion -> Prev State Proc may run before Entry.

	Sync Mode
	Must check input flags every pwm cycle

	Set async to proc, sync issue can recover?
	control proc may overwrite pid state set, but last to complete is always user input?
*/

/*
	FeedbackMode update: match Ramp and PID state to output
	Transition to Run State (Active Control)

	proc if (not run state) or (run state and change feedbackmode)
	Motor_User_Set[Control]ModeCmd check feedback flags,
	check control active flag
	Store control active flag as FeedbackModeFlags.Update
*/
static inline bool Motor_CheckActivateControl(Motor_T * p_motor, Motor_FeedbackMode_T mode)
{
	// return (((Motor_CheckFeedbackModeFlags(p_motor, mode) == false) && (Motor_CheckPositionFeedback(p_motor) == true)) || p_motor->FeedbackModeFlags.Update);
	return (Motor_CheckFeedbackModeFlags(p_motor, mode) == false);
}


void _Motor_User_ActivateControl(Motor_T * p_motor, Motor_FeedbackMode_T mode)
{
	if(Motor_CheckActivateControl(p_motor, mode) == true)
	{
		Critical_Enter(); /* Block PWM Thread, do not proc new flags before matching output with StateMachine */
		Motor_SetFeedbackModeFlags(p_motor, mode); /* Matching output occurs in StateMachine Proc, depends on State */
		StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_CONTROL, mode);
		Critical_Exit();
	}
}

/*
	always State machine checked version
*/
void Motor_User_ReleaseControl(Motor_T * p_motor)
{
	StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_RELEASE, STATE_MACHINE_INPUT_VALUE_NULL); /* no critical for transition, only 1 transistion in run state? cannot conflict? */
}

/*
	Disable control, motor may remain spinning
*/
void Motor_User_DisableControl(Motor_T * p_motor)
{
	Phase_Float(&p_motor->Phase);
	StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_RELEASE, STATE_MACHINE_INPUT_VALUE_NULL); /* no critical for transition, only 1 transistion in run state? cannot conflict? */
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
	if(p_motor->Direction != direction) { StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_DIRECTION, direction); }
	return (direction == p_motor->Direction);
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

	Check SensorMode on input to determine start, or proc may run before checking during entry
	alternatively, implement critical, move check into state machine, share 1 active function this way.
*/
void Motor_User_ActivateCalibrationAdc(Motor_T * p_motor)
{
	StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, MOTOR_CALIBRATION_STATE_ADC);
}

void Motor_User_ActivateCalibrationHall(Motor_T * p_motor)
{
	if(p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_HALL)
	{
		StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, MOTOR_CALIBRATION_STATE_HALL);
	}
}

void Motor_User_ActivateCalibrationEncoder(Motor_T * p_motor)
{
	if(p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_ENCODER)
	{
		StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, MOTOR_CALIBRATION_STATE_ENCODER);
	}
}

void Motor_User_ActivateCalibrationSinCos(Motor_T * p_motor)
{
	if(p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_SIN_COS)
	{
		StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, MOTOR_CALIBRATION_STATE_SIN_COS);
	}
}

void Motor_User_ActivateCalibrationSensor(Motor_T * p_motor)
{
	switch(p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_HALL:		StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, MOTOR_CALIBRATION_STATE_HALL); 		break;
		case MOTOR_SENSOR_MODE_ENCODER:		StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, MOTOR_CALIBRATION_STATE_ENCODER);	break;
		case MOTOR_SENSOR_MODE_SIN_COS:		StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, MOTOR_CALIBRATION_STATE_SIN_COS);	break;
		default: break;
	}
}

/******************************************************************************/
/*   */
/******************************************************************************/
qangle16_t Motor_User_GetMechanicalAngle(Motor_T * p_motor)
{
	qangle16_t angle;

	switch(p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_SENSORLESS: 	angle = 0; 	break;
		case MOTOR_SENSOR_MODE_HALL: 		angle = Encoder_Motor_GetMechanicalTheta(&p_motor->Encoder);	break;
		case MOTOR_SENSOR_MODE_ENCODER: 	angle = Encoder_Motor_GetMechanicalTheta(&p_motor->Encoder);	break;
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
		case MOTOR_SENSOR_MODE_SIN_COS: 	angle = SinCos_GetMechanicalAngle(&p_motor->SinCos); 			break;
#endif
		default: 							angle = 0; 	break;
	}

	return angle;
}

int16_t Motor_User_GetGroundSpeed_Kmh(Motor_T * p_motor)
{
	int16_t speed;

	switch(p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_SENSORLESS: 	speed = 0; 	break;
		case MOTOR_SENSOR_MODE_HALL: 		speed = Encoder_DeltaT_GetGroundSpeed_Kmh(&p_motor->Encoder);	break;
		case MOTOR_SENSOR_MODE_ENCODER: 	speed = Encoder_DeltaD_GetGroundSpeed_Kmh(&p_motor->Encoder);	break;
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
		case MOTOR_SENSOR_MODE_SIN_COS: 	speed =  Linear_Speed_CalcGroundSpeed(&p_motor->Units, p_motor->SpeedFeedback_Frac16); break;
#endif
		default: 							speed = 0; 	break;
	}

	return (p_motor->Direction == MOTOR_DIRECTION_CCW) ? speed : 0 - speed;
}

int16_t Motor_User_GetGroundSpeed_Mph(Motor_T * p_motor)
{
	int16_t speed;

	switch(p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_SENSORLESS: 	speed = 0; 	break;
		case MOTOR_SENSOR_MODE_HALL: 		speed = Encoder_DeltaT_GetGroundSpeed_Mph(&p_motor->Encoder);	break;
		case MOTOR_SENSOR_MODE_ENCODER: 	speed = Encoder_DeltaD_GetGroundSpeed_Mph(&p_motor->Encoder);	break;
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
		case MOTOR_SENSOR_MODE_SIN_COS: 	speed =  Linear_Speed_CalcGroundSpeed(&p_motor->Units, p_motor->SpeedFeedback_Frac16); break;
#endif
		default: 							speed = 0; 	break;
	}

	return (p_motor->Direction == MOTOR_DIRECTION_CCW) ? speed : 0 - speed;
}

void Motor_User_SetGroundSpeed_Kmh(Motor_T * p_motor, uint32_t wheelDiameter_Mm, uint32_t wheelToMotorRatio_Factor, uint32_t wheelToMotorRatio_Divisor)
{
	switch(p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_SENSORLESS: 	break;
		case MOTOR_SENSOR_MODE_HALL: 		Encoder_Motor_SetGroundRatio_Metric(&p_motor->Encoder, wheelDiameter_Mm, wheelToMotorRatio_Factor, wheelToMotorRatio_Divisor);	break;
		case MOTOR_SENSOR_MODE_ENCODER: 	Encoder_Motor_SetGroundRatio_Metric(&p_motor->Encoder, wheelDiameter_Mm, wheelToMotorRatio_Factor, wheelToMotorRatio_Divisor);	break;
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
		case MOTOR_SENSOR_MODE_SIN_COS: 	Linear_Speed_CalcGroundSpeed(&p_motor->Units, p_motor->SpeedFeedback_Frac16); break;
#endif
		default: 	break;
	}
}

void Motor_User_SetGroundSpeed_Mph(Motor_T * p_motor, uint32_t wheelDiameter_Inch10, uint32_t wheelToMotorRatio_Factor, uint32_t wheelToMotorRatio_Divisor)
{
	int16_t speed;

	switch(p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_SENSORLESS: 	break;
		case MOTOR_SENSOR_MODE_HALL: 		Encoder_Motor_SetGroundRatio_US(&p_motor->Encoder, wheelDiameter_Inch10, wheelToMotorRatio_Factor, wheelToMotorRatio_Divisor);	break;
		case MOTOR_SENSOR_MODE_ENCODER: 	Encoder_Motor_SetGroundRatio_US(&p_motor->Encoder, wheelDiameter_Inch10, wheelToMotorRatio_Factor, wheelToMotorRatio_Divisor);	break;
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
		case MOTOR_SENSOR_MODE_SIN_COS: 	 Linear_Speed_CalcGroundSpeed(&p_motor->Units, p_motor->SpeedFeedback_Frac16); break;
#endif
		default: 							break;
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
static uint16_t ScaleFrac16(uint16_t scalar_frac16, uint16_t base_frac16)
{
	return (uint32_t)scalar_frac16 * (uint32_t)base_frac16 / 65536UL;
}

/*
	Speed limit always sets most recent input
*/
void Motor_User_SetSpeedLimitActive(Motor_T * p_motor, uint16_t scalar_frac16)
{
	p_motor->SpeedLimitCcw_Frac16 	= ScaleFrac16(scalar_frac16, p_motor->Parameters.SpeedLimitCcw_Frac16);
	p_motor->SpeedLimitCw_Frac16 	= ScaleFrac16(scalar_frac16, p_motor->Parameters.SpeedLimitCw_Frac16);
	p_motor->SpeedLimit_Frac16 		= (p_motor->Direction == MOTOR_DIRECTION_CCW) ? p_motor->SpeedLimitCcw_Frac16 : p_motor->SpeedLimitCw_Frac16;
}

void Motor_User_ClearSpeedLimitActive(Motor_T * p_motor)
{
	Motor_ResetSpeedLimits(p_motor);
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
	ILimit
*/
/*
	Unchecked
*/
void _Motor_User_SetILimitActive(Motor_T * p_motor, uint16_t scalar_frac16)
{
	p_motor->ILimitActiveSentinel = scalar_frac16;
	p_motor->ILimitMotoring_Frac16 = ScaleFrac16(scalar_frac16, p_motor->Parameters.ILimitMotoring_Frac16);
	p_motor->ILimitGenerating_Frac16 = ScaleFrac16(scalar_frac16, p_motor->Parameters.ILimitGenerating_Frac16);
	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC) { Motor_FOC_ResetSpeedPidILimits(p_motor); }
}

void _Motor_User_ClearILimitActive(Motor_T * p_motor)
{
	Motor_ResetILimits(p_motor);
	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC) { Motor_FOC_ResetSpeedPidILimits(p_motor); }
}

/*
	Check Lower
	E.g.
	LimitParam = 32768 => LimitActive = 32768
	Scalar1 = 32768 => LimitActive = 16384
	Scalar2 = 6553 => LimitActive = 3276
	Scalar3 = 32768 => LimitActive = 3276

	need list to restore previous limit
*/
/*! @return true if set */
bool Motor_User_SetILimitActive(Motor_T * p_motor, uint16_t scalar_frac16, Motor_ILimitActiveId_T id)
{
	bool isSet = (scalar_frac16 < p_motor->ILimitActiveSentinel);
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

Motor_ILimitActiveId_T Motor_User_GetILimitActiveId(Motor_T * p_motor) { return p_motor->ILimitActiveId; }

/******************************************************************************/
/*
	Nvm Param Peristent Limits
*/
/******************************************************************************/
/*
	Persistent SpeedLimit - effective Speed Feedback Mode only
*/
static inline void PropagateSpeedLimitActive(Motor_T * p_motor)
{
#ifdef CONFIG_MOTOR_PROPAGATE_SET_PARAM_ENABLE
	Motor_User_ClearSpeedLimitActive(p_motor);
#else
	(void)p_motor;
#endif
}

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
	PropagateSpeedLimitActive(p_motor);
}

void Motor_User_SetSpeedLimitForwardParam_Frac16(Motor_T * p_motor, uint16_t forward_Frac16)
{
	if(p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW) 	{ p_motor->Parameters.SpeedLimitCcw_Frac16 = forward_Frac16; }
	else 																	{ p_motor->Parameters.SpeedLimitCw_Frac16 = forward_Frac16; }
	PropagateSpeedLimitActive(p_motor);
}

void Motor_User_SetSpeedLimitReverseParam_Frac16(Motor_T * p_motor, uint16_t reverse_Frac16)
{
	if(p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW) 	{ p_motor->Parameters.SpeedLimitCw_Frac16 = reverse_Frac16; }
	else 																	{ p_motor->Parameters.SpeedLimitCcw_Frac16 = reverse_Frac16; }
	PropagateSpeedLimitActive(p_motor);
}

#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
static uint16_t ConvertToSpeedLimitFrac16(Motor_T * p_motor, uint16_t speed_rpm)
{
	int32_t speed_frac16 = _Motor_User_ConvertToSpeedFrac16(p_motor, speed_rpm);
	return (speed_frac16 > UINT16_MAX) ? UINT16_MAX : speed_frac16;
}

void Motor_User_SetSpeedLimitParam_Rpm(Motor_T * p_motor, uint16_t forward_Rpm, uint16_t reverse_Rpm)
{
	Motor_User_SetSpeedLimitParam_Frac16(p_motor, ConvertToSpeedLimitFrac16(p_motor, forward_Rpm), ConvertToSpeedLimitFrac16(p_motor, reverse_Rpm));
}

void Motor_User_SetSpeedLimitForwardParam_Amp(Motor_T * p_motor, uint16_t forward_Rpm)
{
	Motor_User_SetSpeedLimitForwardParam_Frac16(p_motor, ConvertToSpeedLimitFrac16(p_motor, forward_Rpm));
}

void Motor_User_SetSpeedLimitReverseParam_Amp(Motor_T * p_motor, uint16_t reverse_Rpm)
{
	Motor_User_SetSpeedLimitReverseParam_Frac16(p_motor, ConvertToSpeedLimitFrac16(p_motor, reverse_Rpm));
}

uint16_t Motor_User_GetSpeedLimitForwardParam_Amp(Motor_T * p_motor)
{
	uint16_t speedLimit = (p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW) ? p_motor->Parameters.SpeedLimitCcw_Frac16 : p_motor->Parameters.SpeedLimitCw_Frac16;
	return _Motor_User_ConvertToSpeedRpm(p_motor, speedLimit);
}
uint16_t Motor_User_GetSpeedLimitReverseParam_Amp(Motor_T * p_motor)
{
	uint16_t speedLimit = (p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW) ? p_motor->Parameters.SpeedLimitCw_Frac16 : p_motor->Parameters.SpeedLimitCcw_Frac16;
	return _Motor_User_ConvertToSpeedRpm(p_motor, speedLimit);
}
#endif

/*
	Persistent ILimit
*/
static inline void PropagateILimitActive(Motor_T * p_motor)
{
#ifdef CONFIG_MOTOR_PROPAGATE_SET_PARAM_ENABLE
	_Motor_User_ClearILimitActive(p_motor);
#else
	(void)p_motor;
#endif
}

void Motor_User_SetILimitParam_Frac16(Motor_T * p_motor, uint16_t motoring_Frac16, uint16_t generating_Frac16)
{
	p_motor->Parameters.ILimitMotoring_Frac16 = motoring_Frac16;
	p_motor->Parameters.ILimitGenerating_Frac16 = generating_Frac16;
	PropagateILimitActive(p_motor);
}

void Motor_User_SetILimitMotoringParam_Frac16(Motor_T * p_motor, uint16_t motoring_Frac16)
{
	p_motor->Parameters.ILimitMotoring_Frac16 = motoring_Frac16;
	PropagateILimitActive(p_motor);
}

void Motor_User_SetILimitGeneratingParam_Frac16(Motor_T * p_motor, uint16_t generating_Frac16)
{
	p_motor->Parameters.ILimitGenerating_Frac16 = generating_Frac16;
	PropagateILimitActive(p_motor);
}

#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
static uint16_t ConvertToILimitFrac16(Motor_T * p_motor, uint16_t i_amp)
{
	int32_t i_frac16 = _Motor_User_ConvertToIFrac16(p_motor, i_amp);
	return (i_frac16 > UINT16_MAX) ? UINT16_MAX : i_frac16;
}

void Motor_User_SetILimitParam_Amp(Motor_T * p_motor, uint16_t motoring_Amp, uint16_t generating_Amp)
{
	Motor_User_SetILimitParam_Frac16(p_motor, ConvertToILimitFrac16(p_motor, motoring_Amp), ConvertToILimitFrac16(p_motor, generating_Amp));
}

void Motor_User_SetILimitMotoringParam_Amp(Motor_T * p_motor, uint16_t motoring_Amp)
{
	Motor_User_SetILimitMotoringParam_Frac16(p_motor, ConvertToILimitFrac16(p_motor, motoring_Amp));
}

void Motor_User_SetILimitGeneratingParam_Amp(Motor_T * p_motor, uint16_t generating_Amp)
{
	Motor_User_SetILimitGeneratingParam_Frac16(p_motor, ConvertToILimitFrac16(p_motor, generating_Amp));
}

uint16_t Motor_User_GetILimitMotoringParam_Amp(Motor_T * p_motor) 		{ return _Motor_User_ConvertToIAmp(p_motor, p_motor->Parameters.ILimitMotoring_Frac16); }
uint16_t Motor_User_GetILimitGeneratingParam_Amp(Motor_T * p_motor) 	{ return _Motor_User_ConvertToIAmp(p_motor, p_motor->Parameters.ILimitGenerating_Frac16); }
#endif

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
#ifdef CONFIG_MOTOR_PROPAGATE_SET_PARAM_ENABLE
	Motor_ResetSpeedVMatchRatio(p_motor);
	if((p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_HALL) || (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_ENCODER))
	{
		Motor_ResetUnitsEncoder(p_motor);
	}
	/* todo non encoder */
#endif
}

void Motor_User_SetSpeedFeedbackRef_VRpm(Motor_T * p_motor, uint16_t vMotor_V, uint16_t vMotorSpeed_Rpm)
{
	Motor_User_SetSpeedFeedbackRef_Rpm(p_motor, vMotorSpeed_Rpm * Global_Motor_GetVSourceRef() / vMotor_V);
}

void Motor_User_SetSpeedVMatchRef_Rpm(Motor_T * p_motor, uint16_t rpm)
{
	uint32_t min = p_motor->Parameters.SpeedFeedbackRef_Rpm * 3 / 4;
	uint32_t max = p_motor->Parameters.SpeedFeedbackRef_Rpm * 5 / 4;

	if		(rpm > max) { p_motor->Parameters.SpeedVMatchRef_Rpm = max; }
	else if	(rpm < min) { p_motor->Parameters.SpeedVMatchRef_Rpm = min; }
	else 				{ p_motor->Parameters.SpeedVMatchRef_Rpm = rpm; };

#ifdef CONFIG_MOTOR_PROPAGATE_SET_PARAM_ENABLE
	Motor_ResetSpeedVMatchRatio(p_motor);
#endif
}

void Motor_User_SetSpeedVMatchRef_VRpm(Motor_T * p_motor, uint16_t vMotor_V, uint16_t vMotorSpeed_Rpm)
{
	Motor_User_SetSpeedVMatchRef_Rpm(p_motor, vMotorSpeed_Rpm * Global_Motor_GetVSourceRef() / vMotor_V);
}

// void Motor_User_SetIaZero_Adcu(Motor_T * p_motor, uint16_t adcu)
// {
// 	p_motor->Parameters.IaZeroRef_Adcu = adcu;
// // #ifdef CONFIG_MOTOR_PROPAGATE_SET_PARAM_ENABLE
// // 	Motor_ResetUnitsIa(p_motor);
// // #endif
// }
// void Motor_User_SetIbZero_Adcu(Motor_T * p_motor, uint16_t adcu)
// {
// 	p_motor->Parameters.IbZeroRef_Adcu = adcu;
// // #ifdef CONFIG_MOTOR_PROPAGATE_SET_PARAM_ENABLE
// // 	Motor_ResetUnitsIb(p_motor);
// // #endif
// }
// void Motor_User_SetIcZero_Adcu(Motor_T * p_motor, uint16_t adcu)
// {
// 	p_motor->Parameters.IcZeroRef_Adcu = adcu;
// // #ifdef CONFIG_MOTOR_PROPAGATE_SET_PARAM_ENABLE
// // 	Motor_ResetUnitsIc(p_motor);
// // #endif
// }

void Motor_User_SetIaIbIcZero_Adcu(Motor_T * p_motor, uint16_t ia_adcu, uint16_t ib_adcu, uint16_t ic_adcu)
{
	p_motor->Parameters.IaZeroRef_Adcu = ia_adcu;
	p_motor->Parameters.IbZeroRef_Adcu = ib_adcu;
	p_motor->Parameters.IcZeroRef_Adcu = ic_adcu;
#ifdef CONFIG_MOTOR_PROPAGATE_SET_PARAM_ENABLE
	Motor_ResetUnitsIabc(p_motor);
#endif
}

void Motor_User_SetDirectionCalibration(Motor_T * p_motor, Motor_DirectionCalibration_T mode)
{
#ifdef CONFIG_MOTOR_PROPAGATE_SET_PARAM_ENABLE
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
#ifdef CONFIG_MOTOR_PROPAGATE_SET_PARAM_ENABLE
	if(p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_HALL) { Motor_ResetUnitsHallPolePairs(p_motor); }
#endif
}

void Motor_User_SetSensorMode(Motor_T * p_motor, Motor_SensorMode_T mode)
{
	p_motor->Parameters.SensorMode = mode;
#ifdef CONFIG_MOTOR_PROPAGATE_SET_PARAM_ENABLE
	Motor_ResetSensorMode(p_motor);
#endif
}

/******************************************************************************/
/*   */
/******************************************************************************/

#if 	defined(CONFIG_MOTOR_DEBUG_ENABLE)
void Motor_User_SetIPeakRef_Adcu_Debug(Motor_T * p_motor, uint16_t adcu)
{
	p_motor->Parameters.IPeakRef_Adcu = adcu;
#ifdef CONFIG_MOTOR_PROPAGATE_SET_PARAM_ENABLE
	Motor_ResetUnitsIabc(p_motor);
#endif
}

void Motor_User_SetIPeakRef_Adcu(Motor_T * p_motor, uint16_t adcu)
{
	p_motor->Parameters.IPeakRef_Adcu = (adcu >  GLOBAL_MOTOR.I_ZERO_TO_PEAK_ADCU) ?  GLOBAL_MOTOR.I_ZERO_TO_PEAK_ADCU : adcu;
#ifdef CONFIG_MOTOR_PROPAGATE_SET_PARAM_ENABLE
	Motor_ResetUnitsIabc(p_motor);
#endif
}

void Motor_User_SetIPeakRef_MilliV(Motor_T * p_motor, uint16_t min_MilliV, uint16_t max_MilliV)
{
	uint16_t adcuZero = (uint32_t)(max_MilliV + min_MilliV) * ADC_MAX / 2U / Global_Motor_GetAdcVRef();
	uint16_t adcuMax = (uint32_t)max_MilliV * ADC_MAX / Global_Motor_GetAdcVRef();
	Motor_User_SetIPeakRef_Adcu(p_motor, adcuMax - adcuZero);
}
#endif