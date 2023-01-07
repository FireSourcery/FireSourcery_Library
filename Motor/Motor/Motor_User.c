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

	CmdValue (Ramp Target) and CmdMode selectively sync inputs for StateMachine
*/

/*
	ControlFeedbackMode update: match Ramp and PID state to output
	Transition to Run State (Active Control)

	proc if (not run state) or (run state and change feedbackmode)
	Motor_User_Set[Control]ModeCmd check feedback flags,
	check control active flag
	Store control active flag as ControlFeedbackMode.IsDisable
*/
void _Motor_User_ActivateControlMode(Motor_T * p_motor, Motor_FeedbackModeId_T mode)
{
	p_motor->CmdFeedbackMode.State = Motor_ConvertFeedbackModeId(mode).State;
	if(p_motor->ControlFeedbackMode.State != Motor_ConvertFeedbackModeId(mode).State)
	// if(p_motor->CmdFeedbackMode.State != Motor_ConvertFeedbackModeId(mode).State)
	{
		Critical_Enter(); /* Block PWM Thread, do not proc new flags before matching output with StateMachine */
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
	Disable control
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

#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
void Motor_User_ActivateCalibrationSinCos(Motor_T * p_motor)
{
	if(p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_SIN_COS)
	{
		StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, MOTOR_CALIBRATION_STATE_SIN_COS);
	}
}
#endif

void Motor_User_ActivateCalibrationSensor(Motor_T * p_motor)
{
	switch(p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_HALL:		StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, MOTOR_CALIBRATION_STATE_HALL); 		break;
		case MOTOR_SENSOR_MODE_ENCODER:		StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, MOTOR_CALIBRATION_STATE_ENCODER);	break;
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
		case MOTOR_SENSOR_MODE_SIN_COS:		StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, MOTOR_CALIBRATION_STATE_SIN_COS);	break;
#endif
		default: break;
	}
}
/******************************************************************************/
/*   */
/******************************************************************************/


/******************************************************************************/
/*!
	Active Limits

	RefMax -> Param -> [Direction] -> Scalar

	RefMax => 5000 RPM
	Parameters.Limit_Frac16 == 32767 => 2500 RPM
	scalar_frac16 == 52428 => Limit[Active]_Frac16 == 26213 => 2000 RPM

	Limit[Active]_Frac16 is applied every SetUserCmd
*/
/******************************************************************************/
static uint16_t ScaleFrac16(uint16_t scalar_frac16, uint16_t base_frac16) { return (uint32_t)scalar_frac16 * (uint32_t)base_frac16 / 65536UL; }

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
// 		Motor_User_SetSpeedLimitActive(p_motor, scalar_frac16);
// 		p_motor->SpeedLimitActiveId = id;
// 	}
// 	return isSet;
// }

// bool Motor_User_ClearSpeedLimitActive_Id(Motor_T * p_motor, Motor_SpeedLimitActiveId_T id)
// {
// 	if(p_motor->SpeedLimitActiveId == id)
// 	{
// 		Motor_User_ClearSpeedLimitActive(p_motor);
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
	Motor_ResetPidILimits(p_motor);
}

void _Motor_User_ClearILimitActive(Motor_T * p_motor)
{
	Motor_ResetILimits(p_motor);
	Motor_ResetPidILimits(p_motor);
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
	Ground Speed
*/
/******************************************************************************/
int16_t Motor_User_GetGroundSpeed_Kmh(Motor_T * p_motor)
{
	int16_t speed;

	switch(p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_HALL: 		speed = Encoder_DeltaD_GetGroundSpeed_Kmh(&p_motor->Encoder);	break;
		case MOTOR_SENSOR_MODE_ENCODER: 	speed = Encoder_DeltaD_GetGroundSpeed_Kmh(&p_motor->Encoder);	break;
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
		case MOTOR_SENSOR_MODE_SIN_COS: 	speed =  Linear_Speed_CalcGroundSpeed(&p_motor->Units, p_motor->Speed_Frac16); break;
#endif
#if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
		case MOTOR_SENSOR_MODE_SENSORLESS: 	speed = 0; 	break;
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
		case MOTOR_SENSOR_MODE_HALL: 		speed = Encoder_DeltaD_GetGroundSpeed_Mph(&p_motor->Encoder);	break;
		case MOTOR_SENSOR_MODE_ENCODER: 	speed = Encoder_DeltaD_GetGroundSpeed_Mph(&p_motor->Encoder);	break;
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
		case MOTOR_SENSOR_MODE_SIN_COS: 	speed =  Linear_Speed_CalcGroundSpeed(&p_motor->Units, p_motor->Speed_Frac16); break;
#endif
#if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
		case MOTOR_SENSOR_MODE_SENSORLESS: 	speed = 0; 	break;
#endif
		default: 							speed = 0; 	break;
	}

	return (p_motor->Direction == MOTOR_DIRECTION_CCW) ? speed : 0 - speed;
}

void Motor_User_SetGroundSpeed_Kmh(Motor_T * p_motor, uint32_t wheelDiameter_Mm, uint32_t wheelToMotorRatio_Factor, uint32_t wheelToMotorRatio_Divisor)
{
	switch(p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_HALL: 		Encoder_SetGroundRatio_Metric(&p_motor->Encoder, wheelDiameter_Mm, wheelToMotorRatio_Factor, wheelToMotorRatio_Divisor);	break;
		case MOTOR_SENSOR_MODE_ENCODER: 	Encoder_SetGroundRatio_Metric(&p_motor->Encoder, wheelDiameter_Mm, wheelToMotorRatio_Factor, wheelToMotorRatio_Divisor);	break;
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
		case MOTOR_SENSOR_MODE_SIN_COS: 	Linear_Speed_CalcGroundSpeed(&p_motor->Units, p_motor->Speed_Frac16); break;
#endif
#if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
		case MOTOR_SENSOR_MODE_SENSORLESS: 	speed = 0; 	break;
#endif
		default: 	break;
	}
}

void Motor_User_SetGroundSpeed_Mph(Motor_T * p_motor, uint32_t wheelDiameter_Inch10, uint32_t wheelToMotorRatio_Factor, uint32_t wheelToMotorRatio_Divisor)
{
	switch(p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_HALL: 		Encoder_SetGroundRatio_US(&p_motor->Encoder, wheelDiameter_Inch10, wheelToMotorRatio_Factor, wheelToMotorRatio_Divisor);	break;
		case MOTOR_SENSOR_MODE_ENCODER: 	Encoder_SetGroundRatio_US(&p_motor->Encoder, wheelDiameter_Inch10, wheelToMotorRatio_Factor, wheelToMotorRatio_Divisor);	break;
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
		case MOTOR_SENSOR_MODE_SIN_COS: 	 Linear_Speed_CalcGroundSpeed(&p_motor->Units, p_motor->Speed_Frac16); break;
#endif
#if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
		case MOTOR_SENSOR_MODE_SENSORLESS: 	speed = 0; 	break;
#endif
		default: 							break;
	}
}


/******************************************************************************/
/*
	Nvm Params
*/
/******************************************************************************/
typedef void(*Motor_PropagateSet_T)(Motor_T * p_motor);

static inline void PropagateSet(Motor_T * p_motor, Motor_PropagateSet_T reset)
{
#ifdef CONFIG_MOTOR_PROPAGATE_SET_PARAM_ENABLE
	reset(p_motor);
#else
	(void)p_motor; (void)reset;
#endif
}

/******************************************************************************/
/*
	Nvm Param Persistent Limits
*/
/******************************************************************************/
/*
	Persistent SpeedLimit - effective Speed Feedback Mode only
*/
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
	PropagateSet(p_motor, Motor_User_ClearSpeedLimitActive);
}

void Motor_User_SetSpeedLimitForwardParam_Frac16(Motor_T * p_motor, uint16_t forward_Frac16)
{
	if(p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW) 	{ p_motor->Parameters.SpeedLimitCcw_Frac16 = forward_Frac16; }
	else 																	{ p_motor->Parameters.SpeedLimitCw_Frac16 = forward_Frac16; }
	PropagateSet(p_motor, Motor_User_ClearSpeedLimitActive);
}

void Motor_User_SetSpeedLimitReverseParam_Frac16(Motor_T * p_motor, uint16_t reverse_Frac16)
{
	if(p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW) 	{ p_motor->Parameters.SpeedLimitCw_Frac16 = reverse_Frac16; }
	else 																	{ p_motor->Parameters.SpeedLimitCcw_Frac16 = reverse_Frac16; }
	PropagateSet(p_motor, Motor_User_ClearSpeedLimitActive);
}

#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
static uint16_t ConvertToSpeedLimitFrac16(Motor_T * p_motor, uint16_t speed_rpm)
{
	int32_t speed_frac16 = _Motor_ConvertToSpeedFrac16(p_motor, speed_rpm);
	return (speed_frac16 > UINT16_MAX) ? UINT16_MAX : speed_frac16;
}

void Motor_User_SetSpeedLimitParam_Rpm(Motor_T * p_motor, uint16_t forward_Rpm, uint16_t reverse_Rpm)
{
	Motor_User_SetSpeedLimitParam_Frac16(p_motor, ConvertToSpeedLimitFrac16(p_motor, forward_Rpm), ConvertToSpeedLimitFrac16(p_motor, reverse_Rpm));
}

void Motor_User_SetSpeedLimitForwardParam_Rpm(Motor_T * p_motor, uint16_t forward_Rpm)
{
	Motor_User_SetSpeedLimitForwardParam_Frac16(p_motor, ConvertToSpeedLimitFrac16(p_motor, forward_Rpm));
}

void Motor_User_SetSpeedLimitReverseParam_Rpm(Motor_T * p_motor, uint16_t reverse_Rpm)
{
	Motor_User_SetSpeedLimitReverseParam_Frac16(p_motor, ConvertToSpeedLimitFrac16(p_motor, reverse_Rpm));
}

uint16_t Motor_User_GetSpeedLimitForwardParam_Rpm(Motor_T * p_motor)
{
	uint16_t speedLimit = (p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW) ?
		p_motor->Parameters.SpeedLimitCcw_Frac16 : p_motor->Parameters.SpeedLimitCw_Frac16;
	return _Motor_ConvertToSpeedRpm(p_motor, speedLimit);
}

uint16_t Motor_User_GetSpeedLimitReverseParam_Rpm(Motor_T * p_motor)
{
	uint16_t speedLimit = (p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW) ?
		p_motor->Parameters.SpeedLimitCw_Frac16 : p_motor->Parameters.SpeedLimitCcw_Frac16;
	return _Motor_ConvertToSpeedRpm(p_motor, speedLimit);
}
#endif

/*
	Persistent ILimit
*/
void Motor_User_SetILimitParam_Frac16(Motor_T * p_motor, uint16_t motoring_Frac16, uint16_t generating_Frac16)
{
	p_motor->Parameters.ILimitMotoring_Frac16 = motoring_Frac16;
	p_motor->Parameters.ILimitGenerating_Frac16 = generating_Frac16;
	PropagateSet(p_motor, _Motor_User_ClearILimitActive);
}

void Motor_User_SetILimitMotoringParam_Frac16(Motor_T * p_motor, uint16_t motoring_Frac16)
{
	p_motor->Parameters.ILimitMotoring_Frac16 = motoring_Frac16;
	PropagateSet(p_motor, _Motor_User_ClearILimitActive);
}

void Motor_User_SetILimitGeneratingParam_Frac16(Motor_T * p_motor, uint16_t generating_Frac16)
{
	p_motor->Parameters.ILimitGenerating_Frac16 = generating_Frac16;
	PropagateSet(p_motor, _Motor_User_ClearILimitActive);
}

#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
static uint16_t ConvertToILimitFrac16(Motor_T * p_motor, uint16_t i_amp)
{
	int32_t i_frac16 = _Motor_ConvertToIFrac16(p_motor, i_amp);
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

uint16_t Motor_User_GetILimitMotoringParam_Amp(Motor_T * p_motor) 		{ return _Motor_ConvertToIAmp(p_motor, p_motor->Parameters.ILimitMotoring_Frac16); }
uint16_t Motor_User_GetILimitGeneratingParam_Amp(Motor_T * p_motor) 	{ return _Motor_ConvertToIAmp(p_motor, p_motor->Parameters.ILimitGenerating_Frac16); }
#endif

/******************************************************************************/
/*
	Nvm Reference/Calibration
	Optionally propagate values during set, or wait for reboot
*/
/******************************************************************************/
/* SpeedFeedbackRef_Rpm => 100% speed for PID feedback. */
void Motor_User_SetSpeedFeedbackRef_Rpm(Motor_T * p_motor, uint16_t rpm)
{
	p_motor->Parameters.SpeedFeedbackRef_Rpm = rpm;
	if(rpm < p_motor->Parameters.SpeedVRef_Rpm) { p_motor->Parameters.SpeedVRef_Rpm = rpm; }
	PropagateSet(p_motor, Motor_ResetUnitsVSpeed);
	PropagateSet(p_motor, Motor_ResetUnitsSensor);
}

/* SpeedVRef =< SetSpeedFeedbackRef to ensure not match to higher speed */
void Motor_User_SetSpeedVRef_Rpm(Motor_T * p_motor, uint16_t rpm)
{
	p_motor->Parameters.SpeedVRef_Rpm = (rpm <= p_motor->Parameters.SpeedFeedbackRef_Rpm) ? rpm : p_motor->Parameters.SpeedFeedbackRef_Rpm;
	PropagateSet(p_motor, Motor_ResetUnitsVSpeed);
	PropagateSet(p_motor, Motor_ResetUnitsSensor);
}

void Motor_User_SetSpeedFeedbackRef_Kv(Motor_T * p_motor, uint16_t kv)
{
	uint16_t kvRpm = kv * Global_Motor_GetVSource_V();
	if((p_motor->Parameters.SpeedFeedbackRef_Rpm == 0U) || kvRpm < p_motor->Parameters.SpeedFeedbackRef_Rpm)
	{
		Motor_User_SetSpeedFeedbackRef_Rpm(p_motor, kvRpm);
	}
}

void Motor_User_SetSpeedVRef_Kv(Motor_T * p_motor, uint16_t kv)
{
	Motor_User_SetSpeedVRef_Rpm(p_motor, kv * Global_Motor_GetVSource_V());
}

/* Setting Kv overwrites higher SpeedFeedbackRef. SpeedFeedbackRef can be set independently from Kv */
void Motor_User_SetKv(Motor_T * p_motor, uint16_t kv)
{
	p_motor->Parameters.Kv = kv;
	Motor_User_SetSpeedFeedbackRef_Kv(p_motor, kv);
	Motor_User_SetSpeedVRef_Kv(p_motor, kv);
}

void Motor_User_SetIaZero_Adcu(Motor_T * p_motor, uint16_t adcu)
{
	p_motor->Parameters.IaZeroRef_Adcu = adcu;
	// PropagateSet(p_motor, Motor_ResetUnitsIa);
}
void Motor_User_SetIbZero_Adcu(Motor_T * p_motor, uint16_t adcu)
{
	p_motor->Parameters.IbZeroRef_Adcu = adcu;
	// PropagateSet(p_motor, Motor_ResetUnitsIb);
}
void Motor_User_SetIcZero_Adcu(Motor_T * p_motor, uint16_t adcu)
{
	p_motor->Parameters.IcZeroRef_Adcu = adcu;
	// PropagateSet(p_motor, Motor_ResetUnitsIc);
}

void Motor_User_SetIaIbIcZero_Adcu(Motor_T * p_motor, uint16_t ia_adcu, uint16_t ib_adcu, uint16_t ic_adcu)
{
	p_motor->Parameters.IaZeroRef_Adcu = ia_adcu;
	p_motor->Parameters.IbZeroRef_Adcu = ib_adcu;
	p_motor->Parameters.IcZeroRef_Adcu = ic_adcu;
	PropagateSet(p_motor, Motor_ResetUnitsIabc);
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
	PropagateSet(p_motor, Motor_SetDirectionForward);
}

void Motor_User_SetPolePairs(Motor_T * p_motor, uint8_t polePairs)
{
	p_motor->Parameters.PolePairs = polePairs;
	PropagateSet(p_motor, Motor_ResetUnitsSensor);
}

/* Reboot unless deinit is implemented in HAL */
void Motor_User_SetSensorMode(Motor_T * p_motor, Motor_SensorMode_T mode)
{
	p_motor->Parameters.SensorMode = mode;
	PropagateSet(p_motor, Motor_InitSensor);
}

/******************************************************************************/
/*   */
/******************************************************************************/
#if defined(CONFIG_MOTOR_DEBUG_ENABLE)
void Motor_User_SetIPeakRef_Adcu_Debug(Motor_T * p_motor, uint16_t adcu)
{
	p_motor->Parameters.IPeakRef_Adcu = adcu;
	Motor_ResetUnitsIabc(p_motor);
}

void Motor_User_SetIPeakRef_Adcu(Motor_T * p_motor, uint16_t adcu)
{
	p_motor->Parameters.IPeakRef_Adcu = (adcu >  GLOBAL_MOTOR.I_MAX_ZTP_ADCU) ?  GLOBAL_MOTOR.I_MAX_ZTP_ADCU : adcu;
	PropagateSet(p_motor, Motor_ResetUnitsIabc);
}

void Motor_User_SetIPeakRef_MilliV(Motor_T * p_motor, uint16_t min_MilliV, uint16_t max_MilliV)
{
	uint16_t adcuZero = (uint32_t)(max_MilliV + min_MilliV) * GLOBAL_ANALOG.ADC_MAX / 2U / GLOBAL_ANALOG.ADC_VREF_MILLIV;
	uint16_t adcuMax = (uint32_t)max_MilliV * GLOBAL_ANALOG.ADC_MAX / GLOBAL_ANALOG.ADC_VREF_MILLIV;
	Motor_User_SetIPeakRef_Adcu(p_motor, adcuMax - adcuZero);
}
#endif