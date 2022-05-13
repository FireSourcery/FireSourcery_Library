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

uint16_t Motor_User_GetMechanicalAngle(Motor_T * p_motor)
{
	uint16_t angle;

	switch(p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_SENSORLESS: 	angle = 0; 	break;
		case MOTOR_SENSOR_MODE_HALL: 		angle = Encoder_Motor_GetMechanicalTheta(&p_motor->Encoder);	break;
		case MOTOR_SENSOR_MODE_ENCODER: 	angle = Encoder_Motor_GetMechanicalTheta(&p_motor->Encoder);	break;
		case MOTOR_SENSOR_MODE_SIN_COS: 	angle = SinCos_GetMechanicalAngle(&p_motor->SinCos); 			break;
		default: 	break;
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
	State Proc in PWM thread. User Input in Main Thread. May need critical section during input.

	Set async to proc, sync issue can recover?,
	control proc may overwrite pid state set, but last to complete is always user input

	Sync Mode
	Must check input flags every pwm cycle
*/

/*
	Change Feedback mode, match Ramp and PID state to output, depending on freewheel or run state
*/
void _Motor_User_SetFeedbackMode(Motor_T * p_motor, Motor_FeedbackMode_T mode)
{
	if(Motor_CheckFeedbackMode(p_motor, mode) == true)
	{
		Critical_Enter();

		Motor_SetFeedbackModeFlags(p_motor, mode);
		/* Match ouput state, accounting for FeedbackMode and State */
		StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_CONTROL_MODE); // FEEDBACK_MODE + RUN_MODE shared
		p_motor->FeedbackModeFlags.Update = 0U;

		Critical_Exit();
	}
}

/*
	Disable control, motor may remain spinning
*/
void Motor_User_DisableControl(Motor_T * p_motor)
{
	Phase_Float(&p_motor->Phase);
	StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_FLOAT);
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

	if(p_motor->Direction != direction)
	{
		StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_DIRECTION);
		//StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_DIRECTION, direction);
	}

	return (p_motor->UserDirection == p_motor->Direction);
}

bool Motor_User_SetDirectionForward(Motor_T * p_motor)
{
	Motor_Direction_T direction = (p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW) ?
		MOTOR_DIRECTION_CCW : MOTOR_DIRECTION_CW;

	return Motor_User_SetDirection(p_motor, direction);
}

bool Motor_User_SetDirectionReverse(Motor_T * p_motor)
{
	Motor_Direction_T direction = (p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW) ?
		MOTOR_DIRECTION_CW : MOTOR_DIRECTION_CCW;

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
bool _Motor_User_SetILimitActive(Motor_T * p_motor, uint16_t scalar_frac16)
{
	/* if scalar active is < previously set scalar, only need to check one direction */
	// bool isSet = ((uint32_t)scalar_frac16 * p_motor->Parameters.ILimitMotoring_Frac16 / 65536U) < p_motor->ILimitMotoring_Frac16;
	bool isSet = scalar_frac16 < p_motor->ILimitActiveScalar;

	if(isSet == true)
	{
		p_motor->ILimitActiveScalar = scalar_frac16;
		p_motor->ILimitMotoring_Frac16 = (uint32_t)scalar_frac16 * p_motor->Parameters.ILimitMotoring_Frac16 / 65536U;
		p_motor->ILimitGenerating_Frac16 = (uint32_t)scalar_frac16 * p_motor->Parameters.ILimitGenerating_Frac16 / 65536U;

		if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC) { Motor_FOC_ResetOutputLimits(p_motor); }
	}

	return isSet;
}

void _Motor_User_ClearILimitActive(Motor_T * p_motor)
{
	Motor_ResetILimits(p_motor);
	p_motor->ILimitActiveScalar = 0xFFFF;
	p_motor->ILimitActiveId = MOTOR_I_LIMIT_ACTIVE_DISABLE;
	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC) { Motor_FOC_ResetOutputLimits(p_motor); }
}

bool Motor_User_SetILimitActive(Motor_T * p_motor, uint16_t scalar_frac16, Motor_ILimitActiveId_T id)
{
	bool isSet = _Motor_User_SetILimitActive(p_motor, scalar_frac16);
	if(isSet == true) { p_motor->ILimitActiveId = id; }
	return isSet;
}

void Motor_User_ClearILimitActive(Motor_T * p_motor, Motor_ILimitActiveId_T id)
{
	if(p_motor->ILimitActiveId == id)
	{
		_Motor_User_ClearILimitActive(p_motor);
		p_motor->ILimitActiveId = MOTOR_I_LIMIT_ACTIVE_DISABLE;
	}
}

/******************************************************************************/
/*
	Nvm Param Peristent Limits
*/
/******************************************************************************/
/* Persistent SpeedLimit - effective speed control mode only */
void Motor_User_SetSpeedLimitParam_Rpm(Motor_T * p_motor, uint16_t forward_Rpm, uint16_t reverse_Rpm)
{
	uint32_t forward_Frac16 = Motor_ConvertToSpeedFrac16(p_motor, forward_Rpm);
	uint32_t reverse_Frac16 = Motor_ConvertToSpeedFrac16(p_motor, reverse_Rpm);

	if(forward_Frac16 > 65535U) { forward_Frac16 = 65535U; }
	if(reverse_Frac16 > 65535U) { reverse_Frac16 = 65535U; }

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
void Motor_User_SetILimitParam_Amp(Motor_T * p_motor, uint16_t motoring_Amp, uint16_t generating_Amp)
{
	uint32_t limitMotoring_Frac16 = Motor_ConvertToIFrac16(p_motor, motoring_Amp);
	uint32_t limitGenerating_Frac16 = Motor_ConvertToIFrac16(p_motor, generating_Amp);

	if(limitMotoring_Frac16 > 65535U) { limitMotoring_Frac16 = 65535U; }
	if(limitGenerating_Frac16 > 65535U) { limitGenerating_Frac16 = 65535U; }

	p_motor->Parameters.ILimitMotoring_Frac16 = limitMotoring_Frac16;
	p_motor->Parameters.ILimitGenerating_Frac16 = limitGenerating_Frac16;
#ifndef CONFIG_MOTOR_PROPOGATE_SET_PARAM_DISABLE
	_Motor_User_ClearILimitActive(p_motor);
#endif
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
	Motor_User_SetSpeedFeedbackRef_Rpm(p_motor, vMotorSpeed_Rpm * _Motor_GetVRefSupply() / vMotor_V);
}

void Motor_User_SetSpeedVMatchRef_Rpm(Motor_T * p_motor, uint16_t rpm)
{
	uint32_t min = p_motor->Parameters.SpeedFeedbackRef_Rpm / 2;
	uint32_t max = p_motor->Parameters.SpeedFeedbackRef_Rpm * 2;

	if		(rpm > max) { p_motor->Parameters.SpeedVMatchRef_Rpm = max; }
	else if	(rpm < min) { p_motor->Parameters.SpeedVMatchRef_Rpm = min; }
	else 				{ p_motor->Parameters.SpeedVMatchRef_Rpm = rpm; };

	p_motor->Parameters.SpeedVMatchRef_Rpm = rpm;
#ifndef CONFIG_MOTOR_PROPOGATE_SET_PARAM_DISABLE
	Motor_ResetSpeedVMatchRatio(p_motor);
#endif
}

void Motor_User_SetSpeedVMatchRef_VRpm(Motor_T * p_motor, uint16_t vMotor_V, uint16_t vMotorSpeed_Rpm)
{
	Motor_User_SetSpeedVMatchRef_Rpm(p_motor, vMotorSpeed_Rpm * _Motor_GetVRefSupply() / vMotor_V);
}

void Motor_User_SetIRefPeak_Adcu(Motor_T * p_motor, uint16_t adcu)
{
	p_motor->Parameters.IRefPeak_Adcu = (adcu > p_motor->CONFIG.I_SENSOR_PEAK_LIMIT_ADCU) ? p_motor->CONFIG.I_SENSOR_PEAK_LIMIT_ADCU : adcu;
#ifndef CONFIG_MOTOR_PROPOGATE_SET_PARAM_DISABLE
	Motor_ResetUnitsIabc(p_motor);
#endif
}

void Motor_User_SetIRefPeak_MilliV(Motor_T * p_motor, uint16_t min_MilliV, uint16_t max_MilliV)
{
	uint16_t adcuZero = (uint32_t)(max_MilliV + min_MilliV) * ADC_MAX / 2U / _Motor_GetAdcVRef();
	uint16_t adcuMax = (uint32_t)max_MilliV * ADC_MAX / _Motor_GetAdcVRef();
	Motor_User_SetIRefPeak_Adcu(p_motor, adcuMax - adcuZero);
}

void Motor_User_SetIaIbIcZero_Adcu(Motor_T * p_motor, uint16_t ia_adcu, uint16_t ib_adcu, uint16_t ic_adcu)
{
	p_motor->Parameters.IaRefZero_Adcu = ia_adcu;
	p_motor->Parameters.IbRefZero_Adcu = ib_adcu;
	p_motor->Parameters.IcRefZero_Adcu = ic_adcu;
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


/******************************************************************************/
/*
	N Motor Array functions
*/
/******************************************************************************/
//void ProcN(Motor_T * p_motor, uint8_t motorCount, uint32_t (*op)(Motor_T*, uint32_t), uint16_t var)
//{
//	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
//	{
//		op(&p_motor[iMotor], var);
//	}
//}

//alternatively use check stop state?
bool Motor_UserN_CheckStop(Motor_T * p_motor, uint8_t motorCount)
{
	bool isStop = true;

	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		if(p_motor->SpeedFeedback_Frac16 > 0U) { isStop = false; break; }
	}

	return isStop;
}

void Motor_UserN_DisableControl(Motor_T * p_motor, uint8_t motorCount)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_DisableControl(&p_motor[iMotor]);
	}
}

void Motor_UserN_Ground(Motor_T * p_motor, uint8_t motorCount)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_Ground(&p_motor[iMotor]);
	}
}

bool Motor_UserN_SetDirectionForward(Motor_T * p_motor, uint8_t motorCount)
{
	bool isSet = true;

	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		if(Motor_User_SetDirectionForward(&p_motor[iMotor]) == false) { isSet = false; }
	}

	return isSet;
}

bool Motor_UserN_SetDirectionReverse(Motor_T * p_motor, uint8_t motorCount)
{
	bool isSet = true;

	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		if(Motor_User_SetDirectionReverse(&p_motor[iMotor]) == false) { isSet = false; }
	}

	return isSet;
}

bool Motor_UserN_CheckFault(Motor_T * p_motor, uint8_t motorCount)
{
	bool isFault = false;

	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		if(Motor_User_CheckFault(&p_motor[iMotor]) == true) { isFault = true; }
	}

	return isFault;
}

bool Motor_UserN_ClearFault(Motor_T * p_motor, uint8_t motorCount)
{
	bool isClear = true;

	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		if(Motor_User_ClearFault(&p_motor[iMotor]) == false) { isClear = false; }
	}

	return isClear;
}

void Motor_UserN_SetUserControlCmd(Motor_T * p_motor, uint8_t motorCount, int32_t cmd)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_SetUserControlCmd(&p_motor[iMotor], cmd);
	}
}

void Motor_UserN_SetThrottleCmd(Motor_T * p_motor, uint8_t motorCount, uint16_t throttle)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_SetThrottleCmd(&p_motor[iMotor], throttle);
	}
}

void Motor_UserN_SetBrakeCmd(Motor_T * p_motor, uint8_t motorCount, uint16_t brake)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_SetBrakeCmd(&p_motor[iMotor], brake);
	}
}

void Motor_UserN_SetVoltageBrakeCmd(Motor_T * p_motor, uint8_t motorCount)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_SetVoltageBrakeCmd(&p_motor[iMotor]);
	}
}

void Motor_UserN_SetRegenCmd(Motor_T * p_motor, uint8_t motorCount, uint16_t brake)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_SetRegenCmd(&p_motor[iMotor], brake);
	}
}

void Motor_UserN_SetSpeedLimitActive(Motor_T * p_motor, uint8_t motorCount, uint16_t limit_frac16)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_SetSpeedLimitActive(&p_motor[iMotor], limit_frac16);
	}
}

void Motor_UserN_ClearSpeedLimit(Motor_T * p_motor, uint8_t motorCount)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_ClearSpeedLimitActive(&p_motor[iMotor]);
	}
}

void Motor_UserN_SetILimitActive(Motor_T * p_motor, uint8_t motorCount, uint16_t limit_frac16, Motor_ILimitActiveId_T id)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_SetILimitActive(&p_motor[iMotor], limit_frac16, id);
	}
}

void Motor_UserN_ClearILimit(Motor_T * p_motor, uint8_t motorCount, Motor_ILimitActiveId_T id)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_ClearILimitActive(&p_motor[iMotor], id);
	}
}