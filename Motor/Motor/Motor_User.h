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
	@file 	Motor_User.h
	@author FireSourcery
	@brief  User Interface. Motor module public functions.
			Functions include error checking.
	@version V0
*/
/******************************************************************************/
#ifndef MOTOR_USER_H
#define MOTOR_USER_H

#include "Motor_StateMachine.h"
#include "Motor_FOC.h"

#include <stdint.h>
#include <stdbool.h>



/******************************************************************************/
/*!
	Inline StateMachine Wrappers
*/
/******************************************************************************/
/*
	always State machine checked version
*/
static inline void Motor_User_ReleaseControl(Motor_T * p_motor)
{
	StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_RELEASE, STATE_MACHINE_INPUT_VALUE_NULL); /* no critical for transition, only 1 transistion in run state? cannot conflict? */
}

/*
	Disable control
*/
static inline void Motor_User_DisableControl(Motor_T * p_motor)
{
	Phase_Float(&p_motor->Phase);
	StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_RELEASE, STATE_MACHINE_INPUT_VALUE_NULL);
}

static inline void Motor_User_Ground(Motor_T * p_motor)
{
	Phase_Ground(&p_motor->Phase);
//	StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_GROUND);	//alternatively only ground in stop state
}

/*
	Fault - Shared StateMachine InputId
	Fault State - checks exit
	Other States - user initiated transistion to fault state
*/
static inline bool Motor_User_ClearFault(Motor_T * p_motor)
{
	if(StateMachine_GetActiveStateId(&p_motor->StateMachine) == MSM_STATE_ID_FAULT)
		{ StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_FAULT, STATE_MACHINE_INPUT_VALUE_NULL); }

	return (StateMachine_GetActiveStateId(&p_motor->StateMachine) != MSM_STATE_ID_FAULT);
}

static inline void Motor_User_SetFault(Motor_T * p_motor)
{
	if(StateMachine_GetActiveStateId(&p_motor->StateMachine) != MSM_STATE_ID_FAULT)
		{ StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_FAULT, STATE_MACHINE_INPUT_VALUE_NULL); }
}

static inline bool Motor_User_CheckFault(Motor_T * p_motor)
{
	return (StateMachine_GetActiveStateId(&p_motor->StateMachine) == MSM_STATE_ID_FAULT);
}

static inline void Motor_User_ToggleFault(Motor_T * p_motor)
{
	StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_FAULT, STATE_MACHINE_INPUT_VALUE_NULL);
}

/******************************************************************************/
/*!
	User Get Set Wrappers
*/
/******************************************************************************/
/******************************************************************************/
/*
	RAM Variables
	Conversion functions only on user call. Not called regularly
*/
/******************************************************************************/
static inline Motor_StateMachine_StateId_T Motor_User_GetStateId(Motor_T * p_motor) 				{ return StateMachine_GetActiveStateId(&p_motor->StateMachine); }
static inline uint16_t Motor_User_GetAdcu(Motor_T * p_motor, MotorAnalog_Channel_T adcChannel) 		{ return p_motor->AnalogResults.Channels[adcChannel]; }
static inline uint8_t Motor_User_GetAdcu_Msb8(Motor_T * p_motor, MotorAnalog_Channel_T adcChannel) 	{ return Motor_User_GetAdcu(p_motor, adcChannel) >> (GLOBAL_ANALOG.ADC_BITS - 8U); }
static inline int32_t Motor_User_GetHeat_DegC(Motor_T * p_motor, uint16_t scalar) 					{ return Thermistor_ConvertToDegC_Int(&p_motor->Thermistor, p_motor->AnalogResults.Heat_Adcu, scalar); }
static inline float Motor_User_GetHeat_DegCFloat(Motor_T * p_motor) 								{ return Thermistor_ConvertToDegC_Float(&p_motor->Thermistor, p_motor->AnalogResults.Heat_Adcu); }

static inline qangle16_t Motor_User_GetElectricalAngle(Motor_T * p_motor) { return p_motor->ElectricalAngle; }
static inline qangle16_t Motor_User_GetMechanicalAngle(Motor_T * p_motor) { return Motor_GetMechanicalAngle(p_motor); }

typedef int32_t(*Motor_CommutationModeFunctionInt32_T)(Motor_T * p_motor);

static inline int32_t Motor_GetCommutationModeInt32(Motor_T * p_motor, Motor_CommutationModeFunctionInt32_T focFunction, Motor_CommutationModeFunctionInt32_T sixStepFunction)
{
#if 	defined(CONFIG_MOTOR_SIX_STEP_ENABLE) && defined(CONFIG_MOTOR_FOC_ENABLE)
	int32_t value32;
	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC) 				{ value32 = focFunction(p_motor); }
	else /* p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP */ 	{ value32 = sixStepFunction(p_motor); }
	return value32;
#elif 	defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
	(void)focFunction; return sixStepFunction(p_motor);
#else /* defined(CONFIG_MOTOR_FOC_ENABLE) */
	(void)sixStepFunction; return focFunction(p_motor);
#endif
}

/*!
	Speed_Fixed32 set as CCW is positive
	@return speed forward as positive. reverse as negative.
*/
static inline int32_t Motor_User_GetSpeed_FracS16(Motor_T * p_motor) { return Motor_ConvertUserDirection(p_motor, p_motor->Speed_FracS16); }

/*!
	@return IPhase Zero to Peak.

	iPhase motoring as positive. generating as negative.
*/
static inline int32_t Motor_User_GetIPhase_FracS16(Motor_T * p_motor)
{
	return Motor_ConvertUserDirection(p_motor, Motor_GetCommutationModeInt32(p_motor, Motor_FOC_GetIPhase_FracS16, 0U));
}

/*
	BEMF
*/
// static inline int32_t Motor_User_GetVBemf_FracS16(Motor_T * p_motor)
// {
// }

/*
	BEMF during freewheel or V Out during active control
*/
static inline int32_t Motor_User_GetVPhase_FracS16(Motor_T * p_motor)
{
	return Motor_ConvertUserDirection(p_motor, Motor_GetCommutationModeInt32(p_motor, Motor_FOC_GetVPhase_FracS16, 0U));
}

/* Ideal electrical power */
static inline int32_t Motor_User_GetElectricalPower_FracS16(Motor_T * p_motor)
{
	return Motor_GetCommutationModeInt32(p_motor, Motor_FOC_GetElectricalPower_FracS16, 0U);
}


#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
/*!	@return [-32767:32767] Rpm should not exceed int16_t */
static inline int16_t Motor_User_GetSpeed_Rpm(Motor_T * p_motor) 			{ return _Motor_ConvertSpeed_Scalar16ToRpm(p_motor, Motor_User_GetSpeed_FracS16(p_motor)); }
static inline int16_t Motor_User_GetIPhase_Amps(Motor_T * p_motor) 			{ return _Motor_ConvertI_FracS16ToAmps(Motor_User_GetIPhase_FracS16(p_motor)); }
static inline int16_t Motor_User_GetVPhase_Volts(Motor_T * p_motor) 		{ return _Motor_ConvertV_FracS16ToVolts(Motor_User_GetVPhase_FracS16(p_motor)); }
static inline int32_t Motor_User_GetElectricalPower_VA(Motor_T * p_motor) 	{ return _Motor_ConvertPower_FracS16ToWatts(Motor_User_GetElectricalPower_FracS16(p_motor)); }
#endif


//todo check conversion
static inline uint32_t Motor_User_ConvertToVSpeed(Motor_T * p_motor, uint16_t rpm)  { return Linear_Function(&p_motor->UnitsVSpeed, _Motor_ConvertSpeed_RpmToScalar16(p_motor, rpm)); }

/******************************************************************************/
/*
	Nvm Param function
*/
/******************************************************************************/
static inline Motor_CommutationMode_T Motor_User_GetCommutationMode(Motor_T * p_motor) 				{ return p_motor->Parameters.CommutationMode; }
static inline Motor_SensorMode_T Motor_User_GetSensorMode(Motor_T * p_motor) 						{ return p_motor->Parameters.SensorMode; }
static inline Motor_FeedbackModeId_T Motor_User_GetFeedbackMode(Motor_T * p_motor) 					{ return p_motor->Parameters.DefaultFeedbackMode; }
static inline Motor_DirectionCalibration_T Motor_User_GetDirectionCalibration(Motor_T * p_motor) 	{ return p_motor->Parameters.DirectionCalibration; }
static inline uint8_t Motor_User_GetPolePairs(Motor_T * p_motor) 									{ return p_motor->Parameters.PolePairs; }
static inline uint16_t Motor_User_GetSpeedFeedbackRef_Rpm(Motor_T * p_motor) 						{ return p_motor->Parameters.SpeedFeedbackRef_Rpm; }
static inline uint16_t Motor_User_GetSpeedVRef_Rpm(Motor_T * p_motor) 								{ return p_motor->Parameters.VSpeedRef_Rpm; }

static inline void Motor_User_SetCommutationMode(Motor_T * p_motor, Motor_CommutationMode_T mode)  	{ p_motor->Parameters.CommutationMode = mode; }
// static inline void Motor_User_SetAlignMode(Motor_T * p_motor, Motor_AlignMode_T mode) 				{ p_motor->Parameters.AlignMode = mode; }
static inline void Motor_User_SetAlignVoltage(Motor_T * p_motor, uint16_t v_frac16) 				{ p_motor->Parameters.AlignPower_FracU16 = (v_frac16 > GLOBAL_MOTOR.ALIGN_VPWM_MAX) ? GLOBAL_MOTOR.ALIGN_VPWM_MAX : v_frac16; }
static inline void Motor_User_SetAlignTime_Cycles(Motor_T * p_motor, uint16_t cycles) 				{ p_motor->Parameters.AlignTime_Cycles = cycles; }
static inline void Motor_User_SetOpenLoopAccel_Cycles(Motor_T * p_motor, uint16_t cycles) 			{ p_motor->Parameters.OpenLoopAccel_Cycles = cycles; }
static inline void Motor_User_SetRampAccel_Cycles(Motor_T * p_motor, uint16_t cycles) 				{ p_motor->Parameters.RampAccel_Cycles = cycles; }
#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
static inline void Motor_User_SetAlignTime_Millis(Motor_T * p_motor, uint16_t millis) 				{ p_motor->Parameters.AlignTime_Cycles = _Motor_ConvertToControlCycles(p_motor, millis); }
static inline void Motor_User_SetOpenLoopAccel_Millis(Motor_T * p_motor, uint16_t millis) 			{ p_motor->Parameters.OpenLoopAccel_Cycles = _Motor_ConvertToControlCycles(p_motor, millis); }
static inline void Motor_User_SetRampAccel_Millis(Motor_T * p_motor, uint16_t millis) 				{ p_motor->Parameters.RampAccel_Cycles = _Motor_ConvertToControlCycles(p_motor, millis); }
#endif
// static inline void Motor_User_SetVoltageBrakeScalar_Frac16(Motor_T * p_motor, uint16_t scalar_Frac16) 	{ p_motor->Parameters.VoltageBrakeScalar_InvFrac16 = 65535U - scalar_Frac16; }

/* Persistent Control Mode */
static inline void Motor_User_SetFeedbackModeParam(Motor_T * p_motor, Motor_FeedbackModeId_T mode) 	{ p_motor->Parameters.DefaultFeedbackMode = mode; p_motor->ControlFeedbackMode.IsDisable = 1U; }
#ifdef CONFIG_MOTOR_SIX_STEP_ENABLE
static inline void Motor_User_SetPhaseModeParam(Motor_T * p_motor, Phase_Mode_T mode) 				{ p_motor->Parameters.PhasePwmMode = mode; Phase_Polar_ActivateMode(&p_motor->Phase, mode); }
#endif

/******************************************************************************/
/*!
	Extern
*/
/******************************************************************************/
extern void Motor_User_SetVoltageMode(Motor_T * p_motor);
extern void Motor_User_SetVoltageCmdValue(Motor_T * p_motor, int16_t vCmd);
extern void Motor_User_SetVoltageModeCmd(Motor_T * p_motor, int16_t voltage);
extern bool Motor_User_GetVoltageModeIOverLimit(Motor_T * p_motor);
extern void Motor_User_SetScalarMode(Motor_T * p_motor);
extern void Motor_User_SetScalarCmdValue(Motor_T * p_motor, uint32_t scalar);
extern void Motor_User_SetScalarModeCmd(Motor_T * p_motor, uint32_t scalar);
extern void Motor_User_SetTorqueMode(Motor_T * p_motor);
extern void Motor_User_SetTorqueCmdValue(Motor_T * p_motor, int16_t torque);
extern void Motor_User_SetTorqueModeCmd(Motor_T * p_motor, int16_t torque);
extern void Motor_User_SetSpeedMode(Motor_T * p_motor);
extern void Motor_User_SetSpeedCmdValue(Motor_T * p_motor, int16_t speed);
extern void Motor_User_SetSpeedModeCmd(Motor_T * p_motor, int16_t speed);
extern void Motor_User_SetOpenLoopMode(Motor_T * p_motor);
extern void Motor_User_SetOpenLoopCmdValue(Motor_T * p_motor, int16_t ivCmd);
extern void Motor_User_SetOpenLoopModeCmd(Motor_T * p_motor, int16_t ivMagnitude);
extern void Motor_User_SetPositionCmdValue(Motor_T * p_motor, uint16_t angle);
extern void Motor_User_SetDefaultFeedbackMode(Motor_T * p_motor);
extern void Motor_User_SetDefaultFeedbackCmdValue(Motor_T * p_motor, int16_t userCmd);
extern void Motor_User_SetDefaultCmd(Motor_T * p_motor, int16_t userCmd);
extern void Motor_User_SetThrottleCmd(Motor_T * p_motor, uint16_t throttle);
extern void Motor_User_SetBrakeCmd(Motor_T * p_motor, uint16_t brake);
extern void Motor_User_SetVBrakeCmd(Motor_T * p_motor, uint16_t brake);
extern void Motor_User_SetCruise(Motor_T * p_motor);

extern bool Motor_User_SetDirection(Motor_T * p_motor, Motor_Direction_T direction);
extern bool Motor_User_SetDirectionForward(Motor_T * p_motor);
extern bool Motor_User_SetDirectionReverse(Motor_T * p_motor);
extern void Motor_User_ActivateCalibrationHall(Motor_T * p_motor);
extern void Motor_User_ActivateCalibrationEncoder(Motor_T * p_motor);
extern void Motor_User_ActivateCalibrationAdc(Motor_T * p_motor);
extern void Motor_User_ActivateCalibrationSinCos(Motor_T * p_motor);

extern void Motor_User_SetSpeedLimitActive(Motor_T * p_motor, uint16_t scalar_frac16);
extern void Motor_User_ClearSpeedLimitActive(Motor_T * p_motor);
// extern bool Motor_User_SetSpeedLimitActive_Id(Motor_T * p_motor, uint16_t scalar_frac16, Motor_SpeedLimitActiveId_T id);
// extern void Motor_User_ClearSpeedLimitActive_Id(Motor_T * p_motor, Motor_SpeedLimitActiveId_T id);
// extern void _Motor_User_SetILimitActive(Motor_T * p_motor, uint16_t scalar_frac16);
// extern void _Motor_User_ClearLimitActive(Motor_T * p_motor);
extern bool Motor_User_SetILimitActive(Motor_T * p_motor, uint16_t scalar_frac16, Motor_ILimitActiveId_T id);
extern bool Motor_User_ClearILimitActive(Motor_T * p_motor, Motor_ILimitActiveId_T id);

extern int16_t Motor_User_GetGroundSpeed_Mph(Motor_T * p_motor);
extern void Motor_User_SetGroundSpeed_Kmh(Motor_T * p_motor, uint32_t wheelDiameter_Mm, uint32_t wheelToMotorRatio_Factor, uint32_t wheelToMotorRatio_Divisor);
extern void Motor_User_SetGroundSpeed_Mph(Motor_T * p_motor, uint32_t wheelDiameter_Inch10, uint32_t wheelToMotorRatio_Factor, uint32_t wheelToMotorRatio_Divisor);

extern void Motor_User_SetSpeedLimitParam_Frac16(Motor_T * p_motor, uint16_t forward_Frac16, uint16_t reverse_Frac16);
extern void Motor_User_SetSpeedLimitForwardParam_Frac16(Motor_T * p_motor, uint16_t forward_Frac16);
extern void Motor_User_SetSpeedLimitReverseParam_Frac16(Motor_T * p_motor, uint16_t reverse_Frac16);
extern void Motor_User_SetILimitParam_Frac16(Motor_T * p_motor, uint16_t motoring_Frac16, uint16_t generating_Frac16);
extern void Motor_User_SetILimitMotoringParam_Frac16(Motor_T * p_motor, uint16_t motoring_Frac16);
extern void Motor_User_SetILimitGeneratingParam_Frac16(Motor_T * p_motor, uint16_t generating_Frac16);

#if defined(CONFIG_MOTOR_UNIT_CONVERSION_LOCAL)
extern void Motor_User_SetSpeedLimitParam_Rpm(Motor_T * p_motor, uint16_t forward_Rpm, uint16_t reverse_Rpm);
extern void Motor_User_SetSpeedLimitForwardParam_Rpm(Motor_T * p_motor, uint16_t forward_Rpm);
extern void Motor_User_SetSpeedLimitReverseParam_Rpm(Motor_T * p_motor, uint16_t reverse_Rpm);
extern void Motor_User_SetILimitParam_Amp(Motor_T * p_motor, uint16_t motoring_Amp, uint16_t generating_Amp);
extern void Motor_User_SetILimitMotoringParam_Amp(Motor_T * p_motor, uint16_t motoring_Amp);
extern void Motor_User_SetILimitGeneratingParam_Amp(Motor_T * p_motor, uint16_t generating_Amp);
#endif

extern void Motor_User_SetSpeedFeedbackRef_Rpm(Motor_T * p_motor, uint16_t rpm);
extern void Motor_User_SetSpeedFeedbackRef_Kv(Motor_T * p_motor, uint16_t kv);
extern void Motor_User_SetVSpeedRef_Rpm(Motor_T * p_motor, uint16_t rpm);
extern void Motor_User_SetVSpeedRef_Kv(Motor_T * p_motor, uint16_t kv);

extern void Motor_User_SetIaZero_Adcu(Motor_T * p_motor, uint16_t adcu);
extern void Motor_User_SetIbZero_Adcu(Motor_T * p_motor, uint16_t adcu);
extern void Motor_User_SetIcZero_Adcu(Motor_T * p_motor, uint16_t adcu);
extern void Motor_User_SetIaIbIcZero_Adcu(Motor_T * p_motor, uint16_t ia_adcu, uint16_t ib_adcu, uint16_t ic_adcu);
extern void Motor_User_SetDirectionCalibration(Motor_T * p_motor, Motor_DirectionCalibration_T mode);
extern void Motor_User_SetPolePairs(Motor_T * p_motor, uint8_t polePairs);
extern void Motor_User_SetSensorMode(Motor_T * p_motor, Motor_SensorMode_T mode);
#if defined(CONFIG_MOTOR_DEBUG_ENABLE)
extern void Motor_User_SetIPeakRef_Adcu_Debug(Motor_T * p_motor, uint16_t adcu);
extern void Motor_User_SetIPeakRef_Adcu(Motor_T * p_motor, uint16_t adcu);
extern void Motor_User_SetIPeakRef_MilliV(Motor_T * p_motor, uint16_t min_MilliV, uint16_t max_MilliV);
#endif

#endif
