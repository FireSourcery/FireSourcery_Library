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
	UserCmd functions
	User control modes, torque/speed/position, above foc/sixstep commutation layer
	Call regularly to update cmd value

	User input sign +/- indicates along or against Direction selected. NOT virtual CW/CCW.
	Convert sign to direction here. Called less frequently than control loop, 1/Millis.

	SetMode 	- Invokes StateMachine - Sets control mode only
	SetCmdValue - Without invoking StateMachine - Sets cmd value
		Although cmd/ramp value need not set on every state, handle outside
	SetModeCmd 	- Check/sets control mode, and sets cmd value
*/
/******************************************************************************/

extern void _Motor_User_ActivateControlMode(Motor_T * p_motor, Motor_FeedbackMode_T mode);

/*!
	Convert user reference direction to CCW/CW direction
	@param[in] userCmd int16_t[-32768:32767]
	@return int32_t[-32768:32768], Over saturated if input is -32768
*/
static inline int32_t _Motor_User_CalcDirectionalCmd(Motor_T * p_motor, int16_t userCmd)
{
	return (p_motor->Direction == MOTOR_DIRECTION_CCW) ? userCmd : (int32_t)0 - userCmd;
}

/******************************************************************************/
/*!
	Voltage Mode
*/
/******************************************************************************/
static inline void Motor_User_SetVoltageMode(Motor_T * p_motor)
{
	_Motor_User_ActivateControlMode(p_motor, MOTOR_FEEDBACK_MODE_CONSTANT_VOLTAGE);
}

/*!
	@param[in] voltage [-32768:32767]
*/
static inline void Motor_User_SetVoltageCmdValue(Motor_T * p_motor, int16_t voltage)
{
	int32_t input = (voltage > 0) ? voltage : 0; /* Reverse voltage use change direction */
	Linear_Ramp_SetTarget(&p_motor->Ramp, _Motor_User_CalcDirectionalCmd(p_motor, input));
}

static inline void Motor_User_SetVoltageModeCmd(Motor_T * p_motor, int16_t voltage)
{
	Motor_User_SetVoltageMode(p_motor);
	Motor_User_SetVoltageCmdValue(p_motor, voltage);
}

/******************************************************************************/
/*!
	Voltage Freq Mode
*/
/******************************************************************************/
static inline void Motor_User_SetScalarMode(Motor_T * p_motor)
{
	_Motor_User_ActivateControlMode(p_motor, MOTOR_FEEDBACK_MODE_SCALAR_VOLTAGE_FREQ);
}

/*!
	@param[in] voltage [0:65535] temp scalar 65535 => 1
*/
static inline void Motor_User_SetScalarCmdValue(Motor_T * p_motor, uint32_t scalar)
{
	Linear_Ramp_SetTarget(&p_motor->Ramp, scalar);
}

static inline void Motor_User_SetScalarModeCmd(Motor_T * p_motor, uint32_t scalar)
{
	Motor_User_SetScalarMode(p_motor);
	Motor_User_SetScalarCmdValue(p_motor, scalar);
}

/******************************************************************************/
/*!
	Torque Mode
*/
/******************************************************************************/
static inline void Motor_User_SetTorqueMode(Motor_T * p_motor)
{
	_Motor_User_ActivateControlMode(p_motor, MOTOR_FEEDBACK_MODE_CONSTANT_CURRENT);
}

/*!
	@param[in] torque [-32768:32767]
*/
static inline void Motor_User_SetTorqueCmdValue(Motor_T * p_motor, int16_t torque)
{
	int32_t input = (torque > 0) ? ((int32_t)torque * p_motor->ILimitMotoring_Frac16 / 65536) : ((int32_t)torque * p_motor->ILimitGenerating_Frac16 / 65536);
	Linear_Ramp_SetTarget(&p_motor->Ramp, _Motor_User_CalcDirectionalCmd(p_motor, input));
}

static inline void Motor_User_SetTorqueModeCmd(Motor_T * p_motor, int16_t torque)
{
	Motor_User_SetTorqueMode(p_motor);
	Motor_User_SetTorqueCmdValue(p_motor, torque);
}

/******************************************************************************/
/*!
	Speed Mode
*/
/******************************************************************************/
/*!
	Default speed mode is speed torque mode
*/
static inline void Motor_User_SetSpeedMode(Motor_T * p_motor)
{
	_Motor_User_ActivateControlMode(p_motor, MOTOR_FEEDBACK_MODE_CONSTANT_SPEED_CURRENT);
}

/*!
	Direction forward, request -speed, uses forward speed limit, not reverse

	@param[in] speed [-32768:32767]
*/
static inline void Motor_User_SetSpeedCmdValue(Motor_T * p_motor, int16_t speed)
{
	Linear_Ramp_SetTarget(&p_motor->Ramp, _Motor_User_CalcDirectionalCmd(p_motor, (int32_t)speed * p_motor->SpeedLimit_Frac16 / 65536));
}

static inline void Motor_User_SetSpeedModeCmd(Motor_T * p_motor, int16_t speed)
{
	Motor_User_SetSpeedMode(p_motor);
	Motor_User_SetSpeedCmdValue(p_motor, speed);
}

/******************************************************************************/
/*!
	Open Loop
*/
/******************************************************************************/
/*!

*/
static inline void Motor_User_SetOpenLoopMode(Motor_T * p_motor)
{
	_Motor_User_ActivateControlMode(p_motor, MOTOR_FEEDBACK_MODE_OPEN_LOOP);
}

/*!
*/
static inline void Motor_User_SetOpenLoopCmdValue(Motor_T * p_motor, int16_t vPwm)
{
	Linear_Ramp_SetTarget(&p_motor->Ramp, p_motor->Parameters.OpenLoopVPwm_Frac16 / 2U);
}

static inline void Motor_User_SetOpenLoopModeCmd(Motor_T * p_motor, int16_t vPwm)
{
	Motor_User_SetOpenLoopMode(p_motor);
	Motor_User_SetOpenLoopCmdValue(p_motor, vPwm);
}

/******************************************************************************/
/*!
	Position Mode
*/
/******************************************************************************/
/*!
	todo
	@param[in] angle [0:65535]
*/
// static inline void Motor_User_SetPositionCmd(Motor_T * p_motor, uint16_t angle)
// {
//	_Motor_User_SetControlMode(p_motor, MOTOR_CONTROL_MODE_POSITION);
//	Motor_User_SetCmd(p_motor, angle);
// }

/******************************************************************************/
/*!
	User Mode
*/
/******************************************************************************/
static inline void Motor_User_SetUserFeedbackMode(Motor_T * p_motor)
{
	_Motor_User_ActivateControlMode(p_motor, p_motor->Parameters.DefaultFeedbackMode); /* throttle FeedbackMode */
}

static inline void Motor_User_SetUserFeedbackCmdValue(Motor_T * p_motor, int16_t userCmd)
{
	if		(p_motor->FeedbackModeFlags.Speed == 1U) 	{ Motor_User_SetSpeedCmdValue(p_motor, userCmd); }
	else if	(p_motor->FeedbackModeFlags.Current == 1U) 	{ Motor_User_SetTorqueCmdValue(p_motor, userCmd); }
	else 												{ Motor_User_SetVoltageCmdValue(p_motor, userCmd); }

}

/*!
	@param[in] userCmd [-32768:32767]
*/
static inline void Motor_User_SetUserFeedbackModeCmd(Motor_T * p_motor, int16_t userCmd)
{
	Motor_User_SetUserFeedbackMode(p_motor);
	Motor_User_SetUserFeedbackCmdValue(p_motor, userCmd);
}

//todo remove
/******************************************************************************/
/*!
	Throttle and Brake accept uint16_t, wrapped functions use int16_t
	- UserCmd value in configured DefaultFeedbackMode
*/
/******************************************************************************/
/*!
	@param[in] throttle [0:65535] throttle percentage, 65535 => speed limit
*/
static inline void Motor_User_SetThrottleCmd(Motor_T * p_motor, uint16_t throttle)
{
	Motor_User_SetUserFeedbackModeCmd(p_motor, throttle / 2U);
}

/*!
	Always request opposite direction current
	req opposite iq, bound vq to 0 for no plugging brake

	transition from accelerating to decelerating,
	use signed ramp to transition through 0 without discontinuity
	ramp from in-direction torque to 0 to counter-direction torque

	@param[in] brake [0:65535]
*/
static inline void Motor_User_SetBrakeCmd(Motor_T * p_motor, uint16_t brake)
{

	// if(p_motor->FeedbackModeFlags.Hold == 0U)
	// {
	// 	if(Motor_GetSpeed_RPM(p_motor) > 10U)
	// 	{
	Motor_User_SetTorqueModeCmd(p_motor, (int32_t)0 - (int32_t)brake / 2);
	// 	}
	// 	else
	// 	{
	// 		p_motor->FeedbackModeFlags.Hold = 1U;  //clears on throttle
	// 		Phase_Ground(&p_motor->Phase);
	// 	}
	// }
}


/*
	alternatively, V/F mode
*/
// static inline void Motor_User_SetVoltageBrakeCmd(Motor_T * p_motor)
// {
// 	// if(Motor_GetSpeed_RPM(p_motor) > 30U)
// 	// {
// 	// Motor_User_SetVoltageModeCmd(p_motor, p_motor->Speed_Frac16 * p_motor->Parameters.VoltageBrakeScalar_InvFrac16 / 65536U);
// 	// Motor_User_SetVoltageModeCmd(p_motor, p_motor->Speed_Frac16 * voltage / 65536U);
// 	// }
// 	// else
// 	// {
// 	// 	Motor_User_DisableControl(p_motor); //fix repeat
// 	// }
// }

static inline void Motor_User_SetRegenCmd(Motor_T * p_motor, uint16_t brake)
{
	Motor_User_SetScalarModeCmd(p_motor, (65535U - brake)); /* Higher brake => lower voltage */
}

static inline void Motor_User_SetCoast(Motor_T * p_motor)
{
	Motor_User_SetTorqueModeCmd(p_motor, 0U);
}

/******************************************************************************/
/*!
	inline State wrappers
*/
/******************************************************************************/
/*
	Fault - Shared StateMachine InputId
	Fault State - checks exit
	Other States - user initiated transistion to fault state
*/
static inline void Motor_User_ToggleFault(Motor_T * p_motor)
{
	StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_FAULT, STATE_MACHINE_INPUT_VALUE_NULL);
}

static inline bool Motor_User_ClearFault(Motor_T * p_motor)
{
	if(StateMachine_GetActiveStateId(&p_motor->StateMachine) == MSM_STATE_ID_FAULT)
	{
		StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_FAULT, STATE_MACHINE_INPUT_VALUE_NULL);
	}

	return (StateMachine_GetActiveStateId(&p_motor->StateMachine) != MSM_STATE_ID_FAULT);
}

static inline void Motor_User_SetFault(Motor_T * p_motor)
{
	if(StateMachine_GetActiveStateId(&p_motor->StateMachine) != MSM_STATE_ID_FAULT)
	{
		StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_FAULT, STATE_MACHINE_INPUT_VALUE_NULL);
	}
}

static inline bool Motor_User_CheckFault(Motor_T * p_motor)
{
	return (StateMachine_GetActiveStateId(&p_motor->StateMachine) == MSM_STATE_ID_FAULT);
}


/******************************************************************************/
/*!
	Error checked User Get Set Values
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
static inline uint16_t Motor_User_GetVBemf_Frac16(Motor_T * p_motor)								{ return Linear_Voltage_CalcFracU16(&p_motor->UnitsVabc, p_motor->VBemfPeak_Adcu); }
static inline uint16_t Motor_User_GetVBemf_V(Motor_T * p_motor)										{ return Linear_Voltage_CalcV(&p_motor->UnitsVabc, p_motor->VBemfPeak_Adcu); }

static inline qangle16_t Motor_User_GetElectricalAngle(Motor_T * p_motor) { return p_motor->ElectricalAngle; }
static inline qangle16_t Motor_User_GetMechanicalAngle(Motor_T * p_motor) { return Motor_GetMechanicalAngle(p_motor); }

/*!
	@return I zero to peak.

	iPhase motoring as positive. generating as negative.
*/
static inline int32_t Motor_User_GetIPhase_Frac16(Motor_T * p_motor)
{
	int32_t iPhase;

	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
	{
		iPhase = Motor_FOC_GetIMagnitude_Frac16(p_motor);
		//sign = sign of iq, account direction
		// if(p_motor->Direction == MOTOR_DIRECTION_CW) { iPhase = 0 - iPhase; }
	}
	else /* if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP) */
	{
		iPhase = 0U; //todo
	}

	return iPhase;
}

/*!
	Speed_Frac16 set as CCW us positive
	@return speed forward as positive. reverse as negative.
*/
static inline int32_t Motor_User_GetSpeed_Frac16(Motor_T * p_motor)
{
	return (p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW) ? p_motor->Speed_Frac16 : 0 - p_motor->Speed_Frac16;
}

#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
static inline int16_t Motor_User_GetIPhase_Amp(Motor_T * p_motor) { return _Motor_ConvertToIAmp(p_motor, Motor_User_GetIPhase_Frac16(p_motor)); }
/*!	@return [-32767:32767] Rpm should not exceed int16_t */
static inline int16_t Motor_User_GetSpeed_Rpm(Motor_T * p_motor) { return _Motor_ConvertToSpeedRpm(p_motor, Motor_User_GetSpeed_Frac16(p_motor)); }
#endif

/*
	User output only
	default collected format
*/
// typedef union Motor_User_StatusFlags_T
// {
// 	struct
// 	{
// 		// uint32_t ILimitFault 		: 1U;
// 		// uint32_t ILimitActive 		: 1U;
// 		// uint32_t IWarning 			: 1U;
// 		// uint32_t HeatOverLimit 		: 1U;
// 		// uint32_t HeatOverThreshold 	: 1U;
// 		// uint32_t HeatOverWarning 	: 1U;

// 		Thermistor_Status_T HeatStatus 	: 4U;
// 	};
// 	uint32_t State;
// }
// Motor_User_StatusFlags_T;

// static inline Motor_User_StatusFlags_T Motor_User_GetStatusFlags(Motor_T * p_motor)
// {
// 	Motor_User_StatusFlags_T status ;

// 	status.HeatStatus 		= Thermistor_GetStatus(&p_motor->Thermistor);
// 	// status.IWarning 		= p_motor->ControlFlags.ILimitScalarActive;
// 	// status.ILimitActive 	= p_motor->ControlFlags.ILimitActive;
// 	// status.ILimitFault = p_motor->ControlFlags.ILimitActive;

// 	return status;
// }

/******************************************************************************/
/*
	Nvm Param function
*/
/******************************************************************************/
static inline Motor_CommutationMode_T Motor_User_GetCommutationMode(Motor_T * p_motor) 				{ return p_motor->Parameters.CommutationMode; }
static inline Motor_SensorMode_T Motor_User_GetSensorMode(Motor_T * p_motor) 						{ return p_motor->Parameters.SensorMode; }
static inline Motor_FeedbackMode_T Motor_User_GetFeedbackMode(Motor_T * p_motor) 					{ return p_motor->Parameters.DefaultFeedbackMode; }
static inline Motor_DirectionCalibration_T Motor_User_GetDirectionCalibration(Motor_T * p_motor) 	{ return p_motor->Parameters.DirectionCalibration; }
static inline uint8_t Motor_User_GetPolePairs(Motor_T * p_motor) 									{ return p_motor->Parameters.PolePairs; }
static inline uint16_t Motor_User_GetSpeedFeedbackRef_Rpm(Motor_T * p_motor) 						{ return p_motor->Parameters.SpeedFeedbackRef_Rpm; }
static inline uint16_t Motor_User_GetSpeedVRef_Rpm(Motor_T * p_motor) 								{ return p_motor->Parameters.SpeedVRef_Rpm; }

static inline uint32_t Motor_User_ConvertToSpeedV(Motor_T * p_motor, uint16_t rpm) 					{ return Linear_Function(&p_motor->UnitsVSpeed, _Motor_ConvertToSpeedFrac16(p_motor, rpm)); }

static inline void Motor_User_SetCommutationMode(Motor_T * p_motor, Motor_CommutationMode_T mode)  	{ p_motor->Parameters.CommutationMode = mode; }
static inline void Motor_User_SetAlignMode(Motor_T * p_motor, Motor_AlignMode_T mode) 				{ p_motor->Parameters.AlignMode = mode; }
static inline void Motor_User_SetAlignVoltage(Motor_T * p_motor, uint16_t v_frac16) 				{ p_motor->Parameters.AlignVPwm_Frac16 = (v_frac16 > GLOBAL_MOTOR.ALIGN_VPWM_MAX) ? GLOBAL_MOTOR.ALIGN_VPWM_MAX : v_frac16; }
static inline void Motor_User_SetAlignTime_Cycles(Motor_T * p_motor, uint16_t cycles) 				{ p_motor->Parameters.AlignTime_Cycles = cycles; }
// static inline void Motor_User_SetAlignTime_Millis(Motor_T * p_motor, uint16_t millis) 				{ p_motor->Parameters.AlignTime_Cycles = millis; }
static inline void Motor_User_SetOpenLoopAccel_Cycles(Motor_T * p_motor, uint16_t cycles) 			{ p_motor->Parameters.OpenLoopAccel_Cycles = cycles; }
// static inline void Motor_User_SetOpenLoopAccel_Millis(Motor_T * p_motor, uint16_t millis) 			{ p_motor->Parameters.OpenLoopAccel_Cycles = millis; }
static inline void Motor_User_SetRampAccel_Cycles(Motor_T * p_motor, uint16_t cycles) 				{ p_motor->Parameters.RampAccel_Cycles = cycles; }
// static inline void Motor_User_SetRampAccel_Millis(Motor_T * p_motor, uint16_t millis) 				{ p_motor->Parameters.RampAccel_Cycles = millis; }
// static inline void Motor_User_SetVoltageBrakeScalar_Frac16(Motor_T * p_motor, uint16_t scalar_Frac16) 	{ p_motor->Parameters.VoltageBrakeScalar_InvFrac16 = 65535U - scalar_Frac16; }

/* Persistent Control Mode */
static inline void Motor_User_SetFeedbackModeParam(Motor_T * p_motor, Motor_FeedbackMode_T mode) 	{ p_motor->Parameters.DefaultFeedbackMode = mode; p_motor->FeedbackModeFlags.Update = 1U; }
static inline void Motor_User_SetPhaseModeParam(Motor_T * p_motor, Phase_Mode_T mode) 				{ p_motor->Parameters.PhasePwmMode = mode; Phase_Polar_ActivateMode(&p_motor->Phase, mode); }

/******************************************************************************/
/*
	Additional module wrapper
*/
/******************************************************************************/
// static inline Hall_Sensors_T Motor_User_ReadHall(Motor_T * p_motor) { return Hall_ReadSensors(&p_motor->Hall); }
// static inline bool Motor_User_GetHallA(Motor_T * p_motor) { return Hall_GetSensorA(&p_motor->Hall);; }
// static inline bool Motor_User_GetHallB(Motor_T * p_motor) { return Hall_GetSensorB(&p_motor->Hall); }
// static inline bool Motor_User_GetHallC(Motor_T * p_motor) { return Hall_GetSensorC(&p_motor->Hall); }
// static inline uint16_t Motor_User_GetHallRotorAngle(Motor_T * p_motor) { return Hall_GetRotorAngle_Degrees16(&p_motor->Hall); }

/******************************************************************************/
/*!
	Extern
*/
/******************************************************************************/
extern void Motor_User_DisableControl(Motor_T * p_motor);
extern void Motor_User_ReleaseControl(Motor_T * p_motor);
extern void Motor_User_Ground(Motor_T * p_motor);
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
extern void Motor_User_SetSpeedVRef_Rpm(Motor_T * p_motor, uint16_t rpm);
extern void Motor_User_SetSpeedVRef_Kv(Motor_T * p_motor, uint16_t kv);

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
