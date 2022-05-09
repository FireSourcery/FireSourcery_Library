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
	@file 	Motor_User.h
	@author FireSoucery
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


extern void _Motor_User_SetFeedbackMode(Motor_T * p_motor, Motor_FeedbackMode_T mode);

/******************************************************************************/
/*!
	UserCmd functions

	User control modes, torque/speed/position, above foc/sixstep commutation layer

	Call regularly to update cmd value
	Cmd value sets without checking state machine.

	User input sign +/- indicates along or against Direction selected. NOT virtual CW/CCW.
	Convert sign to direction here. Called less frequently than control loop, 1/Millis.

	SetMode functions sets control mode only
	SetCmd functions check/sets control mode, and sets cmd value
*/
/******************************************************************************/

/*!
	@param[in] userCmd [-32768:32767]
	@return int32_t[-32767:32767]
*/
static inline int32_t _Motor_User_CalcDirectionalCmd(Motor_T * p_motor, int32_t userCmd)
{
	return (p_motor->Direction == MOTOR_DIRECTION_CCW) ? userCmd : 0 - userCmd;
}

/******************************************************************************/
/*!
	Voltage Mode
*/
/******************************************************************************/
static inline void Motor_User_SetVoltageMode(Motor_T * p_motor)
{
	_Motor_User_SetFeedbackMode(p_motor, MOTOR_FEEDBACK_MODE_CONSTANT_VOLTAGE);
}

/*!
	@param[in] voltage [-32768:32767]
*/
static inline void Motor_User_SetVoltageCmdValue(Motor_T * p_motor, int16_t voltage)
{
	int32_t input = (voltage > 0) ? (int32_t)voltage * p_motor->ILimitMotoring_Frac16 / 65536 : 0;
	Motor_SetRamp(p_motor, _Motor_User_CalcDirectionalCmd(p_motor, input));
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
static inline void Motor_User_SetVFreqMode(Motor_T * p_motor)
{
	_Motor_User_SetFeedbackMode(p_motor, MOTOR_FEEDBACK_MODE_VOLTAGE_FREQ_SCALAR);
}

/*!
	@param[in] voltage [-32768:32767]
*/
static inline void Motor_User_SetVFreqCmdValue(Motor_T * p_motor, int16_t voltage)
{
	Motor_User_SetVoltageCmdValue(p_motor, voltage);
}

static inline void Motor_User_SetVFreqModeCmd(Motor_T * p_motor, int16_t voltage)
{
	Motor_User_SetVFreqMode(p_motor);
	Motor_User_SetVFreqCmdValue(p_motor, voltage);
}


/******************************************************************************/
/*!
	Torque Mode
*/
/******************************************************************************/
static inline void Motor_User_SetTorqueMode(Motor_T * p_motor)
{
	_Motor_User_SetFeedbackMode(p_motor, MOTOR_FEEDBACK_MODE_CONSTANT_CURRENT);
}

/*!
	@param[in] torque [-32768:32767]
*/
static inline void Motor_User_SetTorqueCmdValue(Motor_T * p_motor, int16_t torque)
{
	int32_t input = (torque > 0) ? (int32_t)torque * p_motor->ILimitMotoring_Frac16 / 65536 : (int32_t)torque * p_motor->ILimitGenerating_Frac16 / 65536;
	Motor_SetRamp(p_motor, _Motor_User_CalcDirectionalCmd(p_motor, input));
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
	_Motor_User_SetFeedbackMode(p_motor, MOTOR_FEEDBACK_MODE_CONSTANT_SPEED_CURRENT);
}

/*!
	@param[in] speed [-32768:32767]
*/
static inline void Motor_User_SetSpeedCmdValue(Motor_T * p_motor, int16_t speed)
{
	int32_t input = (int32_t)speed * p_motor->SpeedLimit_Frac16 / 65536;
	Motor_SetRamp(p_motor, _Motor_User_CalcDirectionalCmd(p_motor, input));
}

static inline void Motor_User_SetSpeedModeCmd(Motor_T * p_motor, int16_t speed)
{
	Motor_User_SetSpeedMode(p_motor);
	Motor_User_SetSpeedCmdValue(p_motor, speed);
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
//	_Motor_User_SetFeedbackMode(p_motor, MOTOR_CONTROL_MODE_POSITION);
//	Motor_User_SetCmd(p_motor, angle);
// }

/******************************************************************************/
/*!
	User Mode
*/
/******************************************************************************/
static inline void Motor_User_SetUserFeedbackMode(Motor_T * p_motor)
{
	_Motor_User_SetFeedbackMode(p_motor, p_motor->Parameters.FeedbackMode);
}

static inline void Motor_User_SetUserFeedbackCmdValue(Motor_T * p_motor, int16_t userCmd)
{
	if(p_motor->FeedbackModeFlags.Speed == 1U)
	{
		Motor_User_SetSpeedCmdValue(p_motor, userCmd);
	}
	else
	{
		(p_motor->FeedbackModeFlags.Current == 1U) ?
			Motor_User_SetTorqueCmdValue(p_motor, userCmd) :
			Motor_User_SetVoltageCmdValue(p_motor, userCmd);
	}
}

static inline void Motor_User_SetUserFeedbackModeCmd(Motor_T * p_motor, int16_t userCmd)
{
	Motor_User_SetUserFeedbackMode(p_motor);
	Motor_User_SetUserFeedbackCmdValue(p_motor, userCmd);
}

/******************************************************************************/
/*!
	Throttle
*/
/******************************************************************************/
/*!
	@param[in] throttle [0:65535] trhottle percentage, 65535 => speed limit
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
static inline void Motor_User_SetVoltageBrakeCmd(Motor_T * p_motor)
{
	// if(Motor_GetSpeed_RPM(p_motor) > 30U)
	// {
	Motor_User_SetVoltageModeCmd(p_motor, p_motor->SpeedFeedback_Frac16 * p_motor->Parameters.VoltageBrakeScalar_InvFrac16 / 65536U);
	// Motor_User_SetVoltageModeCmd(p_motor, p_motor->SpeedFeedback_Frac16 * voltage / 65536U);
	// Motor_User_SetVFreqModeCmd( p_motor,  voltage);

	// }
	// else
	// {
	// 	Motor_User_DisableControl(p_motor);   //fix repeat
	// }
}


/*
	Fault State - checks exit
	Other States - user initated tranisition to fault state
*/
static inline void Motor_User_ProcFault(Motor_T * p_motor)
{
	StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_FAULT);
}

static inline bool Motor_User_ClearFault(Motor_T * p_motor)
{
	if(StateMachine_GetActiveStateId(&p_motor->StateMachine) == MSM_STATE_ID_FAULT)
	{
		StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_FAULT);
	}

	return (StateMachine_GetActiveStateId(&p_motor->StateMachine) != MSM_STATE_ID_FAULT);
}

static inline void Motor_User_SetFault(Motor_T * p_motor)
{
	if(StateMachine_GetActiveStateId(&p_motor->StateMachine) != MSM_STATE_ID_FAULT)
	{
		StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_FAULT);
	}
}

static inline bool Motor_User_CheckFault(Motor_T * p_motor)
{
	return (StateMachine_GetActiveStateId(&p_motor->StateMachine) == MSM_STATE_ID_FAULT);
}

//static inline void Motor_User_ActivatePhase(Motor_T * p_motor)
//{
//
//}

/******************************************************************************/
/*
	RAM Variables
	Conversion functions only on user call. Not called regularly
*/
/******************************************************************************/
static inline Motor_StateMachine_StateId_T Motor_User_GetStateId(Motor_T * p_motor) 						{ return StateMachine_GetActiveStateId(&p_motor->StateMachine); }
static inline uint16_t Motor_User_GetMotorAdcu(Motor_T * p_motor, MotorAnalog_Channel_T adcChannel) 		{ return p_motor->AnalogResults.Channels[adcChannel]; }
static inline uint16_t Motor_User_GetMotorAdcu_Msb8(Motor_T * p_motor, MotorAnalog_Channel_T adcChannel) 	{ return Motor_User_GetMotorAdcu(p_motor, adcChannel) >> (ADC_BITS - 8U); }
static inline int32_t Motor_User_GetHeat_DegC(Motor_T * p_motor, uint16_t scalar) 	{ return Thermistor_ConvertToDegC_Int(&p_motor->Thermistor, p_motor->AnalogResults.Heat_Adcu, scalar); }
static inline float Motor_User_GetHeat_DegCFloat(Motor_T * p_motor) 				{ return Thermistor_ConvertToDegC_Float(&p_motor->Thermistor, p_motor->AnalogResults.Heat_Adcu); }
// static inline uint16_t Motor_User_GetBemf_Frac16(Motor_T * p_motor)		{ return Linear_Voltage_CalcFractionUnsigned16(&p_motor->UnitVabc, BEMF_GetVBemfPeak_Adcu(&p_motor->Bemf)); }
// static inline uint32_t Motor_User_GetBemf_V(Motor_T * p_motor)			{ return Linear_Voltage_CalcV(&p_motor->UnitVabc, BEMF_GetVBemfPeak_Adcu(&p_motor->Bemf)); }

static inline int32_t Motor_User_GetIPhase_Frac16(Motor_T * p_motor)
{
	uint16_t iPhase;

	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
	{
		iPhase = Motor_FOC_GetIMagnitude_Frac16(p_motor);  //or use iq?
	}
	else
	{

	}

	return iPhase;
}

static inline int16_t Motor_User_GetIPhase_Amp(Motor_T * p_motor)
{
	return Motor_ConvertToIAmp(p_motor, Motor_User_GetIPhase_Frac16(p_motor));
}

static inline int32_t Motor_User_GetSpeed_Frac16(Motor_T * p_motor)
{
	return (p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW) ? p_motor->SpeedFeedback_Frac16 : 0 - p_motor->SpeedFeedback_Frac16;
}

/*!
	@return speed_Rpm reverse as negative. forward as positive
*/
static inline int16_t Motor_User_GetSpeed_Rpm(Motor_T * p_motor)
{
	int16_t speed_Rpm = Motor_ConvertToSpeedRpm(p_motor, p_motor->SpeedFeedback_Frac16);
	return (p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW) ? speed_Rpm : 0 - speed_Rpm;
}

/*
	User output only
*/
typedef union Motor_User_StatusFlags_T
{
	struct
	{
		// uint32_t ILimitFault 		: 1U;
		// uint32_t ILimitActive 		: 1U;
		// uint32_t IWarning 			: 1U;
		// uint32_t HeatOverLimit 		: 1U;
		// uint32_t HeatOverThreshold 	: 1U;
		// uint32_t HeatOverWarning 	: 1U;

		Thermistor_Status_T HeatStatus 	: 2U;
	};
	uint32_t State;
}
Motor_User_StatusFlags_T;

static inline Motor_User_StatusFlags_T Motor_User_GetStatusFlags(Motor_T * p_motor)
{
	Motor_User_StatusFlags_T status ;

	status.HeatStatus 		= Thermistor_GetStatus(&p_motor->Thermistor);
	// status.IWarning 		= p_motor->RunStateFlags.ILimitScalarActive;
	// status.ILimitActive 	= p_motor->RunStateFlags.ILimitActive;
	// status.ILimitFault = p_motor->RunStateFlags.ILimitActive;

	return status;
}



/******************************************************************************/
/*
	Nvm Param function
*/
/******************************************************************************/
static inline Motor_CommutationMode_T Motor_User_GetCommutationMode(Motor_T * p_motor, Motor_CommutationMode_T mode) 	{ return p_motor->Parameters.CommutationMode; }
static inline Motor_SensorMode_T Motor_User_GetSensorMode(Motor_T * p_motor, Motor_SensorMode_T mode) 					{ return p_motor->Parameters.SensorMode; }
static inline Motor_DirectionCalibration_T Motor_User_GetDirectionCalibration(Motor_T * p_motor) 						{ return p_motor->Parameters.DirectionCalibration; }

// static inline void Motor_User_SetIaZero_Adcu(Motor_T * p_motor, uint16_t adcu) { p_motor->Parameters.IaRefZero_Adcu = adcu; }
// static inline void Motor_User_SetIbZero_Adcu(Motor_T * p_motor, uint16_t adcu) { p_motor->Parameters.IbRefZero_Adcu = adcu; }
// static inline void Motor_User_SetIcZero_Adcu(Motor_T * p_motor, uint16_t adcu) { p_motor->Parameters.IcRefZero_Adcu = adcu; }

/* Does each max need individual adjustment? */
// static inline void Motor_User_SetIaMax_Adcu(Motor_T * p_motor, uint16_t adcu) { p_motor->Parameters.IaRefMax_Adcu = adcu; }
// static inline void Motor_User_SetIbMax_Adcu(Motor_T * p_motor, uint16_t adcu) { p_motor->Parameters.IbRefMax_Adcu = adcu; }
// static inline void Motor_User_SetIcMax_Adcu(Motor_T * p_motor, uint16_t adcu) { p_motor->Parameters.IcRefMax_Adcu = adcu; }

static inline void Motor_User_SetCommutationMode(Motor_T * p_motor, Motor_CommutationMode_T mode)  	{ p_motor->Parameters.CommutationMode = mode; }
static inline void Motor_User_SetAlignMode(Motor_T * p_motor, Motor_AlignMode_T mode) 				{ p_motor->Parameters.AlignMode = mode; }
static inline void Motor_User_SetAlignVoltage(Motor_T * p_motor, uint16_t v_frac16) 				{ p_motor->Parameters.AlignVoltage_Frac16 = (v_frac16 > CONFIG_MOTOR_ALIGN_VOLTAGE_MAX) ? CONFIG_MOTOR_ALIGN_VOLTAGE_MAX : v_frac16; }
static inline void Motor_User_SetAlignTime_Millis(Motor_T * p_motor, uint16_t millis) 				{ p_motor->Parameters.AlignTime_ControlCycles = millis * (20000U/1000U) ; } //todo define pwm freq
static inline void Motor_User_VoltageBrakeScalar_Frac16(Motor_T * p_motor, uint16_t scalar_Frac16) 	{ p_motor->Parameters.VoltageBrakeScalar_InvFrac16 = 65535U - scalar_Frac16; }

/* Persistent Control Mode */
static inline void Motor_User_SetFeedbackModeParam(Motor_T * p_motor, Motor_FeedbackMode_T mode) 	{ p_motor->Parameters.FeedbackMode = mode; }
static inline void Motor_User_SetPhaseModeParam(Motor_T * p_motor, Phase_Mode_T mode) 				{ p_motor->Parameters.PhasePwmMode = mode; Phase_Polar_ActivateMode(&p_motor->Phase, mode); }
static inline void Motor_User_SetILimitOnHeatParam(Motor_T * p_motor, uint16_t scalar_Frac16) 		{ p_motor->Parameters.ILimitScalarHeat_Frac16 = scalar_Frac16; }

/******************************************************************************/
/*
	Additional module wrapper
*/
/******************************************************************************/
static inline Hall_Sensors_T Motor_User_ReadHall(Motor_T * p_motor) { return Hall_ReadSensors(&p_motor->Hall); }
static inline bool Motor_User_GetHallA(Motor_T * p_motor) { return Hall_GetSensorA(&p_motor->Hall);; }
static inline bool Motor_User_GetHallB(Motor_T * p_motor) { return Hall_GetSensorB(&p_motor->Hall); }
static inline bool Motor_User_GetHallC(Motor_T * p_motor) { return Hall_GetSensorC(&p_motor->Hall); }
static inline uint16_t Motor_User_GetHallRotorAngle(Motor_T * p_motor) { return Hall_GetRotorAngle_Degrees16(&p_motor->Hall); }

/******************************************************************************/
/*!
	extern
*/
/******************************************************************************/
extern void Motor_User_DisableControl(Motor_T * p_motor);
extern void Motor_User_Ground(Motor_T * p_motor);
extern bool Motor_User_SetDirection(Motor_T * p_motor, Motor_Direction_T direction);
extern bool Motor_User_SetDirectionForward(Motor_T * p_motor);
extern bool Motor_User_SetDirectionReverse(Motor_T * p_motor);
extern void Motor_User_ActivateCalibrationHall(Motor_T * p_motor);
extern void Motor_User_ActivateCalibrationEncoder(Motor_T * p_motor);
extern void Motor_User_ActivateCalibrationAdc(Motor_T * p_motor);
extern void Motor_User_ActivateCalibrationSinCos(Motor_T * p_motor);

extern void Motor_User_SetSpeedLimitActiveScalar(Motor_T * p_motor, uint16_t scalar_frac16);
extern void Motor_User_ClearSpeedLimitActive(Motor_T * p_motor );
extern void Motor_User_SetILimitActiveScalar(Motor_T * p_motor, uint16_t scalar_frac16);
extern void Motor_User_ClearILimitActive(Motor_T * p_motor);
extern uint16_t Motor_User_GetMechanialAngle(Motor_T * p_motor);

extern void Motor_User_SetSpeedRefMax_Rpm(Motor_T * p_motor, uint16_t rpm);
extern void Motor_User_SetSpeedRefMax_VRpm(Motor_T * p_motor, uint16_t vMotor, uint16_t vMotorSpeed_Rpm) ;
extern void Motor_User_SetIRefPeak_Adcu(Motor_T * p_motor, uint16_t adcu);
extern void Motor_User_SetIRefPeak_MilliV(Motor_T * p_motor, uint16_t min_MilliV, uint16_t max_MilliV);
extern void Motor_User_SetIaIbIcZero_Adcu(Motor_T * p_motor, uint16_t ia_adcu, uint16_t ib_adcu, uint16_t ic_adcu);

extern void Motor_User_SetILimitParam_Amp(Motor_T * p_motor, uint16_t motoring_Amp, uint16_t generating_Amp);
extern void Motor_User_SetSpeedLimitParam_Rpm(Motor_T * p_motor, uint16_t forward_Rpm, uint16_t reverse_Rpm);
extern void Motor_User_SetDirectionCalibration(Motor_T * p_motor, Motor_DirectionCalibration_T mode);

extern void Motor_UserN_SetCmd(Motor_T * p_motor, uint8_t motorCount, int32_t cmd);
extern void Motor_UserN_SetThrottleCmd(Motor_T * p_motor, uint8_t motorCount, uint16_t throttle);
extern void Motor_UserN_SetBrakeCmd(Motor_T * p_motor, uint8_t motorCount, uint16_t brake);
extern void Motor_UserN_SetVoltageBrakeCmd(Motor_T * p_motor, uint8_t motorCount);
extern bool Motor_UserN_SetDirectionForward(Motor_T * p_motor, uint8_t motorCount);
extern bool Motor_UserN_SetDirectionReverse(Motor_T * p_motor, uint8_t motorCount);
extern void Motor_UserN_DisableControl(Motor_T * p_motor, uint8_t motorCount);
extern void Motor_UserN_Ground(Motor_T * p_motor, uint8_t motorCount);
extern bool Motor_UserN_CheckStop(Motor_T * p_motor, uint8_t motorCount);
extern bool Motor_UserN_CheckErrorFlags(Motor_T * p_motor, uint8_t motorCount);
extern bool Motor_UserN_ClearFault(Motor_T * p_motor, uint8_t motorCount);

#endif
