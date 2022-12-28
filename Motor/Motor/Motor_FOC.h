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
	@file 	Motor_FOC.h
	@author FireSourcery
	@brief  Motor FOC submodule. FOC control functions.
			FOC mode StateMachine mapping
			defined as inline for StateMachine wrapper functions
	@version V0
*/
/******************************************************************************/
#ifndef MOTOR_FOC_H
#define MOTOR_FOC_H

#include "Motor.h"
#include "Transducer/Encoder/Encoder_ISR.h"

/******************************************************************************/
/*
	+/- Sign indicates absolute direction, CW/CCW. NOT along/against direction selected.
	Positive is virtual CCW.
	B and Beta are virtual CCW of A/Alpha.
	Iq sign is relative to rotor direction, NOT Vq direction.

	CCW +Vq +Iq => Forward Motoring Q1
	CCW +Vq -Iq => Forward Regen Q4
	CCW -Vq -Iq => Forward Plugging

	CW -Vq -Iq => Reverse Motoring Q2
	CW -Vq +Iq => Reverse Regen Q3
	CW +Vq +Iq => Reverse Plugging
*/
/******************************************************************************/
/******************************************************************************/
/*!
	Map to Motor Analog Conversions
	Convert current from ADCU to QFrac
*/
/******************************************************************************/
static inline void Motor_FOC_CaptureIa(Motor_T * p_motor)
{
	qfrac16_t iphase = ((int32_t)Linear_ADC_CalcFracS16(&p_motor->UnitsIa, p_motor->AnalogResults.Ia_Adcu) + FOC_GetIa(&p_motor->Foc)) / 2;
	FOC_SetIa(&p_motor->Foc, iphase);
}

static inline void Motor_FOC_CaptureIb(Motor_T * p_motor)
{
	qfrac16_t iphase = ((int32_t)Linear_ADC_CalcFracS16(&p_motor->UnitsIb, p_motor->AnalogResults.Ib_Adcu) + FOC_GetIb(&p_motor->Foc)) / 2;
	FOC_SetIb(&p_motor->Foc, iphase);
}

static inline void Motor_FOC_CaptureIc(Motor_T * p_motor)
{
	qfrac16_t iphase = ((int32_t)Linear_ADC_CalcFracS16(&p_motor->UnitsIc, p_motor->AnalogResults.Ic_Adcu) + FOC_GetIc(&p_motor->Foc)) / 2;
	FOC_SetIc(&p_motor->Foc, iphase);
}

static inline void Motor_FOC_CaptureVa(Motor_T * p_motor) {}
static inline void Motor_FOC_CaptureVb(Motor_T * p_motor) {}
static inline void Motor_FOC_CaptureVc(Motor_T * p_motor) {}

/******************************************************************************/
/*!

*/
/******************************************************************************/
/* IdqMagnitude */
static inline uint32_t Motor_FOC_GetIMagnitude_Frac16(Motor_T * p_motor)
{
	return (uint32_t)FOC_GetIMagnitude(&p_motor->Foc) * 2U;
}

/******************************************************************************/
/*!
	State Machine Helpers
*/
/******************************************************************************/
extern void Motor_FOC_ProcPosition(Motor_T * p_motor);
extern bool Motor_FOC_ProcSpeed(Motor_T * p_motor);
extern void Motor_FOC_ProcSpeedFeedback(Motor_T * p_motor);
extern void Motor_FOC_ProcAngleObserve(Motor_T * p_motor);
extern void Motor_FOC_ActivateOutput(Motor_T * p_motor);
extern void Motor_FOC_ActivateAngle(Motor_T * p_motor, qangle16_t angle, qfrac16_t vq, qfrac16_t vd);
extern void Motor_FOC_SetControlMode(Motor_T * p_motor, Motor_FeedbackMode_T mode);

/******************************************************************************/
/*!
	Voltage Current Feedback - Angle Control State
*/
/******************************************************************************/
/*
	Constant Voltage Mode
	input	RampCmd[-32768:32768]
	output	VqReq[-32768:32767]
*/
/*
	Scalar Voltage Mode
	input	RampCmd[0:65535] => Scalar
	output	VqReq[-32768:32767] = RampCmd / 65536 * VSpeed_Frac16 / 2
			VSpeed_Frac16 =< Speed_Frac16, to not exceed 1

	Overflow caution:
	Linear_Function(&p_motor->UnitsVSpeed) input range
*/
static inline void _Motor_FOC_ProcVoltageMode(Motor_T * p_motor, qfrac16_t vqReq)
{
	bool isOverLimit;
	qfrac16_t iLimit;
	qfrac16_t vqReqOut;

	if		(FOC_GetIq(&p_motor->Foc) > p_motor->VoltageModeILimitCcw_FracS16) 	{ iLimit = p_motor->VoltageModeILimitCcw_FracS16; isOverLimit = true; }
	else if	(FOC_GetIq(&p_motor->Foc) < p_motor->VoltageModeILimitCw_FracS16) 	{ iLimit = p_motor->VoltageModeILimitCw_FracS16; isOverLimit = true; }
	else 																		{ isOverLimit = false; iLimit = vqReq; }

	if((isOverLimit == true) && (p_motor->ControlFlags.VoltageModeILimitActive == false))
	{
		p_motor->ControlFlags.VoltageModeILimitActive = true;
		PID_SetOutputState(&p_motor->PidIq, FOC_GetVq(&p_motor->Foc));
	}
	else /* Alternatively remain set until manual reset */
	{
		p_motor->ControlFlags.VoltageModeILimitActive = false;
	}

	vqReqOut = (p_motor->ControlFlags.VoltageModeILimitActive == true) ? PID_Calc(&p_motor->PidIq, iLimit, FOC_GetIq(&p_motor->Foc)) : vqReq;

	FOC_SetVq(&p_motor->Foc, vqReqOut);
	FOC_SetVd(&p_motor->Foc, 0);
}

static inline void _Motor_FOC_ProcCurrentFeedback(Motor_T * p_motor)
{
	int32_t userOutput = (p_motor->FeedbackModeFlags.Speed == 1U) ? p_motor->SpeedControl_FracS16 : Linear_Ramp_GetOutput(&p_motor->Ramp);

	if(p_motor->FeedbackModeFlags.Current == 1U) /* Current Control mode - proc using last adc measure */
	{
		FOC_SetVq(&p_motor->Foc, PID_Calc(&p_motor->PidIq, userOutput, 	FOC_GetIq(&p_motor->Foc)));
		FOC_SetVd(&p_motor->Foc, PID_Calc(&p_motor->PidId, 0U, 			FOC_GetId(&p_motor->Foc)));
	}
	else /* Voltage Control mode - use current feedback for over current only */
	{
		// if(p_motor->FeedbackModeFlags.Scalar == 1U)
		// {
		// 	userOutput = Linear_Ramp_GetOutput(&p_motor->Ramp) * (Motor_GetVSpeedFrac16_VSpeed(p_motor) / 2) / 65536;
		// 	if		(userOutput > 32767) 	{ userOutput = 32767; }
		// 	else if	(userOutput < -32767) 	{ userOutput = -32767; }
		// }
		_Motor_FOC_ProcVoltageMode(p_motor, userOutput);
	}
}

/*!
	Match Feedback State to Output
	PID update when changing Control/FeedbackMode
	V output is prev VReq, match to start at 0 change in torque
	Iq PID output differs between voltage Iq limit mode and Iq control mode. still need to match
*/
/* FeedbackModeFlags.Scalar => Start from scalar of 1, output speed */
/* FeedbackModeFlags.Speed == 1, SPEED_CURRENT, or SPEED_VOLTAGE */
/* FeedbackModeFlags.Scalar == 0, CONSTANT_CURRENT, or CONSTANT_VOLTAGE, Open Loop */
/* does not set vd coming out of align */
static inline void Motor_FOC_MatchFeedbackLoop(Motor_T * p_motor)
{
	int32_t userOutput = (p_motor->FeedbackModeFlags.Current == 1U) ? FOC_GetIq(&p_motor->Foc) : FOC_GetVq(&p_motor->Foc);; /* q_sqrt(vd vq) */

	if		(p_motor->FeedbackModeFlags.Scalar == 1U) 	{ Linear_Ramp_SetOutputState(&p_motor->Ramp, 65535); }
	else if	(p_motor->FeedbackModeFlags.Speed == 1U) 	{ Linear_Ramp_SetOutputState(&p_motor->Ramp, p_motor->Speed_Frac16 / 2); Motor_SetSpeedOutput(p_motor, userOutput);}
	else 												{ Linear_Ramp_SetOutputState(&p_motor->Ramp, userOutput); }

	PID_SetIntegral(&p_motor->PidIq, FOC_GetVq(&p_motor->Foc));
	PID_SetIntegral(&p_motor->PidId, FOC_GetVd(&p_motor->Foc));
}

/******************************************************************************/
/*!
	State Machine
*/
/******************************************************************************/
#ifdef CONFIG_MOTOR_EXTERN_CONTROL_ENABLE
extern void Motor_ExternControl(Motor_T * p_motor);
#endif
/*
	Main Control Loop - Run State
	StateMachine calls each PWM, ~20kHz
*/
static inline void Motor_FOC_ProcAngleControl(Motor_T * p_motor)
{
	if((p_motor->ControlTimerBase & GLOBAL_MOTOR.CONTROL_ANALOG_DIVIDER) == 0UL)
	{
		AnalogN_Group_PauseQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_I);
		AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IA);
		AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IB);
#if defined(CONFIG_MOTOR_I_SENSORS_ABC)
		AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IC);
#endif
		AnalogN_Group_ResumeQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_I);
	}
	/* Samples chain completes sometime after queue resumes. ADC ISR priority higher than PWM. */
#ifdef CONFIG_MOTOR_EXTERN_CONTROL_ENABLE
	Motor_ExternControl(p_motor);
#endif
// /* ~10us */ Motor_Debug_CaptureTime(p_motor, 1U);
	Linear_Ramp_ProcOutput(&p_motor->Ramp);

	/* User request open loop support, implement outside */
	if(Motor_CheckPositionFeedback(p_motor) == true)
	{
		Motor_FOC_ProcPosition(p_motor);
		Motor_FOC_ProcSpeedFeedback(p_motor);
	}

// /* ~29 us */ Motor_Debug_CaptureTime(p_motor, 2U);
#if 	defined(CONFIG_MOTOR_I_SENSORS_AB)
	FOC_ProcClarkePark_AB(&p_motor->Foc);
#elif 	defined(CONFIG_MOTOR_I_SENSORS_ABC)
	FOC_ProcClarkePark(&p_motor->Foc);
#endif

// /* ~30us */	Motor_Debug_CaptureTime(p_motor, 3U);
	_Motor_FOC_ProcCurrentFeedback(p_motor); /* Set Vd Vq */

	FOC_ProcInvParkInvClarkeSvpwm(&p_motor->Foc);
	Phase_ActivateDuty(&p_motor->Phase, FOC_GetDutyA(&p_motor->Foc), FOC_GetDutyB(&p_motor->Foc), FOC_GetDutyC(&p_motor->Foc));

// /* ~37us */ Motor_Debug_CaptureTime(p_motor, 4U);
}

static inline void Motor_FOC_StartAlign(Motor_T * p_motor)
{
	// Motor_FOC_SetControlMode(p_motor, MOTOR_FEEDBACK_MODE_CONSTANT_CURRENT);
	Linear_Ramp_ZeroOutputState(&p_motor->Ramp);
	Linear_Ramp_SetSlope(&p_motor->Ramp, p_motor->Parameters.AlignTime_Cycles, 0, p_motor->Parameters.AlignVPwm_Frac16 / 2U);
	Linear_Ramp_SetTarget(&p_motor->Ramp, p_motor->Parameters.AlignVPwm_Frac16 / 2U);
}

static inline void Motor_FOC_ProcAlign(Motor_T * p_motor)
{
	Linear_Ramp_ProcOutput(&p_motor->Ramp);
	Motor_FOC_ActivateAngle(p_motor, 0, 0, Linear_Ramp_GetOutput(&p_motor->Ramp));
}

static inline void Motor_FOC_StartOpenLoop(Motor_T * p_motor)
{
	p_motor->OpenLoopSpeed_RPM = 0U;
	Linear_Ramp_ZeroOutputState(&p_motor->Ramp); /* start from 0, since 90 degrees ahead of align */
	Linear_Ramp_SetSlope(&p_motor->Ramp, p_motor->Parameters.RampAccel_Cycles, 0, p_motor->Parameters.OpenLoopVPwm_Frac16 / 2U);
	Linear_Ramp_SetTarget(&p_motor->Ramp, p_motor->Parameters.OpenLoopVPwm_Frac16 / 2U);
}

/*
	OpenLoop
	Blind input angle, constant voltage
	ElectricalAngle => integrate speed to angle
*/
static inline void _Motor_FOC_ProcOpenLoop(Motor_T * p_motor)
{
	p_motor->OpenLoopSpeed_RPM = Linear_Ramp_CalcOutput(&p_motor->OpenLoopRamp, p_motor->OpenLoopSpeed_RPM);
	p_motor->ElectricalAngle += (p_motor->OpenLoopSpeed_RPM << 16U) / (60U * GLOBAL_MOTOR.CONTROL_FREQ);
	// p_motor->Speed_Frac16 = p_motor->OpenLoopSpeed_RPM * 65535 / p_motor->Parameters.SpeedFeedbackRef_Rpm; /* temp convert to frac 16 and back again  for user read */
}

static inline void Motor_FOC_ProcOpenLoop(Motor_T * p_motor)
{
	_Motor_FOC_ProcOpenLoop(p_motor);
	/* proc current, or use AngleControl */ /* todo OpenLoop with current */
	Linear_Ramp_ProcOutput(&p_motor->Ramp);
	Motor_FOC_ActivateAngle(p_motor, p_motor->ElectricalAngle, Linear_Ramp_GetOutput(&p_motor->Ramp), 0);
}

static inline void Motor_FOC_ProcStop(Motor_T * p_motor)
{
	AnalogN_Group_PauseQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_I);
	AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IA);
	AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IB);
#if defined(CONFIG_MOTOR_I_SENSORS_ABC)
	AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IC);
#endif
	AnalogN_Group_ResumeQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_I);

	Motor_FOC_ProcAngleObserve(p_motor);

#if 	defined(CONFIG_MOTOR_I_SENSORS_AB)
	FOC_ProcClarkePark_AB(&p_motor->Foc);
#elif 	defined(CONFIG_MOTOR_I_SENSORS_ABC)
	FOC_ProcClarkePark(&p_motor->Foc);
#endif
}

/* Also Clears Iq for OpenLoop/Align */
static inline void Motor_FOC_SetOutputMatchStop(Motor_T * p_motor)
{
	FOC_SetIq(&p_motor->Foc, 0);
	FOC_SetVq(&p_motor->Foc, 0);
	FOC_SetVd(&p_motor->Foc, 0);
	p_motor->Speed_Frac16 = 0;
}

/*
	From FreeWheel State, match to speed/bemf
*/
static inline void Motor_FOC_SetOutputMatchFreewheel(Motor_T * p_motor)
{
	int32_t vqReq;
	// vqReq = Motor_GetVSpeedFrac16_VBemf(p_motor) / 2;
	vqReq = Motor_GetVSpeedFrac16_VSpeed(p_motor) / 2;
	FOC_SetIq(&p_motor->Foc, 0);
	FOC_SetVq(&p_motor->Foc, vqReq);
	FOC_SetVd(&p_motor->Foc, 0);
}

/******************************************************************************/
/*!
	Extern
*/
/******************************************************************************/
extern void Motor_FOC_StartAlign(Motor_T * p_motor);
extern void Motor_FOC_ProcAlign(Motor_T * p_motor);
extern void Motor_FOC_StartOpenLoop(Motor_T * p_motor);
extern void Motor_FOC_ProcOpenLoop(Motor_T * p_motor);
extern void Motor_FOC_ProcStop(Motor_T * p_motor);

extern void Motor_FOC_SetDirectionCcw(Motor_T * p_motor);
extern void Motor_FOC_SetDirectionCw(Motor_T * p_motor);
extern void Motor_FOC_SetDirection(Motor_T * p_motor, Motor_Direction_T direction);
extern void Motor_FOC_SetDirectionForward(Motor_T * p_motor);

#endif
