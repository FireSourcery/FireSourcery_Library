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
	@version V0
*/
/******************************************************************************/
#ifndef MOTOR_FOC_H
#define MOTOR_FOC_H

#include "Motor.h"

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

static inline void Motor_FOC_CaptureVa(Motor_T * p_motor)
{
	qfrac16_t vphase = ((int32_t)Linear_Voltage_CalcFracS16(&p_motor->UnitsVabc, p_motor->AnalogResults.Va_Adcu) + FOC_GetVBemfA(&p_motor->Foc)) / 2;
	FOC_SetVBemfA(&p_motor->Foc, vphase);
}

static inline void Motor_FOC_CaptureVb(Motor_T * p_motor)
{
	qfrac16_t vphase = ((int32_t)Linear_Voltage_CalcFracS16(&p_motor->UnitsVabc, p_motor->AnalogResults.Vb_Adcu) + FOC_GetVBemfB(&p_motor->Foc)) / 2;
	FOC_SetVBemfB(&p_motor->Foc, vphase);
}

static inline void Motor_FOC_CaptureVc(Motor_T * p_motor)
{
	qfrac16_t vphase = ((int32_t)Linear_Voltage_CalcFracS16(&p_motor->UnitsVabc, p_motor->AnalogResults.Vc_Adcu) + FOC_GetVBemfC(&p_motor->Foc)) / 2;
	FOC_SetVBemfC(&p_motor->Foc, vphase);
}

/******************************************************************************/
/*!

*/
/******************************************************************************/
static inline uint32_t Motor_FOC_GetIMagnitude_Frac16(Motor_T * p_motor) 		{ return (uint32_t)FOC_GetIMagnitude(&p_motor->Foc) * 2U; }
static inline uint32_t Motor_FOC_GetIMagnitudeClarke_Frac16(Motor_T * p_motor) 	{ return (uint32_t)FOC_GetIMagnitude_Clarke(&p_motor->Foc) * 2U; }
static inline uint32_t Motor_FOC_GetVBemfMagnitude_Frac16(Motor_T * p_motor) 	{ return (uint32_t)FOC_GetVBemfMagnitude(&p_motor->Foc) * 2U; }

/* In User Reference Sign */
static inline int32_t Motor_FOC_GetIMagnitudeUser_Frac16(Motor_T * p_motor)
{
	int32_t iPhase = Motor_FOC_GetIMagnitude_Frac16(p_motor);
	return (p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW) ? iPhase : 0 - iPhase;
}

static inline int32_t Motor_FOC_GetVBemfMagnitudeUser_Frac16(Motor_T * p_motor)
{
	int32_t vPhase = Motor_FOC_GetVBemfMagnitude_Frac16(p_motor);
	return (p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW) ? vPhase : 0 - vPhase;
}

/******************************************************************************/
/*!
	Check limits on set
*/
/******************************************************************************/
static inline bool Motor_FOC_CheckIOverThreshold(Motor_T * p_motor)
{
	return !math_isbound(FOC_GetIq(&p_motor->Foc), (int32_t)p_motor->ILimitCw_FracS16 * 7 / 8, (int32_t)p_motor->ILimitCcw_FracS16 * 7 / 8);
}

/******************************************************************************/
/*!
	State Machine Helpers
*/
/******************************************************************************/
extern void Motor_FOC_ProcAngleObserve(Motor_T * p_motor);
extern void Motor_FOC_ProcAngleControl(Motor_T * p_motor);

/******************************************************************************/
/*!
	StateMachine mapping
	defined as inline for StateMachine wrapper functions
*/
/******************************************************************************/
static inline void Motor_FOC_StartAlign(Motor_T * p_motor)
{
	Linear_Ramp_Set(&p_motor->AuxRamp, p_motor->Parameters.AlignTime_Cycles, 0, p_motor->Parameters.AlignVPwm_Frac16 / 2U);
	Motor_SetControlFeedbackOpenLoopCurrent(p_motor);
	FOC_SetIVqReq(&p_motor->Foc, 0);
	FOC_SetTheta(&p_motor->Foc, 0);
}

static inline void Motor_FOC_ProcAlign(Motor_T * p_motor)
{
	Linear_Ramp_ProcOutput(&p_motor->AuxRamp);
	FOC_SetIVdReq(&p_motor->Foc, Linear_Ramp_GetOutput(&p_motor->AuxRamp));
	Motor_FOC_ProcAngleControl(p_motor);
}

static inline void Motor_FOC_StartAlignValidate(Motor_T * p_motor)
{
	p_motor->ControlFeedbackMode.OpenLoop = 0U;
	Motor_ZeroSensorAlign(p_motor);
	Motor_ZeroSensor(p_motor);
	FOC_SetIVdReq(&p_motor->Foc, 0U);
}

static inline void Motor_FOC_StartOpenLoop(Motor_T * p_motor)
{
	p_motor->Speed_Frac16 = 0;
	Linear_Ramp_Set(&p_motor->AuxRamp, p_motor->Parameters.RampAccel_Cycles, 0, _Motor_ConvertDirectionalCmd(p_motor, p_motor->Parameters.OpenLoopVPwm_Frac16 / 2U));
	FOC_SetIVdReq(&p_motor->Foc, 0);
	//alternatively, clamp user input ramp
}


/*
	OpenLoop
	Blind input angle, constant voltage
	ElectricalAngle => integrate speed to angle
*/
static inline void _Motor_FOC_ProcOpenLoopSpeed(Motor_T * p_motor)
{
	// p_motor->OpenLoopSpeed_RPM = Linear_Ramp_CalcOutput(&p_motor->OpenLoopSpeedRamp, p_motor->OpenLoopSpeed_RPM);
	// p_motor->ElectricalAngle += (p_motor->OpenLoopSpeed_RPM << 16U) / (60U * GLOBAL_MOTOR.CONTROL_FREQ);

	/* OpenLoopSpeed_RPM = Speed_Frac16 / 65535 * SpeedFeedbackRef_Rpm */
	p_motor->Speed_Frac16 = Linear_Ramp_CalcOutput(&p_motor->OpenLoopSpeedRamp, p_motor->Speed_Frac16);
	/* (OpenLoopSpeed_RPM * 65536) / (60 * CONTROL_FREQ) */
	p_motor->ElectricalAngle += (p_motor->Speed_Frac16 * p_motor->Parameters.SpeedFeedbackRef_Rpm) / (60U * GLOBAL_MOTOR.CONTROL_FREQ);
}

static inline void Motor_FOC_ProcOpenLoop(Motor_T * p_motor)
{
	_Motor_FOC_ProcOpenLoopSpeed(p_motor);
	Linear_Ramp_ProcOutput(&p_motor->AuxRamp);
	FOC_SetIVqReq(&p_motor->Foc, Linear_Ramp_GetOutput(&p_motor->AuxRamp));
	Motor_FOC_ProcAngleControl(p_motor);
	// Motor_FOC_ActivateAngle(p_motor, p_motor->ElectricalAngle, Linear_Ramp_GetOutput(&p_motor->AuxRamp), 0);
}

static inline void Motor_FOC_SetOpenLoopMagnitude(Motor_T * p_motor, int32_t qMagnitude)
{
	// Linear_Ramp_SetTarget(&p_motor->Ramp, qMagnitude);
}

static inline void Motor_FOC_SetOpenLoopSpeed(Motor_T * p_motor, int32_t speed_Frac16)
{
	// Linear_Ramp_SetTarget(&p_motor->OpenLoopSpeedRamp, speed_Frac16);
}

static inline void Motor_FOC_ProcStop(Motor_T * p_motor)
{
	Motor_FOC_ProcAngleObserve(p_motor);
}

/* Clear State for SetFeedbackMatch */
static inline void Motor_FOC_ClearState(Motor_T * p_motor)
{
	FOC_ClearState(&p_motor->Foc);
}

/*
	From FreeWheel State, match to speed/bemf
*/
static inline void Motor_FOC_SetVSpeed(Motor_T * p_motor)
{
	// FOC_ProcVBemfClarkePark(&p_motor->Foc);
	int32_t vqReq = Linear_Function_FracS16(&p_motor->UnitsVSpeed, p_motor->Speed_Frac16);;
	FOC_SetIq(&p_motor->Foc, 0);
	FOC_SetId(&p_motor->Foc, 0);
	FOC_SetVq(&p_motor->Foc, vqReq);
	FOC_SetVd(&p_motor->Foc, 0);
}

/******************************************************************************/
/*!
	Extern
*/
/******************************************************************************/
extern void Motor_FOC_ActivateOutput(Motor_T * p_motor);
extern void Motor_FOC_ActivateAngle(Motor_T * p_motor, qangle16_t angle, qfrac16_t vq, qfrac16_t vd);
// extern void Motor_FOC_SetControlMode(Motor_T * p_motor, Motor_FeedbackMode_T mode);
extern void Motor_FOC_SetFeedbackMatch(Motor_T * p_motor);

extern void Motor_FOC_SetDirectionCcw(Motor_T * p_motor);
extern void Motor_FOC_SetDirectionCw(Motor_T * p_motor);
extern void Motor_FOC_SetDirection(Motor_T * p_motor, Motor_Direction_T direction);
extern void Motor_FOC_SetDirectionForward(Motor_T * p_motor);

#endif
