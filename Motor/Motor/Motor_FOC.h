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
	qfrac16_t iphase = ((int32_t)Linear_ADC_CalcFracS16(&p_motor->UnitIa, p_motor->AnalogResults.Ia_Adcu) + FOC_GetIa(&p_motor->Foc)) / 2;
	FOC_SetIa(&p_motor->Foc, iphase);
}

static inline void Motor_FOC_CaptureIb(Motor_T * p_motor)
{
	qfrac16_t iphase = ((int32_t)Linear_ADC_CalcFracS16(&p_motor->UnitIb, p_motor->AnalogResults.Ib_Adcu) + FOC_GetIb(&p_motor->Foc)) / 2;
	FOC_SetIb(&p_motor->Foc, iphase);
}

static inline void Motor_FOC_CaptureIc(Motor_T * p_motor)
{
	qfrac16_t iphase = ((int32_t)Linear_ADC_CalcFracS16(&p_motor->UnitIc, p_motor->AnalogResults.Ic_Adcu) + FOC_GetIc(&p_motor->Foc)) / 2;
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
*/
/******************************************************************************/
// static inline void _Motor_FOC_ProcPositionFeedback(Motor_T * p_motor)
// {
// 	bool procSpeed = Timer_Periodic_Poll(&p_motor->SpeedTimer);
// 	qangle16_t electricalAngle; /* FracU16 [0, 65535] map negative portions of qangle16_t */
// 	int32_t speedFeedback_Frac16;

// 	switch(p_motor->Parameters.SensorMode)
// 	{
// 		case MOTOR_SENSOR_MODE_HALL:
// #if defined(CONFIG_MOTOR_HALL_MODE_POLLING) /* todo fix */
// 			if(Hall_PollCaptureRotorAngle(&p_motor->Hall) == true) { Encoder_CapturePulse(&p_motor->Encoder); }
// #endif
// 			electricalAngle = Hall_GetRotorAngle_Degrees16(&p_motor->Hall);
// 			electricalAngle += Encoder_ModeDT_InterpolateAngle(&p_motor->Encoder);
// 			if(procSpeed == true) { speedFeedback_Frac16 = Encoder_ModeDT_PollScalarVelocity(&p_motor->Encoder); }
// 			break;

// 		case MOTOR_SENSOR_MODE_ENCODER:
// 			electricalAngle = Motor_GetEncoderElectricalAngle(p_motor);
// 			electricalAngle += Encoder_ModeDT_InterpolateAngle(&p_motor->Encoder);
// 			if(procSpeed == true) { speedFeedback_Frac16 = Encoder_ModeDT_PollScalarVelocity(&p_motor->Encoder); } 	/* Quadrature capture is signed */
// 			break;

// #if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
// 		case MOTOR_SENSOR_MODE_SIN_COS:
// 			SinCos_CaptureAngle(&p_motor->SinCos, p_motor->AnalogResults.Sin_Adcu, p_motor->AnalogResults.Cos_Adcu);
// 			electricalAngle = SinCos_GetElectricalAngle(&p_motor->SinCos);
// 			if(procSpeed == true) { speedFeedback_Frac16 = _Motor_FOC_CaptureAngleSpeed(p_motor, SinCos_GetMechanicalAngle(&p_motor->SinCos)); }
// 			/* Using position form 50us ago */
// 			//todo group
// 			AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_SIN);
// 			AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_COS);
// 			break;
// #endif
// #if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
// 		case MOTOR_SENSOR_MODE_SENSORLESS:
// 			//todo observer
// 			electricalAngle = 0;
// 			speedFeedback_Frac16 = 0;
// 			p_motor->FeedbackModeFlags.OpenLoop = 1U;
// 			p_motor->FeedbackModeFlags.OpenLoop = 1U;
// 			p_motor->ControlFlags.SensorFeedback = 0U;
// 			p_motor->ControlFlags.SensorFeedback = 0U;
// 			break;
// #endif
// 		default:
// 			electricalAngle = 0;
// 			speedFeedback_Frac16 = 0;
// 			break;
// 	}

// 	if(procSpeed == true)
// 	{
// 		speedFeedback_Frac16 = (speedFeedback_Frac16 + p_motor->SpeedFeedback_Frac16) / 2;
// 		/*
// 			Speed Feedback Loop
// 				SpeedControl_FracS16 update ~1000Hz, Ramp input 1000Hz, RampCmd output 20000Hz
// 			input	RampCmd[-32767:32767] - (speedFeedback_Frac16 / 2)[-32767:32767]
// 			output 	SpeedControl_FracS16[-32767:32767] => IqReq or VqReq
// 		*/
// 		/* PID procs during stop state */
// 		if(p_motor->FeedbackModeFlags.Speed == 1U) { p_motor->SpeedControl_FracS16 = PID_Calc(&p_motor->PidSpeed, p_motor->RampCmd, speedFeedback_Frac16 / 2); };
// 		p_motor->SpeedFeedback_Frac16 = speedFeedback_Frac16;
// 	}

// 	/* Once Per Cycle */
// 	// if(p_motor->ElectricalAngle < 0 && electricalAngle > 0)
// 	if(((p_motor->ElectricalAngle ^ electricalAngle) & 0x8000U) != (uint16_t)0U)
// 	{
// 		p_motor->VBemfPeak_Adcu = p_motor->VBemfPeakTemp_Adcu;
// 		p_motor->VBemfPeakTemp_Adcu = 0U;
// 		p_motor->IPhasePeak_Adcu = p_motor->IPhasePeakTemp_Adcu;
// 		p_motor->IPhasePeakTemp_Adcu = 0U;
// 	}

// 	p_motor->ElectricalAngle = electricalAngle;
// }


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

/******************************************************************************/
/*!
	Voltage Current Feedback
*/
/******************************************************************************/
/*
	Constant Voltage Mode
	input	RampCmd[-32768:32768]
	output	VqReq[-32768:32767] => RampCmd
*/
/*
	Scalar Voltage Mode
	input	RampCmd[0:65535] == Scalar
	output	VqReq[-32768:32767] => Scalar * SpeedFeedback

	SpeedVMatchRatio = SpeedFeedback_Frac16 * SpeedFeedbackRef_Rpm / SpeedVMatchRef_Rpm / 2;
	SpeedVMatchRatio = Linear_Function(&p_motor->SpeedVMatchRatio, p_motor->SpeedFeedback_Frac16) / 2;
	vqReq = RampCmd * SpeedVMatchRatio / 65536;

	SpeedFeedback_Frac16 unsaturated ~[-65535:65535]

	Overflow caution:
	Linear_Function(&p_motor->SpeedVMatchRatio) input range [-65535*3/2:65535*3/2], when VMatchRef == FeedbackRef * 3 / 4
*/
static inline void _Motor_FOC_ProcVoltageMode(Motor_T * p_motor, qfrac16_t vqReq)
{
	bool isOverLimit;
	qfrac16_t vqReqOut;
	qfrac16_t iLimit;

	if		(FOC_GetIq(&p_motor->Foc) > p_motor->VoltageModeILimitCcw_FracS16) 	{ iLimit = p_motor->VoltageModeILimitCcw_FracS16; isOverLimit = true; }
	else if	(FOC_GetIq(&p_motor->Foc) < p_motor->VoltageModeILimitCw_FracS16) 	{ iLimit = p_motor->VoltageModeILimitCw_FracS16; isOverLimit = true; }
	else 																		{ isOverLimit = false; iLimit = vqReq; }

	if((isOverLimit == true) && (p_motor->ControlFlags.VoltageModeILimitActive == false))
	{
		p_motor->ControlFlags.VoltageModeILimitActive = true;
		PID_SetOutputState(&p_motor->PidIq, FOC_GetVq(&p_motor->Foc));
	}
	else /* alternatively remain set until manual reset */
	{
		p_motor->ControlFlags.VoltageModeILimitActive = false;
	}

	vqReqOut = (p_motor->ControlFlags.VoltageModeILimitActive == true) ? PID_Calc(&p_motor->PidIq, iLimit, FOC_GetIq(&p_motor->Foc)) : vqReq;

	FOC_SetVq(&p_motor->Foc, vqReqOut);
	FOC_SetVd(&p_motor->Foc, 0);
}

static inline void _Motor_FOC_ProcCurrentFeedback(Motor_T * p_motor)
{
	int32_t userOutput = (p_motor->FeedbackModeFlags.Speed == 1U) ? p_motor->SpeedControl_FracS16 : p_motor->RampCmd;

	if(p_motor->FeedbackModeFlags.Current == 1U) /* Current Control mode - proc using last adc measure */
	{
		FOC_SetVq(&p_motor->Foc, PID_Calc(&p_motor->PidIq, userOutput, 	FOC_GetIq(&p_motor->Foc)));
		FOC_SetVd(&p_motor->Foc, PID_Calc(&p_motor->PidId, 0U, 			FOC_GetId(&p_motor->Foc)));
	}
	else /* Voltage Control mode - use current feedback for over current only */
	{
		if(p_motor->FeedbackModeFlags.Scalar == 1U)
		{
			userOutput = p_motor->RampCmd * (Linear_Function(&p_motor->SpeedVMatchRatio, p_motor->SpeedFeedback_Frac16 / 2)) / 65536; /* change to linear_frac16 */
			if		(userOutput > 32767) 	{ userOutput = 32767; }
			else if	(userOutput < -32767) 	{ userOutput = -32767; }
		}
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

	if		(p_motor->FeedbackModeFlags.Scalar == 1U) 	{ Motor_SetRampOutput(p_motor, 65535); }
	else if	(p_motor->FeedbackModeFlags.Speed == 1U) 	{ Motor_SetRampOutput(p_motor, p_motor->SpeedFeedback_Frac16 / 2); Motor_SetSpeedOutput(p_motor, userOutput);}
	else 												{ Motor_SetRampOutput(p_motor, userOutput); }

	PID_SetIntegral(&p_motor->PidIq, FOC_GetVq(&p_motor->Foc));
	PID_SetIntegral(&p_motor->PidId, FOC_GetVd(&p_motor->Foc));
}


/******************************************************************************/
/*!
	State Machine
*/
/******************************************************************************/
/*
	Main Control Loop - Run State
	StateMachine calls each PWM, ~20kHz
*/
static inline void Motor_FOC_ProcAngleControl(Motor_T * p_motor)
{
	qangle16_t electricalAngle;

	if(((p_motor->ControlTimerBase & GLOBAL_MOTOR.CONTROL_ANALOG_DIVIDER)) == 0UL)
	{
		AnalogN_Group_PauseQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_I);
		AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IA);
		AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IB);
	#if defined(CONFIG_MOTOR_I_SENSORS_ABC)
		AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IC);
	#endif
		AnalogN_Group_ResumeQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_I);
	}
	/* Samples chain completes sometime after queue resumes. ADC isr priority higher than PWM. */

// /* ~10us */ Motor_Debug_CaptureTime(p_motor, 1U);
	Motor_ProcRamp(p_motor);

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

/* ~37us */ Motor_Debug_CaptureTime(p_motor, 4U);
}

static inline void Motor_FOC_StartAlign(Motor_T * p_motor)
{
	Motor_ZeroRamp(p_motor);
	Motor_SetRampSlope_Ticks(p_motor, p_motor->Parameters.AlignTime_Cycles, 0, p_motor->Parameters.AlignVPwm_Frac16 / 2U);
	Motor_SetRampTarget(p_motor, p_motor->Parameters.AlignVPwm_Frac16 / 2U);
}

static inline void Motor_FOC_ProcAlign(Motor_T * p_motor)
{
	Motor_ProcRamp(p_motor);
	Motor_FOC_ActivateAngle(p_motor, 0, 0, p_motor->RampCmd);
}

static inline void Motor_FOC_StartOpenLoop(Motor_T * p_motor)
{
	p_motor->OpenLoopSpeed_RPM = 0U;
	// todo  Motor_SetRampTarget_Millis
	// Linear_Ramp_SetSlope_Ticks(&p_motor->VPwmRamp,  p_motor->Parameters.RampAccel_Cycles , 0, p_motor->Parameters.OpenLoopVPwm_Frac16 / 2U);
	Motor_ZeroRamp(p_motor); /* start from 0, since 90 degrees ahead of align */
	Motor_SetRampSlope_Ticks(p_motor, p_motor->Parameters.RampAccel_Cycles, 0, p_motor->Parameters.OpenLoopVPwm_Frac16 / 2U);
	Motor_SetRampTarget(p_motor, p_motor->Parameters.OpenLoopVPwm_Frac16 / 2U);
}

/*
	OpenLoop
	Blind input angle, constant voltage
	ElectricalAngle => integrate speed to angle
*/
static inline void _Motor_FOC_ProcOpenLoop(Motor_T * p_motor)
{
	p_motor->OpenLoopSpeed_RPM = Linear_Ramp_CalcCmdNextOutput(&p_motor->OpenLoopRamp, p_motor->OpenLoopSpeed_RPM);
	p_motor->ElectricalAngle += (p_motor->OpenLoopSpeed_RPM << 16U) / (60U * GLOBAL_MOTOR.CONTROL_FREQ);
	// p_motor->SpeedFeedback_Frac16 = p_motor->OpenLoopSpeed_RPM * 65535 / p_motor->Parameters.SpeedFeedbackRef_Rpm; /* temp convert to frac 16 and back again  for user read */
}

static inline void Motor_FOC_ProcOpenLoop(Motor_T * p_motor)
{
	_Motor_FOC_ProcOpenLoop(p_motor);
	/* proc current, or use AngleControl */ /* todo OpenLoop with current */
	Motor_ProcRamp(p_motor);
	Motor_FOC_ActivateAngle(p_motor, p_motor->ElectricalAngle, p_motor->RampCmd, 0);
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
}

/*
	From FreeWheel State, match to speed/bemf
*/
static inline void Motor_FOC_SetOutputMatchFreewheel(Motor_T * p_motor)
{
	int32_t vqReq;
	/*
		Match to Bemf
		Captured VBemfPeak always positive
		todo check bemf capture available.
	*/
	// vqReq = Linear_ADC_CalcFracS16(&p_motor->CONFIG.UNIT_V_ABC, p_motor->VBemfPeak_Adcu);

	/*
		Match to Speed
		User sets larger SpeedVMatchRef_Rpm to ensure not resume to higher speed
		vqReq = p_motor->SpeedFeedback_Frac16 * p_motor->Parameters.SpeedFeedbackRef_Rpm / p_motor->Parameters.SpeedVMatchRef_Rpm / 2;
			SpeedVMatch_Factor = SpeedFeedbackRef_Rpm << 14 / SpeedVMatchRef_Rpm
			vqReq = (p_motor->SpeedFeedback_Frac16 * SpeedVMatch_Factor >> 14) / 2
	*/
	vqReq = Linear_Function(&p_motor->SpeedVMatchRatio, p_motor->SpeedFeedback_Frac16 / 2);
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
