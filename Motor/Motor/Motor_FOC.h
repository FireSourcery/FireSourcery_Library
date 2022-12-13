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
			FOC mode State Machine mapping
			defined as inline for StateMachine wrapper functions
	@version V0
*/
/******************************************************************************/
#ifndef MOTOR_FOC_H
#define MOTOR_FOC_H

#include "Motor.h"

/******************************************************************************/
/*
	+/- Sign indicates absolute direction, CW/CCW. NOT along or against direction selected.
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
	qfrac16_t i_temp = ((int32_t)Linear_ADC_CalcFractionSigned16(&p_motor->UnitIa, p_motor->AnalogResults.Ia_Adcu) + (int32_t)FOC_GetIa(&p_motor->Foc)) / 2;
	FOC_SetIa(&p_motor->Foc, i_temp);
}

static inline void Motor_FOC_CaptureIb(Motor_T * p_motor)
{
	qfrac16_t i_temp = ((int32_t)Linear_ADC_CalcFractionSigned16(&p_motor->UnitIb, p_motor->AnalogResults.Ib_Adcu) + (int32_t)FOC_GetIb(&p_motor->Foc)) / 2;
	FOC_SetIb(&p_motor->Foc, i_temp);
}

static inline void Motor_FOC_CaptureIc(Motor_T * p_motor)
{
	qfrac16_t i_temp = ((int32_t)Linear_ADC_CalcFractionSigned16(&p_motor->UnitIc, p_motor->AnalogResults.Ic_Adcu) + (int32_t)FOC_GetIc(&p_motor->Foc)) / 2;
	FOC_SetIc(&p_motor->Foc, i_temp);
}

static inline void _Motor_FOC_CaptureHall(Motor_T * p_motor)
{
	Encoder_DeltaT_Capture(&p_motor->Encoder);
	Encoder_DeltaT_CaptureExtended(&p_motor->Encoder);
	p_motor->HallAngle = (qangle16_t)Hall_GetRotorAngle_Degrees16(&p_motor->Hall);
	p_motor->InterpolatedAngleIndex = 1U;
}

static inline void Motor_FOC_CaptureHall_ISR(Motor_T * p_motor)
{
	Hall_CaptureSensors_ISR(&p_motor->Hall);
	_Motor_FOC_CaptureHall(p_motor);
}

/******************************************************************************/
/*!
	Private
*/
/******************************************************************************/
/* For SinCos, Sensorless, when not using Encoder module */
static inline int32_t _Motor_FOC_AngleSpeed(Motor_T * p_motor, qangle16_t speedAngle)
{
	int32_t speedDelta = speedAngle - p_motor->SpeedAngle; /* loops if no overflow past 1 full cycle */
	int32_t speedFeedback_Frac16 = (p_motor->SpeedFeedback_Frac16 + Linear_Speed_CalcAngleRpmFrac16(&p_motor->UnitAngleRpm, speedDelta)) / 2;
		// if(p_motor->Direction == MOTOR_DIRECTION_CW) { speedDelta = 0 - speedDelta; } //alraedy signed?
	p_motor->SpeedAngle = speedAngle; /* mechanical angle */
	return speedFeedback_Frac16;
}

/*
	OpenLoop
	Blind input angle, constant voltage
	p_foc->Theta = integrate speed to angle
*/
static inline void _Motor_FOC_ProcOpenLoop(Motor_T * p_motor)
{
	p_motor->OpenLoopSpeed_RPM = Linear_Ramp_ProcIndexOutput(&p_motor->OpenLoopRamp, &p_motor->OpenLoopRampIndex, p_motor->OpenLoopSpeed_RPM);
	// p_motor->ElectricalAngle = p_motor->ElectricalAngle + Encoder_Motor_ConvertMechanicalRpmToElectricalDelta(&p_motor->Encoder, p_motor->OpenLoopSpeed_RPM);
	p_motor->ElectricalAngle = p_motor->ElectricalAngle + Linear_Speed_CalcRpmAngle(&p_motor->UnitAngleRpm, p_motor->OpenLoopSpeed_RPM);
	p_motor->SpeedFeedback_Frac16 = p_motor->OpenLoopSpeed_RPM * 65535 / p_motor->Parameters.SpeedFeedbackRef_Rpm; /* temp convert to frac 16 and back again */
	// p_motor->OpenLoopVPwm = p_motor->RampCmd; proportional and bounded
	FOC_SetVq(&p_motor->Foc, p_motor->Parameters.OpenLoopVPwm / 2U);
}

static inline void _Motor_FOC_ProcPositionFeedback(Motor_T * p_motor)
{
	bool procSpeed = Timer_Periodic_Poll(&p_motor->SpeedTimer);
	uint32_t electricalDelta;
	qangle16_t electricalAngle;
	int32_t speedFeedback_Frac16 = 0U;

	switch(p_motor->Parameters.SensorMode)
	{
// #if defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
// 		case MOTOR_SENSOR_MODE_OPEN_LOOP: /* Sensor mode set indicates OpenLoop Only */
// 			_Motor_FOC_ProcOpenLoop(p_motor);
// 			procSpeed = false;
// 			electricalAngle = p_motor->ElectricalAngle;
// 			speedFeedback_Frac16 = p_motor->SpeedFeedback_Frac16;
// 			break;
// #endif
		case MOTOR_SENSOR_MODE_HALL:
#if defined(CONFIG_MOTOR_HALL_MODE_POLLING)
			if(Hall_PollCaptureSensors(&p_motor->Hall) == true)
			{
				_Motor_FOC_CaptureHall(p_motor);
				electricalAngle = p_motor->HallAngle;
			}
			else
#endif
			{
				// Encoder_Motor_InterpolateElectricalDelta_Bounded(&p_motor->Encoder, &p_motor->InterpolatedAngleIndex);
				electricalDelta = Encoder_Motor_InterpolateElectricalDelta(&p_motor->Encoder, p_motor->InterpolatedAngleIndex);
				if(electricalDelta > 65536U / 6U) { electricalDelta = 65536U / 6U; }
				if(p_motor->Direction == MOTOR_DIRECTION_CW) { electricalDelta = 0 - electricalDelta; };

				electricalAngle = p_motor->HallAngle + electricalDelta;
				p_motor->InterpolatedAngleIndex++;
			}

			if(procSpeed == true) { speedFeedback_Frac16 = Encoder_DeltaT_GetScalarSpeed_WatchStop(&p_motor->Encoder); }
			// speedFeedback_Frac16 = _Motor_FOC_CaptureAngleSpeed(p_motor, electricalAngle);
			break;

		case MOTOR_SENSOR_MODE_ENCODER:
			/*  */
			if(procSpeed == true)
			{
				Encoder_DeltaD_Capture(&p_motor->Encoder); /* speed */
				speedFeedback_Frac16 = Encoder_DeltaD_GetScalarSpeed(&p_motor->Encoder);
			}
			/* Encoder_Motor_GetElectricalTheta returns [0, 65535] maps directly to negative portions of qangle16_t */
			electricalAngle = (qangle16_t)Encoder_Motor_GetElectricalTheta(&p_motor->Encoder);
			break;

#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
		case MOTOR_SENSOR_MODE_SIN_COS:
			SinCos_CaptureAngle(&p_motor->SinCos, p_motor->AnalogResults.Sin_Adcu, p_motor->AnalogResults.Cos_Adcu);
			electricalAngle = SinCos_GetElectricalAngle(&p_motor->SinCos);
			if(procSpeed == true) { speedFeedback_Frac16 = _Motor_FOC_CaptureAngleSpeed(p_motor, SinCos_GetMechanicalAngle(&p_motor->SinCos)); }
			/* Using position form 50us ago */
			//todo group
			AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_SIN);
			AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_COS);
			break;
#endif
#if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
		case MOTOR_SENSOR_MODE_SENSORLESS:
			//todo observer
			electricalAngle = 0;
			speedFeedback_Frac16 = 0;
			p_motor->FeedbackModeFlags.OpenLoop = 1U;
			p_motor->FeedbackModeFlags.OpenLoop = 1U;
			p_motor->ControlFlags.SensorFeedback = 0U;
			p_motor->ControlFlags.SensorFeedback = 0U;
			break;
#endif
		default:
			electricalAngle = 0;
			speedFeedback_Frac16 = 0;
			break;
	}

	if(procSpeed == true)
	{
		if(p_motor->Direction == MOTOR_DIRECTION_CW) { speedFeedback_Frac16 = 0 - speedFeedback_Frac16; };
		speedFeedback_Frac16 = (speedFeedback_Frac16 + p_motor->SpeedFeedback_Frac16) / 2;
		/*
			Speed Feedback Loop
				SpeedControl update 1000Hz, Ramp input 1000Hz, RampCmd output 20000Hz, alternatively use RampIndex += 20?
			input	RampCmd[-32767:32767] - (speedFeedback_Frac16 / 2)[-32767:32767]
			output 	SpeedControl[-32767:32767] => IqReq or VqReq
		*/
		if(p_motor->FeedbackModeFlags.Speed == 1U) { p_motor->SpeedControl = PID_Calc(&p_motor->PidSpeed, p_motor->RampCmd, speedFeedback_Frac16 / 2); };
		p_motor->SpeedFeedback_Frac16 = speedFeedback_Frac16;
	}

	/* Once Per Cycle */
	// if(p_motor->ElectricalAngle < 0 && electricalAngle > 0)
	if(((p_motor->ElectricalAngle ^ electricalAngle) & 0x8000U) != (uint16_t)0U)
	{
		p_motor->VBemfPeak_Adcu = p_motor->VBemfPeakTemp_Adcu;
		p_motor->VBemfPeakTemp_Adcu = 0U;

		p_motor->IPhasePeak_Adcu = p_motor->IPhasePeakTemp_Adcu;
		p_motor->IPhasePeakTemp_Adcu = 0U;
	}

	p_motor->ElectricalAngle = electricalAngle; /* Save for output */
}

static inline void _Motor_FOC_ProcVoltageMode(Motor_T * p_motor, qfrac16_t vqReq)
{
	bool isOverLimit;
	qfrac16_t vqReqOut;

	/* VoltageModeILimit_QFracS16 set to torque direction. Alternatively, use limits stored in SpeedPid  */
	if		(p_motor->VoltageModeILimit_QFracS16 > 0) 	{ isOverLimit = (FOC_GetIq(&p_motor->Foc) > p_motor->VoltageModeILimit_QFracS16); }
	else if	(p_motor->VoltageModeILimit_QFracS16 < 0) 	{ isOverLimit = (FOC_GetIq(&p_motor->Foc) < p_motor->VoltageModeILimit_QFracS16); }
	else 												{ isOverLimit = false; } /* should not occur */

	// if		(p_motor->VoltageModeILimit_QFracS16 > 0) 	{ isOverLimit = (FOC_GetIMagnitude(&p_motor->Foc) > p_motor->VoltageModeILimit_QFracS16); }
	// else if	(p_motor->VoltageModeILimit_QFracS16 < 0) 	{ isOverLimit = (FOC_GetIMagnitude(&p_motor->Foc) > 0 - p_motor->VoltageModeILimit_QFracS16); }
	// else 												{ isOverLimit = false; } /* should not occur */

	if((isOverLimit == true) && (p_motor->ControlFlags.VoltageModeILimitActive == false))
	{
		p_motor->ControlFlags.VoltageModeILimitActive = true;
		PID_SetOutputState(&p_motor->PidIq, FOC_GetVq(&p_motor->Foc));
	}
	else /* alternatively remain set until manual reset */
	{
		p_motor->ControlFlags.VoltageModeILimitActive = false;
	}

	vqReqOut = (p_motor->ControlFlags.VoltageModeILimitActive == true) ?
		PID_Calc(&p_motor->PidIq, p_motor->VoltageModeILimit_QFracS16, FOC_GetIq(&p_motor->Foc)) : vqReq;

	// vqReqOut = PID_Calc(&p_motor->PidIq, p_motor->VoltageModeILimit_QFracS16, FOC_GetIq(&p_motor->Foc));
	// if(vqReq < vqReqOut) { vqReqOut = vqReq; }

	FOC_SetVq(&p_motor->Foc, vqReqOut);
	FOC_SetVd(&p_motor->Foc, 0);
}

static inline void _Motor_FOC_ProcFeedbackLoop(Motor_T * p_motor)
{
	int32_t userOutput = (p_motor->FeedbackModeFlags.Speed == 1U) ? p_motor->SpeedControl : p_motor->RampCmd;

	if(p_motor->FeedbackModeFlags.Current == 1U) /* Current Control mode - proc using last adc measure */
	{
		FOC_SetVq(&p_motor->Foc, PID_Calc(&p_motor->PidIq, userOutput, 	FOC_GetIq(&p_motor->Foc)));
		FOC_SetVd(&p_motor->Foc, PID_Calc(&p_motor->PidId, 0U, 			FOC_GetId(&p_motor->Foc)));
	}
	else /* Voltage Control mode - use current feedback for over current only */
	{
		/*
			Constant Voltage Mode
			input	RampCmd[-32767:32767]
			output	VqReq[-32767:32767] => RampCmd
		*/
		/*
			VFreq Mode
			input	RampCmd[0:65535] == VFreqScalar
			output	VqReq[-32767:32767] => VFreqScalar * SpeedFeedback

			SpeedVMatchRatio = SpeedFeedback_Frac16 * SpeedFeedbackRef_Rpm / SpeedVMatchRef_Rpm / 2;
			SpeedVMatchRatio = Linear_Function(&p_motor->SpeedVMatchRatio, p_motor->SpeedFeedback_Frac16) / 2;
			vqReq = RampCmd * SpeedVMatchRatio / 65536;

			SpeedFeedback_Frac16 unsaturated ~[-65535:65535]

			Overflow caution:
			Linear_Function(&p_motor->SpeedVMatchRatio) input range [-65535*3/2:65535*3/2], when VMatchRef == FeedbackRef * 3 / 4
		*/
		if(p_motor->FeedbackModeFlags.VFreqScalar == 1U)
		{
			userOutput = p_motor->RampCmd * (Linear_Function(&p_motor->SpeedVMatchRatio, p_motor->SpeedFeedback_Frac16 / 2)) / 65536;

			if		(userOutput > 32767) 	{ userOutput = 32767; }
			else if	(userOutput < -32767) 	{ userOutput = -32767; }
		}

		_Motor_FOC_ProcVoltageMode(p_motor, userOutput);

		// _Motor_FOC_ProcVoltageModeILimit(p_motor, userOutput);
		// if(p_motor->ControlFlags.VoltageModeILimitActive == true) { userOutput = PID_Calc(&p_motor->PidIq, p_motor->ILimitVoltageMode_Frac16, FOC_GetIq(&p_motor->Foc)); }
		// FOC_SetVq(&p_motor->Foc, vqReqOut);
		// FOC_SetVd(&p_motor->Foc, 0);
	}
}

static inline void _Motor_FOC_ActivateAngle(Motor_T * p_motor)
{
	FOC_ProcInvParkInvClarkeSvpwm(&p_motor->Foc);
	Phase_ActivateDuty(&p_motor->Phase, FOC_GetDutyA(&p_motor->Foc), FOC_GetDutyB(&p_motor->Foc), FOC_GetDutyC(&p_motor->Foc));
}


/******************************************************************************/
/*!
	Public Call from State Machine
*/
/******************************************************************************/
static void Motor_FOC_ProcAngleObserve(Motor_T * p_motor)
{
	Motor_Debug_CaptureTime(p_motor, 1U);
	//no current sense during pwm float, check bemf
	AnalogN_Group_PauseQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_V);
#if defined(CONFIG_MOTOR_V_SENSORS_ANALOG)
	AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_VA);
	AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_VB);
	AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_VC);
#endif
	AnalogN_Group_ResumeQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_V);

	Motor_Debug_CaptureTime(p_motor, 2U);

	_Motor_FOC_ProcPositionFeedback(p_motor);
}

static inline void Motor_FOC_StartAngleObserve(Motor_T * p_motor)
{

}

/*
	StateMachine calls each PWM, ~20kHz
	shared function -> less states
*/
static void Motor_FOC_ProcAngleControl(Motor_T * p_motor)
{
	AnalogN_Group_PauseQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_I);
	AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IA);
	AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IB);
#if defined(CONFIG_MOTOR_I_SENSORS_ABC)
	AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IC);
#endif
	AnalogN_Group_ResumeQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_I);

	/* Samples chain completes shortly after queue resumes. ADC isr priority higher than PWM. */

	/* ~10us */ Motor_Debug_CaptureTime(p_motor, 1U);

	Motor_ProcRamp(p_motor);

	if(Motor_CheckPositionFeedback(p_motor) == true) { _Motor_FOC_ProcPositionFeedback(p_motor); }
	else { _Motor_FOC_ProcOpenLoop(p_motor); }

	FOC_SetVector(&p_motor->Foc, p_motor->ElectricalAngle);

	/* ~29 us */ Motor_Debug_CaptureTime(p_motor, 2U);

#if 	defined(CONFIG_MOTOR_I_SENSORS_AB)
	FOC_ProcClarkePark_AB(&p_motor->Foc);
#elif 	defined(CONFIG_MOTOR_I_SENSORS_ABC)
	FOC_ProcClarkePark(&p_motor->Foc);
#endif

	/* ~30us */	Motor_Debug_CaptureTime(p_motor, 3U);

	if(Motor_CheckPositionFeedback(p_motor) == true) { _Motor_FOC_ProcFeedbackLoop(p_motor); }

	_Motor_FOC_ActivateAngle(p_motor);

	/* ~37us */ Motor_Debug_CaptureTime(p_motor, 4U);
}

static void Motor_FOC_ProcOpenLoop(Motor_T * p_motor)
{

}

/* Always on Entry, from Stop and Resume */
static inline void Motor_FOC_StartAngleControl(Motor_T * p_motor)
{
	FOC_SetDutyZero(&p_motor->Foc);
	Phase_ActivateDuty(&p_motor->Phase, FOC_GetDutyA(&p_motor->Foc), FOC_GetDutyB(&p_motor->Foc), FOC_GetDutyC(&p_motor->Foc));
	Phase_ActivateSwitchABC(&p_motor->Phase);
}

/******************************************************************************/
/*!
	PID updates when changing Control/FeedbackMode
*/
/******************************************************************************/
/*

*/
static void _Motor_FOC_SetOutputMatch(Motor_T * p_motor, int32_t iq, int32_t vq, int32_t vd)
{
	int32_t userOutput = (p_motor->FeedbackModeFlags.Current == 1U) ? iq : vq;

	if		(p_motor->FeedbackModeFlags.VFreqScalar == 1U) 	{ Motor_SetRampOutput(p_motor, 65535); } 		/* Start from scalar of 1, output speed */
	else if	(p_motor->FeedbackModeFlags.Speed == 1U) 		{ Motor_SetSpeedOutput(p_motor, userOutput);} 	/* SPEED_CURRENT, or SPEED_VOLTAGE */
	else 													{ Motor_SetRampOutput(p_motor, userOutput); } 	/* CONSTANT_CURRENT, or CONSTANT_VOLTAGE */

	PID_SetIntegral(&p_motor->PidIq, vq);
	PID_SetIntegral(&p_motor->PidId, vd);
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
	// vqReq = Linear_ADC_CalcFractionUnsigned16(&p_motor->CONFIG.UNIT_V_ABC, p_motor->VBemfPeak_Adcu) >> 1U;

	/*
		Match to Speed
		User sets larger SpeedVMatchRef_Rpm to ensure not resume to higher speed
		vqReq = p_motor->SpeedFeedback_Frac16 * p_motor->Parameters.SpeedFeedbackRef_Rpm / p_motor->Parameters.SpeedVMatchRef_Rpm / 2;
			SpeedVMatch_Factor = SpeedFeedbackRef_Rpm << 14 / SpeedVMatchRef_Rpm
			vqReq = (p_motor->SpeedFeedback_Frac16 * SpeedVMatch_Factor >> 14) / 2
	*/
	vqReq = Linear_Function(&p_motor->SpeedVMatchRatio, p_motor->SpeedFeedback_Frac16 / 2);
	_Motor_FOC_SetOutputMatch(p_motor, 0, vqReq, 0);
}

static inline void Motor_FOC_SetOutputMatchStop(Motor_T * p_motor)
{
	_Motor_FOC_SetOutputMatch(p_motor, 0, 0, 0);
}

/*
	V output is prev VReq, match to start at 0 change in torque
	Iq PID output differs between voltage Iq limit mode and Iq control mode. still need to match
*/
static inline void Motor_FOC_SetOutputMatchRun(Motor_T * p_motor)
{
	_Motor_FOC_SetOutputMatch(p_motor, FOC_GetIq(&p_motor->Foc), FOC_GetVq(&p_motor->Foc), FOC_GetVd(&p_motor->Foc));
}


/******************************************************************************/
/*!
	User
*/
/******************************************************************************/

/* IdqMagnitude */
static inline uint32_t Motor_FOC_GetIMagnitude_Frac16(Motor_T * p_motor)
{
	return (uint32_t)FOC_GetIMagnitude(&p_motor->Foc) * 2U;
}

/*
	Call from user must also set Vector Sine/Cosine, not set during position read
*/
//static inline void Motor_FOC_ActivateAngle(Motor_T * p_motor, qangle16_t angle, qfrac16_t vq, qfrac16_t vd)
//{
////	p_motor->ElectricalAngle = angle;
//	FOC_SetVq(&p_motor->Foc, vq);
//	FOC_SetVd(&p_motor->Foc, vd);
//	FOC_SetVector(&p_motor->Foc, angle); //angle -90
//	_Motor_FOC_ActivateAngle(p_motor);
//}


/******************************************************************************/
/*!
	Extern
*/
/******************************************************************************/
extern void Motor_FOC_SetOutputLimitsCcw(Motor_T * p_motor);
extern void Motor_FOC_SetOutputLimitsCw(Motor_T * p_motor);
extern void Motor_FOC_SetDirectionCcw(Motor_T * p_motor);
extern void Motor_FOC_SetDirectionCw(Motor_T * p_motor);
extern void Motor_FOC_SetDirection(Motor_T * p_motor, Motor_Direction_T direction);
extern void Motor_FOC_ResetSpeedPidILimits(Motor_T * p_motor);
extern void Motor_FOC_SetDirectionForward(Motor_T * p_motor);

#endif
