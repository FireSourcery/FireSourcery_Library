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
	@file 	Motor_FOC.c
	@author FireSourcery
	@brief
	@version V0
*/
/******************************************************************************/
#include "Motor_FOC.h"

/******************************************************************************/
/*!
	Position Sensor Feedback - Speed, Angle
*/
/******************************************************************************/
static inline qangle16_t Motor_FOC_PollSensorAngle(Motor_T * p_motor)
{
	qangle16_t electricalAngle; /* FracU16 [0, 65535] maps to negative portions of qangle16_t */

	switch(p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_HALL:
#if defined(CONFIG_MOTOR_HALL_MODE_POLLING)
			if(Hall_PollCaptureRotorAngle(&p_motor->Hall) == true) { Encoder_CapturePulse(&p_motor->Encoder); }
#endif
			electricalAngle = Hall_GetRotorAngle_Degrees16(&p_motor->Hall);
			electricalAngle += Encoder_ModeDT_InterpolateAngularDisplacement(&p_motor->Encoder);
			break;

		case MOTOR_SENSOR_MODE_ENCODER:
			electricalAngle = Motor_GetEncoderElectricalAngle(p_motor);
			// electricalAngle += Encoder_ModeDT_InterpolateAngularDisplacement(&p_motor->Encoder);
			break;

#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
		case MOTOR_SENSOR_MODE_SIN_COS:
			SinCos_CaptureAngle(&p_motor->SinCos, p_motor->AnalogResults.Sin_Adcu, p_motor->AnalogResults.Cos_Adcu);
			electricalAngle = SinCos_GetElectricalAngle(&p_motor->SinCos);
			//todo group
			AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_SIN);
			AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_COS);
			break;
#endif
#if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
		case MOTOR_SENSOR_MODE_SENSORLESS:
			//todo observer
			electricalAngle = 0;
			p_motor->FeedbackMode.OpenLoop = 1U;
			p_motor->FeedbackMode.OpenLoop = 1U;
			p_motor->ControlFlags.SensorFeedback = 0U;
			p_motor->ControlFlags.SensorFeedback = 0U;
			break;
#endif
		default: electricalAngle = 0; break;
	}

	return electricalAngle;
}

void Motor_FOC_ProcSensorAngle(Motor_T * p_motor)
{
	qangle16_t electricalAngle = Motor_FOC_PollSensorAngle(p_motor);
	/* Once Per Cycle */
	if(qangle16_cycle(p_motor->ElectricalAngle, electricalAngle) == true)
	{
		p_motor->VBemfPeak_Adcu = p_motor->VBemfPeakTemp_Adcu;
		p_motor->VBemfPeakTemp_Adcu = 0U;
		p_motor->IPhasePeak_Adcu = p_motor->IPhasePeakTemp_Adcu;
		p_motor->IPhasePeakTemp_Adcu = 0U;
	}
	p_motor->ElectricalAngle = electricalAngle;
}

/* For SinCos, Sensorless, when not using Encoder module */
// static inline int32_t PollAngleSpeed(Motor_T * p_motor, qangle16_t speedAngle)
// {
// 	int32_t speedDelta = speedAngle - p_motor->SpeedAngle; /* loops if no overflow past 1 full cycle */
// 	int32_t speedFeedback_Frac16 = (p_motor->Speed_Frac16 + Linear_Speed_CalcAngleRpmFrac16(&p_motor->UnitsAngleRpm, speedDelta)) / 2;
// 	p_motor->SpeedAngle = speedAngle; /* mechanical angle */
// 	return speedFeedback_Frac16;
// }

/* returns [-65536:65536] as [-1:1] unsaturated */
static inline int32_t Motor_FOC_PollSensorSpeed(Motor_T * p_motor)
{
	int32_t speed_Frac16;
	switch(p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_HALL:		speed_Frac16 = Encoder_ModeDT_PollScalarVelocity(&p_motor->Encoder); break;
		case MOTOR_SENSOR_MODE_ENCODER: 	speed_Frac16 = Encoder_ModeDT_PollScalarVelocity(&p_motor->Encoder); break;
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
		case MOTOR_SENSOR_MODE_SIN_COS:		speed_Frac16 = PollAngleSpeed(p_motor, SinCos_GetMechanicalAngle(&p_motor->SinCos));	break;
#endif
#if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
		case MOTOR_SENSOR_MODE_SENSORLESS: break;
#endif
		default: speed_Frac16 = 0; break;
	}
	return speed_Frac16;
}

bool Motor_FOC_ProcSpeed(Motor_T * p_motor)
{
	bool procSpeed = Timer_Periodic_Poll(&p_motor->SpeedTimer);
	if(procSpeed == true) { p_motor->Speed_Frac16 = (Motor_FOC_PollSensorSpeed(p_motor) + p_motor->Speed_Frac16) / 2; }
	return procSpeed;
}

void Motor_FOC_ProcSpeedFeedback(Motor_T * p_motor)
{
	if(Motor_FOC_ProcSpeed(p_motor) == true) { Motor_ProcSpeedFeedback(p_motor, p_motor->Speed_Frac16 / 2); }
}

/******************************************************************************/
/*!
	Feedback
*/
/******************************************************************************/
/*!
	Match Feedback State to Output
	PID update when changing Control/FeedbackMode
	V output is prev VReq, match to start at 0 change in torque
	Iq PID output differs between voltage Iq limit mode and Iq control mode. still need to match
*/
/* FeedbackMode.Scalar => Start from scalar of 1, output speed */
/* FeedbackMode.Speed == 1, SPEED_CURRENT, or SPEED_VOLTAGE */
/* FeedbackMode.Scalar == 0, CONSTANT_CURRENT, or CONSTANT_VOLTAGE, Open Loop */
void Motor_FOC_MatchFeedbackLoop(Motor_T * p_motor)
{
	int32_t userOutput = (p_motor->FeedbackMode.Current == 1U) ? FOC_GetIq(&p_motor->Foc) : FOC_GetVq(&p_motor->Foc);; /* q_sqrt(vd vq) */

	if		(p_motor->FeedbackMode.Scalar == 1U) 	{ Linear_Ramp_SetOutputState(&p_motor->Ramp, 65535); }
	else if	(p_motor->FeedbackMode.Speed == 1U) 	{ Linear_Ramp_SetOutputState(&p_motor->Ramp, p_motor->Speed_Frac16 / 2); PID_SetOutputState(&p_motor->PidSpeed, userOutput); }
	else 											{ Linear_Ramp_SetOutputState(&p_motor->Ramp, userOutput); }

	PID_SetIntegral(&p_motor->PidIq, FOC_GetVq(&p_motor->Foc));
	PID_SetIntegral(&p_motor->PidId, FOC_GetVd(&p_motor->Foc));
}

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

	vqReqOut = (p_motor->ControlFlags.VoltageModeILimitActive == true) ? PID_Proc(&p_motor->PidIq, iLimit, FOC_GetIq(&p_motor->Foc)) : vqReq;

	FOC_SetVq(&p_motor->Foc, vqReqOut);
	FOC_SetVd(&p_motor->Foc, 0);
}

static inline void _Motor_FOC_ProcCurrentFeedback(Motor_T * p_motor)
{
	int32_t userOutput = (p_motor->FeedbackMode.Speed == 1U) ? PID_GetOutput(&p_motor->PidSpeed) : Linear_Ramp_GetOutput(&p_motor->Ramp);

	if(p_motor->FeedbackMode.Current == 1U) /* Current Control mode - proc using last adc measure */
	{
		FOC_SetVq(&p_motor->Foc, PID_Proc(&p_motor->PidIq, userOutput, 					FOC_GetIq(&p_motor->Foc)));
		FOC_SetVd(&p_motor->Foc, PID_Proc(&p_motor->PidId, FOC_GetIdReq(&p_motor->Foc), FOC_GetId(&p_motor->Foc))); /* todo regularize */
	}
	else /* Voltage Control mode - use current feedback for over current only */
	{
		// if(p_motor->FeedbackMode.Scalar == 1U)
		// {
		// 	userOutput = Linear_Ramp_GetOutput(&p_motor->Ramp) * (Motor_GetVSpeedFrac16_Speed(p_motor) / 2) / 65536;
		// 	if		(userOutput > 32767) 	{ userOutput = 32767; }
		// 	else if	(userOutput < -32767) 	{ userOutput = -32767; }
		// }
		_Motor_FOC_ProcVoltageMode(p_motor, userOutput);
	}
}


#ifdef CONFIG_MOTOR_EXTERN_CONTROL_ENABLE
extern void Motor_ExternControl(Motor_T * p_motor);
#endif

/*
	Feedback Control Loop
	StateMachine calls each PWM, ~20kHz
*/
void Motor_FOC_ProcAngleControl(Motor_T * p_motor)
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
	if(Motor_CheckOpenLoop(p_motor) == false)
	{
		Motor_FOC_ProcSensorAngle(p_motor);
		Motor_FOC_ProcSpeedFeedback(p_motor);
		FOC_SetVector(&p_motor->Foc, p_motor->ElectricalAngle);
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

/*
	FreeWheel and Stop State
*/
void Motor_FOC_ProcAngleObserve(Motor_T * p_motor)
{
#if defined(CONFIG_MOTOR_V_SENSORS_ANALOG)
	if((p_motor->ControlTimerBase & GLOBAL_MOTOR.CONTROL_ANALOG_DIVIDER) == 0UL)
	{
		AnalogN_Group_PauseQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_V);
		AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_VA);
		AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_VB);
		AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_VC);
		AnalogN_Group_ResumeQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_V);
	}
#endif
	Motor_FOC_ProcSensorAngle(p_motor);
	Motor_FOC_ProcSpeed(p_motor);
// Debug_LED();
}

/******************************************************************************/
/*!

*/
/******************************************************************************/
/*
	Enables PWM Output - From Stop and Freewheel
*/
void Motor_FOC_ActivateOutput(Motor_T * p_motor)
{
	FOC_ZeroSvpwm(&p_motor->Foc);
	Phase_ActivateDuty(&p_motor->Phase, FOC_GetDutyA(&p_motor->Foc), FOC_GetDutyB(&p_motor->Foc), FOC_GetDutyC(&p_motor->Foc));
	Phase_ActivateSwitchABC(&p_motor->Phase);
}

/*
	Feed Forward Angle without ClarkPark on Current
*/
void Motor_FOC_ActivateAngle(Motor_T * p_motor, qangle16_t angle, qfrac16_t vq, qfrac16_t vd)
{
	FOC_SetVq(&p_motor->Foc, vq);
	FOC_SetVd(&p_motor->Foc, vd);
	FOC_SetVector(&p_motor->Foc, angle);
	FOC_ProcInvParkInvClarkeSvpwm(&p_motor->Foc);
	Phase_ActivateDuty(&p_motor->Phase, FOC_GetDutyA(&p_motor->Foc), FOC_GetDutyB(&p_motor->Foc), FOC_GetDutyC(&p_motor->Foc));
}

/*
	Unchecked by StateMachine
*/
void Motor_FOC_SetControlMode(Motor_T * p_motor, Motor_FeedbackMode_T mode)
{
	if(p_motor->FeedbackMode.State != mode.State)
	{
		p_motor->FeedbackMode = mode;
		Motor_FOC_MatchFeedbackLoop(p_motor);
	}
}

/******************************************************************************/
/*!
	FOC Direction
*/
/******************************************************************************/
/*
	Set on Direction change
	Iq/Id PID always Vq/Vd. Clip opposite user direction range, no plugging.
	Voltage FeedbackMode active during over current only.
*/
void Motor_FOC_SetDirectionCcw(Motor_T * p_motor)
{
	Motor_SetDirectionCcw(p_motor);
	PID_SetOutputLimits(&p_motor->PidIq, 0, INT16_MAX);
	PID_SetOutputLimits(&p_motor->PidId, INT16_MIN / 2, INT16_MAX / 2); /* Symmetrical for now */
}

void Motor_FOC_SetDirectionCw(Motor_T * p_motor)
{
	Motor_SetDirectionCw(p_motor);
	PID_SetOutputLimits(&p_motor->PidIq, INT16_MIN, 0);
	PID_SetOutputLimits(&p_motor->PidId, INT16_MIN / 2, INT16_MAX / 2);
}

void Motor_FOC_SetDirection(Motor_T * p_motor, Motor_Direction_T direction)
{
	if(direction == MOTOR_DIRECTION_CCW) 	{ Motor_FOC_SetDirectionCcw(p_motor); }
	else 									{ Motor_FOC_SetDirectionCw(p_motor); }
}

void Motor_FOC_SetDirectionForward(Motor_T * p_motor)
{
	if(p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW) 	{ Motor_FOC_SetDirectionCcw(p_motor); }
	else 																	{ Motor_FOC_SetDirectionCw(p_motor); }
}