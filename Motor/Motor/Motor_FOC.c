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
static inline qangle16_t Motor_FOC_PollPositionSensorAngle(Motor_T * p_motor)
{
	qangle16_t electricalAngle; /* FracU16 [0, 65535] map negative portions of qangle16_t */

	switch(p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_HALL:
#if defined(CONFIG_MOTOR_HALL_MODE_POLLING) /* todo fix */
			if(Hall_PollCaptureRotorAngle(&p_motor->Hall) == true) { Encoder_CapturePulse(&p_motor->Encoder); }
#endif
			electricalAngle = Hall_GetRotorAngle_Degrees16(&p_motor->Hall);
			electricalAngle += Encoder_ModeDT_InterpolateAngle(&p_motor->Encoder);
			break;

		case MOTOR_SENSOR_MODE_ENCODER:
			electricalAngle = Motor_GetEncoderElectricalAngle(p_motor);
			electricalAngle += Encoder_ModeDT_InterpolateAngle(&p_motor->Encoder);
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
			p_motor->FeedbackModeFlags.OpenLoop = 1U;
			p_motor->FeedbackModeFlags.OpenLoop = 1U;
			p_motor->ControlFlags.SensorFeedback = 0U;
			p_motor->ControlFlags.SensorFeedback = 0U;
			break;
#endif
		default: electricalAngle = 0; break;
	}

	return electricalAngle;
}

/* For SinCos, Sensorless, when not using Encoder module */
// static inline int32_t PollAngleSpeed(Motor_T * p_motor, qangle16_t speedAngle)
// {
// 	int32_t speedDelta = speedAngle - p_motor->SpeedAngle; /* loops if no overflow past 1 full cycle */
// 	int32_t speedFeedback_Frac16 = (p_motor->SpeedFeedback_Frac16 + Linear_Speed_CalcAngleRpmFrac16(&p_motor->UnitAngleRpm, speedDelta)) / 2;
// 	p_motor->SpeedAngle = speedAngle; /* mechanical angle */
// 	return speedFeedback_Frac16;
// }

/* returns [-65536:65536] as [-1:1] unsaturated */
static inline int32_t Motor_FOC_PollPositionSensorSpeed(Motor_T * p_motor)
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

void Motor_FOC_ProcPosition(Motor_T * p_motor)
{
	qangle16_t electricalAngle = Motor_FOC_PollPositionSensorAngle(p_motor);
	if(qangle16_cycle(p_motor->ElectricalAngle, electricalAngle) == true) /* Once Per Cycle (p_motor->ElectricalAngle < 0 && electricalAngle > 0) */
	{
		p_motor->VBemfPeak_Adcu = p_motor->VBemfPeakTemp_Adcu;
		p_motor->VBemfPeakTemp_Adcu = 0U;
		p_motor->IPhasePeak_Adcu = p_motor->IPhasePeakTemp_Adcu;
		p_motor->IPhasePeakTemp_Adcu = 0U;
	}
	p_motor->ElectricalAngle = electricalAngle;
	FOC_SetVector(&p_motor->Foc, electricalAngle);
}

bool Motor_FOC_ProcSpeed(Motor_T * p_motor)
{
	bool procSpeed = Timer_Periodic_Poll(&p_motor->SpeedTimer);
	if(procSpeed == true) { p_motor->SpeedFeedback_Frac16 = (Motor_FOC_PollPositionSensorSpeed(p_motor) + p_motor->SpeedFeedback_Frac16) / 2; }
	return procSpeed;
}

void Motor_FOC_ProcSpeedFeedback(Motor_T * p_motor)
{
	if(Motor_FOC_ProcSpeed(p_motor) == true) { Motor_ProcSpeedFeedback(p_motor, p_motor->SpeedFeedback_Frac16 / 2); }
}

/******************************************************************************/
/*!
*/
/******************************************************************************/
/*
	FreeWheel and Stop State
*/
void Motor_FOC_ProcAngleObserve(Motor_T * p_motor)
{
	qangle16_t electricalAngle;
#if defined(CONFIG_MOTOR_V_SENSORS_ANALOG)
	if(((p_motor->ControlTimerBase & GLOBAL_MOTOR.CONTROL_ANALOG_DIVIDER)) == 0UL)
	{
		AnalogN_Group_PauseQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_V);
		AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_VA);
		AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_VB);
		AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_VC);
		AnalogN_Group_ResumeQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_V);
	}
#endif
	Motor_FOC_ProcPosition(p_motor);
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
	FOC_SetDutyZero(&p_motor->Foc);
	Phase_ActivateDuty(&p_motor->Phase, FOC_GetDutyA(&p_motor->Foc), FOC_GetDutyB(&p_motor->Foc), FOC_GetDutyC(&p_motor->Foc));
	Phase_ActivateSwitchABC(&p_motor->Phase);
}

/*
	OpenLoop/User Activate
	must also set Vector Sine/Cosine, not set during position read,
	angle control loop must set vector before feedback calc
*/
void Motor_FOC_ActivateAngle(Motor_T * p_motor, qangle16_t angle, qfrac16_t vq, qfrac16_t vd)
{
//	p_motor->ElectricalAngle = angle;
	FOC_SetVq(&p_motor->Foc, vq);
	FOC_SetVd(&p_motor->Foc, vd);
	FOC_SetVector(&p_motor->Foc, angle);
	FOC_ProcInvParkInvClarkeSvpwm(&p_motor->Foc);
	Phase_ActivateDuty(&p_motor->Phase, FOC_GetDutyA(&p_motor->Foc), FOC_GetDutyB(&p_motor->Foc), FOC_GetDutyC(&p_motor->Foc));
}

/******************************************************************************/
/*!
*/
/******************************************************************************/
/*
	Set on Direction change
	Iq/Id PID always Vq/Vd, clip opposite user direction range, no plugging.
	Voltage Feedback Mode active during over current only.
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