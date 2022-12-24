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
*/
/******************************************************************************/
void Motor_FOC_PollPositionSensor(Motor_T * p_motor)
{

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

/******************************************************************************/
/*!
*/
/******************************************************************************/
/*
	Call from user must also set Vector Sine/Cosine, not set during position read
	angl control loop must set vector before feedback calc
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

/* 1 Step Align */
void Motor_FOC_Align(Motor_T * p_motor)
{
	Motor_FOC_ActivateAngle(p_motor, 0, 0, p_motor->Parameters.AlignVPwm_Frac16 / 2U);
}

/* Alternate Soft Align */
void Motor_FOC_StartAlign(Motor_T * p_motor)
{
	// Motor_SetRampTarget_Millis
	Motor_ZeroRamp(p_motor);
	Motor_SetRampSlope_Millis(p_motor, p_motor->Parameters.AlignTime_Ms, 0, p_motor->Parameters.AlignVPwm_Frac16 / 2U);
	Motor_SetRampTarget(p_motor, p_motor->Parameters.AlignVPwm_Frac16 / 2U);
}

void Motor_FOC_ProcAlign(Motor_T * p_motor)
{
	Motor_ProcRamp(p_motor);
	Motor_FOC_ActivateAngle(p_motor, 0, 0, p_motor->RampCmd);
}

/* Alternate OpenLoop */
void Motor_FOC_StartOpenLoop(Motor_T * p_motor)
{
	p_motor->OpenLoopSpeed_RPM = 0U;
	// todo  Motor_SetRampTarget_Millis
	// Linear_Ramp_SetSlope_Ticks(&p_motor->VPwmRamp, _Motor_ConvertToControlCycles(p_motor, p_motor->Parameters.RampAccel_Ms), 0, p_motor->Parameters.OpenLoopVPwm_Frac16 / 2U);
	Motor_ZeroRamp(p_motor); /* start from 0, since 90 degrees ahead of align */
	Motor_SetRampSlope_Millis(p_motor, p_motor->Parameters.RampAccel_Ms, 0, p_motor->Parameters.OpenLoopVPwm_Frac16 / 2U);
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

void Motor_FOC_ProcOpenLoop(Motor_T * p_motor)
{
	_Motor_FOC_ProcOpenLoop(p_motor);
	/* proc current, or use AngleControl */
	Motor_ProcRamp(p_motor);
	Motor_FOC_ActivateAngle(p_motor, p_motor->ElectricalAngle, p_motor->RampCmd, 0);
}


void Motor_FOC_ProcStop(Motor_T * p_motor)
{
#if defined(CONFIG_MOTOR_V_SENSORS_ANALOG)
	AnalogN_Group_PauseQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_V);
	AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_VA);
	AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_VB);
	AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_VC);
	AnalogN_Group_ResumeQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_V);
#endif
	AnalogN_Group_PauseQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_I);
	AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IA);
	AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IB);
#if defined(CONFIG_MOTOR_I_SENSORS_ABC)
	AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IC);
#endif
	AnalogN_Group_ResumeQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_I);

	_Motor_FOC_ProcPositionFeedback(p_motor);

	FOC_SetVector(&p_motor->Foc, p_motor->ElectricalAngle);

#if 	defined(CONFIG_MOTOR_I_SENSORS_AB)
	FOC_ProcClarkePark_AB(&p_motor->Foc);
#elif 	defined(CONFIG_MOTOR_I_SENSORS_ABC)
	FOC_ProcClarkePark(&p_motor->Foc);
#endif
}