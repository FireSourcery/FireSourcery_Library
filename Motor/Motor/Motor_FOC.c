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
void Motor_FOC_EnqueueVabc(Motor_T * p_motor)
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
#else
	(void)p_motor;
#endif
}

void Motor_FOC_EnqueueIabc(Motor_T * p_motor)
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
}

/******************************************************************************/
/*!
	Feedback Loops
*/
/******************************************************************************/
static inline void ProcInnerFeedback(Motor_T * p_motor)
{
	if(p_motor->ControlFeedbackMode.Current == 1U) /* Current Control mode - proc using last adc measure */
	{
		FOC_SetVq(&p_motor->Foc, PID_Proc(&p_motor->PidIq, FOC_GetIVqReq(&p_motor->Foc), FOC_GetIq(&p_motor->Foc)));
		FOC_SetVd(&p_motor->Foc, PID_Proc(&p_motor->PidId, FOC_GetIVdReq(&p_motor->Foc), FOC_GetId(&p_motor->Foc)));
	}
	else /* Voltage Control mode - use current feedback for over current only */
	{
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
		if(p_motor->ControlFeedbackMode.Scalar == 1U)
		{
			// FOC_SetIVqReq(&p_motor->Foc, math_clamp(Linear_Ramp_GetOutput(&p_motor->Ramp) * (Motor_GetKvSpeed_Frac16(p_motor) / 2 / Scalar) / 65536, -32767, 32767));
		}

		if(math_isbound(FOC_GetIq(&p_motor->Foc), p_motor->ILimitCw_FracS16, p_motor->ILimitCcw_FracS16) == true)
		{
			FOC_SetVq(&p_motor->Foc, FOC_GetIVqReq(&p_motor->Foc));
			FOC_SetVd(&p_motor->Foc, FOC_GetIVdReq(&p_motor->Foc));
		}
		else
		{
			p_motor->ControlFeedbackMode.Current == 1U;
			PID_SetOutputState(&p_motor->PidIq, FOC_GetVq(&p_motor->Foc));
			PID_SetOutputState(&p_motor->PidId, FOC_GetVd(&p_motor->Foc));
			if		(FOC_GetIq(&p_motor->Foc) < p_motor->ILimitCw_FracS16) 	{ FOC_SetIVqReq(&p_motor->Foc, p_motor->ILimitCw_FracS16); }
			else if	(FOC_GetIq(&p_motor->Foc) > p_motor->ILimitCcw_FracS16) { FOC_SetIVqReq(&p_motor->Foc, p_motor->ILimitCcw_FracS16); }
			FOC_SetIVdReq(&p_motor->Foc, 0U);
		}
	}
}

/*
	Speed Feedback Loop
	SpeedControl_FracS16 update ~1000Hz, Ramp input 1000Hz, RampCmd output 20000Hz
	input	RampCmd[-32767:32767] - (speedFeedback_Frac16 / 2)[-32767:32767]
			accepts over saturated inputs
	output 	SpeedControl_FracS16[-32767:32767] => IqReq or VqReq
*/
static inline void ProcOuterFeedback(Motor_T * p_motor)
{
	int32_t speed_FracS16;
	Linear_Ramp_ProcOutput(&p_motor->Ramp);

	if((Motor_ProcSensorSpeed(p_motor) == true) && (p_motor->ControlFeedbackMode.Speed == 1U))
	{
		speed_FracS16 = p_motor->Speed_Frac16 / 2;
		FOC_SetIVqReq(&p_motor->Foc, PID_Proc(&p_motor->PidSpeed, Linear_Ramp_GetOutput(&p_motor->Ramp), speed_FracS16));
		FOC_SetIVdReq(&p_motor->Foc, 0);
	}
	else if(p_motor->ControlFeedbackMode.Speed == 0U)
	{
		speed_FracS16 = p_motor->Speed_Frac16 / 2;
		// if(Motor_CheckSpeedOverLimit(Motor_T * p_motor) == true)
		// if(math_isbound(speed_FracS16, p_motor->SpeedLimitCw_FracS16, p_motor->SpeedLimitCcw_FracS16) == true)
		// {
			FOC_SetIVqReq(&p_motor->Foc, Linear_Ramp_GetOutput(&p_motor->Ramp));
			FOC_SetIVdReq(&p_motor->Foc, 0);
		// }
		// else
		// {
		// 	p_motor->ControlFeedbackMode.Speed = 1U;
		// 	PID_SetOutputState(&p_motor->PidSpeed, Linear_Ramp_GetOutput(&p_motor->Ramp));
		// 	if		(speed_FracS16 < p_motor->SpeedLimitCw_FracS16) 	{ Linear_Ramp_SetOutputState(&p_motor->Ramp, p_motor->SpeedLimitCw_FracS16); }
		// 	else if	(speed_FracS16 > p_motor->SpeedLimitCcw_FracS16) 	{ Linear_Ramp_SetOutputState(&p_motor->Ramp, p_motor->SpeedLimitCcw_FracS16); }
		// }
	}

}

/*!
	Match Feedback State to Output
	Update PID state when changing ControlFeedbackMode
*/
void Motor_FOC_ProcFeedbackMatch(Motor_T * p_motor)
{
	int32_t qReq = (p_motor->ControlFeedbackMode.Current == 1U) ? FOC_GetIq(&p_motor->Foc) : FOC_GetVq(&p_motor->Foc);; /* q_sqrt(vd vq) */

	if		(p_motor->ControlFeedbackMode.Speed == 1U) 		{ Linear_Ramp_SetOutputState(&p_motor->Ramp, p_motor->Speed_Frac16 / 2); PID_SetOutputState(&p_motor->PidSpeed, qReq); }
	else if	(p_motor->ControlFeedbackMode.Scalar == 1U) 	{ Linear_Ramp_SetOutputState(&p_motor->Ramp, 65535); }
	else 													{ Linear_Ramp_SetOutputState(&p_motor->Ramp, qReq); } /* CONSTANT_CURRENT, or CONSTANT_VOLTAGE */

	PID_SetOutputState(&p_motor->PidIq, FOC_GetVq(&p_motor->Foc));
	PID_SetOutputState(&p_motor->PidId, FOC_GetVd(&p_motor->Foc));
}

/******************************************************************************/
/*!

*/
/******************************************************************************/
static inline void ProcClarkePark(Motor_T * p_motor)
{
#if 	defined(CONFIG_MOTOR_I_SENSORS_AB)
	FOC_ProcClarkePark_AB(&p_motor->Foc);
#elif 	defined(CONFIG_MOTOR_I_SENSORS_ABC)
	FOC_ProcClarkePark(&p_motor->Foc);
#endif
}

/* Vd Vq to SVPWM */
static void ActivateAngle(Motor_T * p_motor)
{
	FOC_ProcInvParkInvClarkeSvpwm(&p_motor->Foc);
	Phase_ActivateDuty(&p_motor->Phase, FOC_GetDutyA(&p_motor->Foc), FOC_GetDutyB(&p_motor->Foc), FOC_GetDutyC(&p_motor->Foc));
}

static void ProcInnerFeedbackOutput(Motor_T * p_motor)
{
	ProcClarkePark(p_motor);
	ProcInnerFeedback(p_motor); /* Set Vd Vq */
	ActivateAngle(p_motor);
}

/******************************************************************************/
/*!

*/
/******************************************************************************/
#ifdef CONFIG_MOTOR_EXTERN_CONTROL_ENABLE
extern void Motor_ExternControl(Motor_T * p_motor);
#endif

/*
	Feedback Control Loop
	StateMachine calls each PWM, ~20kHz
*/
void Motor_FOC_ProcAngleControl(Motor_T * p_motor)
{
	Motor_FOC_EnqueueIabc(p_motor); /* Samples chain completes sometime after queue resumes. ADC ISR priority higher than PWM. */
#ifdef CONFIG_MOTOR_EXTERN_CONTROL_ENABLE
	Motor_ExternControl(p_motor);
#endif
// /* ~10us */ Motor_Debug_CaptureTime(p_motor, 1U);
	// if(Motor_CheckOpenLoop(p_motor) == false)
	// {
	p_motor->ElectricalAngle = Motor_PollSensorAngle(p_motor);
	FOC_SetTheta(&p_motor->Foc, p_motor->ElectricalAngle);
	ProcOuterFeedback(p_motor);
	// }
// /* ~29 us */ Motor_Debug_CaptureTime(p_motor, 2U);
	ProcInnerFeedbackOutput(p_motor);
// /* ~37us */ Motor_Debug_CaptureTime(p_motor, 4U);
}

/*
	FreeWheel and Stop State
*/
void Motor_FOC_ProcAngleVBemf(Motor_T * p_motor)
{
	Motor_FOC_EnqueueVabc(p_motor);
	p_motor->ElectricalAngle = Motor_PollSensorAngle(p_motor);
	Motor_ProcSensorSpeed(p_motor);
	FOC_SetTheta(&p_motor->Foc, p_motor->ElectricalAngle);
	FOC_ProcVBemfClarkePark(&p_motor->Foc);
}

/******************************************************************************/
/*!
	StateMachine mapping
	defined as inline for StateMachine wrapper functions
*/
/******************************************************************************/
void Motor_FOC_StartAlign(Motor_T * p_motor)
{
	Motor_SetControlFeedbackOpenLoopCurrent(p_motor);
	Linear_Ramp_Set(&p_motor->AuxRamp, p_motor->Parameters.AlignTime_Cycles, 0, p_motor->Parameters.AlignPower_Frac16 / 2U);
	FOC_SetIVqReq(&p_motor->Foc, 0);
	FOC_SetTheta(&p_motor->Foc, 0);
}

void Motor_FOC_ProcAlign(Motor_T * p_motor)
{
	Motor_FOC_EnqueueIabc(p_motor);
	FOC_SetIVdReq(&p_motor->Foc, Linear_Ramp_ProcOutput(&p_motor->AuxRamp));
	ProcInnerFeedbackOutput(p_motor);
}

void Motor_FOC_StartAlignValidate(Motor_T * p_motor)
{
	Motor_ZeroSensorAlign(p_motor);
	Motor_ZeroSensor(p_motor);
	FOC_SetIVdReq(&p_motor->Foc, 0);
	p_motor->ControlFeedbackMode.OpenLoop = 0U;	// p_motor->ControlFeedbackMode = p_motor->CmdFeedbackMode;
}

void Motor_FOC_StartOpenLoop(Motor_T * p_motor)
{
	// Motor_SetControlFeedbackOpenLoopCurrent(p_motor);
	// p_motor->ControlFeedbackMode.Current = 0U;
	p_motor->Speed_Frac16 = 0;
	Linear_Ramp_Set(&p_motor->AuxRamp, p_motor->Parameters.RampAccel_Cycles, 0, _Motor_ConvertDirectionalCmd(p_motor, p_motor->Parameters.OpenLoopPower_Frac16 / 2U));
	Linear_Ramp_SetTarget(&p_motor->OpenLoopSpeedRamp, _Motor_ConvertDirectionalCmd(p_motor, p_motor->Parameters.OpenLoopSpeed_Frac16 / 2U) * 2);
	Linear_Ramp_SetOutputState(&p_motor->OpenLoopSpeedRamp, 0);
	FOC_SetIVdReq(&p_motor->Foc, 0);
	// alternatively, clamp user input ramp
}

/*
	OpenLoop
	Blind input angle, constant voltage/current
	ElectricalAngle => integrate speed to angle
*/
static void _Motor_FOC_ProcOpenLoopSpeed(Motor_T * p_motor)
{
	/* OpenLoopSpeed_RPM = Speed_Frac16 / 65535 * SpeedFeedbackRef_Rpm */
	/* (OpenLoopSpeed_RPM * 65536 * 6) / (60 *CONTROL_FREQ) */
	p_motor->Speed_Frac16 = Linear_Ramp_ProcOutput(&p_motor->OpenLoopSpeedRamp);
	p_motor->ElectricalAngle += ((p_motor->Speed_Frac16 * p_motor->Parameters.SpeedFeedbackRef_Rpm * p_motor->Parameters.PolePairs) / ((int32_t)60 * GLOBAL_MOTOR.CONTROL_FREQ));
}

void Motor_FOC_ProcOpenLoop(Motor_T * p_motor)
{
	Motor_FOC_EnqueueIabc(p_motor);
	_Motor_FOC_ProcOpenLoopSpeed(p_motor);
	FOC_SetTheta(&p_motor->Foc, p_motor->ElectricalAngle);
	FOC_SetIVqReq(&p_motor->Foc, Linear_Ramp_ProcOutput(&p_motor->AuxRamp)); // Linear_Ramp_ProcOutput(&p_motor->Ramp);
	ProcInnerFeedbackOutput(p_motor);
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
	FOC_SetTheta(&p_motor->Foc, angle);
	ActivateAngle(p_motor);
}

/******************************************************************************/
/*!
	FOC Direction
*/
/******************************************************************************/
/*
	Set on Direction change
	Iq/Id PID always Vq/Vd. Clip opposite user direction range, no plugging.
	Voltage ControlFeedbackMode active during over current only.
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