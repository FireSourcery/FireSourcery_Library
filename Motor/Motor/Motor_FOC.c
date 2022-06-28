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
	@file 	Motor_FOC.c
	@author FireSoucery
	@brief
	@version V0
*/
/******************************************************************************/
#include "Motor_FOC.h"

/*!
	@param[in] all [0:32767]
*/
// static void SetOutputLimits(Motor_T * p_motor, uint16_t iOutCcw, uint16_t iOutCw, uint16_t vOutCcw, uint16_t vOutCw)
// {
// 	if(p_motor->FeedbackModeFlags.Speed == 1U)
// 	{
// 		if(p_motor->FeedbackModeFlags.Current == 1U) 	{ PID_SetOutputLimits(&p_motor->PidSpeed, 0 - iOutCw, iOutCcw); } /* Speed PID Output is Iq */
// 		else 											{ PID_SetOutputLimits(&p_motor->PidSpeed, 0 - vOutCw, vOutCcw); } /* Speed PID Output is Vq */
// 	}
// }

static void ResetSpeedPidOutputLimitsCcw(Motor_T * p_motor)
{
	// SetOutputLimits(p_motor, p_motor->ILimitMotoring_Frac16 / 2U, p_motor->ILimitGenerating_Frac16 / 2U, INT16_MAX, 0);
	PID_SetOutputLimits(&p_motor->PidSpeed, 0 - p_motor->ILimitGenerating_Frac16 / 2, p_motor->ILimitMotoring_Frac16 / 2);
}

static void ResetSpeedPidOutputLimitsCw(Motor_T * p_motor)
{
	// SetOutputLimits(p_motor, p_motor->ILimitGenerating_Frac16 / 2U, p_motor->ILimitMotoring_Frac16 / 2U, 0, INT16_MAX);
	PID_SetOutputLimits(&p_motor->PidSpeed, 0 - p_motor->ILimitMotoring_Frac16 / 2, p_motor->ILimitGenerating_Frac16 / 2);
}

/* Set on Limits change */
void Motor_FOC_ResetSpeedPidOutputLimits(Motor_T * p_motor)
{
	if((p_motor->FeedbackModeFlags.Speed == 1U) && (p_motor->FeedbackModeFlags.Current == 1U)) /* Speed PID Output is Iq */
	{
		(p_motor->Direction == MOTOR_DIRECTION_CCW) ? ResetSpeedPidOutputLimitsCcw(p_motor) : ResetSpeedPidOutputLimitsCw(p_motor);
	}
}

// static void SetDirectionCcw(Motor_T * p_motor)
// {
// 	ResetSpeedPidOutputLimitsCcw(p_motor);
// 	// SetOutputLimits(p_motor, p_motor->ILimitMotoring_Frac16 / 2U, p_motor->ILimitGenerating_Frac16 / 2U, INT16_MAX, 0);
// 	/*
// 		Iq/Id PID always Vq/Vd, clip opposite user direction range, no plugging.
// 		Voltage Feedback Mode active during over current only.
// 	*/
// 	PID_SetOutputLimits(&p_motor->PidIq, 0, INT16_MAX);
// 	PID_SetOutputLimits(&p_motor->PidId, INT16_MIN / 2, INT16_MAX / 2);
// }

// static void SetDirectionCw(Motor_T * p_motor)
// {
// 	ResetSpeedPidOutputLimitsCw(p_motor);
// 	// SetOutputLimits(p_motor, p_motor->ILimitGenerating_Frac16 / 2U, p_motor->ILimitMotoring_Frac16 / 2U, 0, INT16_MAX);
// 	PID_SetOutputLimits(&p_motor->PidIq, INT16_MIN, 0);
// 	PID_SetOutputLimits(&p_motor->PidId, INT16_MIN / 2, INT16_MAX / 2);
// }

void Motor_FOC_SetDirectionCcw(Motor_T * p_motor)
{
	Motor_SetDirectionCcw(p_motor);
	// SetDirectionCcw(p_motor);
	if(p_motor->FeedbackModeFlags.Speed == 1U)
	{
		if(p_motor->FeedbackModeFlags.Current == 1U) 	{ ResetSpeedPidOutputLimitsCcw(p_motor); }					/* Speed PID Output is Iq */
		else 											{ PID_SetOutputLimits(&p_motor->PidSpeed, 0, INT16_MAX); } 	/* Speed PID Output is Vq */
	}
	PID_SetOutputLimits(&p_motor->PidIq, 0, INT16_MAX);
	PID_SetOutputLimits(&p_motor->PidId, INT16_MIN / 2, INT16_MAX / 2);
}

void Motor_FOC_SetDirectionCw(Motor_T * p_motor)
{
	Motor_SetDirectionCw(p_motor);
	// SetDirectionCw(p_motor);
	if(p_motor->FeedbackModeFlags.Speed == 1U)
	{
		if(p_motor->FeedbackModeFlags.Current == 1U) 	{ ResetSpeedPidOutputLimitsCw(p_motor); }					/* Speed PID Output is Iq */
		else 											{ PID_SetOutputLimits(&p_motor->PidSpeed, INT16_MIN, 0); } 	/* Speed PID Output is Vq */
	}
	PID_SetOutputLimits(&p_motor->PidIq, INT16_MIN, 0);
	PID_SetOutputLimits(&p_motor->PidId, INT16_MIN / 2, INT16_MAX / 2);
}

/* Set on Direction change */
void Motor_FOC_SetDirection(Motor_T * p_motor, Motor_Direction_T direction)
{
	(direction == MOTOR_DIRECTION_CCW) ? Motor_FOC_SetDirectionCcw(p_motor) : Motor_FOC_SetDirectionCw(p_motor);
}

void Motor_FOC_SetDirectionForward(Motor_T * p_motor)
{
	(p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW) ? Motor_FOC_SetDirectionCcw(p_motor) : Motor_FOC_SetDirectionCw(p_motor);
}

