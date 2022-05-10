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
static void SetOutputLimits(Motor_T * p_motor, int16_t speedIOutCcw, int16_t speedIOutCw, int16_t vOutCcw, int16_t vOutCw)
{
	if(p_motor->FeedbackModeFlags.Speed == 1U)
	{
		(p_motor->FeedbackModeFlags.Current == 1U) ?
			PID_SetOutputLimits(&p_motor->PidSpeed, 0 - speedIOutCw, speedIOutCcw) :  	/* Speed PID is Iq */
			PID_SetOutputLimits(&p_motor->PidSpeed, 0 - vOutCw, vOutCcw);  	/* Speed PID is Vq, VLimit output proptional to ILimit */
	}

	/*
		Iq/Id PID always Vq/Vd, full range, no plugging.
		Voltage Feedback Mode active during over current only
	*/
	PID_SetOutputLimits(&p_motor->PidIq, 0 - vOutCw, vOutCcw);
	PID_SetOutputLimits(&p_motor->PidId, 0 - p_motor->ILimitMotoring_Frac16 / 2, p_motor->ILimitMotoring_Frac16 / 2); /* Id use 50% of Iq Motoring) */
}

void Motor_FOC_SetOutputLimitsCcw(Motor_T * p_motor)
{
	SetOutputLimits(p_motor, p_motor->ILimitMotoring_Frac16 / 2U, p_motor->ILimitGenerating_Frac16 / 2U, 32767, 0);
}

void Motor_FOC_SetOutputLimitsCw(Motor_T * p_motor)
{
	SetOutputLimits(p_motor, p_motor->ILimitGenerating_Frac16 / 2U, p_motor->ILimitMotoring_Frac16 / 2U, 0, 32767);
}

void Motor_FOC_SetDirectionCcw(Motor_T * p_motor) 	{ Motor_SetDirectionCcw(p_motor); Motor_FOC_SetOutputLimitsCcw(p_motor); }
void Motor_FOC_SetDirectionCw(Motor_T * p_motor) 	{ Motor_SetDirectionCw(p_motor); Motor_FOC_SetOutputLimitsCw(p_motor); }

void Motor_FOC_SetDirection(Motor_T * p_motor, Motor_Direction_T direction)
{
	(direction == MOTOR_DIRECTION_CCW) ? Motor_FOC_SetDirectionCcw(p_motor) : Motor_FOC_SetDirectionCw(p_motor);
}

void Motor_FOC_SetDirectionForward(Motor_T * p_motor)
{
	(p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW) ? Motor_FOC_SetDirectionCcw(p_motor) : Motor_FOC_SetDirectionCw(p_motor);
}
