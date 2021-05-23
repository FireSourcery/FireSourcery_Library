/**************************************************************************/
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
/**************************************************************************/
/**************************************************************************/
/*!
    @file 	PID.c
    @author FireSoucery
    @brief
    @version V0

    Created on: Nov 26, 2019
*/
/**************************************************************************/
#include "PID.h"

#include <stdbool.h>
#include <stdint.h>

/*
	Standard PID calculation
	control = (Kp * error)
			+ (Ki * ErrorSum * SampleTime)
			+ (Kd * (error - ErrorPrev) / SampleTime)

 */
static inline int32_t CalcPID(PID_T * p_pid, int32_t error)
{
	int32_t control = 0;

	int32_t proportional, intergral, derivative;

	proportional = (p_pid->KpFactor * error / p_pid->KpDivisor);

	p_pid->ErrorSum += error;

	if 		(p_pid->ErrorSum > p_pid->OutMax) {p_pid->ErrorSum = p_pid->OutMax;}
	else if (p_pid->ErrorSum < p_pid->OutMin) {p_pid->ErrorSum = p_pid->OutMin;}

	intergral = (p_pid->KiFactor * p_pid->ErrorSum / p_pid->KiDivisorCalcFreq);

//	p_pid->ErrorSum += error * p_pid->KiFactor; on the fly ki adjustments
//
//	if 		(p_pid->ErrorSum > p_pid->OutMax) {p_pid->ErrorSum = p_pid->OutMax;}
//	else if (p_pid->ErrorSum < p_pid->OutMin) {p_pid->ErrorSum = p_pid->OutMin;}
//
//	intergral = (p_pid->ErrorSum / p_pid->KiDivisorCalcFreq);

	derivative = (p_pid->KdFactorCalcFreq * (error - p_pid->ErrorPrev) / p_pid->KdDivisor);

	p_pid->ErrorPrev = error;

	control = proportional + intergral + derivative;

	if 		(control > p_pid->OutMax) {control = p_pid->OutMax;}
	else if (control < p_pid->OutMin) {control = p_pid->OutMin;}

	return control;
}

int32_t PID_Calc(PID_T *p_pid, int32_t setpoint, int32_t feedback)
{
	return CalcPID(p_pid, feedback-setpoint);
}

void PID_SetTunings(PID_T * p_pid, int32_t kpFactor, int32_t kpDivisor, int32_t kiFactor, int32_t kiDivisor, int32_t kdFactor, int32_t kdDivisor)
{
	p_pid->KpFactor 	= kpFactor;
	p_pid->KpDivisor 	= kpDivisor;
	p_pid->KiFactor 	= kiFactor;
	p_pid->KiDivisor 	= kiDivisor;
	p_pid->KdFactor 	= kdFactor;
	p_pid->KdDivisor 	= kdDivisor;

	p_pid->KiDivisorCalcFreq = kiDivisor * p_pid->CalcFreq;
	p_pid->KdFactorCalcFreq = kdFactor * p_pid->CalcFreq;

	if (p_pid->Direction == PID_DIRECTION_REVERSE)
	{
		p_pid->KpFactor = 0 - kpFactor;
		p_pid->KiFactor = 0 - kiFactor;
		p_pid->KdFactor = 0 - kdFactor;
	}
}

void PID_SetCalcFreq(PID_T * p_pid, uint32_t calcFreq)
{
   if (calcFreq > 0U)
   {
      p_pid->CalcFreq = calcFreq;
   }
}

void PID_SetOutputLimits(PID_T * p_pid, uint32_t min, uint32_t max)
{
	if (max > min)
	{
		p_pid->OutMin = min;
		p_pid->OutMax = max;
#ifdef CONFIG_PID_MODE_POINTER
	   if		(*p_pid->p_Control > p_pid->OutMax) {*p_pid->p_Control = p_pid->OutMax;}
	   else if	(*p_pid->p_Control < p_pid->OutMin) {*p_pid->p_Control = p_pid->OutMin;}
#endif
		if		(p_pid->ErrorSum > p_pid->OutMax) {p_pid->ErrorSum = p_pid->OutMax;}
		else if	(p_pid->ErrorSum < p_pid->OutMin) {p_pid->ErrorSum = p_pid->OutMin;}
	}
}

void PID_SetDirection(PID_T * p_pid, PID_Direction_T direction)
{
	if ((direction != p_pid->Direction))
	{
		p_pid->KpFactor = 0 - p_pid->KpFactor;
		p_pid->KiFactor = 0 - p_pid->KiFactor;
		p_pid->KdFactor = 0 - p_pid->KdFactor;
	}
	p_pid->Direction = direction;
}

//int32_t PID_GetKp(PID_T * p_pid) {return  p_pid->Kp;}
//int32_t PID_GetKi(PID_T * p_pid) {return  p_pid->Ki;}
//int32_t PID_GetKd(PID_T * p_pid) {return  p_pid->Kd;}

PID_Direction_T PID_GetDirection(PID_T *p_pid)
{
	return p_pid->Direction;
}

void PID_Init
(
	PID_T * p_pid,
#ifdef CONFIG_PID_MODE_POINTER
	int32_t * p_setpoint, int32_t * p_feedback, int32_t * p_control,
#endif
	int32_t kpFactor, int32_t kpDivisor,
	int32_t kiFactor, int32_t kiDivisor,
	int32_t kdFactor, int32_t kdDivisor,
	uint32_t calcFreq,
	int32_t outMin, int32_t outMax
)
{
#ifdef CONFIG_PID_MODE_POINTER
	p_pid->p_Setpoint = p_setPoint;
	p_pid->p_Output = p_output;
	p_pid->p_Feedback = p_feedback;
#endif

	p_pid->CalcFreq = calcFreq;

	PID_SetTunings(p_pid, kpFactor, kpDivisor, kiFactor, kiDivisor, kdFactor, kdDivisor);
	PID_SetOutputLimits(p_pid, outMin, outMax);
	p_pid->ErrorSum = 0;
	p_pid->ErrorPrev = 0;

	p_pid->Direction = PID_DIRECTION_DIRECT;
}

//bool PID_PollCalc(PID_T *p_pid)
//{
//	bool status;
//	if (p_pid->CalcCounter < p_pid->CalcPeriod)
//	{
//		p_pid->CalcCounter++;
//		status = false;
//	}
//	else
//	{
//		p_pid->CalcCounter = 0;
//		p_pid->Control = CalcPID(p_pid, p_pid->Setpoint - p_pid->Feedback);
//		status = true;
//	}
//	return status;
//}
