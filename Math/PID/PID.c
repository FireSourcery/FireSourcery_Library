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
    @file 	PID.c
    @author FireSoucery
    @brief
    @version V0
*/
/******************************************************************************/
#include "PID.h"

#include <stdbool.h>
#include <stdint.h>

/*
	Standard PID calculation
	control = (Kp * error)
			+ (Ki * ErrorSum * SampleTime)
			+ (Kd * (error - ErrorPrev) / SampleTime)
 */
static inline int32_t CalcPid(PID_T * p_pid, int32_t error)
{
	int32_t control, proportional, integral, derivative;

	proportional = (p_pid->Params.KpFactor * error / p_pid->Params.KpDivisor);

//	p_pid->ErrorSum += error;

	integral = (p_pid->Params.KiFactor * p_pid->ErrorSum / p_pid->KiDivisorFreq);

	if 		(integral > p_pid->Params.OutMax) 	{integral = p_pid->Params.OutMax;}
	else if (integral < p_pid->Params.OutMin) 	{integral = p_pid->Params.OutMin;}
	else 										{p_pid->ErrorSum += error;} //stop accumulating integral if output is past limits

	if(p_pid->Params.Mode == PID_MODE_PID)
	{
		derivative = (p_pid->KdFactorFreq * (error - p_pid->ErrorPrev) / p_pid->Params.KdDivisor);
		p_pid->ErrorPrev = error;
		control = proportional + integral + derivative;
	}
	else //(p_pid->Mode == PID_MODE_PI)
	{
		control = proportional + integral;
	}

	if 		(control > p_pid->Params.OutMax) {control = p_pid->Params.OutMax;}
	else if (control < p_pid->Params.OutMin) {control = p_pid->Params.OutMin;}

	return control;
}

int32_t PID_Calc(PID_T * p_pid, int32_t setpoint, int32_t feedback)
{
	return CalcPid(p_pid, setpoint - feedback);
}


void PID_SetIntegral(PID_T * p_pid, int32_t integral)
{
	p_pid->ErrorSum = integral * p_pid->KiDivisorFreq / p_pid->Params.KiFactor;
}

void PID_SetFreq(PID_T * p_pid, uint32_t calcFreq)
{
   if (calcFreq > 0U)
   {
      p_pid->Params.CalcFreq = calcFreq;
   }
}

void PID_SetTunings(PID_T * p_pid, int32_t kpFactor, int32_t kpDivisor, int32_t kiFactor, int32_t kiDivisor, int32_t kdFactor, int32_t kdDivisor)
{
	p_pid->Params.KpFactor 		= kpFactor;
	p_pid->Params.KpDivisor 	= kpDivisor;
	p_pid->Params.KiFactor 		= kiFactor;
	p_pid->Params.KiDivisor 	= kiDivisor;
	p_pid->Params.KdFactor 		= kdFactor;
	p_pid->Params.KdDivisor 	= kdDivisor;

	if (p_pid->Params.Direction == PID_DIRECTION_REVERSE)
	{
		p_pid->Params.KpFactor = 0 - kpFactor;
		p_pid->Params.KiFactor = 0 - kiFactor;
		p_pid->Params.KdFactor = 0 - kdFactor;
	}

	p_pid->KiDivisorFreq = p_pid->Params.KiDivisor * p_pid->Params.CalcFreq;
	p_pid->KdFactorFreq = p_pid->Params.KdFactor * p_pid->Params.CalcFreq;
}

void PID_SetTunings_FractionSigned16(PID_T * p_pid, int32_t kp, int32_t ki, int32_t kd)
{
	PID_SetTunings(p_pid, kp, 32768, ki, 32768, kd, 32768);
}

void PID_SetOutputLimits(PID_T * p_pid, uint32_t min, uint32_t max)
{
	if (max > min)
	{
		p_pid->Params.OutMin = min;
		p_pid->Params.OutMax = max;
		if		(p_pid->ErrorSum > p_pid->Params.OutMax) {p_pid->ErrorSum = p_pid->Params.OutMax;}
		else if	(p_pid->ErrorSum < p_pid->Params.OutMin) {p_pid->ErrorSum = p_pid->Params.OutMin;}
	}
}

void PID_SetDirection(PID_T * p_pid, PID_Direction_T direction)
{
	if (direction != p_pid->Params.Direction)
	{
		p_pid->Params.KpFactor = 0 - p_pid->Params.KpFactor;
		p_pid->Params.KiFactor = 0 - p_pid->Params.KiFactor;
		p_pid->Params.KdFactor = 0 - p_pid->Params.KdFactor;
	}
	p_pid->Params.Direction = direction;
}

void PID_Reset(PID_T * p_pid)
{
	p_pid->ErrorSum 	= 0;
	p_pid->ErrorPrev 	= 0;
}

//int32_t PID_GetKp_Frac16(PID_T * p_pid) {return  p_pid->Params.Kp;}
//int32_t PID_GetKi_Frac16(PID_T * p_pid) {return  p_pid->Params.Ki;}
//int32_t PID_GetKd_Frac16(PID_T * p_pid) {return  p_pid->Params.Kd;}

int32_t PID_GetKp_Int(PID_T * p_pid, uint16_t scalar) {return scalar * p_pid->Params.KpFactor / p_pid->Params.KpDivisor;}
int32_t PID_GetKi_Int(PID_T * p_pid, uint16_t scalar) {return scalar *  p_pid->Params.KiFactor / p_pid->Params.KiDivisor;}
int32_t PID_GetKd_Int(PID_T * p_pid, uint16_t scalar) {return scalar *  p_pid->Params.KdFactor / p_pid->Params.KdDivisor;}

PID_Direction_T PID_GetDirection(PID_T *p_pid)
{
	return p_pid->Params.Direction;
}

void PID_Init(PID_T * p_pid)
{
	if (p_pid->CONFIG.P_PARAMS != 0U)
	{
		memcpy(&p_pid->Params, p_pid->CONFIG.P_PARAMS, sizeof(PID_Params_T));
	}

	p_pid->KiDivisorFreq = p_pid->Params.KiDivisor * p_pid->Params.CalcFreq;
	p_pid->KdFactorFreq = p_pid->Params.KdFactor * p_pid->Params.CalcFreq;

	p_pid->ErrorSum = 0;
	p_pid->ErrorPrev = 0;
}

void PID_Init_Params
(
	PID_T * p_pid,
	uint32_t calcFreq,
	int32_t kpFactor, int32_t kpDivisor,
	int32_t kiFactor, int32_t kiDivisor,
	int32_t kdFactor, int32_t kdDivisor,
	int32_t outMin, int32_t outMax
)
{
	PID_SetCalcFreq(p_pid, calcFreq);
	PID_SetTunings(p_pid, kpFactor, kpDivisor, kiFactor, kiDivisor, kdFactor, kdDivisor);
	PID_SetOutputLimits(p_pid, outMin, outMax);
	p_pid->ErrorSum = 0;
	p_pid->ErrorPrev = 0;
	p_pid->Params.Direction = PID_DIRECTION_DIRECT;
}
