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
	@file 	PID.c
	@author FireSourcery
	@brief
	@version V0
*/
/******************************************************************************/
#include "PID.h"
#include <string.h>


static void ResetRuntime(PID_T * p_pid)
{
	p_pid->KiDivisorFreq 	= p_pid->Params.KiDivisor * p_pid->Params.CalcFreq;
	p_pid->KdFactorFreq 	= p_pid->Params.KdFactor * p_pid->Params.CalcFreq;
	p_pid->ErrorSumOverflow = INT32_MAX / p_pid->Params.KiFactor;
}

void PID_Init(PID_T * p_pid)
{
	if(p_pid->CONFIG.P_PARAMS != 0U) { memcpy(&p_pid->Params, p_pid->CONFIG.P_PARAMS, sizeof(PID_Params_T)); }
	p_pid->OutputMin = -65535;
	p_pid->OutputMax = 65535;
	ResetRuntime(p_pid);
	PID_Reset(p_pid);
}

void PID_Init_Args
(
	PID_T * p_pid,
	uint32_t calcFreq,
	int32_t kpFactor, int32_t kpDivisor,
	int32_t kiFactor, int32_t kiDivisor,
	int32_t kdFactor, int32_t kdDivisor,
	int32_t outMin, int32_t outMax
)
{
	PID_SetFreq(p_pid, calcFreq);
	PID_SetTunings(p_pid, kpFactor, kpDivisor, kiFactor, kiDivisor, kdFactor, kdDivisor);
	PID_SetOutputLimits(p_pid, outMin, outMax);
	ResetRuntime(p_pid);
	PID_Reset(p_pid);
}


static int32_t GetIntegral(PID_T * p_pid)
{
	int32_t integral;

	if((p_pid->ErrorSum > p_pid->ErrorSumOverflow) || (p_pid->ErrorSum < 0 - p_pid->ErrorSumOverflow))
	{
		integral = p_pid->ErrorSum / p_pid->KiDivisorFreq * p_pid->Params.KiFactor;
	}
	else
	{
		integral = p_pid->Params.KiFactor * p_pid->ErrorSum / p_pid->KiDivisorFreq;
	}

	return integral;
}

static void SetIntegral(PID_T * p_pid, int32_t integral)
{
	int32_t integralOverflow = INT32_MAX / p_pid->KiDivisorFreq;

	if((integral > integralOverflow) || (integral < 0 - integralOverflow))
	{
		p_pid->ErrorSum = integral / p_pid->Params.KiFactor * p_pid->KiDivisorFreq;
	}
	else
	{
		p_pid->ErrorSum = integral * p_pid->KiDivisorFreq / p_pid->Params.KiFactor;
	}
}


/*
	Standard PID calculation
	control = (Kp * error) + (Ki * ErrorSum * SampleTime) + (Kd * (error - ErrorPrev) / SampleTime)
*/
static inline int32_t CalcPid(PID_T * p_pid, int32_t error)
{
	int32_t control, proportional, integral, derivative;

	proportional = (p_pid->Params.KpFactor * error / p_pid->Params.KpDivisor);

	integral = GetIntegral(p_pid);

	if(integral > p_pid->OutputMax)
	{
		integral = p_pid->OutputMax;
		if(error < 0) { p_pid->ErrorSum += error; } /* only if error sum becomes out of sync, add error if error is negative */ //may need p_pid->OutputMax > 0
	}
	else if(integral < p_pid->OutputMin)
	{
		integral = p_pid->OutputMin;
		if(error > 0) { p_pid->ErrorSum += error; } /* only if error sum becomes out of sync, add error if error is positive */
	}
	else
	{
		p_pid->ErrorSum += error; /* stops accumulating integral if output is past limits */
	}

	if(p_pid->Params.Mode == PID_MODE_PID)
	{
		derivative = (p_pid->KdFactorFreq * (error - p_pid->ErrorPrev) / p_pid->Params.KdDivisor);
		p_pid->ErrorPrev = error;
		control = proportional + integral + derivative;
	}
	else /* (p_pid->Mode == PID_MODE_PI) */
	{
		control = proportional + integral;
	}

	if		(control > p_pid->OutputMax) { control = p_pid->OutputMax; }
	else if	(control < p_pid->OutputMin) { control = p_pid->OutputMin; }

	return control;
}

int32_t PID_Calc(PID_T * p_pid, int32_t setpoint, int32_t feedback)
{
	return CalcPid(p_pid, setpoint - feedback);
}

/*
	Compute Time variables Set
*/
void PID_Reset(PID_T * p_pid)
{
	p_pid->ErrorSum = 0;
	p_pid->ErrorPrev = 0;
}

void PID_SetIntegral(PID_T * p_pid, int32_t integral)
{
	if		(integral > p_pid->OutputMax) 	{ SetIntegral(p_pid, p_pid->OutputMax); }
	else if	(integral < p_pid->OutputMin) 	{ SetIntegral(p_pid, p_pid->OutputMin); }
	else 									{ SetIntegral(p_pid, integral);}
}

void PID_SetOutputState(PID_T * p_pid, int32_t integral)
{
	PID_SetIntegral(p_pid, integral);
}

void PID_SetOutputLimits(PID_T * p_pid, int32_t min, int32_t max)
{
	int32_t integral;

	if(max > min)
	{
		p_pid->OutputMin = min;
		p_pid->OutputMax = max;
		integral = GetIntegral(p_pid); /* Reset integral with limits */
		if		(integral > p_pid->OutputMax) { SetIntegral(p_pid, p_pid->OutputMax); }
		else if	(integral < p_pid->OutputMin) { SetIntegral(p_pid, p_pid->OutputMin); }
	}
}

/*
	Persistent Params Set
*/
void PID_SetFreq(PID_T * p_pid, uint32_t calcFreq)
{
	if(calcFreq > 0U) { p_pid->Params.CalcFreq = calcFreq; }
}

void PID_SetTunings(PID_T * p_pid, int32_t kpFactor, int32_t kpDivisor, int32_t kiFactor, int32_t kiDivisor, int32_t kdFactor, int32_t kdDivisor)
{
	p_pid->Params.KpFactor 		= kpFactor;
	p_pid->Params.KpDivisor 	= kpDivisor;
	p_pid->Params.KiFactor 		= kiFactor;
	p_pid->Params.KiDivisor 	= kiDivisor;
	p_pid->Params.KdFactor 		= kdFactor;
	p_pid->Params.KdDivisor 	= kdDivisor;
	ResetRuntime(p_pid);
}

void PID_SetTunings_FractionSigned16(PID_T * p_pid, int32_t kp, int32_t ki, int32_t kd)
{
	/* scale down ki overflow */
	// int32_t kiFactor = ki >> 12U;
	// int32_t kiDivisor = 32768 >> 12U;
	// while ()

	PID_SetTunings(p_pid, kp, 32768, ki, 32768, kd, 32768);
}

int32_t PID_GetKp_FractionSigned16(PID_T * p_pid) { return 32768 * p_pid->Params.KpFactor / p_pid->Params.KpDivisor; }
int32_t PID_GetKi_FractionSigned16(PID_T * p_pid) { return 32768 * p_pid->Params.KiFactor / p_pid->Params.KiDivisor; }
int32_t PID_GetKd_FractionSigned16(PID_T * p_pid) { return 32768 * p_pid->Params.KdFactor / p_pid->Params.KdDivisor; }

int32_t PID_GetKp_Int(PID_T * p_pid, uint16_t scalar) { return scalar * p_pid->Params.KpFactor / p_pid->Params.KpDivisor; }
int32_t PID_GetKi_Int(PID_T * p_pid, uint16_t scalar) { return scalar * p_pid->Params.KiFactor / p_pid->Params.KiDivisor; }
int32_t PID_GetKd_Int(PID_T * p_pid, uint16_t scalar) { return scalar * p_pid->Params.KdFactor / p_pid->Params.KdDivisor; }
