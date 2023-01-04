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


static void ResetGains(PID_T * p_pid)
{
// 	p_pid->KiDivisorFreq 	= p_pid->Params.KiDivisor * p_pid->Params.SampleFreq;
// 	p_pid->KdFactorFreq 	= p_pid->Params.KdFactor * p_pid->Params.SampleFreq;
// #ifdef CONFIG_PID_INTEGRAL_32BITS
// 	p_pid->ErrorSumOverflow = INT32_MAX / p_pid->Params.KiFactor;
// 	p_pid->IntegralOverflow = INT32_MAX / p_pid->KiDivisorFreq;
// #endif
	PID_SetKp_Fixed32(p_pid, p_pid->Params.Kp_Fixed32);
	PID_SetKi_Fixed32(p_pid, p_pid->Params.Ki_Fixed32);
	// PID_SetKd_Fixed32(p_pid, p_pid->Params.Kd_Fixed32);
}

void PID_Init(PID_T * p_pid)
{
	if(p_pid->CONFIG.P_PARAMS != 0U) { memcpy(&p_pid->Params, p_pid->CONFIG.P_PARAMS, sizeof(PID_Params_T)); }
	PID_SetOutputLimits(p_pid, INT16_MIN, INT16_MAX);
	ResetGains(p_pid);
	PID_Reset(p_pid);
}

// void PID_Init_Args
// (
// 	PID_T * p_pid, uint32_t calcFreq,
// 	int32_t kpFactor, int32_t kpDivisor,
// 	int32_t kiFactor, int32_t kiDivisor,
// 	int32_t kdFactor, int32_t kdDivisor,
// 	int32_t outMin, int32_t outMax
// )
// {
// 	PID_SetFreq(p_pid, calcFreq);
// 	PID_SetTunings(p_pid, kpFactor, kpDivisor, kiFactor, kiDivisor, kdFactor, kdDivisor);
// 	PID_SetOutputLimits(p_pid, outMin, outMax);
// 	ResetGains(p_pid);
// 	PID_Reset(p_pid);
// }

// #ifdef CONFIG_PID_INTEGRAL_32BITS
// static int32_t GetIntegral(PID_T * p_pid)
// {
// 	return ((p_pid->ErrorSum > p_pid->ErrorSumOverflow) || (p_pid->ErrorSum < 0 - p_pid->ErrorSumOverflow)) ?
// 		(p_pid->ErrorSum / p_pid->KiDivisorFreq * p_pid->Params.KiFactor) : (p_pid->Params.KiFactor * p_pid->ErrorSum / p_pid->KiDivisorFreq);
// }

// static void SetIntegral(PID_T * p_pid, int32_t integral)
// {
// 	p_pid->ErrorSum = ((integral > p_pid->IntegralOverflow) || (integral < 0 - p_pid->IntegralOverflow)) ?
// 		(integral / p_pid->Params.KiFactor * p_pid->KiDivisorFreq) : (integral * p_pid->KiDivisorFreq / p_pid->Params.KiFactor);
// }
// #endif

static inline int16_t GetIntegral(PID_T * p_pid) { return (p_pid->Integral32 >> 16); }
static inline void SetIntegral(PID_T * p_pid, int16_t integral) { p_pid->Integral32 = ((int32_t)integral << 16); }

/*
	Conventional parallel PID calculation
	control = (Kp * error) + (Ki * ErrorSum * SampleTime) + (Kd * (error - ErrorPrev) / SampleTime)
*/
static inline int32_t CalcPid(PID_T * p_pid, int32_t error)
{
	int32_t control, proportional, integral32, integral, derivative, integralMin, integralMax;

	// proportional = (p_pid->Params.KpFactor * error / p_pid->Params.KpDivisor);

	proportional = (p_pid->Params.PropGain * error >> p_pid->Params.PropGainShift); /* Inclusive of 16 shift */

	// integral = GetIntegral(p_pid);
	/* Backward rectangular approximation */
	/* Store as Sum */
	/* SumFactor * (errorSum >> (0 - SumFactorShift)) >> 16 */

	/* Forward rectangular approximation */
	/* Store as Integral. Allows compute time gain adjustment. */
	integral32 = p_pid->Integral32 + (p_pid->Params.IntegralGain * error >> p_pid->Params.IntegralGainShift);
	integral = integral32 >> 16;

	/* Dynamic Clamp */
	integralMin = math_min(p_pid->OutputMin - proportional, 0);
	integralMax = math_max(p_pid->OutputMax - proportional, 0);

	if 		(integral > integralMax) 	{ integral = integralMax; if(error < 0) { SetIntegral(p_pid, integralMax); } }
	else if	(integral < integralMin) 	{ integral = integralMin; if(error > 0) { SetIntegral(p_pid, integralMin); } }
	else 								{ p_pid->Integral32 = integral32; }

	// if(p_pid->Params.Mode == PID_MODE_PID)
	// {
	// 	derivative = (p_pid->KdFactorFreq * (error - p_pid->ErrorPrev) / p_pid->Params.KdDivisor);
	// 	p_pid->ErrorPrev = error;
	// 	control = proportional + integral + derivative;
	// }
	// else /* (p_pid->Mode == PID_MODE_PI) */
	{
		control = proportional + integral;
	}

	// if		(control > p_pid->OutputMax) { control = p_pid->OutputMax; }
	// else if	(control < p_pid->OutputMin) { control = p_pid->OutputMin; }

	// return control;

	return math_clamp(control, p_pid->OutputMin, p_pid->OutputMax);
}

int32_t PID_Proc(PID_T * p_pid, int32_t setpoint, int32_t feedback)
{
	return CalcPid(p_pid, setpoint - feedback);
}

/*
	Compute Time variables Set
*/
void PID_Reset(PID_T * p_pid)
{
	p_pid->Integral32 = 0;
	p_pid->ErrorPrev = 0;
}

void PID_SetIntegral(PID_T * p_pid, int16_t integral)
{
	int32_t integralLimited = math_clamp(integral, p_pid->OutputMin, p_pid->OutputMax);
	SetIntegral(p_pid, integralLimited);
}

void PID_SetOutputState(PID_T * p_pid, int16_t integral)
{
	PID_SetIntegral(p_pid, integral);
}

void PID_SetOutputLimits(PID_T * p_pid, int16_t min, int16_t max)
{
	int32_t integral;

	if(max > min)
	{
		p_pid->OutputMin = min;
		p_pid->OutputMax = max;
		// integral = GetIntegral(p_pid); /* Reset integral with limits */
		// if		(integral > p_pid->OutputMax) { SetIntegral(p_pid, p_pid->OutputMax); }
		// else if	(integral < p_pid->OutputMin) { SetIntegral(p_pid, p_pid->OutputMin); }
	}
}

/*
	Persistent Params Set
*/
void PID_SetFreq(PID_T * p_pid, uint32_t calcFreq)
{
	if(calcFreq > 0U) { p_pid->Params.SampleFreq = calcFreq; }
}

/*
	Proportional(k) = Kp * error(k) = kp_fixed32 * error(k) >> 16
	PropFactor >> PropDivisorShift = kp_fixed32 >> 16
*/
void PID_SetKp_Fixed32(PID_T * p_pid, int32_t kp_Fixed32)
{
	uint8_t maxShift = q_maxshift_signed(kp_Fixed32); /* kp_fixed32 / 65536 * 65536, error max is (32,767 - (-32,768)) */
	p_pid->Params.PropGainShift = (maxShift < 16U) ? maxShift : 16U;
	p_pid->Params.PropGain = kp_Fixed32 >> (16U - p_pid->Params.PropGainShift);
	// PropFactorShift = q_log2_ceiling(kp_Fixed32/65536);
	// PropFactor * error(k) >> (16 - PropFactorShift)
	/*
		65536 -> 16,384, 2
		65535 -> 32,767, 1
	*/
}

int32_t PID_GetKp_Fixed32(PID_T * p_pid) { return p_pid->Params.PropGain << (16U - p_pid->Params.PropGainShift); }

/*
	Integral(k) = Ki * (error(k) + errorSum(k-1)) / SampleFreq
				= ki_fixed32 * (error(k) + errorSum(k-1)) / SampleFreq >> 16
*/
/* Integral clamped to 32,767, ErrorSum Max = INT32_MAX / SumFactor = 32,767 << SumDivisorShift / SumFactor */
//ki = 70000
//factor = ki * (l << log2(SampleFreq) / SampleFreq = 57,344
//divisor = 65536 * (l << log2(SampleFreq))) = 1,073,741,824

// PropFactor kiTime 1 << 14,-13
// PropFactorShift = -14,-13

//factor = ki * (l << log2( ) / SampleFreq = 28,672
//divisor = 65536 * (l << log2( ))) = 536,870,912
//  14 * (error(k) + errorSum(k-1)) >> 18 = 32,768
//613,566,756
void PID_SetKi_Fixed32(PID_T * p_pid, int32_t ki_Fixed32)
{
	int32_t kiTime = ki_Fixed32 / p_pid->Params.SampleFreq;
	int32_t shift = 15U - q_log2_ceiling(kiTime);

	p_pid->Params.IntegralGainShift = shift;
	p_pid->Params.IntegralGain = ki_Fixed32 << shift / p_pid->Params.SampleFreq;
}

// int32_t PID_GetKi_Fixed32(PID_T * p_pid) { return p_pid->SumFactor << (16U - p_pid->SumDivisorShift); }

// void PID_SetTunings(PID_T * p_pid, int32_t kpFactor, int32_t kpDivisor, int32_t kiFactor, int32_t kiDivisor, int32_t kdFactor, int32_t kdDivisor)
// {
// 	p_pid->Params.KpFactor 		= kpFactor;
// 	p_pid->Params.KpDivisor 	= kpDivisor;
// 	p_pid->Params.KiFactor 		= kiFactor;
// 	p_pid->Params.KiDivisor 	= kiDivisor;
// 	p_pid->Params.KdFactor 		= kdFactor;
// 	p_pid->Params.KdDivisor 	= kdDivisor;
// 	ResetRuntime(p_pid);
// }



