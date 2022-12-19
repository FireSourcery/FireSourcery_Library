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
	@file 	Linear.c
	@author FireSourcery
	@brief
	@version V0
*/
/******************************************************************************/
#include "Linear.h"


/******************************************************************************/
/*!
	Linear
	f(x) = (factor * (x - x0) / divisor) + y0
	f(xRef) = yRef

	x0 implementation: equations include 2 offsets. always include 1 additional operation
	alternatively
		option 1. save y-intercept as shifted to preserve precision
		option 2. selectively calc with 1 offset at function call.
			reuse procedure inv functions, + supplement inv frac16 functions
	Overflow factor, divisor > 131,071
*/
/******************************************************************************/
/*
	x0 = 0
*/
void Linear_Init(Linear_T * p_linear, int32_t factor, int32_t divisor, int32_t y0, int32_t yRef)
{
	p_linear->XOffset = 0;
	p_linear->YOffset = y0;
#ifdef CONFIG_LINEAR_DIVIDE_SHIFT
	/*
		if factor > divisor, bound with yref.
		invf overflow if  (divisor * yRef) > INT32_MAX
	*/
	p_linear->YReference = yRef;
	p_linear->XReference = linear_invf(factor, divisor, y0, yRef); /* (yRef - y0)*divisor/factor */

	/*
		Allow max input of x = XReference * 2
			XReference * 2 * Slope_Shifted <= INT32_MAX

			(1 << SlopeShift) <= INT32_MAX / (XReference * 2 * factor) * divisor
			(1 << SlopeShift) <= INT32_MAX / 2 / factor * divisor / XReference
		//todo determine max shift, factor > 65536
	*/
	p_linear->Slope = (factor << LINEAR_DIVIDE_SHIFT) / divisor;
	p_linear->SlopeShift = LINEAR_DIVIDE_SHIFT;

	/*	Iterative log2	*/
	while((p_linear->XReference > INT32_MAX / p_linear->Slope) && (p_linear->SlopeShift > 0U))
	{
		p_linear->Slope = p_linear->Slope >> 1U;
		p_linear->SlopeShift--;
	}

	//todo maxleftshift, if factor > divisor, invslope can be > 14
	p_linear->InvSlope = (divisor << LINEAR_DIVIDE_SHIFT) / factor;
	p_linear->InvSlopeShift = LINEAR_DIVIDE_SHIFT;

	while((p_linear->YReference - y0 > INT32_MAX / p_linear->InvSlope) && (p_linear->InvSlopeShift > 0U))
	{
		p_linear->InvSlope = p_linear->InvSlope >> 1U;
		p_linear->InvSlopeShift--;
	}

#elif defined(CONFIG_LINEAR_DIVIDE_NUMERICAL)
	p_linear->SlopeFactor = factor;
	p_linear->SlopeDivisor = divisor;
	p_linear->YOffset = y0;
	p_linear->YReference = yRef;
#endif
}

// void Linear_Init(Linear_T * p_linear, int32_t factor, int32_t divisor, int32_t x0, int32_t xRef)
// {

// }

// void Linear_Init(Linear_T * p_linear, int32_t factor, int32_t divisor, int32_t x0, int32_t y0, int32_t yRef)
// {

// }

// void Linear_Init(Linear_T * p_linear, int32_t factor, int32_t divisor, int32_t x0, int32_t y0, int32_t xRef)
// {

// }

/*
	Map [x0:xRef] to [y0:yRef]. Interpolate from (x0, y0) to (xRef, yRef).
	Derive slope
	User input yRef > y0, XRef > x0
*/
void Linear_Init_Map(Linear_T * p_linear, int32_t x0, int32_t xRef, int32_t y0, int32_t yRef)
{
#ifdef CONFIG_LINEAR_DIVIDE_SHIFT
	p_linear->YReference = yRef;
	p_linear->XReference = xRef;
	p_linear->Slope = ((yRef - y0) << LINEAR_DIVIDE_SHIFT) / (xRef - x0);
	p_linear->SlopeShift = LINEAR_DIVIDE_SHIFT;
	p_linear->InvSlope = ((xRef - x0) << LINEAR_DIVIDE_SHIFT) / (yRef - y0);
	p_linear->InvSlopeShift = LINEAR_DIVIDE_SHIFT;
	p_linear->XOffset = x0;
	p_linear->YOffset = y0;
#endif
}

/******************************************************************************/
/*!
	Scalar
*/
/******************************************************************************/
/* scalar may be compile time constant, can compiler unroll loop to optimize? */
int32_t Linear_Function_Scalar(const Linear_T * p_linear, int32_t x, uint16_t scalar)
{
	int32_t factor = x * p_linear->Slope;
	int32_t result = 0U;

	/*
		Loop N = Log_[Divisor*=](scalar)
		scalar 1000
			[Divisor*=] == 10 => 4
			[Divisor*=] == 2 => 10
	*/
	for(uint16_t iDivisor = 1U; scalar >= iDivisor; iDivisor *= 4U) 	/* scalar / iDivisor > 0U */
	{
		if(factor < INT32_MAX / scalar * iDivisor) /* (factor < INT32_MAX / (scalar / iDivisor)) */
		{
			result = Linear_Function(p_linear, x * scalar / iDivisor) * iDivisor;
			break;
		}
	}

	if(result == 0) { result = Linear_Function(p_linear, x) * scalar; }

	return result;
}

int32_t Linear_InvFunction_Scalar(const Linear_T * p_linear, int32_t y, uint16_t scalar)
{
	(void)p_linear;
	(void)y;
	(void)scalar;
	return 0; //todo
}