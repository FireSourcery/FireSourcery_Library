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
	@file 	Linear.c
	@author FireSoucery
	@brief
	@version V0
*/
/******************************************************************************/
#include "Linear.h"

/*
	f(x) = (factor * (x - x0) / divisor) + y0
	f(x[100%] == xref) = yRef

	factor > divisor, bound with yref
	todo divisor > factor, bound with xref

	x0 implementation: equations include 2 offsets. always include 1 additional operation
	alternatively
		option 1. save y-intercept as shifted to preserve precision
		option 2. selectively calc with 1 offset at function call.
			reuse procedure inv functions, + supplement inv frac16 functions
*/

void Linear_Init(Linear_T * p_linear, int32_t factor, int32_t divisor, int32_t y0, int32_t yRef)
{
#ifdef CONFIG_LINEAR_DIVIDE_SHIFT
	p_linear->YReference = yRef;
	p_linear->XReference = linear_invf(factor, divisor, y0, yRef); /* divisor*yRef/factor */

	//todo determine max shift if factor > 65536
	p_linear->Slope = (factor << 14U) / divisor;
	p_linear->SlopeShift = 14U;

	//todo non iterative, log2 
	while((p_linear->XReference > INT32_MAX / p_linear->Slope) && (p_linear->SlopeShift > 0U))
	{
		p_linear->Slope = p_linear->Slope >> 1U;
		p_linear->SlopeShift--;
	}

	//todo maxleftshift, if factor > divisor, invslope can be > 14
	p_linear->InvSlope = (divisor << 14U) / factor;
	p_linear->InvSlopeShift = 14U;

	while((p_linear->YReference - y0 > INT32_MAX / p_linear->InvSlope) && (p_linear->InvSlopeShift > 0U))
	{
		p_linear->InvSlope = p_linear->InvSlope >> 1U;
		p_linear->InvSlopeShift--;
	}

	p_linear->XOffset = 0;
	p_linear->YOffset = y0;
#elif defined(CONFIG_LINEAR_DIVIDE_NUMERICAL)
	// p_linear->SlopeFactor = factor;
	// p_linear->SlopeDivisor = divisor;
	// p_linear->YOffset = y0;
	// p_linear->YReference = yRef;
#endif
}

void Linear_Init_X0(Linear_T * p_linear, int32_t factor, int32_t divisor, int32_t x0, int32_t yRef)
{

}

/*
	derive slope
*/
void Linear_Init_XRefYRef(Linear_T * p_linear, int32_t x0, int32_t xRef, int32_t yRef)
{

}


/******************************************************************************/
/*!
	Saturate to uint16_t, q0.16 [0, 65535]
	f([-XRef:XRef]) => [0:65536]
*/
/******************************************************************************/ 
uint16_t _Linear_SatUnsigned16(int32_t frac16)
{
	uint16_t fracU16;

	if		(frac16 > 65535) 	{ fracU16 = 65535U; }
	else if	(frac16 < 0) 		{ fracU16 = 0U; }
	else 						{ fracU16 = (uint16_t)frac16; }

	return fracU16;
}

/* negative returns zero */
uint16_t Linear_Function_FractionUnsigned16(const Linear_T * p_linear, int32_t x)
{
	return _Linear_SatUnsigned16(Linear_Function_Fraction16(p_linear, x));
}

uint16_t _Linear_SatUnsigned16_Abs(int32_t frac16)
{
	uint16_t fracU16;

	if		(frac16 < 0) 		{ fracU16 = (uint16_t)(0 - frac16); }
	else if	(frac16 > 65535) 	{ fracU16 = 65535U; }
	else 						{ fracU16 = (uint16_t)frac16; }

	return fracU16;
}

/* negative returns abs */
uint16_t Linear_Function_FractionUnsigned16_Abs(const Linear_T * p_linear, int32_t x)
{
	return _Linear_SatUnsigned16_Abs(Linear_Function_Fraction16(p_linear, x));
}

/* y_frac16 in q0.16 format is handled by q16.16 case */
int32_t Linear_InvFunction_FractionUnsigned16(const Linear_T * p_linear, uint16_t y_fracU16)
{
	return Linear_InvFunction_Fraction16(p_linear, y_fracU16);
}

/******************************************************************************/
/*!
	Saturate to int16_t, q1.15 [-32768, 32767]
	f([-XRef:XRef]) => [-32768:32767]
*/
/******************************************************************************/ 
int16_t _Linear_SatSigned16(int32_t frac16)
{
	int16_t fracS16;

	if		(frac16 > 32767) 	{ fracS16 = 32767; }
	else if	(frac16 < -32768) 	{ fracS16 = -32768; }
	else 						{ fracS16 = (int16_t)frac16; }

	return fracS16;
}

/* */
int16_t Linear_Function_FractionSigned16(const Linear_T * p_linear, int32_t x)
{
	return _Linear_SatSigned16(Linear_Function_Fraction16(p_linear, x) / 2);
}

/* y_frac16 use q1.15 */
int32_t Linear_InvFunction_FractionSigned16(const Linear_T * p_linear, int16_t y_fracS16)
{
	return Linear_InvFunction_Fraction16(p_linear, (int32_t)y_fracS16 * 2);
}


/******************************************************************************/
/*!
	Scalar
*/
/******************************************************************************/ 
int32_t Linear_Function_Scalar(const Linear_T * p_linear, int32_t x, uint16_t scalar)
{
	int32_t factor = x * p_linear->Slope;
	int32_t result = 0U;

	/* scalar maybe compile time constant, can compiler unroll loop to optimize? */

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
 





