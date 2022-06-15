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
	@file 	Linear.h
	@author FireSoucery
	@brief  Mathematical linear function.
			e.g. Dynamic look up table, unit/ADC conversion using factor and divisor.
	@version V0
*/
/******************************************************************************/
#ifndef LINEAR_H
#define LINEAR_H

#include "Config.h"
#include "math_linear.h"
#include <stdint.h>

typedef struct Linear_Tag
{
#if defined(CONFIG_LINEAR_DIVIDE_SHIFT)
	int32_t Slope;			/* y = (x - XOffset) * Slope >> SlopeShift + YOffset */
	uint8_t SlopeShift;
	int32_t InvSlope;		/* x = (y - YOffset) * InvSlope >> InvSlopeShift + XOffset */
	uint8_t InvSlopeShift;
#elif defined(CONFIG_LINEAR_DIVIDE_NUMERICAL)
	int32_t SlopeFactor;
	int32_t SlopeDivisor;
#endif
	int32_t XOffset;
	int32_t YOffset;
	/* One Ref is derived */
	int32_t XReference;		/* f(x[0:XRef]) => frac16(x)[0:65536] */
	int32_t YReference;		/* f(x)[0:YRef] => frac16(x)[0:65536] */
}
Linear_T;

#if defined(CONFIG_LINEAR_DIVIDE_SHIFT)
#define LINEAR_CONFIG(factor, divisor, y0, yRef)												\
{																								\
	.Slope 				= ((int32_t)65536 << 14U) / ((int32_t)(divisor) * yRef / (factor)),		\
	.SlopeShift 		= 14U,																	\
	.InvSlope 			= (((int32_t)(divisor) * (yRef) / (factor)) << 14U) / 65536, 			\
	.InvSlopeShift 		= 14U,																	\
	.XOffset 			= 0,																	\
	.YOffset 			= y0,																	\
 	.XReference 		= ((int32_t)(divisor) * (yRef) / (factor)), 							\
	.YReference 		= yRef, 																\
}
#elif defined(CONFIG_LINEAR_DIVIDE_NUMERICAL)
#define LINEAR_CONFIG(factor, divisor, offset, rangeRef)				\
{																		\
																		\
}
#endif

/******************************************************************************/
/*!
	Protected
*/
/******************************************************************************/
extern int32_t _Linear_Sat(int32_t min, int32_t max, int32_t value);

static inline int16_t _Linear_SatSigned16(int32_t frac16) { return _Linear_Sat(INT16_MIN, INT16_MAX, frac16); }
static inline uint16_t _Linear_SatUnsigned16(int32_t frac16) { return _Linear_Sat(0, UINT16_MAX, frac16); }
static inline uint16_t _Linear_SatUnsigned16_Abs(int32_t frac16)
{
	int32_t sat = _Linear_Sat(0 - (int32_t)UINT16_MAX, UINT16_MAX, frac16);
	return (sat < 0) ? 0 - sat: sat;
}


/******************************************************************************/
/*!
	@brief Linear Essential Functions
	@{
*/
/******************************************************************************/

/******************************************************************************/
/*!
	@brief
*/
/******************************************************************************/
/*
	((x - x0) * factor / divisor) + y0

	Overflow: Slope * (x - x0) must be < INT32_MAX [2,147,483,647]
*/
static inline int32_t Linear_Function(const Linear_T * p_linear, int32_t x)
{
#if defined(CONFIG_LINEAR_DIVIDE_SHIFT)
	return linear_f_shift(p_linear->Slope, p_linear->SlopeShift, p_linear->XOffset, p_linear->YOffset, x);
#elif defined(CONFIG_LINEAR_DIVIDE_NUMERICAL)
	return linear_f(p_linear->SlopeFactor, p_linear->SlopeDivisor, p_linear->XOffset, p_linear->YOffset, x);
#endif
}

/*
	((y - y0) * divisor / factor) + x0;
*/
static inline int32_t Linear_InvFunction(const Linear_T * p_linear, int32_t y)
{
#if defined(CONFIG_LINEAR_DIVIDE_SHIFT)
	return linear_invf_shift(p_linear->InvSlope, p_linear->InvSlopeShift, p_linear->XOffset, p_linear->YOffset, y);
#elif defined(CONFIG_LINEAR_DIVIDE_NUMERICAL)
	return linear_invf(p_linear->SlopeFactor, p_linear->SlopeDivisor, p_linear->XOffset, p_linear->YOffset, y);
#endif
}

/******************************************************************************/
/*!
	@brief Fraction16
	Fraction in q16.16 [-2,147,483,648, 2,147,483,647]
	f([-XRef:XRef]) => [-65536:65536] <= [-YRef:YRef], Can oversaturate 1

	@param[in] x overflow limit ~q1.16
	@{
*/
/******************************************************************************/
/*  */
static inline int32_t Linear_Function_Frac16(const Linear_T * p_linear, int32_t x)
{
	return linear_f16(p_linear->XOffset, p_linear->XReference, x);
}

/*!
	y_frac16 in q16.16 format
	@param[in] y_frac16 overflow limit ~q1.16
*/
static inline int32_t Linear_InvFunction_Frac16(const Linear_T * p_linear, int32_t y_frac16)
{
	return linear_invf16(p_linear->XOffset, p_linear->XReference, y_frac16);
}

/******************************************************************************/
/*! @} */
/******************************************************************************/

static inline int32_t Linear_Function_Round(const Linear_T * p_linear, int32_t x)
{
#if defined(CONFIG_LINEAR_DIVIDE_SHIFT)
	return Linear_Function(p_linear, x);
#elif defined(CONFIG_LINEAR_DIVIDE_NUMERICAL)
	return linear_f_rounded(p_linear->SlopeFactor, p_linear->SlopeDivisor, p_linear->XOffset, p_linear->YOffset, x);
#endif
}

static inline int32_t Linear_InvFunction_Round(const Linear_T * p_linear, int32_t y)
{
#if defined(CONFIG_LINEAR_DIVIDE_SHIFT)
	return Linear_InvFunction(p_linear, y);
#elif defined(CONFIG_LINEAR_DIVIDE_NUMERICAL)
	return linear_invf_rounded(p_linear->SlopeFactor, p_linear->SlopeDivisor, p_linear->XOffset, p_linear->YOffset, y);
#endif
}

// extern int16_t _Linear_SatSigned16(int32_t frac16);
// extern uint16_t _Linear_SatUnsigned16(int32_t frac16);
// extern uint16_t _Linear_SatUnsigned16_Abs(int32_t frac16);

extern void Linear_Init(Linear_T * p_linear, int32_t factor, int32_t divisor, int32_t y0, int32_t yRef);
extern void Linear_Init_Map(Linear_T * p_linear, int32_t x0, int32_t xRef, int32_t y0, int32_t yRef);

extern int32_t Linear_Function_Sat(const Linear_T * p_linear, int32_t x);
extern int32_t Linear_InvFunction_Sat(const Linear_T * p_linear, int32_t y);
extern int32_t Linear_Function_Scalar(const Linear_T * p_linear, int32_t x, uint16_t scalar);
extern int32_t Linear_InvFunction_Scalar(const Linear_T * p_linear, int32_t y, uint16_t scalar);

extern uint16_t Linear_Function_FractionUnsigned16(const Linear_T * p_linear, int32_t x);
extern uint16_t Linear_Function_FractionUnsigned16_Abs(const Linear_T * p_linear, int32_t x);
extern int32_t Linear_InvFunction_FractionUnsigned16(const Linear_T * p_linear, uint16_t y_fracU16);
extern int16_t Linear_Function_FractionSigned16(const Linear_T * p_linear, int32_t x);
extern int32_t Linear_InvFunction_FractionSigned16(const Linear_T * p_linear, int16_t y_fracS16);

#endif
