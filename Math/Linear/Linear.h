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
    @brief  Mathematical linear function.
     	 	e.g. dynamic look up table, Unit/ADC conversion using factor and divisor.
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
	int32_t SlopeFactor; 		// m
	int32_t SlopeDivisor;
#ifdef CONFIG_LINEAR_SHIFT_DIVIDE
	uint8_t SlopeDivisor_Shift; // y = x * SlopeFactor >> SlopeDivisor_Shift + Intercept
	uint8_t SlopeFactor_Shift;  // x = (y - Intercept) * SlopeDivisor >> SlopeFactor_Shift
#endif
	int32_t Intercept;			// b, y0
	int32_t X0;
	int32_t Y0;
	int32_t RangeReference; 	// [0:RangeReference] => [0:65536]
} Linear_T;

#ifdef CONFIG_LINEAR_SHIFT_DIVIDE
#define LINEAR_CONFIG(factor, divisor, intercept, rangeRef)				\
{																		\
	.SlopeFactor 				= ((int32_t)factor << 16U) / divisor;  	\
	.SlopeDivisor_Shift 		= 16U;									\
	.SlopeDivisor 				= ((int32_t)divisor << 16U) / factor; 	\
	.SlopeFactor_Shift 			= 16U;									\
	.Intercept 					= intercept;							\
 	.RangeReference 			= rangeRef; 							\
}
#elif defined(CONFIG_LINEAR_NUMIRICAL_DIVIDE)
#define LINEAR_CONFIG(factor, divisor, offset, rangeRef)				\
{																		\
	.SlopeFactor 				= ((int32_t)factor << 16U) / divisor;  	\
	.SlopeDivisor 				= ((int32_t)divisor << 16U) / factor; 	\
	.Intercept 					= (int32_t)offset << 16U;				\
 	.RangeReference 			= rangeRef; 							\
}
#endif

/*
 * Shift divide case overflow caution
 *  x must be < INT32_MAX (2,147,483,647) / SlopeFactor
 */
static inline int32_t Linear_Function(const Linear_T * p_linear, int32_t x)
{
#ifdef CONFIG_LINEAR_SHIFT_DIVIDE
	return linear_f_shift(p_linear->SlopeFactor, p_linear->SlopeDivisor_Shift, p_linear->Intercept, x);
#elif defined(CONFIG_LINEAR_NUMERICAL_DIVIDE)
	return linear_f(p_linear->SlopeFactor, p_linear->SlopeDivisor, p_linear->Intercept, x);
#endif
}

/*
 * return ( (y - b) * m_divisor / m_factor );
 */
static inline int32_t Linear_InvFunction(const Linear_T * p_linear, int32_t y)
{
#ifdef CONFIG_LINEAR_SHIFT_DIVIDE
	return linear_invf_shift(p_linear->SlopeDivisor,  p_linear->SlopeFactor_Shift, p_linear->Intercept, y);
#elif defined(CONFIG_LINEAR_NUMERICAL_DIVIDE)
	return linear_invf(p_linear->SlopeFactor, p_linear->SlopeDivisor, p_linear->Intercept, y);
#endif
}

static inline int32_t Linear_Function_Round(const Linear_T * p_linear, int32_t x)
{
#ifdef CONFIG_LINEAR_SHIFT_DIVIDE
	return Linear_Function(p_linear, x);
#elif defined(CONFIG_LINEAR_NUMERICAL_DIVIDE)
	return linear_f_rounded(p_linear->SlopeFactor, p_linear->SlopeDivisor, p_linear->Intercept, x);
#endif
}
static inline int32_t Linear_InvFunction_Round(const Linear_T * p_linear, int32_t y)
{
#ifdef CONFIG_LINEAR_SHIFT_DIVIDE
	return Linear_InvFunction(p_linear, y);
#elif defined(CONFIG_LINEAR_NUMERICAL_DIVIDE)
	return linear_invf_rounded(p_linear->SlopeFactor, p_linear->SlopeDivisor, p_linear->Intercept, y);
#endif
}


/******************************************************************************/
/*!
	@brief Fraction16

	Fraction in q16.16 [-2,147,483,648, 2,147,483,647]
		65536 	=> 1 * RangeReference
 		-65536 	=> -1 * RangeReference
 */
/******************************************************************************/
/*
 *
 */
static inline int32_t Linear_Function_Fraction16(const Linear_T * p_linear, int32_t x)
{
#ifdef CONFIG_LINEAR_SHIFT_DIVIDE
	return linear_f_frac16_shift(p_linear->SlopeFactor, p_linear->SlopeDivisor_Shift, p_linear->Intercept, p_linear->RangeReference, x);
#elif defined(CONFIG_LINEAR_NUMERICAL_DIVIDE)
	return linear_f_frac16(p_linear->SlopeFactor, p_linear->SlopeDivisor, p_linear->Intercept, p_linear->RangeReference, x);
#endif
}

/*
 * y_frac16 in q16.16 format
 */
static inline int32_t Linear_InvFunction_Fraction16(const Linear_T * p_linear, int32_t y_frac16)
{
#ifdef CONFIG_LINEAR_SHIFT_DIVIDE
	return linear_invf_frac16_shift(p_linear->SlopeDivisor, p_linear->SlopeFactor_Shift, p_linear->Intercept, p_linear->RangeReference, y_frac16);
#elif defined(CONFIG_LINEAR_NUMERICAL_DIVIDE)
	return linear_invf_frac16(p_linear->SlopeFactor, p_linear->SlopeDivisor, p_linear->Intercept, p_linear->RangeReference, x);
#endif
}


/******************************************************************************/
/*!
	Saturate to uint16_t
		q0.16 [0, 65535]
		65536 => 1 * RangeReference, (Note: returns 65535 max)
 */
/******************************************************************************/
/*
 *  negative returns zero
 */
static inline uint16_t Linear_Function_FractionUnsigned16(const Linear_T * p_linear, int32_t x)
{
	int32_t frac16 = Linear_Function_Fraction16(p_linear, x);

	if (frac16 > 65535)		{frac16 = 65535;}
	else if (frac16 < 0) 	{frac16 = 0;}

	return (uint16_t)frac16;
}

/*
 *  negative returns abs
 */
static inline uint16_t Linear_Function_FractionUnsigned16_Abs(const Linear_T * p_linear, int32_t x)
{
	int32_t frac16 = Linear_Function_Fraction16(p_linear, x);

	if (frac16 < 0)		{frac16 = -frac16;}
	if (frac16 > 65535)	{frac16 = 65535;}

	return (uint16_t)frac16;
}

/*
 * y_frac16 in q0.16 format is handled by q16.16 case
 */
static inline int32_t Linear_InvFunction_FractionUnsigned16(const Linear_T * p_linear, uint16_t y_frac16)
{
	return Linear_InvFunction_Fraction16(p_linear, y_frac16);
}

/******************************************************************************/
/*!
	Saturate to int16_t
		q1.15 [-32768, 32767]
		32768 	=> 	1 * RangeReference (Note: returns 32767 max)
		-32768 	=> -1 * RangeReference
 */
/******************************************************************************/
/*
 *
 */
static inline int16_t Linear_Function_FractionSigned16(const Linear_T * p_linear, int32_t x)
{
	int32_t frac16 = Linear_Function_Fraction16(p_linear, x) / 2;

	if (frac16 > 32767)			{frac16 = 32767;}
	else if (frac16 < -32768) 	{frac16 = -32768;}

	return (int16_t)frac16;
}

/*
 * y_frac16 use  q1.15
 */
static inline int32_t Linear_InvFunction_FractionSigned16(const Linear_T * p_linear, int16_t y_fracSigned16)
{
	return Linear_InvFunction_Fraction16(p_linear, y_fracSigned16 * 2);
}

#ifdef CONFIG_LINEAR_SHIFT_DIVIDE
extern void Linear_Init(Linear_T * p_linear, int16_t factor, int16_t divisor, int16_t intercept, int32_t rangeRef);
#elif defined(CONFIG_LINEAR_NUMIRICAL_DIVIDE)
extern void Linear_Init(  Linear_T * p_linear, int32_t factor, int32_t divisor, int32_t intercept, int32_t rangeRef);
#endif

extern void Linear_Init_X0(Linear_T * p_linear, int16_t factor, int16_t divisor, int32_t offset_x0, int32_t rangeRef);

#endif





//static inline int32_t Linear_Function_RangeBound(const Linear_T * p_linear, int32_t x)
//{
//#ifdef CONFIG_LINEAR_SHIFT_DIVIDE
//	return linear_f_shift(p_linear->SlopeFactor, p_linear->SlopeDivisor_Shift, p_linear->Intercept, x);
//#elif defined(CONFIG_LINEAR_NUMERICAL_DIVIDE)
//	return linear_f(p_linear->SlopeFactor, p_linear->SlopeDivisor, p_linear->Intercept, x);
//#endif
//}
//
//static inline int32_t Linear_InvFunction_RangeBound(const Linear_T * p_linear, int32_t y)
//{
//#ifdef CONFIG_LINEAR_SHIFT_DIVIDE
//	return linear_invf_shift(p_linear->SlopeDivisor, p_linear->SlopeFactor_Shift, p_linear->Intercept, y);
//#elif defined(CONFIG_LINEAR_NUMERICAL_DIVIDE)
//	return linear_invf(p_linear->SlopeFactor, p_linear->SlopeDivisor, p_linear->Intercept, y);
//#endif
//}
