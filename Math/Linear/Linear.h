/*******************************************************************************/
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
/*******************************************************************************/
/*******************************************************************************/
/*!
    @file 	Linear.c
    @author FireSoucery
    @brief  Mathematical linear function.
     	 	e.g. dynamic look up table, Unit/ADC conversion using factor and divisor.
    @version V0
*/
/*******************************************************************************/
#ifndef LINEAR_H
#define LINEAR_H

#include "Config.h"

#include "math_linear.h"

#include <stdint.h>

typedef struct Linear_Tag
{
	int32_t SlopeFactor; 	//m
	int32_t SlopeDivisor;
#ifdef CONFIG_LINEAR_SHIFT_DIVIDE
	uint8_t SlopeDivisor_Shift; // y = x * SlopeFactor >> SlopeDivisor_Shift + Offset
	uint8_t SlopeFactor_Shift;  // x = (y - Offset) * SlopeDivisor >> SlopeFactor_Shift
#endif
	int32_t Offset;			//b, y offset
//	int32_t OffsetX;
//	int32_t OffsetY; //2 offsets for x offset mode precision
	int32_t RangeReference; //100 percent, x-intercept is 0 percent
} Linear_T;

//typedef const struct Linear_Tag
//{
//#ifdef CONFIG_LINEAR_SHIFT_DIVIDE
//	const int16_t SLOPE_FACTOR;
//	const int16_t SLOPE_DIVISOR;
//#elif defined(CONFIG_LINEAR_NUMERICAL_DIVIDE)
//	const int32_t SLOPE_FACTOR;
//	const int32_t SLOPE_DIVISOR;
//#endif
//	const int32_t OFFSET;			//b, y offset
//	const int32_t RANGE_REFERENCE; //100 percent, x-intercept is 0 percent
//} Linear_Init_T;

static inline int32_t Linear_Function(Linear_T * p_linear, int32_t x)
{
#ifdef CONFIG_LINEAR_SHIFT_DIVIDE
	return linear_f_shift(p_linear->SlopeFactor, p_linear->SlopeDivisor_Shift, p_linear->Offset, x);
#elif defined(CONFIG_LINEAR_NUMERICAL_DIVIDE)
	return linear_f(p_linear->SlopeFactor, p_linear->SlopeDivisor, p_linear->Offset, x);
#endif
}

/*
 * return ( (y - b) * m_divisor / m_factor );
 */
static inline int32_t Linear_InvFunction(Linear_T * p_linear, int32_t y)
{
#ifdef CONFIG_LINEAR_SHIFT_DIVIDE
	return linear_invf_shift(p_linear->SlopeDivisor, p_linear->SlopeFactor_Shift, p_linear->Offset, y);
#elif defined(CONFIG_LINEAR_NUMERICAL_DIVIDE)
	return linear_invf(p_linear->SlopeFactor, p_linear->SlopeDivisor, p_linear->Offset, y);
#endif
}


/*
 * returns 	65536 => 100 percent, unsat
 * 			-65536 => -100 percent
 */
static inline int32_t Linear_Function_Fraction16(Linear_T * p_linear, int32_t x) //Range16 Percent16
{
#ifdef CONFIG_LINEAR_SHIFT_DIVIDE
	return linear_f_frac16_shift(p_linear->SlopeFactor, p_linear->Offset, p_linear->RangeReference, x);
#elif defined(CONFIG_LINEAR_NUMERICAL_DIVIDE)
	return linear_f_frac16(p_linear->SlopeFactor, p_linear->SlopeDivisor, p_linear->Offset, p_linear->RangeReference, x);
#endif
}

/*
 * returns 	q0.16
 * 			65536 => 100 percent, unsat
 * 			-percent returns invalid  or abs or zero??
 */
static inline uint16_t Linear_Function_FractionUnsigned16(Linear_T * p_linear, int32_t x)
{
	return (uint16_t) Linear_Function_Fraction16(p_linear, x);
}

//static inline uint16_t Linear_FunctionAbs_FractionUnsigned16(Linear_T * p_linear, int32_t x)
//{
//	int32_t temp = Linear_Function_Fraction16(p_linear, x);
//	if t
//	return (uint16_t) Linear_Function_Fraction16(p_linear, x);
//}

/*
 * returns q1.15
 * 			32768 => 100 percent, unsat
 * 			-32768 => -100 percent, unsat
 */
static inline int16_t Linear_Function_FractionSigned16(Linear_T * p_linear, int32_t x)
{
	return Linear_Function_Fraction16(p_linear, x) / 2U;
}

static inline int32_t Linear_Function_Round(Linear_T * p_linear, int32_t x)
{
#ifdef CONFIG_LINEAR_SHIFT_DIVIDE
	return linear_f_shift(p_linear->SlopeFactor, p_linear->SlopeDivisor_Shift, p_linear->Offset, x);
#elif defined(CONFIG_LINEAR_NUMERICAL_DIVIDE)
	return linear_f_rounded(p_linear->SlopeFactor, p_linear->SlopeDivisor, p_linear->Offset, x);
#endif
}

/*
 * todo
 */
static inline int32_t Linear_InvFunction_Fraction16(Linear_T * p_linear, int32_t y_frac16)
{
#ifdef CONFIG_LINEAR_SHIFT_DIVIDE
//	return linear_invf_frac16_shift(p_linear->SlopeFactor, p_linear->SlopeDivisor, p_linear->Offset, p_linear->RangeReference, x);
#elif defined(CONFIG_LINEAR_NUMERICAL_DIVIDE)
	return linear_invf_frac16(p_linear->SlopeFactor, p_linear->SlopeDivisor, p_linear->Offset, p_linear->RangeReference, x);
#endif
}

static inline int16_t Linear_InvFunction_FractionUnsigned16(Linear_T * p_linear, uint32_t y_frac16)
{
	return Linear_InvFunction_Fraction16(p_linear, y_frac16);
}

static inline int16_t Linear_InvFunction_FractionSigned16(Linear_T * p_linear, int32_t y_frac16)
{
	return Linear_InvFunction_Fraction16(p_linear, y_frac16*2U);
}


static inline int32_t Linear_InvFunction_Round(Linear_T * p_linear, int32_t y)
{
#ifdef CONFIG_LINEAR_SHIFT_DIVIDE
	return linear_invf_shift(p_linear->SlopeFactor, p_linear->SlopeDivisor_Shift, p_linear->Offset, y);
#elif defined(CONFIG_LINEAR_NUMERICAL_DIVIDE)
	return linear_invf_rounded(p_linear->SlopeFactor, p_linear->SlopeDivisor, p_linear->Offset, y);
#endif
}

//extern void Linear_Init(Linear_T * p_linear, int32_t factor, int32_t divisor, int32_t offset, int32_t rangeRef);

#if defined(CONFIG_LINEAR_SHIFT_DIVIDE)
extern void Linear_Init_X0(Linear_T * p_linear, int32_t factor, int32_t divisor, int32_t x0, int32_t rangeRef);
#endif

#endif

//static inline int32_t Linear_Function_RangeBound(Linear_T * p_linear, int32_t x)
//{
//#ifdef CONFIG_LINEAR_SHIFT_DIVIDE
//	return linear_f_shift(p_linear->SlopeFactor, p_linear->SlopeDivisor_Shift, p_linear->Offset, x);
//#elif defined(CONFIG_LINEAR_NUMERICAL_DIVIDE)
//	return linear_f(p_linear->SlopeFactor, p_linear->SlopeDivisor, p_linear->Offset, x);
//#endif
//}
//
//static inline int32_t Linear_InvFunction_RangeBound(Linear_T * p_linear, int32_t y)
//{
//#ifdef CONFIG_LINEAR_SHIFT_DIVIDE
//	return linear_invf_shift(p_linear->SlopeDivisor, p_linear->SlopeFactor_Shift, p_linear->Offset, y);
//#elif defined(CONFIG_LINEAR_NUMERICAL_DIVIDE)
//	return linear_invf(p_linear->SlopeFactor, p_linear->SlopeDivisor, p_linear->Offset, y);
//#endif
//}
