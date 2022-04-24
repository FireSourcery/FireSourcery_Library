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
    @brief  Linear
    @version V0
*/
/******************************************************************************/
#include "Linear.h"

/*
 * f(in) = ((factor * (in - x0)) / divisor) + y0
 *
 * factor > divisor, bound with yref
 * divisor > factor, bound with xref
 */
void Linear_Init(Linear_T * p_linear, int32_t factor, int32_t divisor, int32_t y0, int32_t yRef)
{
#ifdef CONFIG_LINEAR_DIVIDE_SHIFT
	p_linear->YReference 		= yRef;
	p_linear->XReference 		= linear_invf(factor, divisor, y0, yRef); // divisor * yRef / factor;

	//todo determine max shift if factor > 65536
	p_linear->Slope 			= (factor << 14U) / divisor;
	p_linear->SlopeShift 		= 14U;

	//todo non iterative, log2,
	while ((p_linear->XReference > INT32_MAX / p_linear->Slope) && (p_linear->SlopeShift > 0U))
	{
		p_linear->Slope = p_linear->Slope >> 1U;
		p_linear->SlopeShift--;
	}

	//todo maxleftshift divide, if factor > divisor, invslope can be > 14
	p_linear->InvSlope 			= (divisor << 14U) / factor;
	p_linear->InvSlopeShift 	= 14U;

	while ((p_linear->YReference - y0 > INT32_MAX / p_linear->InvSlope) && (p_linear->InvSlopeShift > 0U))
	{
		p_linear->InvSlope = p_linear->InvSlope >> 1U;
		p_linear->InvSlopeShift--;
	}

	p_linear->XOffset 			= 0;
	p_linear->YOffset 			= y0;
#elif defined(CONFIG_LINEAR_DIVIDE_NUMERICAL)
	p_linear->SlopeFactor 		= factor;
	p_linear->SlopeDivisor 		= divisor;
	p_linear->YOffset 			= y0;
	p_linear->YReference 		= yRef;
#endif
}


/*
 * derive slope
 */
void Linear_Init_XRefYRef(Linear_T * p_linear, int32_t x0, int32_t y0, int32_t xRef, int32_t yRef)
{

}
#ifdef CONFIG_LINEAR_DIVIDE_SHIFT

//static void InitCommonXReference(Linear_T * p_linear)
//{
//	p_linear->XReference = linear_invf_shift(p_linear->SlopeDivisor, p_linear->SlopeFactor_Shift, p_linear->XOffset, p_linear->YOffset, p_linear->YReference);
//}
//
//static void InitCommon_Shift(Linear_T * p_linear, int16_t factor, int16_t divisor, int32_t yRef, uint8_t shift)
//{
//	p_linear->SlopeFactor 			= ((int32_t)factor << shift) / divisor;
//	p_linear->SlopeDivisor_Shift 	= shift;
//	p_linear->SlopeDivisor 			= ((int32_t)divisor << shift) / factor; //InvF factor
//	p_linear->SlopeFactor_Shift 	= shift;
//	p_linear->YReference 			= yRef;
//}
//
////static void InitCommon(Linear_T * p_linear, int16_t factor, int16_t divisor, int32_t yRef)
////{
////	InitCommon_Shift(p_linear, factor, divisor, yRef, 16U);
////}
//
//
///*
// * f(in) = (factor * in) / divisor + y0
// *
// * f(in:[0 percent]) 	= 0
// * f(in:[100 percent]) 	= yRef
// */
//void Linear_Init_Y0(Linear_T * p_linear, int32_t factor, int32_t divisor, int32_t y0, int32_t yRef)
//{
//	InitCommon_Shift(p_linear, factor, divisor, yRef, 16U);
//	p_linear->XOffset = 0;
//	p_linear->YOffset = y0;
//	InitCommonXReference(p_linear);
//}
//
///*
// * f(in) = (factor * (in - x0)) / divisor
// *
// * option 1. save y-intercept as shifted.
// * option 2. reuse procedure of inv functions, + supplement inv frac16 functions
// * option 3. equations include 2 offsets. y = m*(x-x0) + y0
// */
//void Linear_Init_X0(Linear_T * p_linear, int16_t factor, int16_t divisor, int32_t x0, int32_t yRef)
//{
//	InitCommon_Shift(p_linear, factor, divisor, yRef, 16U);
//	p_linear->XOffset = x0;
//	p_linear->YOffset = 0;
//
////#if defined(CONFIG_LINEAR_DIVIDE_SHIFT)
////	Linear_Init(p_linear, factor, divisor, 0U, yRef);
////	p_linear->Intercept = 0 - MaxLeftShiftDivide(factor * offset_x0, divisor, 16U); // = 0 - (offset_x0 * factor << 16U) / divisor;
////#else
////
////#endif
//}
//
//void Linear_Init_Shift(Linear_T * p_linear, int32_t factor, int32_t divisor, int32_t x0, int32_t y0, int32_t yRef, uint8_t shift)
//{
//	InitCommon_Shift(p_linear, factor, divisor, yRef, shift);
//	p_linear->XOffset = x0;
//	p_linear->YOffset = y0;
//}
#endif









