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
 */
void Linear_Init(Linear_T * p_linear, int32_t factor, int32_t divisor, int32_t y0, int32_t yRef)
{
#ifdef CONFIG_LINEAR_DIVIDE_SHIFT
	p_linear->SlopeFactor 			= (factor << 14U) / divisor;
	p_linear->SlopeDivisor_Shift 	= 14U;
	p_linear->SlopeDivisor 			= (divisor << 14U) / factor;
	p_linear->SlopeFactor_Shift 	= 14U;
	p_linear->YReference 			= yRef;
	p_linear->XReference 			= divisor * yRef / factor;
	p_linear->XOffset 				= 0;
	p_linear->YOffset 				= y0;
#elif defined(CONFIG_LINEAR_DIVIDE_NUMERICAL)
	p_linear->SlopeFactor 		= factor;
	p_linear->SlopeDivisor 		= divisor;
	p_linear->Intercept 		= intercept;
	p_linear->YReference 		= yRef;
	InitCommonXReference(p_linear);
#endif
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





//#if defined(CONFIG_LINEAR_DIVIDE_SHIFT)
///*
// * CONFIG_LINEAR_DIVIDE_SHIFT mode
// * Right shift must retain sign bit,
// * user factor, divisor, input must be less than 16 bits
// */
//static inline int32_t MaxLeftShiftDivide(int32_t factor, int32_t divisor, uint8_t leftShift)
//{
//	int32_t result = 0;
//	int32_t shiftedValue = (1U << leftShift);
//
//	if (shiftedValue % divisor == 0U) /* when divisor is a whole number factor of shifted. */
//	{
//		result = factor * (shiftedValue / divisor);
//	}
//	else
//	{
//		for (uint8_t maxShift = leftShift; maxShift > 0U; maxShift--)
//		{
//			if(factor > 0)
//			{
//				if (factor <= (INT32_MAX >> maxShift))
//				{
//					result = (factor << maxShift) / divisor; //max shift before divide
//
//					if (result <= (INT32_MAX >> (leftShift - maxShift)))
//					{
//						result = result << (leftShift - maxShift); //remaining shift
//					}
//					else /* error, will overflow 32 bit even using ((factor << 0)/divisor) << leftShift */
//					{
//						result = 0;
//					}
//					break;
//				}
//			}
//			else
//			{
//				if((factor << maxShift) < 0) //still negative after shifting
//				{
//					result = (factor << maxShift) / divisor;
//
//					if ((result << (leftShift - maxShift)) < 0)
//					{
//						result = result << (leftShift - maxShift);
//					}
//					else  /* error, will overflow 32 bit even using ((factor << 0)/divisor) << leftShift */
//					{
//						result = 0;
//					}
//					break;
//				}
//			}
//		}
//	}
//
//	return result;
//}
//#endif



