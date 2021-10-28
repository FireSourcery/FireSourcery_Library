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
    @brief  Linear
    @version V0
*/
/*******************************************************************************/
#include "Linear.h"

/*
 * CONFIG_LINEAR_SHIFT_DIVIDE mode
 * Right shift must retain sign bit,
 * user factor, divisor, input must be less than 16 bits
 */

static inline int32_t MaxLeftShiftDivide(int32_t factor, int32_t divisor, uint8_t leftShift)
{
	int32_t result = 0;
	int32_t shiftedValue = (1U << leftShift);

	if (shiftedValue % divisor == 0U) /* when divisor is a whole number factor of shifted. */
	{
		result = factor * (shiftedValue / divisor);
	}
	else
	{
		for (uint8_t maxShift = leftShift; maxShift > 0U; maxShift--)
		{
			if(factor > 0)
			{
				if (factor <= (INT32_MAX >> maxShift))
				{
					result = (factor << maxShift) / divisor; //max shift before divide

					if (result <= (INT32_MAX >> (leftShift - maxShift)))
					{
						result = result << (leftShift - maxShift); //remaining shift
					}
					else /* error, will overflow 32 bit even using ((factor << 0)/divisor) << leftShift */
					{
						result = 0;
					}
					break;
				}
			}
			else
			{
				if((factor << maxShift) < 0) //still negative after shifting
				{
					result = (factor << maxShift) / divisor;

					if ((result << (leftShift - maxShift)) < 0)
					{
						result = result << (leftShift - maxShift);
					}
					else  /* error, will overflow 32 bit even using ((factor << 0)/divisor) << leftShift */
					{
						result = 0;
					}
					break;
				}
			}
		}
	}

	return result;
}

/*
 * f(in) = ((factor * in) / divisor + offset) =>
 *
 * 0 percent 	=> f([-offset*divisor/factor]) 	= 0
 * 100 percent 	=> f([(rangeRef - offset)*divisor/factor]) = rangeRef
 */
void Linear_Init(Linear_T * p_linear, int32_t factor, int32_t divisor, int32_t offset, int32_t rangeRef)
{
#ifdef CONFIG_LINEAR_SHIFT_DIVIDE
	p_linear->SlopeFactor 			= (factor << 16U) / divisor;
	p_linear->SlopeDivisor_Shift 	= 16U;
	p_linear->SlopeDivisor 			= (divisor << 16U) / factor; //InvF factor
	p_linear->SlopeFactor_Shift 	= 16U;
	p_linear->Offset = offset << 16U;
#elif defined(CONFIG_LINEAR_NUMIRICAL_DIVIDE)
	p_linear->SlopeFactor 	= factor;
	p_linear->SlopeDivisor 	= divisor;
	p_linear->Offset = offset;
#endif
	p_linear->RangeReference = rangeRef;
}

/*
 * f(in) = ((factor * (in - x0)) / divisor) =>
 */
#if defined(CONFIG_LINEAR_SHIFT_DIVIDE)
void Linear_Init_X0(Linear_T * p_linear, int32_t factor, int32_t divisor, int32_t offset_x0, int32_t rangeRef)
{
	Linear_Init(p_linear, factor, divisor,  0, rangeRef);
	p_linear->Offset = 0 - MaxLeftShiftDivide(factor * offset_x0, divisor, 16U);
//	p_linear->Offset = 0 - (offset_x0 * factor << 16U) / divisor;
}
#endif

//#if defined(CONFIG_LINEAR_NUMIRICAL_DIVIDE)
//   F  return frac16  efficiently. F16 invalid
//void Linear_Init_Frac16(Linear_T * p_linear, int16_t factor, int16_t divisor, int32_t offset, int32_t rangeRef)
//{
//	p_linear->SlopeFactor 	= factor * 65536;
//	p_linear->SlopeDivisor 	= divisor * rangeRef;
//	p_linear->Offset 		= offset * 65536 / yref;
//	p_linear->RangeReference = rangeRef;
//}
//#endif
