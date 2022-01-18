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
 * f(in) = ((factor * in) / divisor + intercept)
 *
 * f(in:[0 percent]) 	= 0
 * f(in:[100 percent]) 	= rangeRef
 */
#ifdef CONFIG_LINEAR_DIVIDE_SHIFT
void Linear_Init(Linear_T * p_linear, int32_t factor, int32_t divisor, int32_t intercept, int32_t rangeRef)
{
	p_linear->SlopeFactor 			= ((int32_t)factor << 16U) / divisor;
	p_linear->SlopeDivisor_Shift 	= 16U;
	p_linear->SlopeDivisor 			= ((int32_t)divisor << 16U) / factor; //InvF factor
	p_linear->SlopeFactor_Shift 	= 16U;
	p_linear->Intercept 			= intercept << 16U;
	p_linear->RangeReference 		= rangeRef;
}

void Linear_Init_Shift(Linear_T * p_linear, int32_t factor, int32_t divisor, int32_t intercept, int32_t rangeRef, uint8_t shift)
{
	p_linear->SlopeFactor 			= ((int32_t)factor << shift) / divisor;
	p_linear->SlopeDivisor_Shift 	= shift;
	p_linear->SlopeDivisor 			= ((int32_t)divisor << shift) / factor; //InvF factor
	p_linear->SlopeFactor_Shift 	= shift;
	p_linear->Intercept 			= intercept << shift;
	p_linear->RangeReference 		= rangeRef;
}

#elif defined(CONFIG_LINEAR_DIVIDE_NUMERICAL)
void Linear_Init(Linear_T * p_linear, int32_t factor, int32_t divisor, int32_t intercept, int32_t rangeRef)
{
	p_linear->SlopeFactor 		= factor;
	p_linear->SlopeDivisor 		= divisor;
	p_linear->Intercept 		= intercept;
	p_linear->RangeReference 	= rangeRef;
}
#endif





#if defined(CONFIG_LINEAR_DIVIDE_SHIFT)
/*
 * CONFIG_LINEAR_DIVIDE_SHIFT mode
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
#endif


/*
 * f(in) = ((factor * (in - x0)) / divisor)
 */

/*
 * option 1
 * save shifted intercept as shifted
 *
 * option 2. full/partial y = m*(x-x0) implementation. reuse procedure of inv functions, + supplement inv frac16 functions
 *
 * todo option 3. equations include 2 offsets. y = m*(x-x0) + y0
 */
void Linear_Init_X0(Linear_T * p_linear, int16_t factor, int16_t divisor, int32_t offset_x0, int32_t rangeRef)
{
#if defined(CONFIG_LINEAR_DIVIDE_SHIFT)
	Linear_Init(p_linear, factor, divisor, 0U, rangeRef);
	p_linear->Intercept = 0 - MaxLeftShiftDivide(factor * offset_x0, divisor, 16U);
//	p_linear->Intercept = 0 - (offset_x0 * factor << 16U) / divisor;
#else
	Linear_Init(p_linear, factor, divisor, (0 - (offset_x0 * factor / divisor)), rangeRef);
#endif
}
