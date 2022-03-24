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
    @file 	Linear_Ramp.h
    @author FireSoucery
    @brief 	dynamic look up table
			linear_f(index) = user units

    @version V0
*/
/******************************************************************************/
#ifndef LINEAR_RAMP_H
#define LINEAR_RAMP_H

#include "Linear.h"

#include <stdint.h>


static inline void Linear_Ramp_SetTarget(Linear_T * p_linear, int32_t target)
{
	p_linear->YReference = target;
}

static inline int32_t Linear_Ramp_GetTarget(Linear_T * p_linear)
{
	return p_linear->YReference;
}

//shallow slope use index for more accurate fixed point
static inline int32_t Linear_Ramp_ProcOutputIncIndex(Linear_T * p_linear, uint32_t * p_index, uint32_t indexIncreament, int32_t currentRampValue)
{
	int32_t newRampValue;

	if (currentRampValue != p_linear->YReference)
	{
		newRampValue = Linear_Function(p_linear, *p_index); //index overflow?

		/*
		 * Always set ramp in rising slope, eliminates check slope sign
		 */
		if(newRampValue > p_linear->YReference)
		{
			if(currentRampValue < p_linear->YReference) //Alleviate bounce at boundary
			{
				newRampValue = p_linear->YReference;
			}

			if (*p_index > 0U) {*p_index -= indexIncreament;}
		}
		else if(newRampValue < p_linear->YReference)
		{
			if(currentRampValue > p_linear->YReference)
			{
				newRampValue = p_linear->YReference;
			}
			*p_index += indexIncreament;
		}
	}
	else
	{
		newRampValue = currentRampValue;
		//sync error may cause index and current value to mismatch
	}

	return newRampValue;
}

static inline int32_t Linear_Ramp_ProcOutput(Linear_T * p_linear, uint32_t * p_index, int32_t currentRampValue)
{
	return Linear_Ramp_ProcOutputIncIndex(p_linear, p_index, 1U, currentRampValue);
}

static inline void Linear_Ramp_SetIndex(Linear_T * p_linear, uint32_t * p_index, int32_t rampValue)
{
	*p_index = Linear_InvFunction(p_linear, rampValue);
}

//non index version
//static inline int32_t Linear_Ramp_CalcNextOutput(Linear_T * p_linear, int32_t currentRampValue)
//{
//	int32_t newRampValue;
//
//	if(currentRampValue < p_linear->YReference)
//	{
//		newRampValue = ((currentRampValue << p_linear->SlopeDivisor_Shift) + p_linear->SlopeFactor) >> p_linear->SlopeDivisor_Shift;
//	}
//	else if(currentRampValue > p_linear->YReference)
//	{
//		newRampValue = ((currentRampValue << p_linear->SlopeDivisor_Shift) - p_linear->SlopeFactor) >> p_linear->SlopeDivisor_Shift;
//	}
//
//	return newRampValue;
//}

//positive and negative slope
//static inline int32_t Linear_Ramp_ProcIncIndex(Linear_T * p_linear, uint32_t * p_index, uint32_t indexIncreament)
//{
//	int32_t rampValue = Linear_Function(p_linear, *p_index);
//
//	if(p_linear->SlopeFactor < (int32_t)0) /* slope is negative, inc ramp if greater than final */
//	{
//		if(rampValue < p_linear->YReference)
//		{
//			rampValue = p_linear->YReference;
//		}
//		else
//		{
//			*p_index += indexIncreament;
//		}
//	}
//	else if(p_linear->SlopeFactor > (int32_t)0) /* slope is positive, inc ramp if less than final */
//	{
//		if(rampValue > p_linear->YReference)	{rampValue = p_linear->YReference;}
//	}
//	/* else slope is 0, continue to return the same value */
//
////	if(rampValue != p_linear->YReference)
////	{
////		*p_index += indexIncreament;
////	}
//
//	return rampValue;
//}

#endif
