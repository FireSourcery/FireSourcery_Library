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
    @file 	Linear_Ramp.h
    @author FireSoucery
    @brief 	dynamic look up table
			linear_f(index) = user units

    @version V0
*/
/*******************************************************************************/
#ifndef LINEAR_RAMP_H
#define LINEAR_RAMP_H

#include "Linear.h"

#include <stdint.h>

static inline int16_t Linear_Ramp_CalcTarget(Linear_T * p_linear, uint32_t index)
{
	return Linear_Function(p_linear, index);
}

//static inline int16_t Linear_Ramp_CalcTarget_IncIndexOne(Linear_T *p_linear, uint32_t *p_index)
//{
//	int32_t target = Linear_Function(p_linear, *p_index);
//
//	if (p_linear->SlopeFactor >= 0U) //slope is positive
//	{
//		if(target < p_linear->RangeReference)
//		{
//			p_index++;
//		}
//	}
//	else //slope is negative
//	{
//		if(target > p_linear->RangeReference)
//		{
//			p_index++;
//		}
//	}
//
//	return target;
//}

static inline int16_t Linear_Ramp_CalcTarget_IncIndex(Linear_T *p_linear, uint32_t *p_index, uint32_t indexIncreament)
{
	int32_t target = Linear_Function(p_linear, *p_index);

	if (p_linear->SlopeFactor >= (int32_t)0) //slope is positive
	{
		if(target < p_linear->RangeReference)
		{
			*p_index += indexIncreament;
		}
		else
		{
			target = p_linear->RangeReference;
		}
	}
	else //slope is negative
	{
		if(target > p_linear->RangeReference)
		{
			*p_index += indexIncreament;
		}
		else
		{
			target = p_linear->RangeReference;
		}
	}

	return target;
}

//uint16_t Linear_Ramp_CalcIndex(Linear_T * p_linear, int16_t rampValue)
//{
//
//}
#endif
