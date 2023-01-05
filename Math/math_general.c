/******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSourcery / The Firebrand Forge Inc

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
	@file 	math_general.h
	@author FireSourcery
	@brief  Re-implementation of C++ library not found in C
	@version V0
*/
/******************************************************************************/
#include "math_general.h"

uint32_t math_muldiv64_unsigned(uint32_t value, uint32_t factor, uint32_t divisor)
{
	return (uint64_t)value * factor / divisor;
	// MaxLeftShiftDivide(uint32_t factor, uint32_t divisor, uint8_t targetShift)
}

// static uint32_t MaxLeftShiftDivide(uint32_t factor, uint32_t divisor, uint8_t targetShift)
// {
// 	uint32_t result = 0;
// 	uint32_t shiftedValue = (1U << targetShift);

// 	if(shiftedValue % divisor == 0U) /* when divisor is a whole number factor of shifted. */
// 	{
// 		result = factor * (shiftedValue / divisor);
// 	}
// 	else
// 	{
// 		for(uint8_t maxShift = targetShift; maxShift > 0U; maxShift--)
// 		{
// 			if(factor <= (UINT32_MAX >> maxShift))
// 			{
// 				result = (factor << maxShift) / divisor;
// 				if(result <= (UINT32_MAX >> (targetShift - maxShift))) { result = result << (targetShift - maxShift); }
// 				else { result = 0U; } /* error, will overflow 32 bit even using ((factor << 0) / divisor) << leftShift */
// 				break;
// 			}
// 		}
// 	}

// 	return result;
// }