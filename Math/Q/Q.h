/**************************************************************************/
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
/**************************************************************************/
/**************************************************************************/
/*!
	@file 	Q.h
	@author FireSoucery
	@brief 	Q module common functions
	@version V0
*/
/**************************************************************************/
#ifndef Q_MATH_H_
#define Q_MATH_H_

#include <stdint.h>

static inline int16_t q_sat16(int32_t x)
{
	int16_t sat;

	if 		(x > (int32_t)INT16_MAX) 	{sat = INT16_MAX;}
	else if (x < (int32_t)INT16_MIN) 	{sat = INT16_MIN;}
	else 								{sat = (int16_t)x;}

	return sat;
}

/*!
	@brief Calculates square root

	Babylonian method
	y[n] = ( y[n-1] + (x / y[n-1]) ) / 2

	<https://en.wikipedia.org/wiki/Methods_of_computing_square_roots#Babylonian_method>
 */
static inline uint16_t q_sqrt(int32_t x)
{
	uint8_t iteration = 0;
	uint32_t yPrev;
	uint32_t y;

	if (x > 0)
	{
		/*
		 * Set y initial to value such that 0 < x <= UINT32_MAX is solved in 6 iterations or less
		 */
		if ((uint32_t) x > (uint32_t) 1048576U) /* (1 << 20) */
		{
			yPrev = (uint32_t) 8192U; /* 8192*8192 == (1 << 26), solve 0x7FFFFFFF in 6 iterations */
		}
		else
		{
			yPrev = (uint32_t) 128U; /* 128*128 == (1 << 14), solve < 1048576 */
		}

		do
		{
			y = (yPrev + ((uint32_t) x / yPrev)) / (uint32_t) 2U;

			if (y == yPrev)
			{
				break;
			}

			iteration++;
			yPrev = y;

		} while (iteration < 6U);
	}
	else
	{
		y = (uint32_t) 0U;
	}

	return (uint16_t) y;
}

#endif
