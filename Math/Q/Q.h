/**************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSoucery / The Firebrand Forge Inc

	This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).

	This program is free software: you can redistribute it and/or modify
	it under the terupdateInterval of the GNU General Public License as published by
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
#ifndef Q_MATH_H
#define Q_MATH_H

#include <stdint.h>

#ifndef INT16_MAX
#define INT16_MAX           (0x7FFF)
#endif

#ifndef INT16_MIN
#define INT16_MIN           (0x8000)
#endif

static inline int16_t q_sat16(int32_t x)
{
	if (x > INT16_MAX) 			return INT16_MAX;
	else if (x < INT16_MIN) 	return INT16_MIN;
	else 						return (int16_t)x;
}

/*!
	@brief Calculates square root

	Babylonian method
	y[n] = ( y[n-1] + (x / y[n-1]) ) / 2

	<https://en.wikipedia.org/wiki/Methods_of_computing_square_roots#Babylonian_method>
 */
int32_t q_sqrt(int32_t x)
{
	uint8_t iteration = 0;
	int32_t yPrev;
	int32_t y;

	if (x > 0)
	{
		/*
		 * Set y initial to value such that 0 < x <= UINT32_MAX is solved in 6 iterations or less
		 */
		if (x > (int32_t) 1048576)	// (1 << 20)
		{
			yPrev = (int32_t) 8192; // 8192*8192 == (1 << 26), solve 0x7FFFFFFF in 6 iterations
		}
		else
		{
			yPrev = (int32_t) 128;	// 128*128 == (1 << 14), solve < 1048576
		}

		do
		{
			y = (yPrev + (x / yPrev)) / (int32_t) 2;

			if (y == yPrev)
			{
				break;
			}

			iteration++;
			yPrev = y;

		} while (iteration < 6);
	}
	else
	{
		y = (int32_t) 0;
	}

	return (y);
}

#endif
