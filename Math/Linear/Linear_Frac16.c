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
    @file 	Linear_Frac16.c
    @author FireSoucery
    @brief  Linear
    @version V0
*/
/******************************************************************************/
#include "Linear_Frac16.h"

/*
 * INT32_MAX == 200%, leave room for oversaturation
 * i.e. overflow at x >= 2*XRef
 */
void Linear_Frac16_Init(Linear_T * p_linear, int32_t factor, int32_t divisor, int32_t y0, int32_t yRef)
{
	p_linear->SlopeFactor 			= (65536 << 14U) / (divisor * yRef / factor); /* scale factor to 65536 */
	p_linear->SlopeDivisor_Shift 	= 14U;
	p_linear->SlopeDivisor 			= ((divisor * yRef / factor) << 14U) / 65536;
	p_linear->SlopeFactor_Shift 	= 14U;
	p_linear->XOffset 				= 0;
	p_linear->YOffset 				= y0;
	p_linear->XReference 			= (divisor * yRef / factor);
	p_linear->YReference 			= yRef;
}


void Linear_Frac16_Init_X0(Linear_T * p_linear, int32_t factor, int32_t divisor, int32_t x0, int32_t yRef)
{
	p_linear->SlopeFactor 			= (65536 << 14U) / (divisor);
	p_linear->SlopeDivisor_Shift 	= 14U;
	p_linear->SlopeDivisor 			= (divisor << 14U) / 65536;
	p_linear->SlopeFactor_Shift 	= 14U;
	p_linear->XOffset 				= x0;
	p_linear->YOffset 				= 0;
	p_linear->XReference 			= (divisor * yRef / factor);
	p_linear->YReference 			= yRef;
}
