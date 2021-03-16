/*******************************************************************************/
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

#ifdef CONFIG_LINEAR_NUMIRICAL_DIVIDE
void Linear_Init(Linear_T * p_linear, int32_t factor, int32_t divisor, int32_t offset)
{
	p_linear->SlopeFactor 	= factor;
	p_linear->SlopeDivisor 	= divisor;
	p_linear->Offset 		= offset;
}

#elif defined(CONFIG_LINEAR_SHIFT_DIVIDE)
/*
 * Left shift must retain sign bit
 */
void Linear_Init(Linear_T * p_linear, int16_t factor, int16_t divisor, int32_t offset)
{
	p_linear->SlopeFactor 	= (factor << 16) / divisor;
	p_linear->SlopeDivisor 	= (divisor << 16) / factor;

	p_linear->SlopeFactor 	= (factor << shift) / divisor;
	p_linear->SlopeDivisor_Shift = shift;

	p_linear->Offset = offset;
}
#endif
