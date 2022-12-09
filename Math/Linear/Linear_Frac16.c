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
	@file 	Linear_Frac16.c
	@author FireSourcery
	@brief  Linear
	@version V0
*/
/******************************************************************************/
#include "Linear_Frac16.h"

/******************************************************************************/
/*!
	Linear with

	frac16 conversion returns without division, as frac16 calc is performed more frequently
	adcu to physical(user yref) returns without division
	division in physical to adcu, frac16 to physical units
	Shift 14 to allow oversaturation f([-2*XRef:2*XRef]) == [-2*YRef:2*YRef] before overflow
		i.e 2x input range, before overflow, while retaining sign bit
	p_linear->YReference in Units
	p_linear->YOffset in Frac16;
*/
/******************************************************************************/

/*
	Init using slope, y. Derive XRef
	Sets 100% input, x, as f(xRef) == yRef
	f(x>2XRef) will overflow
	Shift 14 to allow oversaturation f([-2*XRef:2*XRef]) == [-2*YRef:2*YRef] before overflow
	Scales factor to 65536
	Scales divisor to xref
*/
void Linear_Frac16_Init(Linear_T * p_linear, int32_t factor, int32_t divisor, int32_t y0_Frac16, int32_t yRef_Units)
{
	p_linear->YReference = yRef_Units;
	p_linear->XReference = linear_invf(factor, divisor, 0, yRef_Units); /* (yRef_Units - 0)*divisor/factor */
	p_linear->Slope = (65536 << 14U) / p_linear->XReference; /* x0 == 0 */
	p_linear->SlopeShift = 14U;
	p_linear->InvSlope = (p_linear->XReference << 14U) / 65536; //todo maxleftshift, if factor > divisor, invslope can be > 14
	p_linear->InvSlopeShift = 14U;
	p_linear->XOffset = 0;
	p_linear->YOffset = y0_Frac16;
}

/*!
	Map [x0, xRef] to [y0, 65535]
	Init using (x0, y0), (xref, yref). Derive slope
	@param[in] y0_Frac16 frac16 offset
	@param[in] yRef user units equivalent to 65536, 100%
*/
void Linear_Frac16_Init_Map(Linear_T * p_linear, int32_t x0, int32_t xRef, int32_t y0_Frac16, int32_t yRef_Units)
{
	p_linear->Slope = (65536 << 14U) / (xRef - x0);
	p_linear->SlopeShift = 14U;
	p_linear->InvSlope = ((xRef - x0) << 14U) / 65536; //todo maxleftshift, if factor > divisor, invslope can be > 14
	p_linear->InvSlopeShift = 14U;
	p_linear->XOffset = x0;
	p_linear->YOffset = y0_Frac16;
	p_linear->XReference = xRef; /* Unused for calculations, User info */
	p_linear->YReference = yRef_Units; /* Retain for "Units" conversion only */
}

// void Linear_Frac16_Init_X0XRef(Linear_T * p_linear, int32_t factor, int32_t divisor, int32_t x0, int32_t xRef)
// {

// }

