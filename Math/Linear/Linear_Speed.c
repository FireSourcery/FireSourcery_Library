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
    @file 	Linear_Speed.c
    @author FireSoucery
    @brief  Linear
    @version V0
*/
/******************************************************************************/
#include "Linear_Speed.h"


static inline uint32_t speed_a16torpm(uint32_t angle16, uint32_t sampleFreq)
{
	uint32_t rpm;

	if (angle16 < (UINT32_MAX / sampleFreq * 60U))
	{
		rpm =  (angle16 * sampleFreq * 60U >> 16U);
	}
	else
	{
		rpm =  (angle16 * sampleFreq >> 16U) * 60U;
	}

	return rpm;
}

static inline uint32_t speed_rpmtoa16(uint32_t rpm, uint32_t sampleFreq)
{
	uint32_t angle;

	if (rpm < (UINT32_MAX >> 16U))
	{
		angle =  (rpm << 16U) / (60U * sampleFreq);
	}
	else
	{
		angle = (1U << 16U) / 60U * rpm / sampleFreq;
	}

	return angle;
}

/******************************************************************************/
/*!
	f(angle) = rpm
	f16(angle) = speed_frac16

	f(angleMax) = speedRef_Rpm
 */
/******************************************************************************/
void Linear_Speed_InitAngleRpm(Linear_T * p_linear, uint32_t sampleFreq, uint8_t angleBits, uint16_t speedRef_Rpm)
{
	p_linear->YReference 			= speedRef_Rpm;
	p_linear->XReference 			= (speedRef_Rpm << angleBits) / (sampleFreq * 60U);

//	(65536 << 14U) / (divisor * yRef / factor);

	p_linear->SlopeFactor 			= sampleFreq * 60U;
	p_linear->SlopeDivisor_Shift 	= angleBits;

	//todo non iterative
	while (p_linear->XReference > (INT32_MAX / p_linear->SlopeFactor))
	{
		p_linear->SlopeFactor 			= p_linear->SlopeFactor >> 1U;
		p_linear->SlopeDivisor_Shift 	= p_linear->SlopeDivisor_Shift - 1U;
	}

	p_linear->SlopeDivisor 			= (1U << angleBits) / (60U * sampleFreq);
	p_linear->SlopeFactor_Shift 	= 0;

//	while((((p_linear->YReference << angleBits) / (60U * sampleFreq)) << p_linear->SlopeFactor_Shift) < INT32_MAX / 2U)
	while((p_linear->YReference * p_linear->SlopeDivisor < INT32_MAX / 2U) && (angleBits + p_linear->SlopeFactor_Shift < 16U))
	{
		p_linear->SlopeFactor_Shift 	= p_linear->SlopeFactor_Shift + 1U;
		p_linear->SlopeDivisor 			= (1U << (angleBits + p_linear->SlopeFactor_Shift)) / (60U * sampleFreq);
	}

	p_linear->XOffset 				= 0;
	p_linear->YOffset 				= 0;
}





