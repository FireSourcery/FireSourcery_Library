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

	//todo frac16 first
 */
/******************************************************************************/
void Linear_Speed_InitAngleRpm(Linear_T * p_linear, uint32_t sampleFreq, uint8_t angleBits, uint16_t speedRef_Rpm)
{
	Linear_Frac16_Init(p_linear, sampleFreq * 60U, (1U << angleBits), 0, speedRef_Rpm);

//	p_linear->YReference 	= speedRef_Rpm;
//	p_linear->XReference 	= (speedRef_Rpm << angleBits) / (sampleFreq * 60U);
//
//	p_linear->Slope 		= sampleFreq * 60U;
//	p_linear->SlopeShift 	= angleBits;
//
//	//todo non iterative
//	while ((p_linear->XReference > INT32_MAX / p_linear->Slope) && (p_linear->SlopeShift > 0U))
//	{
//		p_linear->Slope = p_linear->Slope >> 1U;
//		p_linear->SlopeShift--;
//	}
//
//	//todo maxleftshift divide
//	p_linear->InvSlope 			= (1U << (angleBits + (30U - angleBits))) / (60U * sampleFreq);
//	p_linear->InvSlopeShift 	= (30U - angleBits);
//
//	while ((p_linear->YReference > INT32_MAX / p_linear->InvSlope) && (p_linear->InvSlopeShift > 0U))
//	{
//		p_linear->InvSlope = p_linear->InvSlope >> 1U;
//		p_linear->InvSlopeShift--;
//	}
//
//	p_linear->XOffset 				= 0;
//	p_linear->YOffset 				= 0;
}


void Linear_Speed_InitElectricalAngleRpm(Linear_T * p_linear, uint32_t sampleFreq, uint8_t angleBits, uint8_t polePairs, uint16_t speedRef_Rpm)
{
	Linear_Frac16_Init(p_linear, sampleFreq * 60U / polePairs, (1U << angleBits), 0, speedRef_Rpm);
}

