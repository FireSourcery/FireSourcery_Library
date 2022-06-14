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
	@brief
	@version V0
*/
/******************************************************************************/
#include "Linear_Speed.h"

static inline uint32_t speed_angle16torpm(uint32_t angle16, uint32_t sampleFreq)
{
	uint32_t rpm;
	if(angle16 < (UINT32_MAX / sampleFreq * 60U)) 	{ rpm = (angle16 * sampleFreq * 60U >> 16U); }
	else 											{ rpm = (angle16 * sampleFreq >> 16U) * 60U; }
	return rpm;
}

static inline uint32_t speed_rpmtoangle16(uint32_t rpm, uint32_t sampleFreq)
{
	uint32_t angle;
	if(rpm < (UINT32_MAX >> 16U)) 	{ angle = (rpm << 16U) / (60U * sampleFreq); }
	else 							{ angle = (1U << 16U) / 60U * rpm / sampleFreq; }
	return angle;
}

/******************************************************************************/
/*!
	f(angle) = rpm
	f16(angle) = speed_frac16
	f(angleMax == XRef) = speedRef_Rpm

	SlopeFactor = sampleFreq * 60U
	SlopeDivisor = (1U << angleBits)
 */
 /******************************************************************************/
void Linear_Speed_InitAngleRpm(Linear_T * p_linear, uint32_t sampleFreq, uint8_t angleBits, uint16_t speedRef_Rpm)
{
	Linear_Frac16_Init(p_linear, (sampleFreq * 60U), (1U << angleBits), 0, speedRef_Rpm);
}

void Linear_Speed_InitElectricalAngleRpm(Linear_T * p_linear, uint32_t sampleFreq, uint8_t angleBits, uint8_t polePairs, uint16_t speedRef_Rpm)
{
	Linear_Frac16_Init(p_linear, (sampleFreq * 60U / polePairs), (1U << angleBits), 0, speedRef_Rpm);
}

