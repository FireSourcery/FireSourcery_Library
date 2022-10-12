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
	@file 	Linear_Speed.h
	@author FireSourcery
	@brief
	@version V0
*/
/******************************************************************************/
#ifndef LINEAR_SPEED_H
#define LINEAR_SPEED_H

#include "Linear_Frac16.h"
#include <stdbool.h>

static inline int32_t Linear_Speed_CalcAngleRpm(const Linear_T * p_linear, uint16_t angle)
{
	return Linear_Frac16_Units(p_linear, angle);
}

static inline int32_t Linear_Speed_CalcRpmAngle(const Linear_T * p_linear, uint32_t rpm)
{
	return Linear_Frac16_InvUnits(p_linear, rpm);
}

static inline int32_t Linear_Speed_CalcAngleRpmFrac16(const Linear_T * p_linear, int16_t angle)
{
	return Linear_Frac16(p_linear, angle);
}

static inline int32_t Linear_Speed_CalcRpmFrac16Angle(const Linear_T * p_linear, uint32_t rpm)
{
	return Linear_Frac16_Inv(p_linear, rpm);
}

extern void Linear_Speed_InitAngleRpm(Linear_T * p_linear, uint32_t sampleFreq, uint8_t angleBits, uint16_t speedRef_Rpm);
extern void Linear_Speed_InitElectricalAngleRpm(Linear_T * p_linear, uint32_t sampleFreq, uint8_t angleBits, uint8_t polePairs, uint16_t speedRef_Rpm);

#endif
