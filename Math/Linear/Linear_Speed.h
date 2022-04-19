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
    @file 	Linear_Speed.h
    @author FireSoucery
    @brief	Linear Frac16 calc without division
    @version V0
*/
/******************************************************************************/
#ifndef LINEAR_SPEED_H
#define LINEAR_SPEED_H

#include "Linear.h"

#include <stdbool.h>

static inline int32_t CalcLinearSpeedDeltaAngle(int16_t angle, int16_t anglePrev, bool isDirectionPositive)
{
	uint16_t deltaAngle;

	if(isDirectionPositive)
	{
		if(angle < anglePrev) /* overflow on increment */
		{
			deltaAngle = (uint32_t)65535U - anglePrev + angle + 1U;
		}
		else /* normal case */
		{
			deltaAngle = angle - anglePrev;
		}
	}
	else
	{
		if(angle > anglePrev) /* overflow on decrement */
		{
			deltaAngle = (uint32_t)65535U + anglePrev - angle + 1U;
		}
		else /* normal case */
		{
			deltaAngle = anglePrev - angle;
		}
	}

	return deltaAngle;
}

static inline int32_t Linear_Speed_CalcAngleRpm(const Linear_T * p_linear, int16_t angle, int16_t anglePrev, bool isDirectionPositive)
{
	uint16_t deltaAngle = CalcLinearSpeedDeltaAngle(angle, anglePrev, isDirectionPositive);
	return Linear_Function(p_linear, deltaAngle);
}


static inline int32_t Linear_Speed_CalcRpmAngle(const Linear_T * p_linear, uint32_t rpm)
{
	return Linear_InvFunction(p_linear, rpm);
}

static inline int32_t Linear_Speed_CalcAngleRpmFrac16(const Linear_T * p_linear, int16_t angle, int16_t anglePrev, bool isDirectionPositive)
{
	uint16_t deltaAngle = CalcLinearSpeedDeltaAngle(angle, anglePrev, isDirectionPositive);
	return Linear_Function_Fraction16(p_linear, deltaAngle);
}

static inline int32_t Linear_Speed_CalcRpmFrac16Angle(const Linear_T * p_linear, uint32_t rpm)
{
	return Linear_InvFunction_Fraction16(p_linear, rpm);
}

#endif
