#include <Motor/Utility/MotAnalogMonitor/MotAnalogMonitor_Motor.h>
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
    @file 	MotAnalogUser_Motor.h
    @author FireSoucery
    @brief	Parse conditions and actuate
    @version V0
*/
/**************************************************************************/
#include "MotAnalogUser.h"

#include "Motor/Motor/Motor.h"


//#include "Math/Linear/Linear_ADC.h"
//#include "Math/Linear/Linear.h"

#include <stdint.h>
#include <stdbool.h>


//returns difference, absolute value
uint32_t Bound_CheckUpper(Bound_t * p_bound, int32_t var)
{
	if (var > p_bound->UpperLimit)	return (var - p_bound->UpperLimit);
	else 							return (0);
}

uint32_t Bound_CheckLower(Bound_t * p_bound, int32_t var)
{
	if (var < p_bound->LowerLimit)	return (p_bound->LowerLimit - var);
	else 							return (0);
}

//int32_t Bound_CheckUpperSigned(Bound_t * bound, int32_t Var)
//{
//	if (Var > bound->UpperLimit)	return (Var - bound->UpperLimit);
//	else 							return (0);
//}
//
//int32_t Bound_CheckLowerSigned(Bound_t * bound)
//{
//
//}

void Bound_Proc(Bound_t * p_bound, int32_t var)
{
	if (Bound_CheckUpper(p_bound, var) != 0)
	{
		p_bound->RespondOverLimit(Bound_CheckUpper(p_bound, var));
	}
	else if (Bound_CheckLower(p_bound, var) != 0)
	{
		p_bound->RespondUnderLimt(Bound_CheckLower(p_bound, var));
	}
}

static inline void ProcMotAnalogMonitorMotorLimit
(
	uint32_t value,
	uint32_t upperLimit,
	uint32_t lowerLimit,
	void (*respondOverLimit)(Motor_T *),
	void (*respondUnderLimit)(Motor_T *),
	Motor_T * p_motor
)
{

}

void MotAnalogMonitor_Motor_ProcLimit(
	const MotAnalogMonitor_T * p_monitor,
	Motor_T * p_motorDest,
	uint8_t motorCount,
	void (*respondOverLimit)(Motor_T*),
	void (*respondUnderLimit)(Motor_T *))
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		ProcMotAnalogMonitorMotorLimit(&p_motorDest[iMotor]);
	}
}


void MotAnalogMonitor_Motor_WriteMotor(const MotAnalogMonitor_T * p_monitor, Motor_T * p_motorDest)
{

}
