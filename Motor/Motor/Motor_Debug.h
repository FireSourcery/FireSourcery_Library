
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
	@file 	Motor_Debug.h
	@author FireSourcery
	@brief
	@version V0
*/
/******************************************************************************/
#ifndef MOTOR_DEBUG_H
#define MOTOR_DEBUG_H

#include "Motor.h"

#include "System/SysTime/SysTime.h"

static inline void Motor_CaptureRefTime(Motor_T * p_motor)
{
#if defined(CONFIG_MOTOR_DEBUG_ENABLE)
	p_motor->MicrosRef = SysTime_GetMicros();
#else
	(void)p_motor; (void)index;
#endif
}

static inline void Motor_Debug_CaptureTime(Motor_T * p_motor, uint8_t index)
{
#if defined(CONFIG_MOTOR_DEBUG_ENABLE)
	p_motor->DebugTime[index] = SysTime_GetMicros() - p_motor->MicrosRef;
#else
	(void)p_motor; (void)index;
#endif
}

static inline void Motor_Debug_CapturePeriod(Motor_T * p_motor, uint8_t index)
{
#if defined(CONFIG_MOTOR_DEBUG_ENABLE)
	p_motor->DebugTime[index + 1] = SysTime_GetMicros() - p_motor->DebugTime[index];
	p_motor->DebugTime[index] = SysTime_GetMicros();
#else
	(void)p_motor; (void)index;
#endif
}

#if defined(CONFIG_MOTOR_DEBUG_ENABLE)
extern void Debug_LED(void);
extern void Debug_LedOn(void);
extern void Debug_LedOff(void);
#endif

#endif