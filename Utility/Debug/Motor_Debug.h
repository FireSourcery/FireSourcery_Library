
/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

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
    @file   Motor_Debug.h
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_DEBUG_H
#define MOTOR_DEBUG_H

#include "System/SysTime/SysTime.h"


#if  defined(CONFIG_MOTOR_DEBUG_ENABLE)
    uint32_t MicrosRef;
    volatile bool DebugFlag;
    volatile uint32_t DebugError;
    volatile uint32_t DebugTime[10U];
    volatile uint32_t DebugTimeABC[3U];
    volatile uint32_t DebugCounter;
    volatile uint32_t DebugCounter2;
#endif

static inline void Motor_Debug_CaptureRefTime(MotorPtr_T p_motor)
{
#if defined(CONFIG_MOTOR_DEBUG_ENABLE)
    p_motor->MicrosRef = SysTime_GetMicros();
#else
    (void)p_motor; (void)index;
#endif
}

static inline void Motor_Debug_CaptureTime(MotorPtr_T p_motor, uint8_t index)
{
#if defined(CONFIG_MOTOR_DEBUG_ENABLE)
    p_motor->DebugTime[index] = SysTime_GetMicros() - p_motor->MicrosRef;
#else
    (void)p_motor; (void)index;
#endif
}

static inline void Motor_Debug_CapturePeriod(MotorPtr_T p_motor, uint8_t index)
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