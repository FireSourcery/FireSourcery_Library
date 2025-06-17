
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

*/
/******************************************************************************/
#ifndef MOTOR_DEBUG_H
#define MOTOR_DEBUG_H

#include "System/SysTime/SysTime.h"
#include "Motor.h"

// // Add motor health monitoring
// typedef struct FOC_Diagnostics
// {
//     uint32_t overCurrentCount;
//     uint32_t overVoltageCount;
//     uint32_t commutationErrorCount;
//     uint32_t sensorErrorCount;
//     ufract16_t maxCurrent;
//     ufract16_t avgPower;
//     fract16_t temperatureEst;
// }
// FOC_Diagnostics_T;

// static inline void FOC_UpdateDiagnostics(FOC_T * p_foc, FOC_Diagnostics_T * p_diag)
// {
//     ufract16_t iMag = FOC_GetIMagnitude(p_foc);
//     ufract16_t vMag = FOC_GetVMagnitude(p_foc);

//     /* Track maximum current */
//     if (iMag > p_diag->maxCurrent)
//     {
//         p_diag->maxCurrent = iMag;
//     }

//     /* Count fault conditions */
//     if (iMag > MOTOR_CURRENT_LIMIT)
//     {
//         p_diag->overCurrentCount++;
//     }

//     if (vMag > MOTOR_VOLTAGE_LIMIT)
//     {
//         p_diag->overVoltageCount++;
//     }

//     /* Estimate temperature from I²R losses */
//     accum32_t power = FOC_GetTotalPower(p_foc);
//     p_diag->avgPower = (p_diag->avgPower * 15 + power) >> 4;  /* IIR filter */
// }

//todo replace with trace
static inline void Motor_Debug_CaptureRefTime(Motor_State_T * p_motor)
{
#ifndef NDEBUG
    p_motor->MicrosRef = SysTime_GetMicros();
#else
    (void)p_motor;
#endif
}

static inline void Motor_Debug_CaptureTime(Motor_State_T * p_motor, uint8_t index)
{
#ifndef NDEBUG
    p_motor->DebugTime[index] = SysTime_GetMicros() - p_motor->MicrosRef;
#else
    (void)p_motor; (void)index;
#endif
}

static inline void Motor_Debug_CapturePeriod(Motor_State_T * p_motor, uint8_t index)
{
#ifndef NDEBUG
    p_motor->DebugTime[index + 1] = SysTime_GetMicros() - p_motor->DebugTime[index];
    p_motor->DebugTime[index] = SysTime_GetMicros();
#else
    (void)p_motor; (void)index;
#endif
}

#ifndef NDEBUG
extern void Debug_LED(void);
extern void Debug_LedOn(void);
extern void Debug_LedOff(void);
extern void Debug_Beep(void);
#endif

#endif