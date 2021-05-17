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
    @file 	PID.h
    @author FireSoucery
    @brief
    @version V0

    Created on: Nov 26, 2019
*/
/**************************************************************************/
#ifndef PID_H
#define PID_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
	PID_DIRECTION_DIRECT, PID_DIRECTION_REVERSE,
} PID_Direction_T;

typedef struct PID_Tag
{
//	PID_Mode_T	  		Mode;
	PID_Direction_T 	Direction;

#ifdef CONFIG_PID_MODE_POINTER
	volatile int32_t * p_Setpoint;
	volatile int32_t * p_Feedback;		//Process Variable
	volatile int32_t * p_Control; 		//Control Variable
#else
	volatile int32_t Setpoint;
	volatile int32_t Feedback;	//Process Variable
	volatile int32_t Control; 	//Control Variable
#endif

	int32_t KpFactor;		// user input, time base in seconds
	int32_t KpDivisor;		// user input, time base in seconds
	int32_t KiFactor;		// user input, time base in seconds
	int32_t KiDivisor;		// user input, time base in seconds
	int32_t KdFactor;		// user input, time base in seconds
	int32_t KdDivisor;		// user input, time base in seconds

	int32_t KiDivisorCalcFreq; 	//calculation frequency adjusted
	int32_t KdFactorCalcFreq; 	//calculation frequency adjusted

	uint32_t CalcFreq;

	volatile int32_t ErrorSum;
	volatile int32_t ErrorPrev;

//	uint32_t SampleTime; 				// unit in timer ticks
//	uint32_t TimerFreq;					// convert sample time to standard units
//	volatile uint32_t CalcPeriod; 	// isr count per compute
//	volatile uint32_t CalcCounter;

//	uint32_t TimePrev;
//	uint32_t InputPrev;

	int32_t OutMin;
	int32_t OutMax;
}
PID_T;

extern int32_t PID_Calc(PID_T *p_pid, int32_t setpoint, int32_t feedback);;
extern void PID_SetTunings(PID_T * p_pid, int32_t kpFactor, int32_t kpDivisor, int32_t kiFactor, int32_t kiDivisor, int32_t kdFactor, int32_t kdDivisor);
extern void PID_SetCalcFreq(PID_T * p_pid, uint32_t calcFreq);
extern void PID_SetOutputLimits(PID_T * p_pid, uint32_t min, uint32_t max);
extern void PID_SetDirection(PID_T * p_pid, PID_Direction_T direction);
extern PID_Direction_T PID_GetDirection(PID_T *p_pid);
extern void PID_Init
(
	PID_T * p_pid,
#ifdef CONFIG_PID_MODE_POINTER
	int32_t * p_setpoint, int32_t * p_feedback, int32_t * p_control,
#endif
	int32_t kpFactor, int32_t kpDivisor,
	int32_t kiFactor, int32_t kiDivisor,
	int32_t kdFactor, int32_t kdDivisor,
	uint32_t calcFreq,
	int32_t outMin, int32_t outMax
);

#endif /* PID_H */
