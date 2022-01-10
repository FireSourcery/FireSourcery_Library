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
    @file 	PID.h
    @author FireSoucery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef PID_H
#define PID_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
	PID_DIRECTION_DIRECT,
	PID_DIRECTION_REVERSE,
}
PID_Direction_T;

typedef enum
{
	PID_MODE_PI,
	PID_MODE_PID,
}
PID_Mode_T;

typedef struct PID_Params_Tag
{
	uint32_t CalcFreq;
	PID_Mode_T	  		Mode;
	PID_Direction_T 	Direction;
	int32_t KpFactor;
	int32_t KpDivisor;
	int32_t KiFactor;
	int32_t KiDivisor;
	int32_t KdFactor;
	int32_t KdDivisor;
	int32_t OutMin;
	int32_t OutMax;
}
PID_Params_T;

typedef const struct
{
	const PID_Params_T * const P_PARAMS;
//	uint32_t CALC_FREQ;
}
PID_Config_T;

typedef struct PID_Tag
{
	const PID_Config_T CONFIG;
	PID_Params_T Params;

//#ifdef CONFIG_PID_MODE_POINTER
//	volatile int32_t * p_Setpoint;		//Cmd
//	volatile int32_t * p_Feedback;		//Process Variable
//	volatile int32_t * p_Control; 		//Control Variable
//#endif

	//	uint32_t CalcFreq;
//	PID_Mode_T	  		Mode;
//	PID_Direction_T 	Direction;
//	int32_t KpFactor;
//	int32_t KpDivisor;
//	int32_t KiFactor;
//	int32_t KiDivisor;
//	int32_t KdFactor;
//	int32_t KdDivisor;
//	int32_t OutMin;
//	int32_t OutMax;
//
	int32_t KiDivisorFreq; 	//calculation frequency adjusted
	int32_t KdFactorFreq; 	//calculation frequency adjusted

	int32_t ErrorSum;
	int32_t ErrorPrev;

//	uint32_t SampleTime; 				// unit in timer ticks
//	uint32_t TimerFreq;					// convert sample time to standard units
//	volatile uint32_t CalcPeriod; 	// isr count per compute
//	volatile uint32_t CalcCounter;

//	uint32_t TimePrev;
//	uint32_t InputPrev;
}
PID_T;

#define PID_CONFIG(p_Params)			\
{										\
	.CONFIG = 							\
	{									\
		.P_PARAMS = p_Params,			\
	}									\
}

extern int32_t PID_Calc(PID_T *p_pid, int32_t setpoint, int32_t feedback);;
extern void PID_SetTunings(PID_T * p_pid, int32_t kpFactor, int32_t kpDivisor, int32_t kiFactor, int32_t kiDivisor, int32_t kdFactor, int32_t kdDivisor);
extern void PID_SetFreq(PID_T * p_pid, uint32_t calcFreq);
extern void PID_SetOutputLimits(PID_T * p_pid, uint32_t min, uint32_t max);
extern void PID_SetDirection(PID_T * p_pid, PID_Direction_T direction);
extern PID_Direction_T PID_GetDirection(PID_T *p_pid);
extern void PID_Init_Args
(
	PID_T * p_pid,
//#ifdef CONFIG_PID_MODE_POINTER
//	int32_t * p_setpoint, int32_t * p_feedback, int32_t * p_control,
//#endif
	int32_t kpFactor, int32_t kpDivisor,
	int32_t kiFactor, int32_t kiDivisor,
	int32_t kdFactor, int32_t kdDivisor,
	uint32_t calcFreq,
	int32_t outMin, int32_t outMax
);

#endif /* PID_H */
