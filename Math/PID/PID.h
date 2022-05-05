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

typedef enum PID_Mode_Tag
{
	PID_MODE_PI,
	PID_MODE_PID,
}
PID_Mode_T;

typedef struct __attribute__((aligned (4U))) PID_Params_Tag
{
	PID_Mode_T Mode;
	uint32_t CalcFreq;
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

typedef const struct PID_Config_Tag
{
	const PID_Params_T * const P_PARAMS;
}
PID_Config_T;

typedef struct PID_Tag
{
	const PID_Config_T CONFIG;
	PID_Params_T Params;
	int32_t KiDivisorFreq; 	/* KpDivisor * CalcFreq */
	int32_t KdFactorFreq; 	/* KdFactor * CalcFreq */
	int32_t ErrorSum;
	int32_t ErrorPrev;
}
PID_T;

#define PID_CONFIG(p_Params)			\
{										\
	.CONFIG = 							\
	{									\
		.P_PARAMS = p_Params,			\
	}									\
}

extern void PID_Init(PID_T * p_pid);
extern void PID_Init_Params
(
	PID_T * p_pid,
	uint32_t calcFreq,
	int32_t kpFactor, int32_t kpDivisor,
	int32_t kiFactor, int32_t kiDivisor,
	int32_t kdFactor, int32_t kdDivisor,
	int32_t outMin, int32_t outMax
);
extern int32_t PID_Calc(PID_T *p_pid, int32_t setpoint, int32_t feedback);
extern void PID_Reset(PID_T * p_pid); 
extern void PID_SetIntegral(PID_T * p_pid, int32_t integral);
extern void PID_SetTunings(PID_T * p_pid, int32_t kpFactor, int32_t kpDivisor, int32_t kiFactor, int32_t kiDivisor, int32_t kdFactor, int32_t kdDivisor);
extern void PID_SetFreq(PID_T * p_pid, uint32_t calcFreq);
extern void PID_SetOutputLimits(PID_T * p_pid, uint32_t min, uint32_t max);

#endif /* PID_H */
