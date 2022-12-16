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
	@file 	Global_Motor.h
	@author FireSourcery
	@brief  Per Motor State Control.
	@version V0
*/
/******************************************************************************/
#ifndef GLOBAL_MOTOR_H
#define GLOBAL_MOTOR_H

#include "Config.h"
#include <stdint.h>
#include <stdbool.h>

/* Global Static Const  */
typedef const struct
{
	const uint16_t CONTROL_FREQ;
	const uint16_t V_MAX_VOLTS;
	const uint16_t I_MAX_AMP; 				/* Motor I controller rating. pass to Linear_ADC. Unit conversion, UI input/output, param set. */
	const uint16_t I_ZERO_TO_PEAK_ADCU; 	/* Sensor calibration *//* Zero-To-Peak, derived from sensor hardware */
	const uint16_t VABC_R1;
	const uint16_t VABC_R2;
	const uint16_t ALIGN_VOLTAGE_MAX;
	const uint32_t CONTROL_ANALOG_DIVIDER; 	/* In Pow2 - 1 */
	const uint8_t INIT_WAIT;
}
Global_Motor_T;

/* MISRA violation */
/* Define in Main App */
extern const Global_Motor_T GLOBAL_MOTOR;

extern uint16_t Global_Motor_GetAdcVRef(void);
extern uint16_t Global_Motor_GetVSourceRef(void);
extern void Global_Motor_InitAdcVRef_MilliV(uint16_t adcVRef_MilliV);
extern void Global_Motor_InitVSourceRef_V(uint16_t vRefSupply);

#endif
