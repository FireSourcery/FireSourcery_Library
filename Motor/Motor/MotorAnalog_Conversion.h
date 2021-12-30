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
    @file 	.h
    @author FireSoucery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_ANALOG_CONVERSION_H
#define MOTOR_ANALOG_CONVERSION_H

#include "Peripheral/Analog/AnalogN/AnalogN.h"

//typedef enum
//{
//	MOTOR_ANALOG_CONVERSION_BEMF_A_INDEX_VPOS = 0U,
//} MotorAnalog_ConversionBemfAIndex_T;

/*
 * Misra violation
 */

extern const Analog_ConversionVirtual_T MOTOR_ANALOG_VIRTUAL_BEMF_A;
//extern const Analog_ConversionVirtual_T MOTOR_ANALOG_VIRTUAL_BEMF_A_REPEAT;
extern const Analog_ConversionVirtual_T MOTOR_ANALOG_VIRTUAL_BEMF_B;
extern const Analog_ConversionVirtual_T MOTOR_ANALOG_VIRTUAL_BEMF_C;
extern const Analog_ConversionVirtual_T MOTOR_ANALOG_VIRTUAL_BEMF_REMAINDER;
extern const Analog_ConversionVirtual_T MOTOR_ANALOG_VIRTUAL_FOC_IABC;
extern const Analog_ConversionVirtual_T MOTOR_ANALOG_VIRTUAL_FOC_REMAINDER;
extern const Analog_ConversionVirtual_T MOTOR_ANALOG_VIRTUAL_IDLE;

#endif
