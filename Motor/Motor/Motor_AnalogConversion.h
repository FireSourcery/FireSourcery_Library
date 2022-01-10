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


/*
 * Misra violation
 */

extern const Analog_VirtualConversionChannel_T MOTOR_ANALOG_VIRTUAL_VPOS_PWM_ON;
extern const Analog_VirtualConversionChannel_T MOTOR_ANALOG_VIRTUAL_VPOS;
extern const Analog_VirtualConversionChannel_T MOTOR_ANALOG_VIRTUAL_VA;
extern const Analog_VirtualConversionChannel_T MOTOR_ANALOG_VIRTUAL_VB;
extern const Analog_VirtualConversionChannel_T MOTOR_ANALOG_VIRTUAL_VC;
extern const Analog_VirtualConversionChannel_T MOTOR_ANALOG_VIRTUAL_IA;
extern const Analog_VirtualConversionChannel_T MOTOR_ANALOG_VIRTUAL_IB;
extern const Analog_VirtualConversionChannel_T MOTOR_ANALOG_VIRTUAL_IC;
extern const Analog_VirtualConversionChannel_T MOTOR_ANALOG_VIRTUAL_HEAT;

//extern const Analog_VirtualConversionChannel_T MOTOR_ANALOG_VIRTUAL_BEMF_REMAINDER;
//extern const Analog_VirtualConversionChannel_T MOTOR_ANALOG_VIRTUAL_FOC_IABC;
//extern const Analog_VirtualConversionChannel_T MOTOR_ANALOG_VIRTUAL_FOC_REMAINDER;
//extern const Analog_VirtualConversionChannel_T MOTOR_ANALOG_VIRTUAL_IDLE;

#endif
