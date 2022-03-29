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
#ifndef MOT_ANALOG_CONVERSION_H
#define MOT_ANALOG_CONVERSION_H

#include "Peripheral/Analog/AnalogN/AnalogN.h"

/*
 * Misra violation  extern data
 */
//extern extern const Analog_VirtualConversionChannel_T MOT_ANALOG_VIRTUAL_MONITOR;
//extern extern const Analog_VirtualConversionChannel_T MOT_ANALOG_VIRTUAL_USER;

extern const Analog_VirtualConversionChannel_T MOT_ANALOG_VIRTUAL_HEAT_PCB;
extern const Analog_VirtualConversionChannel_T MOT_ANALOG_VIRTUAL_HEAT_MOSFETS_TOP;
extern const Analog_VirtualConversionChannel_T MOT_ANALOG_VIRTUAL_HEAT_MOSFETS_BOT;
extern const Analog_VirtualConversionChannel_T MOT_ANALOG_VIRTUAL_VPOS;
extern const Analog_VirtualConversionChannel_T MOT_ANALOG_VIRTUAL_VACC;
extern const Analog_VirtualConversionChannel_T MOT_ANALOG_VIRTUAL_VSENSE;
extern const Analog_VirtualConversionChannel_T MOT_ANALOG_VIRTUAL_THROTTLE;
extern const Analog_VirtualConversionChannel_T MOT_ANALOG_VIRTUAL_BRAKE;


#endif
