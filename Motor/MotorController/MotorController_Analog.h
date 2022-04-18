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

#define MOT_ANALOG_CONVERSIONS_CONFIG(VPosPin, p_VPosHost, VAccPin, p_VAccHost, VSensePin, p_VSenseHost, HeatPcbPin, p_HeatPcbHost, HeatMosTopPin, p_HeatMosTopHost, HeatMosBotPin, p_HeatMosBotHost, ThrottlePin, p_ThrottleHost, BrakePin, p_BrakeHost, p_Mot) \
{																																																	\
	.CONVERSION_VPOS 				=	CONFIG_ANALOG_N_CONVERSION(MOT_ANALOG_CHANNEL_VPOS, 				0, 	p_Mot, &((p_Mot)->AnalogResults.Channels[0U]), VPosPin, 		p_VPosHost), 		\
	.CONVERSION_VACC 				=	CONFIG_ANALOG_N_CONVERSION(MOT_ANALOG_CHANNEL_VACC, 				0, 	p_Mot, &((p_Mot)->AnalogResults.Channels[0U]), VAccPin, 		p_VAccHost), 		\
	.CONVERSION_VSENSE 				=	CONFIG_ANALOG_N_CONVERSION(MOT_ANALOG_CHANNEL_VSENSE, 				0, 	p_Mot, &((p_Mot)->AnalogResults.Channels[0U]), VSensePin, 		p_VSenseHost), 		\
	.CONVERSION_HEAT_PCB 			=	CONFIG_ANALOG_N_CONVERSION(MOT_ANALOG_CHANNEL_HEAT_PCB, 			0, 	p_Mot, &((p_Mot)->AnalogResults.Channels[0U]), HeatPcbPin, 		p_HeatPcbHost), 	\
	.CONVERSION_HEAT_MOSFETS_TOP 	=	CONFIG_ANALOG_N_CONVERSION(MOT_ANALOG_CHANNEL_HEAT_MOSFETS_TOP, 	0, 	p_Mot, &((p_Mot)->AnalogResults.Channels[0U]), HeatMosTopPin, 	p_HeatMosTopHost), 	\
	.CONVERSION_HEAT_MOSFETS_BOT 	=	CONFIG_ANALOG_N_CONVERSION(MOT_ANALOG_CHANNEL_HEAT_MOSFETS_BOT, 	0,	p_Mot, &((p_Mot)->AnalogResults.Channels[0U]), HeatMosBotPin, 	p_HeatMosBotHost), 	\
	.CONVERSION_THROTTLE 			=	CONFIG_ANALOG_N_CONVERSION(MOT_ANALOG_CHANNEL_THROTTLE, 			0, 	p_Mot, &((p_Mot)->AnalogResults.Channels[0U]), ThrottlePin, 	p_ThrottleHost), 	\
	.CONVERSION_BRAKE 				=	CONFIG_ANALOG_N_CONVERSION(MOT_ANALOG_CHANNEL_BRAKE, 				0, 	p_Mot, &((p_Mot)->AnalogResults.Channels[0U]), BrakePin, 		p_BrakeHost), 		\
}

#endif
