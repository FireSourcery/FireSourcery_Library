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

#define MOT_ANALOG_CONVERSIONS_CONFIG(VPosPin, VPosHost, VAccPin, VAccHost, VSensePin, VSenseHost, HeatPcbPin, HeatPcbHost, HeatMosTopPin, HeatMosTopHost, HeatMosBotPin, HeatMosBotHost, ThrottlePin, ThrottleHost, BrakePin, BrakeHost, p_Hosts, p_Mot) \
{																																																					\
	.CONVERSION_VPOS 				=	CONFIG_ANALOG_N_CONVERSION(MOT_ANALOG_CHANNEL_VPOS, 				0U, 	p_Mot, &((p_Mot)->AnalogResults.Channels[0U]), VPosPin, 		&(p_Hosts)[VPosHost]), 			\
	.CONVERSION_VACC 				=	CONFIG_ANALOG_N_CONVERSION(MOT_ANALOG_CHANNEL_VACC, 				0U, 	p_Mot, &((p_Mot)->AnalogResults.Channels[0U]), VAccPin, 		&(p_Hosts)[VAccHost]), 			\
	.CONVERSION_VSENSE 				=	CONFIG_ANALOG_N_CONVERSION(MOT_ANALOG_CHANNEL_VSENSE, 				0U, 	p_Mot, &((p_Mot)->AnalogResults.Channels[0U]), VSensePin, 		&(p_Hosts)[VSenseHost]), 		\
	.CONVERSION_HEAT_PCB 			=	CONFIG_ANALOG_N_CONVERSION(MOT_ANALOG_CHANNEL_HEAT_PCB, 			0U, 	p_Mot, &((p_Mot)->AnalogResults.Channels[0U]), HeatPcbPin, 		&(p_Hosts)[HeatPcbHost]), 		\
	.CONVERSION_HEAT_MOSFETS_TOP 	=	CONFIG_ANALOG_N_CONVERSION(MOT_ANALOG_CHANNEL_HEAT_MOSFETS_TOP, 	0U, 	p_Mot, &((p_Mot)->AnalogResults.Channels[0U]), HeatMosTopPin, 	&(p_Hosts)[HeatMosTopHost]), 	\
	.CONVERSION_HEAT_MOSFETS_BOT 	=	CONFIG_ANALOG_N_CONVERSION(MOT_ANALOG_CHANNEL_HEAT_MOSFETS_BOT, 	0U,		p_Mot, &((p_Mot)->AnalogResults.Channels[0U]), HeatMosBotPin, 	&(p_Hosts)[HeatMosBotHost]), 	\
	.CONVERSION_THROTTLE 			=	CONFIG_ANALOG_N_CONVERSION(MOT_ANALOG_CHANNEL_THROTTLE, 			0U, 	p_Mot, &((p_Mot)->AnalogResults.Channels[0U]), ThrottlePin, 	&(p_Hosts)[ThrottleHost]), 		\
	.CONVERSION_BRAKE 				=	CONFIG_ANALOG_N_CONVERSION(MOT_ANALOG_CHANNEL_BRAKE, 				0U, 	p_Mot, &((p_Mot)->AnalogResults.Channels[0U]), BrakePin, 		&(p_Hosts)[BrakeHost]), 		\
	.ADCS_GROUP_V 					= 	{ .Flags = ((1U << VPosHost) | (1U << VAccHost) | (1U << VSenseHost)),				},																						\
	.ADCS_GROUP_HEAT 				= 	{ .Flags = ((1U << HeatPcbHost) | (1U << HeatMosTopHost) | (1U << HeatMosBotHost)),	},																						\
	.ADCS_GROUP_USER 				= 	{ .Flags = ((1U << ThrottleHost) | (1U << BrakeHost)),								},																						\
}

#endif