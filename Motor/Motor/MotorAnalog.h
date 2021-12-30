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
#ifndef MOTOR_ANALOG_H
#define MOTOR_ANALOG_H

#include "Peripheral/Analog/AnalogN/AnalogN.h"

#define MOTOR_ANALOG_CHANNEL_MOTOR_COUNT 	8U

/*!
	@brief Virtual channel identifiers, index into arrays containing Analog channel
 */
typedef enum
{
	MOTOR_ANALOG_CHANNEL_VPOS, 			/* Physically shared channel, but save results in contiguous memory */
	MOTOR_ANALOG_CHANNEL_VA,
	MOTOR_ANALOG_CHANNEL_VB,
	MOTOR_ANALOG_CHANNEL_VC,
	MOTOR_ANALOG_CHANNEL_IA,
	MOTOR_ANALOG_CHANNEL_IB,
	MOTOR_ANALOG_CHANNEL_IC,
	MOTOR_ANALOG_CHANNEL_HEAT_MOTOR,	/* Temperature */
} MotorAnalog_ChannelId_T;

/*
 * Results Buffer
 */
typedef union
{
	struct
	{
		analog_adcresult_t VPos_ADCU;
		analog_adcresult_t Va_ADCU;
		analog_adcresult_t Vb_ADCU;
		analog_adcresult_t Vc_ADCU;
		analog_adcresult_t Ia_ADCU;
		analog_adcresult_t Ib_ADCU;
		analog_adcresult_t Ic_ADCU;
		analog_adcresult_t HeatMotor_ADCU;
	};
	analog_adcresult_t Channels[MOTOR_ANALOG_CHANNEL_MOTOR_COUNT];
}
MotorAnalog_Map_T;


#endif
