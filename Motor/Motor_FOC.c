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
    @file 	Motor_FOC.h
    @author FireSoucery
    @brief  Motor FOC submodule. FOC control functions.
    @version V0
*/
/**************************************************************************/

#include "Motor_FOC.h"
#include "Motor.h"

#include "Config.h"
//#include "Default.h"

#include "Transducer/Encoder/Encoder_Motor.h"
#include "Transducer/Encoder/Encoder.h"

#include "Math/Q/QFrac16.h"
#include "Math/Linear/Linear.h"

#include <stdbool.h>
#include <stdint.h>




/*
	For motor controlled virtualized analog module
 */
/*!
	@brief FOC mode ADC channels to be sampled

	Sample all channels sequentially on PWM trigger
	Measure Ia, Ib, Ic first, so there is longer time for FOC calculations
	remaining channels will continue processing as FOC calculations are performed
 */
//const uint8_t FOC_ANALOG_CONVERSION_CHANNELS_ANGLE_CONTROL[] =
//{
//	[0] = MOTOR_ANALOG_CHANNEL_IA,
//	[1] = MOTOR_ANALOG_CHANNEL_IB,
//	[2] = MOTOR_ANALOG_CHANNEL_IC,
//};
//
//void (* FOC_ANALOG_CONVERSION_CHANNEL_FUNCTIONS_ANGLE_CONTROL[])(Motor_T *) =
//{
//	[0] = Motor_FOC_ConvertIa,
//	[1] = Motor_FOC_ConvertIb,
//	[2] = Motor_FOC_ConvertIc,
//};
//
//Analog_Conversion_T FOC_ANALOG_CONVERSION_ANGLE_CONTROL =
//{
//	.p_VirtualChannels 	= FOC_ANALOG_CONVERSION_CHANNELS_ANGLE_CONTROL,
//	.ChannelCount = sizeof(FOC_ANALOG_CONVERSION_CHANNELS_ANGLE_CONTROL)/sizeof(uint8_t),
//	.Config = 0,
//	.p_OnCompleteChannels = (void (*(*))(void *))FOC_ANALOG_CONVERSION_CHANNEL_FUNCTIONS_ANGLE_CONTROL,
//	.OnCompleteConversion = (void (*)(void *)) &Motor_FOC_ActivateCurrentFeedbackWrapper,
//};
//
//const uint8_t FOC_ANALOG_CONVERSION_CHANNELS_2[] =
//{
//	MOTOR_ANALOG_CHANNEL_VA,
//	MOTOR_ANALOG_CHANNEL_VB,
//	MOTOR_ANALOG_CHANNEL_VC,
//
//	MOTOR_ANALOG_CHANNEL_VBUS,
////	MOTOR_ANALOG_CHANNEL_VACC,
////	MOTOR_ANALOG_CHANNEL_VSENSE,
//
//	MOTOR_ANALOG_CHANNEL_HEAT_MOTOR,
//	MOTOR_ANALOG_CHANNEL_HEAT_PCB,
//	MOTOR_ANALOG_CHANNEL_HEAT_MOSFETS,
//	MOTOR_ANALOG_CHANNEL_THROTTLE,
//	MOTOR_ANALOG_CHANNEL_BRAKE,
//
//};
//
//Analog_Conversion_T FOC_ANALOG_CONVERSION_2 =
//{
//	{.p_VirtualChannels 	= FOC_ANALOG_CONVERSION_CHANNELS_2},
//	.ChannelCount = sizeof(FOC_ANALOG_CONVERSION_CHANNELS_2)/sizeof(uint8_t),
//	.Config = 0,
//	.p_OnCompleteChannels = 0,
//	.OnCompleteConversion = 0,
//};
