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
#include "Motor.h"

extern void Motor_AnalogCallback_CaptureVa(Motor_T * p_motor);
extern void Motor_AnalogCallback_CaptureVb(Motor_T * p_motor);
extern void Motor_AnalogCallback_CaptureVc(Motor_T * p_motor);
extern void Motor_AnalogCallback_CaptureIa(Motor_T * p_motor);
extern void Motor_AnalogCallback_CaptureIb(Motor_T * p_motor);
extern void Motor_AnalogCallback_CaptureIc(Motor_T * p_motor);

#define MOTOR_ANALOG_CONVERSIONS_CONFIG(VaPin, p_VaHost, VbPin, p_VbHost, VcPin, p_VcHost, IaPin, p_IaHost,IbPin, p_IbHost,IcPin, p_IcHost, HeatPin, p_HeatHost, SinPin, p_SinHost,CosPin, p_CosHost, p_Motor) \
{																																														\
	.CONVERSION_VA =	CONFIG_ANALOG_N_CONVERSION(MOTOR_ANALOG_CHANNEL_VA, (Analog_Callback_T)Motor_AnalogCallback_CaptureVa, p_Motor, &((p_Motor)->AnalogResults.Channels[0U]), VaPin, p_VaHost), 		\
	.CONVERSION_VB =	CONFIG_ANALOG_N_CONVERSION(MOTOR_ANALOG_CHANNEL_VB, (Analog_Callback_T)Motor_AnalogCallback_CaptureVb, p_Motor, &((p_Motor)->AnalogResults.Channels[0U]), VbPin, p_VbHost), 		\
	.CONVERSION_VC =	CONFIG_ANALOG_N_CONVERSION(MOTOR_ANALOG_CHANNEL_VC, (Analog_Callback_T)Motor_AnalogCallback_CaptureVc, p_Motor, &((p_Motor)->AnalogResults.Channels[0U]), VcPin, p_VcHost), 		\
	.CONVERSION_IA =	CONFIG_ANALOG_N_CONVERSION(MOTOR_ANALOG_CHANNEL_IA, (Analog_Callback_T)Motor_AnalogCallback_CaptureIa, p_Motor, &((p_Motor)->AnalogResults.Channels[0U]), IaPin, p_IaHost), 		\
	.CONVERSION_IB =	CONFIG_ANALOG_N_CONVERSION(MOTOR_ANALOG_CHANNEL_IB, (Analog_Callback_T)Motor_AnalogCallback_CaptureIb, p_Motor, &((p_Motor)->AnalogResults.Channels[0U]), IbPin, p_IbHost), 		\
	.CONVERSION_IC =	CONFIG_ANALOG_N_CONVERSION(MOTOR_ANALOG_CHANNEL_IC, (Analog_Callback_T)Motor_AnalogCallback_CaptureIc, p_Motor, &((p_Motor)->AnalogResults.Channels[0U]), IcPin, p_IcHost), 		\
	.CONVERSION_HEAT =	CONFIG_ANALOG_N_CONVERSION(MOTOR_ANALOG_CHANNEL_HEAT, 	0U, p_Motor, &((p_Motor)->AnalogResults.Channels[0U]), HeatPin,		p_HeatHost), 						\
	.CONVERSION_SIN =	CONFIG_ANALOG_N_CONVERSION(MOTOR_ANALOG_CHANNEL_SIN, 	0U, p_Motor, &((p_Motor)->AnalogResults.Channels[0U]), SinPin, 		p_SinHost), 						\
	.CONVERSION_COS =	CONFIG_ANALOG_N_CONVERSION(MOTOR_ANALOG_CHANNEL_COS, 	0U, p_Motor, &((p_Motor)->AnalogResults.Channels[0U]), CosPin, 		p_CosHost), 						\
}

#endif
