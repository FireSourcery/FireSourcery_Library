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

extern const Analog_VirtualConversionChannel_T MOTOR_ANALOG_VIRTUAL_SIN;
extern const Analog_VirtualConversionChannel_T MOTOR_ANALOG_VIRTUAL_COS;
//extern const Analog_VirtualConversionChannel_T MOTOR_ANALOG_VIRTUAL_BEMF_REMAINDER;
//extern const Analog_VirtualConversionChannel_T MOTOR_ANALOG_VIRTUAL_FOC_IABC;
//extern const Analog_VirtualConversionChannel_T MOTOR_ANALOG_VIRTUAL_FOC_REMAINDER;
//extern const Analog_VirtualConversionChannel_T MOTOR_ANALOG_VIRTUAL_IDLE;

//todo seperate vitrual
#define MOTOR_ANALOG_CONVERSIONS_CONFIG(VaPin, p_VaAnalogHost, VbPin, p_VbAnalogHost, VcPin, p_VcAnalogHost, IaPin, p_IaAnalogHost,IbPin, p_IbAnalogHost,IcPin, p_IcAnalogHost,HeatPin, p_HeatAnalogHost,SinPin, p_SinAnalogHost,CosPin, p_CosAnalogHost, p_Motor) \
{																																						\
	.CONVERSION_VA =	CONFIG_ANALOG_N_CONVERSION(&MOTOR_ANALOG_VIRTUAL_VA, 	VaPin, 		&((p_Motor)->AnalogResults.Channels[0U]), p_Motor, p_VaAnalogHost), 		\
	.CONVERSION_VB =	CONFIG_ANALOG_N_CONVERSION(&MOTOR_ANALOG_VIRTUAL_VB, 	VbPin, 		&((p_Motor)->AnalogResults.Channels[0U]), p_Motor, p_VbAnalogHost), 		\
	.CONVERSION_VC =	CONFIG_ANALOG_N_CONVERSION(&MOTOR_ANALOG_VIRTUAL_VC, 	VcPin, 		&((p_Motor)->AnalogResults.Channels[0U]), p_Motor, p_VcAnalogHost), 		\
	.CONVERSION_IA =	CONFIG_ANALOG_N_CONVERSION(&MOTOR_ANALOG_VIRTUAL_IA, 	IaPin, 		&((p_Motor)->AnalogResults.Channels[0U]), p_Motor, p_IaAnalogHost), 		\
	.CONVERSION_IB =	CONFIG_ANALOG_N_CONVERSION(&MOTOR_ANALOG_VIRTUAL_IB, 	IbPin, 		&((p_Motor)->AnalogResults.Channels[0U]), p_Motor, p_IbAnalogHost), 		\
	.CONVERSION_IC =	CONFIG_ANALOG_N_CONVERSION(&MOTOR_ANALOG_VIRTUAL_IC, 	IcPin, 		&((p_Motor)->AnalogResults.Channels[0U]), p_Motor, p_IcAnalogHost), 		\
	.CONVERSION_HEAT =	CONFIG_ANALOG_N_CONVERSION(&MOTOR_ANALOG_VIRTUAL_HEAT, 	HeatPin, 	&((p_Motor)->AnalogResults.Channels[0U]), p_Motor, p_HeatAnalogHost), 		\
	.CONVERSION_SIN =	CONFIG_ANALOG_N_CONVERSION(&MOTOR_ANALOG_VIRTUAL_SIN, 	SinPin, 	&((p_Motor)->AnalogResults.Channels[0U]), p_Motor, p_SinAnalogHost), 		\
	.CONVERSION_COS =	CONFIG_ANALOG_N_CONVERSION(&MOTOR_ANALOG_VIRTUAL_COS, 	CosPin, 	&((p_Motor)->AnalogResults.Channels[0U]), p_Motor, p_CosAnalogHost), 		\
}


//			.CONVERSION_VPOS =
//			{
//				.CONVERSION =
//				{
//					.P_VIRTUAL 				= &MOTOR_ANALOG_VIRTUAL_VPOS,
////					.OPTIONS = {0U},
//					.PIN 					= BOARD_VPOS_ADC_PIN,
//					.P_RESULTS_BUFFER 		= &Motors[0U].AnalogResults.Channels[0U],
//					.P_CALLBACK_CONTEXT 	= &Motors[0U],
//				},
//				.P_ANALOG = &Analogs[BOARD_VPOS_ANALOG_HOST],
//			},
//
//			.CONVERSION_VA =
//			{
//				.CONVERSION =
//				{
//					.P_VIRTUAL 				= &MOTOR_ANALOG_VIRTUAL_VA,
////					.OPTIONS = {0U},
//					.PIN 					= BOARD_MOTOR0_VA_ADC_PIN,
//					.P_RESULTS_BUFFER 		= &Motors[0U].AnalogResults.Channels[0U],
//					.P_CALLBACK_CONTEXT 	= &Motors[0U],
//				},
//				.P_ANALOG = &Analogs[BOARD_MOTOR0_VA_ANALOG_HOST],
//			},
//
//			.CONVERSION_VB =
//			{
//				.CONVERSION =
//				{
//					.P_VIRTUAL 				= &MOTOR_ANALOG_VIRTUAL_VB,
////					.OPTIONS = {0U},
//					.PIN 					= BOARD_MOTOR0_VB_ADC_PIN,
//					.P_RESULTS_BUFFER 		= &Motors[0U].AnalogResults.Channels[0U],
//					.P_CALLBACK_CONTEXT 	= &Motors[0U],
//				},
//				.P_ANALOG = &Analogs[BOARD_MOTOR0_VB_ANALOG_HOST],
//			},
//
//			.CONVERSION_VC =
//			{
//				.CONVERSION =
//				{
//					.P_VIRTUAL 				= &MOTOR_ANALOG_VIRTUAL_VC,
////					.OPTIONS = {0U},
//					.PIN 					= BOARD_MOTOR0_VC_ADC_PIN,
//					.P_RESULTS_BUFFER 		= &Motors[0U].AnalogResults.Channels[0U],
//					.P_CALLBACK_CONTEXT 	= &Motors[0U],
//				},
//				.P_ANALOG = &Analogs[BOARD_MOTOR0_VC_ANALOG_HOST],
//			},
//
//			.CONVERSION_IA =
//			{
//				.CONVERSION =
//				{
//					.P_VIRTUAL 				= &MOTOR_ANALOG_VIRTUAL_IA,
////					.OPTIONS = {0U},
//					.PIN 					= BOARD_MOTOR0_IA_ADC_PIN,
//					.P_RESULTS_BUFFER 		= &Motors[0U].AnalogResults.Channels[0U],
//					.P_CALLBACK_CONTEXT 	= &Motors[0U],
//				},
//				.P_ANALOG = &Analogs[BOARD_MOTOR0_IA_ANALOG_HOST],
//			},
//
//			.CONVERSION_IB =
//			{
//				.CONVERSION =
//				{
//					.P_VIRTUAL 				= &MOTOR_ANALOG_VIRTUAL_IB,
////					.OPTIONS = {0U},
//					.PIN 					= BOARD_MOTOR0_IB_ADC_PIN,
//					.P_RESULTS_BUFFER 		= &Motors[0U].AnalogResults.Channels[0U],
//					.P_CALLBACK_CONTEXT 	= &Motors[0U],
//				},
//				.P_ANALOG = &Analogs[BOARD_MOTOR0_IB_ANALOG_HOST],
//			},
//
//			.CONVERSION_IC =
//			{
//				.CONVERSION =
//				{
//					.P_VIRTUAL 				= &MOTOR_ANALOG_VIRTUAL_IC,
////					.OPTIONS = {0U},
//					.PIN 					= BOARD_MOTOR0_IC_ADC_PIN,
//					.P_RESULTS_BUFFER 		= &Motors[0U].AnalogResults.Channels[0U],
//					.P_CALLBACK_CONTEXT 	= &Motors[0U],
//				},
//				.P_ANALOG = &Analogs[BOARD_MOTOR0_IC_ANALOG_HOST],
//			},
//
//			.CONVERSION_HEAT =
//			{
//				.CONVERSION =
//				{
//					.P_VIRTUAL 				= &MOTOR_ANALOG_VIRTUAL_HEAT,
////					.OPTIONS = {0U},
//					.PIN 					= BOARD_MOTOR0_HEAT_ADC_PIN,
//					.P_RESULTS_BUFFER 		= &Motors[0U].AnalogResults.Channels[0U],
//					.P_CALLBACK_CONTEXT 	= &Motors[0U],
//				},
//				.P_ANALOG = &Analogs[BOARD_MOTOR0_HEAT_ANALOG_HOST],
//			},
//
//			.CONVERSION_SIN =
//			{
//				.CONVERSION =
//				{
//					.P_VIRTUAL 				= &MOTOR_ANALOG_VIRTUAL_SIN,
//					.PIN 					= BOARD_MOTOR0_SIN_ADC_PIN,
//					.P_RESULTS_BUFFER 		= &Motors[0U].AnalogResults.Channels[0U],
//					.P_CALLBACK_CONTEXT 	= &Motors[0U],
//				},
//				.P_ANALOG = &Analogs[BOARD_MOTOR0_HEAT_ANALOG_HOST],
//			},
//
//			.CONVERSION_COS =
//			{
//				.CONVERSION =
//				{
//					.P_VIRTUAL 				= &MOTOR_ANALOG_VIRTUAL_COS,
//					.PIN 					= BOARD_MOTOR0_COS_ADC_PIN,
//					.P_RESULTS_BUFFER 		= &Motors[0U].AnalogResults.Channels[0U],
//					.P_CALLBACK_CONTEXT 	= &Motors[0U],
//				},
//				.P_ANALOG = &Analogs[BOARD_MOTOR0_HEAT_ANALOG_HOST],
//			},
			//			.CONVERSION_OPTION_PWM_ON =
			//			{
			//				.CONVERSION =
			//				{
			//					.P_VIRTUAL 				= 0U,
			//					.PIN 					= 0U,
			//					.P_RESULTS_BUFFER 		= 0U,
			//					.P_CALLBACK_CONTEXT 	= &Motors[0U],
			//
			//					.OPTIONS =
			//					{
			//			//						.FLAGS 			= {.IsValid = 1U, .HwTriggerConversion = 1U, },
			//						.IS_VALID 		= true,
			//						.HW_TRIGGER 	= true,
			//			//						.ON_OPTIONS 	= Motor_CapturePwmOnFlag,
			//					},
			//				},
			//				.P_ANALOG = &Analogs[1U], //BOARD_ANALOG_HOST_MOTOR0_
			//			},

			//			.CONVERSION_OPTION_RESTORE =
			//			{
			//				.CONVERSION =
			//				{
			//					.P_VIRTUAL 				= 0U,
			//					.PIN 					= 0U,
			//					.P_RESULTS_BUFFER 		= 0U,
			//					.P_CALLBACK_CONTEXT 	= &Motors[0U],
			//
			//					.OPTIONS =
			//					{
			////						.FLAGS 			= {.IsValid = 1U, .HwTriggerConversion = 0U, },
			//						.IS_VALID 		= true,
			//						.HW_TRIGGER 	= false,
			//						.ON_OPTIONS 	= 0U,
			//					},
			//				},
			//				.P_ANALOG = &Analogs[1U], //BOARD_ANALOG_HOST_MOTOR0_MOTOR1
			//			},

extern void Motor_CapturePwmOnFlag(Motor_T * p_motor);

#endif
