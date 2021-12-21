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
    @file 	MotorAnalog.c
    @author FireSoucery
    @brief  Motor Analog Conversion. Common Virtual component only.
    @version V0
*/
/******************************************************************************/
#include "MotorAnalog.h"
#include "Motor.h"
#include "Motor_SixStep.h"
#include "Motor_FOC.h"
#include "Peripheral/Analog/AnalogN/AnalogN.h"

#include <stdint.h>
#include <stdbool.h>


/******************************************************************************/
/*!
    @brief  Conversions
*/
/******************************************************************************/



/******************************************************************************/
/*!
    @brief  Conversion
*/
/******************************************************************************/
static const Analog_ConversionVirtualChannel_T CHANNELS_BEMF_A[] =
{
	[0U] = {MOTOR_ANALOG_CHANNEL_VPOS,	0U},
	[1U] = {MOTOR_ANALOG_CHANNEL_IB, 	(Analog_OnComplete_T)&Motor_CaptureIBusIb},			//capture as bus current
	[2U] = {MOTOR_ANALOG_CHANNEL_VA, 	(Analog_OnComplete_T)&Motor_SixStep_CaptureBemfA},	//capture as emf for zcd
//	[3] = {MOTOR_ANALOG_CHANNEL_IC,
//	[4] = {MOTOR_ANALOG_CHANNEL_HEAT_MOSFETS,
};

const Analog_ConversionVirtual_T MOTOR_ANALOG_VIRTUAL_BEMF_A =
{
	.P_CHANNELS 	= CHANNELS_BEMF_A,
	.CHANNEL_COUNT 	= sizeof(CHANNELS_BEMF_A)/sizeof(Analog_ConversionVirtualChannel_T),
	.ON_COMPLETE 	= 0U,
	.OPTIONS =		{.HwTriggerConversion = 1U },
};

//static const Analog_ConversionVirtualChannel_T CHANNELS_BEMF_A_REPEAT[] =
//{
//	[0U] = {MOTOR_ANALOG_CHANNEL_VA, 	(Analog_OnComplete_T)&Motor_SixStep_CaptureBemfA},
//};
//
//const Analog_ConversionVirtual_T MOTOR_ANALOG_VIRTUAL_BEMF_A_REPEAT =
//{
//	.P_CHANNELS 	= CHANNELS_BEMF_A_REPEAT,
//	.CHANNEL_COUNT 	= sizeof(CHANNELS_BEMF_A_REPEAT)/sizeof(Analog_ConversionVirtualChannel_T),
//	.ON_COMPLETE 	= 0U,
//	.OPTIONS =		{.HwTriggerConversion = 0U, .ContinuousConversion = 1U, .CaptureLocalPeak = 1U },
//};

/******************************************************************************/
/*!
    @brief  Conversion
*/
/******************************************************************************/
static const Analog_ConversionVirtualChannel_T CHANNELS_BEMF_B[] =
{
	[0U] = {MOTOR_ANALOG_CHANNEL_VPOS, 	0U},
	[1U] = {MOTOR_ANALOG_CHANNEL_IC, 	(Analog_OnComplete_T)&Motor_CaptureIBusIc},
	[2U] = {MOTOR_ANALOG_CHANNEL_VB, 	(Analog_OnComplete_T)&Motor_SixStep_CaptureBemfB},
//	MOTOR_ANALOG_CHANNEL_IA,
//	MOTOR_ANALOG_CHANNEL_HEAT_MOSFETS,
};

const Analog_ConversionVirtual_T MOTOR_ANALOG_VIRTUAL_BEMF_B =
{
	.P_CHANNELS 	= CHANNELS_BEMF_B,
	.CHANNEL_COUNT 	= sizeof(CHANNELS_BEMF_B)/sizeof(Analog_ConversionVirtualChannel_T),
	.ON_COMPLETE 	= 0U,//MotorAnalog_LoadBemfB,

};

//void MotorAnalog_LoadBemfB(Motor_T * p_motor)
//{
////	Analog_ActivateConversion(p_motor->CONFIG.P_ANALOG , &CONVERSION_BEMF_B);
//}

/******************************************************************************/
/*!
    @brief  Conversion
*/
/******************************************************************************/
static const Analog_ConversionVirtualChannel_T CHANNELS_BEMF_C[] =
{
	[0U] = {MOTOR_ANALOG_CHANNEL_VPOS, 	0U},
	[1U] = {MOTOR_ANALOG_CHANNEL_IA, 	(Analog_OnComplete_T)&Motor_CaptureIBusIa},
	[2U] = {MOTOR_ANALOG_CHANNEL_VC, 	(Analog_OnComplete_T)&Motor_SixStep_CaptureBemfC},
//	MOTOR_ANALOG_CHANNEL_IB,
//	MOTOR_ANALOG_CHANNEL_HEAT_MOSFETS,
};

const Analog_ConversionVirtual_T MOTOR_ANALOG_VIRTUAL_BEMF_C =
{
	.P_CHANNELS 	= CHANNELS_BEMF_C,
	.CHANNEL_COUNT 	= sizeof(CHANNELS_BEMF_C)/sizeof(Analog_ConversionVirtualChannel_T),
	.ON_COMPLETE 	= 0U,//MotorAnalog_LoadBemfC,
};

//void MotorAnalog_LoadBemfC(Motor_T * p_motor)
//{
////	Analog_ActivateConversion(p_motor->CONFIG.P_ANALOG , &CONVERSION_BEMF_C);
//}


/******************************************************************************/
/*!
    @brief  Conversion
*/
/******************************************************************************/
static const Analog_ConversionVirtualChannel_T CHANNELS_BEMF_REMAINDER[] =
{
//	[0U] = {MOTOR_ANALOG_CHANNEL_HEAT_MOSFETS, 	0U},
	[0U] = {MOTOR_ANALOG_CHANNEL_HEAT_MOTOR, 	0U},
};

const Analog_ConversionVirtual_T MOTOR_ANALOG_VIRTUAL_BEMF_REMAINDER =
{
	.P_CHANNELS 	= CHANNELS_BEMF_REMAINDER,
	.CHANNEL_COUNT 	= sizeof(CHANNELS_BEMF_REMAINDER)/sizeof(Analog_ConversionVirtualChannel_T),
};

//void MotorAnalog_EnqueueBemfRemainder(Motor_T * p_motor)
//{
//
//}
/******************************************************************************/
/*!
    @brief  Conversion
*/
/******************************************************************************/
/*!
	@brief FOC mode ADC channels to be sampled

	Sample all channels sequentially on PWM trigger
	Measure Ia, Ib, Ic first, so there is longer time for FOC calculations
	remaining channels will continue processing as FOC calculations are performed
 */
static const Analog_ConversionVirtualChannel_T CHANNELS_IABC[] =
{
	[0U] = {MOTOR_ANALOG_CHANNEL_IA, (Analog_OnComplete_T)&Motor_FOC_CaptureIa},
	[1U] = {MOTOR_ANALOG_CHANNEL_IB, (Analog_OnComplete_T)&Motor_FOC_CaptureIb},
	[2U] = {MOTOR_ANALOG_CHANNEL_IC, (Analog_OnComplete_T)&Motor_FOC_CaptureIc},
};

const Analog_ConversionVirtual_T MOTOR_ANALOG_VIRTUAL_IABC =
{
	.P_CHANNELS = CHANNELS_IABC,
	.CHANNEL_COUNT = sizeof(CHANNELS_IABC) / sizeof(Analog_ConversionVirtualChannel_T),
	.ON_COMPLETE = (Analog_OnComplete_T)&Motor_FOC_ProcCurrentFeedback,
};

//void MotorAnalog_ActivateIabc(Motor_T * p_motor)
//{
//	AnalogN_LoadConversion(p_motor->CONFIG.P_ANALOG , &CONVERSION_IABC, p_motor->CONFIG.P_ANALOG_MAP);
//}

const Analog_ConversionVirtual_T MOTOR_ANALOG_VIRTUAL_IABC_OBSERVE =
{
	.P_CHANNELS = CHANNELS_IABC,
	.CHANNEL_COUNT = sizeof(CHANNELS_IABC) / sizeof(Analog_ConversionVirtualChannel_T),
	.ON_COMPLETE = 0U,
};


/******************************************************************************/
/*!
    @brief  Conversion
*/
/******************************************************************************/
static const Analog_ConversionVirtualChannel_T CHANNELS_FOC_REMAINDER[] =
{
	[0U] = {MOTOR_ANALOG_CHANNEL_VA, 			0U},
	[1U] = {MOTOR_ANALOG_CHANNEL_VB, 			0U},
	[2U] = {MOTOR_ANALOG_CHANNEL_VC, 			0U},
	[3U] = {MOTOR_ANALOG_CHANNEL_VPOS, 			0U},
//	[4U] = {MOTOR_ANALOG_CHANNEL_HEAT_MOSFETS, 	0U},
//	[5U] = {MOTOR_ANALOG_CHANNEL_HEAT_MOTOR, 	0U},
};

const Analog_ConversionVirtual_T MOTOR_ANALOG_VIRTUAL_FOC_REMAINDER =
{
	.P_CHANNELS 	= CHANNELS_FOC_REMAINDER,
	.CHANNEL_COUNT 	= sizeof(CHANNELS_FOC_REMAINDER)/sizeof(Analog_ConversionVirtualChannel_T),
	.ON_COMPLETE 	= 0U,
};

//void MotorAnalog_EnqueueFocRemainder(Motor_T * p_motor)
//{
//	Analog_EnqueueConversion(p_motor->CONFIG.P_ANALOG , &CONVERSION_FOC_REMAINDER);
//}
//
/******************************************************************************/
/*!
    @brief  Conversion when motor is stoped
*/
/******************************************************************************/
static const Analog_ConversionVirtualChannel_T CHANNELS_IDLE[] =
{
	{MOTOR_ANALOG_CHANNEL_VPOS, 			0U},
	{MOTOR_ANALOG_CHANNEL_VA, 				(Analog_OnComplete_T)&Motor_SixStep_CaptureBemfA},
	{MOTOR_ANALOG_CHANNEL_VB, 				(Analog_OnComplete_T)&Motor_SixStep_CaptureBemfB},
	{MOTOR_ANALOG_CHANNEL_VC, 				(Analog_OnComplete_T)&Motor_SixStep_CaptureBemfC},
	{MOTOR_ANALOG_CHANNEL_IA, 				(Analog_OnComplete_T)&Motor_FOC_CaptureIa}, //temp //not capture as ibus
	{MOTOR_ANALOG_CHANNEL_IB, 				(Analog_OnComplete_T)&Motor_FOC_CaptureIb},
	{MOTOR_ANALOG_CHANNEL_IC, 				(Analog_OnComplete_T)&Motor_FOC_CaptureIc},
//	{MOTOR_ANALOG_CHANNEL_HEAT_MOSFETS, 	0U},
//	{MOTOR_ANALOG_CHANNEL_HEAT_PCB, 		0U},
	{MOTOR_ANALOG_CHANNEL_HEAT_MOTOR, 		0U},
};

const Analog_ConversionVirtual_T MOTOR_ANALOG_VIRTUAL_IDLE =
{
	.P_CHANNELS 	= CHANNELS_IDLE,
	.CHANNEL_COUNT 	= sizeof(CHANNELS_IDLE)/sizeof(Analog_ConversionVirtualChannel_T),
	.ON_COMPLETE 	= 0U,
};


/******************************************************************************/
/*!
    Common Channels For all Motors
*/
/******************************************************************************/

/******************************************************************************/
/*!
    @brief  Conversion, defer to ms sample
*/
/******************************************************************************/
static const Analog_ConversionVirtualChannel_T CHANNELS_MONITOR[] =
{
	[0U] = {MOTOR_ANALOG_CHANNEL_HEAT_MOSFETS, 	0U},
	[1U] = {MOTOR_ANALOG_CHANNEL_HEAT_PCB, 		0U},
	[2U] = {MOTOR_ANALOG_CHANNEL_VACC,			0U},
	[3U] = {MOTOR_ANALOG_CHANNEL_VSENSE,		0U},
	//	[2U] = {MOTOR_ANALOG_CHANNEL_HEAT_MOTOR, 	0U},
};

const Analog_ConversionVirtual_T MOTOR_ANALOG_VIRTUAL_MONITOR =
{
	.P_CHANNELS 	= CHANNELS_MONITOR,
	.CHANNEL_COUNT 	= sizeof(CHANNELS_MONITOR)/sizeof(Analog_ConversionVirtualChannel_T),
	.ON_COMPLETE 	= 0U,
};

//void MotorAnalog_EnqueueMonitor(Motor_T * p_motor)
//{
////	Analog_Conversion_T * saveConversion = MotorAnalog.p_ActiveConversion;
////
////	Analog_ActivateConversion(p_motor->CONFIG.P_ANALOG , &CONVERSION_MONITOR); 	//todo return to load bemf
////	Analog_EnqueueConversion(p_motor->CONFIG.P_ANALOG , saveConversion); 		//todo return to load bemf
//}

/******************************************************************************/
/*!
    @brief  Conversion
*/
/******************************************************************************/
static const Analog_ConversionVirtualChannel_T CHANNELS_USER[] =
{
	{MOTOR_ANALOG_CHANNEL_BRAKE,		0U},
	{MOTOR_ANALOG_CHANNEL_THROTTLE,		0U},
//	{MOTOR_ANALOG_CHANNEL_VACC,			0U},
//	{MOTOR_ANALOG_CHANNEL_VSENSE,		0U},
};

const Analog_ConversionVirtual_T MOTOR_ANALOG_VIRTUAL_USER =
{
	.P_CHANNELS 	= CHANNELS_USER,
	.CHANNEL_COUNT 	= sizeof(CHANNELS_USER)/sizeof(Analog_ConversionVirtualChannel_T),
	.ON_COMPLETE 	= 0U,
};

//void MotorAnalog_EnqueueUser(Motor_T * p_motor)
//{
//	Analog_EnqueueConversion(p_motor->CONFIG.P_ANALOG , &CONVERSION_USER);
////	Analog_DequeueConversion(p_motor->CONFIG.P_ANALOG );	//starts chain if no conversion is currently active
//}

/******************************************************************************/
/*!
    @brief  Conversion when motor is stoped
*/
/******************************************************************************/
//static const Analog_ConversionVirtualChannel_T CHANNELS_IDLE[] =
//{
////	{MOTOR_ANALOG_CHANNEL_VPOS, 			0U},
//	{MOTOR_ANALOG_CHANNEL_HEAT_MOSFETS, 	0U},
//	{MOTOR_ANALOG_CHANNEL_HEAT_PCB, 		0U},
////	{MOTOR_ANALOG_CHANNEL_HEAT_MOTOR, 		0U},
//	{MOTOR_ANALOG_CHANNEL_VACC,			0U},
//	{MOTOR_ANALOG_CHANNEL_VSENSE,		0U},
//};
//
//const Analog_ConversionVirtual_T MOTOR_ANALOG_VIRTUAL_IDLE =
//{
//	.P_CHANNELS 	= CHANNELS_IDLE,
//	.CHANNEL_COUNT 	= sizeof(CHANNELS_IDLE)/sizeof(Analog_ConversionVirtualChannel_T),
//	.ON_COMPLETE 	= 0U,
//};

//void MotorAnalog_EnqueueIdle(Motor_T * p_motor)
//{
//	Analog_EnqueueConversion(p_motor->CONFIG.P_ANALOG, &CONVERSION_IDLE);
////	Analog_DequeueConversion(p_motor->CONFIG.P_ANALOG );
//}
