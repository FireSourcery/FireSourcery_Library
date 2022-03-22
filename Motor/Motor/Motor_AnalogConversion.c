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
#include "Motor/Motor/Motor_AnalogConversion.h"
#include "MotorAnalog.h"
#include "Motor.h"
#include "Motor_SixStep.h"
#include "Motor_FOC.h"
#include "Peripheral/Analog/AnalogN/AnalogN.h"

#include <stdint.h>
#include <stdbool.h>


//void Motor_CapturePwmOnFlag(Motor_T * p_motor)
//{
//	p_motor->IsPwmOn = true;
////	p_motor->PwmOnTime = Debug_CaptureElapsed(6);
//
//}

/*
 * Sets options only
 */
//const Analog_Conversion_T MOTOR_ANALOG_CONVERSION_OPTION_PWM_ON =
//{
//	.P_VIRTUAL = 0U,
//	.OPTIONS = {.IsValid = 1U, .HwTriggerConversion = 1U, },
//	.ON_OPTIONS = SetPwmOnFlag,
//};
//
//const Analog_Conversion_T MOTOR_ANALOG_CONVERSION_OPTION_RESTORE =
//{
//	.P_VIRTUAL = 0U,
//	.OPTIONS = {.IsValid = 1U, .HwTriggerConversion = 0U, },
//};


/******************************************************************************/
/*!
    @brief  Conversion
*/
/******************************************************************************/
static void ProcVPos(Motor_T * p_motor)
{
	//check over volt
}

const Analog_VirtualConversionChannel_T MOTOR_ANALOG_VIRTUAL_VPOS =
{
	.CHANNEL = MOTOR_ANALOG_CHANNEL_VPOS,
	.ON_COMPLETE = (Analog_Callback_T)ProcVPos,
};

/******************************************************************************/
/*!
    @brief  Conversion
*/
/******************************************************************************/
static void CaptureVa(Motor_T * p_motor)
{
	if (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP)
	{
		Motor_SixStep_CaptureBemfA(p_motor);
	}
	else
	{

	}
}

const Analog_VirtualConversionChannel_T MOTOR_ANALOG_VIRTUAL_VA =
{
	.CHANNEL = MOTOR_ANALOG_CHANNEL_VA,
	.ON_COMPLETE = (Analog_Callback_T)&CaptureVa
};

/******************************************************************************/
/*!
    @brief  Conversion
*/
/******************************************************************************/
static void CaptureVb(Motor_T * p_motor)
{
	if (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP)
	{
		Motor_SixStep_CaptureBemfB(p_motor);
	}
	else
	{

	}
}

const Analog_VirtualConversionChannel_T MOTOR_ANALOG_VIRTUAL_VB =
{
	.CHANNEL = MOTOR_ANALOG_CHANNEL_VB,
	.ON_COMPLETE = (Analog_Callback_T)&CaptureVb
};

/******************************************************************************/
/*!
    @brief  Conversion
*/
/******************************************************************************/
static void CaptureVc(Motor_T * p_motor)
{
	if (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP)
	{
		Motor_SixStep_CaptureBemfC(p_motor);
	}
	else
	{

	}
}

const Analog_VirtualConversionChannel_T MOTOR_ANALOG_VIRTUAL_VC =
{
	.CHANNEL = MOTOR_ANALOG_CHANNEL_VC,
	.ON_COMPLETE = (Analog_Callback_T)&CaptureVc
};

/******************************************************************************/
/*!
    @brief  Conversion
*/
/******************************************************************************/
static void CaptureIa(Motor_T * p_motor)
{
	if (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP)
	{
		Motor_SixStep_CaptureIBusA(p_motor);
	}
	else
	{
		Motor_FOC_CaptureIa(p_motor);
	}
}

const Analog_VirtualConversionChannel_T MOTOR_ANALOG_VIRTUAL_IA =
{
	.CHANNEL = MOTOR_ANALOG_CHANNEL_IA,
	.ON_COMPLETE = (Analog_Callback_T)CaptureIa,
};

/******************************************************************************/
/*!
    @brief  Conversion
*/
/******************************************************************************/
static void CaptureIb(Motor_T * p_motor)
{
	if (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP)
	{
		Motor_SixStep_CaptureIBusB(p_motor);
	}
	else
	{
		Motor_FOC_CaptureIb(p_motor);
	}
}

const Analog_VirtualConversionChannel_T MOTOR_ANALOG_VIRTUAL_IB =
{
	.CHANNEL = MOTOR_ANALOG_CHANNEL_IB,
	.ON_COMPLETE = (Analog_Callback_T)CaptureIb,
};


/******************************************************************************/
/*!
    @brief  Conversion
*/
/******************************************************************************/
static void CaptureIc(Motor_T * p_motor)
{
	if (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP)
	{
		Motor_SixStep_CaptureIBusC(p_motor);
	}
	else
	{
		Motor_FOC_CaptureIc(p_motor);
	}
}

const Analog_VirtualConversionChannel_T MOTOR_ANALOG_VIRTUAL_IC =
{
	.CHANNEL = MOTOR_ANALOG_CHANNEL_IC,
	.ON_COMPLETE = (Analog_Callback_T)CaptureIc,
};

/******************************************************************************/
/*!
    @brief  Conversion
*/
/******************************************************************************/
const Analog_VirtualConversionChannel_T MOTOR_ANALOG_VIRTUAL_HEAT =
{
	.CHANNEL = MOTOR_ANALOG_CHANNEL_HEAT,
	.ON_COMPLETE = 0U,
};




///******************************************************************************/
///*!
//    @brief  Conversion
//*/
///******************************************************************************/

//static const Analog_VirtualConversionChannel_T CHANNELS_BEMF_A[] =
//{
//	[0U] = {MOTOR_ANALOG_CHANNEL_VPOS,	Motor_SixStep_CaptureBemfA_DEBUG},
//	[1U] = {MOTOR_ANALOG_CHANNEL_IB, 	(Analog_OnComplete_T)&Motor_CaptureIBusB},		//capture as bus current
//	[2U] = {MOTOR_ANALOG_CHANNEL_VA, 	(Analog_OnComplete_T)&Motor_SixStep_CaptureBemfA},	//capture as emf for zcd
////	[3] = {MOTOR_ANALOG_CHANNEL_IC,
//};
//
//const Analog_VirtualConversion_T MOTOR_ANALOG_VIRTUAL_BEMF_A =
//{
//	.P_CHANNELS 	= CHANNELS_BEMF_A,
//	.CHANNEL_COUNT 	= sizeof(CHANNELS_BEMF_A)/sizeof(Analog_VirtualConversionChannel_T),
//	.ON_COMPLETE 	= 0U,
//	.OPTIONS =		{.HwTriggerConversion = 1U },
//};
//
///******************************************************************************/
///*!
//    @brief  Conversion
//*/
///******************************************************************************/
//static const Analog_VirtualConversionChannel_T CHANNELS_BEMF_B[] =
//{
//	[0U] = {MOTOR_ANALOG_CHANNEL_VPOS, 	0U},
//	[1U] = {MOTOR_ANALOG_CHANNEL_IC, 	(Analog_OnComplete_T)&Motor_CaptureIBusC},
//	[2U] = {MOTOR_ANALOG_CHANNEL_VB, 	(Analog_OnComplete_T)&Motor_SixStep_CaptureBemfB},
////	MOTOR_ANALOG_CHANNEL_IA,
//};
//
//const Analog_VirtualConversion_T MOTOR_ANALOG_VIRTUAL_BEMF_B =
//{
//	.P_CHANNELS 	= CHANNELS_BEMF_B,
//	.CHANNEL_COUNT 	= sizeof(CHANNELS_BEMF_B)/sizeof(Analog_VirtualConversionChannel_T),
//	.ON_COMPLETE 	= 0U,
//	.OPTIONS =		{.HwTriggerConversion = 1U },
//};
//
///******************************************************************************/
///*!
//    @brief  Conversion
//*/
///******************************************************************************/
//static const Analog_VirtualConversionChannel_T CHANNELS_BEMF_C[] =
//{
//	[0U] = {MOTOR_ANALOG_CHANNEL_VPOS, 	0U},
//	[1U] = {MOTOR_ANALOG_CHANNEL_IA, 	(Analog_OnComplete_T)&Motor_CaptureIBusA},
//	[2U] = {MOTOR_ANALOG_CHANNEL_VC, 	(Analog_OnComplete_T)&Motor_SixStep_CaptureBemfC},
////	MOTOR_ANALOG_CHANNEL_IB,
//};
//
//const Analog_VirtualConversion_T MOTOR_ANALOG_VIRTUAL_BEMF_C =
//{
//	.P_CHANNELS 	= CHANNELS_BEMF_C,
//	.CHANNEL_COUNT 	= sizeof(CHANNELS_BEMF_C)/sizeof(Analog_VirtualConversionChannel_T),
//	.ON_COMPLETE 	= 0U,
//	.OPTIONS =		{.HwTriggerConversion = 1U },
//};







/******************************************************************************/
/*!
    @brief  Conversion
*/
/******************************************************************************/
//static const Analog_VirtualConversionChannel_T CHANNELS_BEMF_REMAINDER[] =
//{
//	[0U] = {MOTOR_ANALOG_CHANNEL_HEAT_MOTOR, 	0U},
//};
//
//const Analog_VirtualConversion_T MOTOR_ANALOG_VIRTUAL_BEMF_REMAINDER =
//{
//	.P_CHANNELS 	= CHANNELS_BEMF_REMAINDER,
//	.CHANNEL_COUNT 	= sizeof(CHANNELS_BEMF_REMAINDER)/sizeof(Analog_VirtualConversionChannel_T),
//};

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
//static const Analog_VirtualConversionChannel_T CHANNELS_IABC[] =
//{
//	[0U] = {MOTOR_ANALOG_CHANNEL_IA, (Analog_OnComplete_T)&Motor_FOC_CaptureIa},
//	[1U] = {MOTOR_ANALOG_CHANNEL_IB, (Analog_OnComplete_T)&Motor_FOC_CaptureIb},
//	[2U] = {MOTOR_ANALOG_CHANNEL_IC, (Analog_OnComplete_T)&Motor_FOC_CaptureIc},
//};
//
//const Analog_VirtualConversion_T MOTOR_ANALOG_VIRTUAL_FOC_IABC =
//{
//	.P_CHANNELS = CHANNELS_IABC,
//	.CHANNEL_COUNT = sizeof(CHANNELS_IABC) / sizeof(Analog_VirtualConversionChannel_T),
//	.ON_COMPLETE = (Analog_OnComplete_T)&Motor_FOC_ProcCurrentFeedback,
////	.OPTIONS =		{.HwTriggerConversion = 1U },
//};
//
//
///******************************************************************************/
///*!
//    @brief  Conversion
//*/
///******************************************************************************/
//static const Analog_VirtualConversionChannel_T CHANNELS_FOC_REMAINDER[] =
//{
//	[0U] = {MOTOR_ANALOG_CHANNEL_VA, 			0U},
//	[1U] = {MOTOR_ANALOG_CHANNEL_VB, 			0U},
//	[2U] = {MOTOR_ANALOG_CHANNEL_VC, 			0U},
//	[3U] = {MOTOR_ANALOG_CHANNEL_VPOS, 			0U},
//	[5U] = {MOTOR_ANALOG_CHANNEL_HEAT_MOTOR, 	0U},
//};
//
//const Analog_VirtualConversion_T MOTOR_ANALOG_VIRTUAL_FOC_REMAINDER =
//{
//	.P_CHANNELS 	= CHANNELS_FOC_REMAINDER,
//	.CHANNEL_COUNT 	= sizeof(CHANNELS_FOC_REMAINDER)/sizeof(Analog_VirtualConversionChannel_T),
//	.ON_COMPLETE 	= 0U,
//};
//
///******************************************************************************/
///*!
//    @brief  Conversion when motor is stopped
//*/
///******************************************************************************/
//static const Analog_VirtualConversionChannel_T CHANNELS_IDLE[] =
//{
//	{MOTOR_ANALOG_CHANNEL_VPOS, 			0U},
//	{MOTOR_ANALOG_CHANNEL_VA, 				(Analog_OnComplete_T)&Motor_SixStep_CaptureBemfA},
//	{MOTOR_ANALOG_CHANNEL_VB, 				(Analog_OnComplete_T)&Motor_SixStep_CaptureBemfB},
//	{MOTOR_ANALOG_CHANNEL_VC, 				(Analog_OnComplete_T)&Motor_SixStep_CaptureBemfC},
//	{MOTOR_ANALOG_CHANNEL_IA, 				0U},
//	{MOTOR_ANALOG_CHANNEL_IB, 				0U},
//	{MOTOR_ANALOG_CHANNEL_IC, 				0U},
//	{MOTOR_ANALOG_CHANNEL_HEAT_MOTOR, 		0U},
//};
//
//const Analog_VirtualConversion_T MOTOR_ANALOG_VIRTUAL_IDLE =
//{
//	.P_CHANNELS 	= CHANNELS_IDLE,
//	.CHANNEL_COUNT 	= sizeof(CHANNELS_IDLE)/sizeof(Analog_VirtualConversionChannel_T),
//	.ON_COMPLETE 	= 0U,
//};
