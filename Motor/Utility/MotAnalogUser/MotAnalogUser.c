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
    @file 	Motor.h
    @author FireSoucery
    @brief
    @version V0
*/
/**************************************************************************/
#include "MotAnalogUser.h"

#include "Peripheral/Pin/Debounce.h"
#include "Peripheral/Pin/Pin.h"

#include "Math/Linear/Linear_ADC.h"
#include "Math/Linear/Linear.h"

#include <stdint.h>
#include <stdbool.h>


void MotAnalogUser_Init(MotAnalogUser_T * p_motorUser, const MotAnalogUser_Config_T * p_config)
{
	p_motorUser->p_Config = p_config;

	Debounce_Init(&p_motorUser->PinBrake, 		&p_config->HAL_PIN_BRAKE, 		&p_config->P_DEBOUNCE_TIMER, 5U);	//5millis
	Debounce_Init(&p_motorUser->PinThrottle,	&p_config->HAL_PIN_THROTTLE, 	&p_config->P_DEBOUNCE_TIMER, 5U);	//5millis
	Debounce_Init(&p_motorUser->PinForward, 	&p_config->HAL_PIN_FORWARD, 	&p_config->P_DEBOUNCE_TIMER, 5U);	//5millis
	Debounce_Init(&p_motorUser->PinReverse, 	&p_config->HAL_PIN_REVERSE, 	&p_config->P_DEBOUNCE_TIMER, 5U);	//5millis

	//uncalibrated default
	Linear_ADC_Init(&p_motorUser->UnitThrottle, 	0U, 4095U, 50U);
	Linear_ADC_Init(&p_motorUser->UnitBrake,		0U, 4095U, 50U);
}


//void MotAnalogUser_CaptureInput_IO(MotAnalogUser_T * p_motorUser)
//{
//	Debounce_CaptureState_IO(&p_motorUser->PinBrake);
//	Debounce_CaptureState_IO(&p_motorUser->PinThrottle);
//	Debounce_CaptureState_IO(&p_motorUser->PinForward);
//	Debounce_CaptureState_IO(&p_motorUser->PinReverse);
//}
//
//bool MotAnalogUser_GetInput_IO(MotAnalogUser_T * p_motorUser)
//{
//	return Debounce_GetState(&p_motorUser->PinBrake);
//}

//void MotAnalogUser_SetInputThrottle(MotAnalogUser_T * p_motorUser, uint16_t throttle)
//{
//	p_motorUser->InputValueThrottlePrev 	= p_motorUser->InputValueThrottle;
//	p_motorUser->InputValueThrottle 		= throttle;
//}
//void MotorInterface_Input_SetBrake(MotorInterface_Input_T * p_input, uint16_t throttle)
//{
//	p_input->InputValueThrottlePrev 	= p_input->InputValueThrottle;
//	p_input->InputValueThrottle 		= throttle;
//}
//

static inline bool GetThrottleRelease(MotAnalogUser_T * p_motorUser)
{
	bool isReleased;

	if (p_motorUser->InputValueThrottle < p_motorUser->InputValueThrottlePrev)
	{
		if ((p_motorUser->InputValueThrottlePrev - p_motorUser->InputValueThrottle) > (65535U/10U))
		{
			isReleased == true;
		}
	}
	else
	{
		isReleased == false;
	}

	return isReleased;
}

void MotAnalogUser_CaptureInput(MotAnalogUser_T * p_motorUser)
{
	Debounce_CaptureState_IO(&p_motorUser->PinBrake);
	Debounce_CaptureState_IO(&p_motorUser->PinThrottle);
	Debounce_CaptureState_IO(&p_motorUser->PinForward);
	Debounce_CaptureState_IO(&p_motorUser->PinReverse);

	p_motorUser->InputSwitchBrake 		= Debounce_GetState(&p_motorUser->PinBrake);
	p_motorUser->InputSwitchThrottle 	= Debounce_GetState(&p_motorUser->PinThrottle);
	p_motorUser->InputSwitchForward 	= Debounce_GetState(&p_motorUser->PinForward);
	p_motorUser->InputSwitchReverse 	= Debounce_GetState(&p_motorUser->PinReverse);

//	if(p_motorUser->InputSwitchNeutralEnable = true)
//	{
//	p_motorUser->InputSwitchNeutral = Debounce_GetState(p_motorUser->PinNeutral);
//	}
//	else
	{
		p_motorUser->InputSwitchNeutral = p_motorUser->InputSwitchForward | p_motorUser->InputSwitchReverse ? false : true;
	}

	p_motorUser->InputValueThrottlePrev 	= p_motorUser->InputValueThrottle;
	p_motorUser->InputValueBrakePrev 		= p_motorUser->InputValueBrake;
	p_motorUser->InputValueThrottle 		= Linear_ADC_CalcFractionUnsigned16(&p_motorUser->UnitThrottle, 	*p_motorUser->p_Config->P_THROTTLE_ADCU);
	p_motorUser->InputValueBrake 			= Linear_ADC_CalcFractionUnsigned16(&p_motorUser->UnitBrake, 		*p_motorUser->p_Config->P_BRAKE_ADCU);

//	p_motorUser->IsThrottleRelease = GetThrottleRelease(p_motorUser);
	p_motorUser->IsThrottleRelease = false;
}

