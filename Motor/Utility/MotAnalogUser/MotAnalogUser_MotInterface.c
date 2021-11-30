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
//#include "MotInterface.h"

#include "Transducer/Debounce/Debounce.h"
#include "Peripheral/Pin/Pin.h"

#include "Math/Linear/Linear_ADC.h"
#include "Math/Linear/Linear.h"

#include <stdint.h>
#include <stdbool.h>



////common interface version

//
//void AnalogUser_CaptureInputToInterface_IO(AnalogUser_T * p_motorUser, MotorInterface_Input_T * p_motorInterface)
//{
//	bool inputSwitchBrake;
//	bool inputSwitchThrottle;
//	bool inputSwitchForward;
//	bool inputSwitchReverse;
//	bool inputSwitchNeutral;
//
//	uint16_t  inputValueThrottle 		= Linear_ADC_CalcFractionUnsigned16(&p_motorUser-> UnitThrottle, 	*p_motorUser->p_Config->P_THROTTLE_ADCU);
//	uint16_t  inputValueBrake 			= Linear_ADC_CalcFractionUnsigned16(&p_motorUser-> UnitBrake, 		*p_motorUser->p_Config->P_BRAKE_ADCU);
//
//	MotorInterface_Input_SetThrottle(p_motorInterface, inputValueThrottle);
//	MotorInterface_Input_SetBrake(p_motorInterface, inputValueBrake);
//
//	Debounce_CaptureState_IO(&p_motorUser-> PinBrake);
//	Debounce_CaptureState_IO(&p_motorUser-> PinThrottle);
//	Debounce_CaptureState_IO(&p_motorUser-> PinForward);
//	Debounce_CaptureState_IO(&p_motorUser-> PinReverse);
//
//	inputSwitchBrake 		= Debounce_GetState(&p_motorUser-> PinBrake);
//	inputSwitchThrottle 	= Debounce_GetState(&p_motorUser-> PinThrottle);
//	inputSwitchForward 		= Debounce_GetState(&p_motorUser-> PinForward);
//	inputSwitchReverse 		= Debounce_GetState(&p_motorUser-> PinReverse);
////	inputSwitchNeutral 		= Debounce_GetState(p_motorUser->AnalogUserAnalog.PinNeutral);
//	inputSwitchNeutral 		= inputSwitchForward | inputSwitchReverse ? false : true;
//
//	MotorInterface_Input_SetSwitchThrottle(p_motorInterface, inputSwitchThrottle);
//	MotorInterface_Input_SetSwitchBrake(p_motorInterface, inputSwitchBrake);
//	MotorInterface_Input_SetSwitchForward(p_motorInterface, inputSwitchForward);
//	MotorInterface_Input_SetSwitchReverse(p_motorInterface, inputSwitchReverse);
//	MotorInterface_Input_SetSwitchNeutral(p_motorInterface, inputSwitchNeutral);
//
//	//	p_motorUser->IsThrottleRelease = AnalogUser_Analog_CalcThrottleRelease();
//}

