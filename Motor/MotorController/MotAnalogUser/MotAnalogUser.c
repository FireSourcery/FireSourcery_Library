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
	@file 	MotAnalogUser.c
	@author FireSoucery
	@brief
	@version V0
*/
/******************************************************************************/
#include "MotAnalogUser.h"
#include <string.h>

void MotAnalogUser_Init(MotAnalogUser_T * p_user)
{
	if(p_user->CONFIG.P_PARAMS != 0U)
	{
		memcpy(&p_user->Params, p_user->CONFIG.P_PARAMS, sizeof(MotAnalogUser_Params_T));
		Linear_ADC_Init(&p_user->UnitThrottle, p_user->Params.ThrottleZero_Adcu, p_user->Params.ThrottleMax_Adcu, 1000U);
		Linear_ADC_Init(&p_user->UnitBrake, p_user->Params.BrakeZero_Adcu, p_user->Params.BrakeMax_Adcu, 1000U);
	}
	else
	{
		Linear_ADC_Init(&p_user->UnitThrottle, 0U, 4095U, 1000U);
		Linear_ADC_Init(&p_user->UnitBrake, 0U, 4095U, 1000U);
	}

	Debounce_Init(&p_user->ReversePin, 5U);
	if(p_user->Params.UseForwardPin == true) 			{ Debounce_Init(&p_user->ForwardPin, 5U); }
	if(p_user->Params.UseNeutralPin == true) 			{ Debounce_Init(&p_user->NeutralPin, 5U); }
	if(p_user->Params.UseThrottleEdgePin == true) 		{ Debounce_Init(&p_user->ThrottleEdgePin, 5U); }
	if(p_user->Params.UseBrakeEdgePin == true) 			{ Debounce_Init(&p_user->BrakeEdgePin, 5U); }
	if(p_user->Params.UseBistateBrakePin == true) 		{ Debounce_Init(&p_user->BistateBrakePin, 5U); }
	if(p_user->Params.UseThrottleSafetyPin == true) 	{ Debounce_Init(&p_user->ThrottleSafetyPin, 5U); }

	MotAnalogUser_SetPinInvert(p_user, p_user->Params.InvertPins);

	p_user->ThrottlePrev_Frac16 = 0U;
	p_user->Throttle_Frac16 = 0U;
	p_user->BrakePrev_Frac16 = 0U;
	p_user->Brake_Frac16 = 0U;
}

void MotAnalogUser_SetParams(MotAnalogUser_T * p_user, const MotAnalogUser_Params_T * p_param)
{
	memcpy(&p_user->Params, p_param, sizeof(MotAnalogUser_Params_T));
}

void MotAnalogUser_SetBrakeRange(MotAnalogUser_T * p_user, uint16_t zero_Adcu, uint16_t max_Adcu)
{
	p_user->Params.BrakeZero_Adcu = zero_Adcu;
	p_user->Params.BrakeMax_Adcu = max_Adcu;
	Linear_ADC_Init(&p_user->UnitBrake, p_user->Params.BrakeZero_Adcu, p_user->Params.BrakeMax_Adcu, 1000U);
}

void MotAnalogUser_SetThrottleRange(MotAnalogUser_T * p_user, uint16_t zero_Adcu, uint16_t max_Adcu)
{
	p_user->Params.ThrottleZero_Adcu = zero_Adcu;
	p_user->Params.ThrottleMax_Adcu = max_Adcu;
	Linear_ADC_Init(&p_user->UnitThrottle, p_user->Params.ThrottleZero_Adcu, p_user->Params.ThrottleMax_Adcu, 1000U);
}

/*
	Non Propagating set. Reboot for Pin HAL config to take effect.
	todo prevent set if P_HAL_PIN == 0U
*/
void MotAnalogUser_SetBrakeAdc(MotAnalogUser_T * p_user, uint16_t zero_Adcu, uint16_t max_Adcu, bool useBrakeEdgePin) //range error
{
	MotAnalogUser_SetBrakeRange(p_user, zero_Adcu, max_Adcu);
	p_user->Params.UseBrakeEdgePin = useBrakeEdgePin;
}

void MotAnalogUser_SetThrottleAdc(MotAnalogUser_T * p_user, uint16_t zero_Adcu, uint16_t max_Adcu, bool useThrottleEdgePin) //range error
{
	MotAnalogUser_SetThrottleRange(p_user, zero_Adcu, max_Adcu);
	p_user->Params.UseThrottleEdgePin = useThrottleEdgePin;
}

// void MotAnalogUser_SetBrakeUnits_Frac16(MotAnalogUser_T * p_user, uint16_t zero_ , uint16_t max_ )
// void MotAnalogUser_SetThrottleUnits_Frac16(MotAnalogUser_T * p_user, uint16_t zero_ , uint16_t max_ )

void MotAnalogUser_SetBistateBrake(MotAnalogUser_T * p_user, bool useBistateBrake, uint16_t bistateBrakeIntensity_Frac16)
{
	p_user->Params.UseBistateBrakePin = useBistateBrake;
	p_user->Params.BistateBrakeValue_Frac16 = bistateBrakeIntensity_Frac16;
}

void MotAnalogUser_SetThrottleSafety(MotAnalogUser_T * p_user, bool useThrottleSafety)
{
	p_user->Params.UseThrottleSafetyPin = useThrottleSafety;
}

void MotAnalogUser_SetDirectionPins(MotAnalogUser_T * p_user, MotAnalogUser_DirectionPins_T pins)
{
	switch(pins)
	{
		case MOT_ANALOG_USER_DIRECTION_PINS_FNR:
			p_user->Params.UseForwardPin = true;
			p_user->Params.UseNeutralPin = true;
			break;

		case MOT_ANALOG_USER_DIRECTION_PINS_FR:
			p_user->Params.UseForwardPin = true;
			p_user->Params.UseNeutralPin = false;
			break;

		case MOT_ANALOG_USER_DIRECTION_PINS_R:
			p_user->Params.UseForwardPin = false;
			p_user->Params.UseNeutralPin = false;
			break;

		default: break;
	}
}

void MotAnalogUser_SetPinInvert(MotAnalogUser_T * p_user, MotAnalogUser_InvertPins_T invertPins)
{
	if(p_user->Params.InvertPins.State != invertPins.State)
	{
		p_user->Params.InvertPins.State = invertPins.State;
		(invertPins.BistateBrake == true) ? Debounce_EnableInvert(&p_user->BistateBrakePin) : Debounce_DisableInvert(&p_user->BistateBrakePin);
		(invertPins.Reverse == true) ? Debounce_EnableInvert(&p_user->ReversePin) : Debounce_DisableInvert(&p_user->ReversePin);
		(invertPins.Forward == true) ? Debounce_EnableInvert(&p_user->ForwardPin) : Debounce_DisableInvert(&p_user->ForwardPin);
		(invertPins.Neutral == true) ? Debounce_EnableInvert(&p_user->NeutralPin) : Debounce_DisableInvert(&p_user->NeutralPin);
		(invertPins.BrakeEdge == true) ? Debounce_EnableInvert(&p_user->BrakeEdgePin) : Debounce_DisableInvert(&p_user->BrakeEdgePin);
		(invertPins.ThrottleEdge == true) ? Debounce_EnableInvert(&p_user->ThrottleEdgePin) : Debounce_DisableInvert(&p_user->ThrottleEdgePin);
	}
}

