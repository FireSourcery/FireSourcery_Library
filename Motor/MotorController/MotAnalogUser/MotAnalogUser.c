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
    @file 	Motor.h
    @author FireSoucery
    @brief

    @version V0
*/
/******************************************************************************/
#include "MotAnalogUser.h"

#include "Transducer/Debounce/Debounce.h"
#include "Peripheral/Pin/Pin.h"

#include "Math/Linear/Linear_ADC.h"
#include "Math/Linear/Linear.h"

#include <stdint.h>
#include <stdbool.h>

#include <string.h>

void MotAnalogUser_Init(MotAnalogUser_T * p_user)
{
	if (p_user->CONFIG.P_PARAMS != 0U)
	{
		memcpy(&p_user->Params, p_user->CONFIG.P_PARAMS, sizeof(MotAnalogUser_Params_T));
		Linear_ADC_Init(&p_user->UnitThrottle, 	p_user->Params.ThrottleZero_ADCU, 	p_user->Params.ThrottleMax_ADCU, 	1000U);
		Linear_ADC_Init(&p_user->UnitBrake,		p_user->Params.BrakeZero_ADCU, 		p_user->Params.BrakeMax_ADCU, 		1000U);
	}
	else
	{
		Linear_ADC_Init(&p_user->UnitThrottle, 	0U, 4095U, 1000U);
		Linear_ADC_Init(&p_user->UnitBrake,		0U, 4095U, 1000U);
	}

	Debounce_Init(&p_user->PinBrake, 		5U);	//5millis
	Debounce_Init(&p_user->PinThrottle,		5U);	//5millis
	Debounce_Init(&p_user->PinForward, 		5U);	//5millis
	Debounce_Init(&p_user->PinReverse, 		5U);	//5millis

	p_user->ThrottlePrev_Frac16 = 0U;
	p_user->Throttle_Frac16 = 0U;
	p_user->BrakePrev_Frac16 = 0U;
	p_user->Brake_Frac16 = 0U;
}

void MotAnalogUser_SetParams(MotAnalogUser_T * p_user, const MotAnalogUser_Params_T * p_param)
{
	memcpy(&p_user->Params, p_param, sizeof(MotAnalogUser_Params_T));
}

void MotAnalogUser_CalibrateBrake(MotAnalogUser_T * p_user, uint16_t zero_ADCU, uint16_t max_ADCU)
{
	p_user->Params.BrakeZero_ADCU = zero_ADCU;
	p_user->Params.BrakeMax_ADCU = max_ADCU;
	Linear_ADC_Init(&p_user->UnitBrake, p_user->Params.BrakeZero_ADCU, p_user->Params.BrakeMax_ADCU, 1000U);
}

void MotAnalogUser_CalibrateThrottle(MotAnalogUser_T * p_user, uint16_t zero_ADCU, uint16_t max_ADCU)
{
	p_user->Params.ThrottleZero_ADCU = zero_ADCU;
	p_user->Params.ThrottleMax_ADCU = max_ADCU;
	Linear_ADC_Init(&p_user->UnitThrottle, p_user->Params.ThrottleZero_ADCU, p_user->Params.ThrottleMax_ADCU, 1000U);
}
