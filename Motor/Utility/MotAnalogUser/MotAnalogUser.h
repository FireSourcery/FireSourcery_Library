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
    @brief  Analog user controls. 1 instance for all motor, input
    @version V0
*/
/**************************************************************************/
#ifndef MOT_ANALOG_USER_H
#define MOT_ANALOG_USER_H

#include "Motor/Motor/Config.h"

#include "Transducer/Debounce/Debounce.h"
#include "Peripheral/Pin/Pin.h"

#include "Math/Linear/Linear_ADC.h"
#include "Math/Linear/Linear.h"

#include <stdint.h>
#include <stdbool.h>

//#ifdef CONFIG_MOTOR_ADC_8
//	typedef uint8_t adc_t;
//#elif defined(CONFIG_MOTOR_ADC_16)
//	typedef uint16_t adc_t;
//#endif

typedef const struct
{
	const uint16_t * const P_THROTTLE_ADCU;
	const uint16_t * const P_BRAKE_ADCU;
//	const uint32_t * P_BRAKE_ADC_CALIBRATION; //nvm version
//	const uint32_t * P_THROTTLE_ADC_CALIBRATION;
}
MotAnalogUser_Config_T;

typedef struct
{
	MotAnalogUser_Config_T  CONFIG;

	Linear_T UnitThrottle;
	Linear_T UnitBrake;

	Debounce_T PinBrake;
	Debounce_T PinThrottle;
	Debounce_T PinForward;
	Debounce_T PinReverse;

	bool InputSwitchNeutral;
	bool IsThrottleRelease;

	uint16_t InputValueThrottle;
	uint16_t InputValueThrottlePrev;
	uint16_t InputValueBrake;
	uint16_t InputValueBrakePrev;

	//input values regs/buffer
	bool InputSwitchBrake;
	bool InputSwitchThrottle;
	bool InputSwitchForward;
	bool InputSwitchReverse;
}
MotAnalogUser_T;

#define MOT_ANALOG_USER_CONFIG(p_Millis, p_Brake_PinHal, Brake_PinId, p_Throttle_PinHal, Throttle_PinId, p_Forward_PinHal, Forward_PinId, p_Reverse_PinHal, Reverse_PinId, p_ThrottleAdcu, p_BrakeAdcu)	\
{																\
	.CONFIG =													\
	{															\
		.P_THROTTLE_ADCU =		p_ThrottleAdcu,					\
		.P_BRAKE_ADCU =			p_BrakeAdcu, 					\
	},															\
	.PinBrake 		= DEBOUNCE_CONFIG(p_Brake_PinHal, 		Brake_PinId, 		p_Millis),  	\
	.PinThrottle 	= DEBOUNCE_CONFIG(p_Throttle_PinHal, 	Throttle_PinId, 	p_Millis),  	\
	.PinForward 	= DEBOUNCE_CONFIG(p_Forward_PinHal, 	Forward_PinId, 		p_Millis),  	\
	.PinReverse 	= DEBOUNCE_CONFIG(p_Reverse_PinHal, 	Reverse_PinId, 		p_Millis),  	\
}

extern void MotAnalogUser_Init(MotAnalogUser_T * p_motorUser );
extern void MotAnalogUser_CaptureInput(MotAnalogUser_T * p_motorUser);

#endif
