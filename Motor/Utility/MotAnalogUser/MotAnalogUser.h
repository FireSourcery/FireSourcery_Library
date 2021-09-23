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

#include "Peripheral/Pin/Debounce.h"
#include "Peripheral/Pin/Pin.h"

#include "Math/Linear/Linear_ADC.h"
#include "Math/Linear/Linear.h"

#include <stdint.h>
#include <stdbool.h>

#ifdef CONFIG_MOTOR_ADC_8
	typedef uint8_t adc_t;
#elif defined(CONFIG_MOTOR_ADC_16)
	typedef uint16_t adc_t;
#endif

typedef const struct
{
	const HAL_Pin_T HAL_PIN_BRAKE;
	const HAL_Pin_T HAL_PIN_THROTTLE;
	const HAL_Pin_T HAL_PIN_FORWARD;
	const HAL_Pin_T HAL_PIN_REVERSE;
	const HAL_Pin_T HAL_PIN_NEUTRAL;
	volatile const uint32_t * const P_DEBOUNCE_TIMER;
	volatile const adc_t * const P_THROTTLE_ADCU;
	volatile const adc_t * const P_BRAKE_ADCU;
	const uint32_t LINEAR_V_ADC_VREF;
	const uint32_t LINEAR_V_ADC_BITS;
}
MotAnalogUser_Config_T;

typedef struct
{
	MotAnalogUser_Config_T * p_Config;

	Linear_T UnitThrottle;
	Linear_T UnitBrake;

	Debounce_T PinBrake;
	Debounce_T PinThrottle;
	Debounce_T PinForward;
	Debounce_T PinReverse;

	bool InputSwitchNeutralEnable;

	//input values regs/buffer
	volatile bool InputSwitchBrake;
	volatile bool InputSwitchThrottle;
	volatile bool InputSwitchForward;
	volatile bool InputSwitchReverse;
	volatile bool InputSwitchNeutral;
	volatile bool IsThrottleRelease;

	volatile uint16_t InputValueThrottle;
	volatile uint16_t InputValueThrottlePrev;
	volatile uint16_t InputValueBrake;
	volatile uint16_t InputValueBrakePrev;
}
MotAnalogUser_T;

extern void MotAnalogUser_Init(MotAnalogUser_T * p_motorUser, const MotAnalogUser_Config_T * p_config);
extern void MotAnalogUser_CaptureInput(MotAnalogUser_T * p_motorUser);

#endif
