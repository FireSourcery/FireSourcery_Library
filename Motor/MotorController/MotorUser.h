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
    @brief  Per Motor State Control.
    @version V0
*/
/**************************************************************************/
#ifndef MOTOR_USER_H
#define MOTOR_USER_H

#include "../Motor.h"
#include "../HAL_Motor.h"

#include "Peripheral/Pin/Debounce.h"
#include "Peripheral/Pin/Pin.h"

#include "Math/Linear/Linear_ADC.h"
#include "Math/Linear/Linear.h"

#include <stdint.h>
#include <stdbool.h>

#ifdef CONFIG_MOTOR_ADC_8
	typedef uint16_t adc_t;
#elif defined(CONFIG_MOTOR_ADC_16)
	typedef uint16_t adc_t;
#endif

typedef enum
{
	MOTOR_INPUT_MODE_ANALOG,
	MOTOR_INPUT_MODE_SERIAL,
	MOTOR_INPUT_MODE_CAN,
} MotorUser_InputMode_T;

typedef struct
{
	Linear_T UnitThrottle;
	Linear_T UnitBrake;

	Debounce_T PinBrake;
	Debounce_T PinThrottle;
	Debounce_T PinForward;
	Debounce_T PinReverse;

} MotorUser_Analog_T;

typedef const struct
{
	const HAL_Pin_T HAL_PIN_BRAKE;
	const HAL_Pin_T HAL_PIN_THROTTLE;
	const HAL_Pin_T HAL_PIN_FORWARD;
	const HAL_Pin_T HAL_PIN_REVERSE;

	volatile const adc_t * const P_THROTTLE_ADCU;
	volatile const adc_t * const P_BRAKE_ADCU;

	const uint32_t LINEAR_V_ADC_VREF;
	const uint32_t LINEAR_V_ADC_BITS;

	const uint32_t * const P_DEBOUNCE_TIMER;
} MotorUser_Init_T;

typedef struct
{
	const MotorUser_Init_T * P_MOTOR_USER_CONST;

	MotorUser_Analog_T MotorUserAnalog;
//	Serial_T Serial;

	//todo alarm

	MotorUser_InputMode_T InputMode;

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

} MotorUser_T;

/*
 * Misra Violation
 */
extern MotorUser_T MotorUserMain;


static inline bool MotorUser_Analog_GetThrottleRelease()
{
	bool isReleased;

	if (MotorUserMain.InputValueThrottle < MotorUserMain.InputValueThrottlePrev)
	{
		if ((MotorUserMain.InputValueThrottlePrev - MotorUserMain.InputValueThrottle) > (65535U/10U))
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


static inline void MotorUser_Analog_CaptureInput_IO()
{
	Debounce_CaptureState_IO(&MotorUserMain.MotorUserAnalog.PinBrake);
	Debounce_CaptureState_IO(&MotorUserMain.MotorUserAnalog.PinThrottle);
	Debounce_CaptureState_IO(&MotorUserMain.MotorUserAnalog.PinForward);
	Debounce_CaptureState_IO(&MotorUserMain.MotorUserAnalog.PinReverse);

	MotorUserMain.InputSwitchBrake 		= Debounce_GetState(&MotorUserMain.MotorUserAnalog.PinBrake);
	MotorUserMain.InputSwitchThrottle 	= Debounce_GetState(&MotorUserMain.MotorUserAnalog.PinThrottle);
	MotorUserMain.InputSwitchForward 	= Debounce_GetState(&MotorUserMain.MotorUserAnalog.PinForward);
	MotorUserMain.InputSwitchReverse 	= Debounce_GetState(&MotorUserMain.MotorUserAnalog.PinReverse);
	//MotorUserMain.InputSwitchNeutral = Debounce_GetState(&MotorUserMain.MotorUserAnalog.PinNeutral);
	MotorUserMain.InputSwitchNeutral = MotorUserMain.InputSwitchForward | MotorUserMain.InputSwitchReverse ? false : true;

	MotorUserMain.InputValueThrottlePrev 	= MotorUserMain.InputValueThrottle;
	MotorUserMain.InputValueBrakePrev 		= MotorUserMain.InputValueBrake;
	MotorUserMain.InputValueThrottle 		= Linear_ADC_CalcFractionUnsigned16(&MotorUserMain.MotorUserAnalog.UnitThrottle, 	*MotorUserMain.P_MOTOR_USER_CONST->P_THROTTLE_ADCU);
	MotorUserMain.InputValueBrake 			= Linear_ADC_CalcFractionUnsigned16(&MotorUserMain.MotorUserAnalog.UnitBrake, 		*MotorUserMain.P_MOTOR_USER_CONST->P_BRAKE_ADCU);

//	MotorUserMain.IsThrottleRelease = MotorUser_Analog_GetThrottleRelease();


}

static inline void MotorUser_Serial_CaptureInput_IO()
{
//	if(Thread_PollTimerCompletePeriodic(&TimerSeconds) == true)
//	{
//		Serial_RestartRx_IO(&Serial1);
//	}
}


static inline void MotorUser_CaptureInput_IO()
{
	switch (MotorUserMain.InputMode)
	{
	case MOTOR_INPUT_MODE_ANALOG:
		break;

	case MOTOR_INPUT_MODE_SERIAL:
		break;


//	case MOTOR_INPUT_MODE_DISABLED:
		break;

	default:
		break;
	}

}





static inline void MotorUser_WriteOutput()
{


}

//priority brake thread
//static inline void MotorUser_PollBrake(Motor_T * p_motor)
//{
//	Debounce_CaptureState_IO(&MotorUserMain.MotorUserAnalog.PinBrake);
//	MotorUserMain.InputSwitchBrake 		= Debounce_GetState(&MotorUserMain.MotorUserAnalog.PinBrake);
//
//	if (MotorUserMain.InputSwitchBrake == true)
//	{
//			Motor_SetUserCmd(p_motor, (MotorUserMain.InputValueBrake + MotorUserMain.InputValueBrake) / 2U);
//			Motor_SetRampDecelerate(p_motor, 0);
//	}
//}

extern void MotorUser_Init(MotorUser_Init_T * p_init);
#endif
