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
    @brief  Analog user controls. 1 instance for all motor, input
    @version V0
*/
/******************************************************************************/
#ifndef MOT_ANALOG_USER_H
#define MOT_ANALOG_USER_H

#include "Motor/Motor/Config.h"

#include "Transducer/Debounce/Debounce.h"
#include "Peripheral/Pin/Pin.h"

#include "Math/Linear/Linear_ADC.h"
#include "Math/Linear/Linear.h"

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
	MOT_ANALOG_USER_CMD_SET_BRAKE,
	MOT_ANALOG_USER_CMD_SET_THROTTLE,
	MOT_ANALOG_USER_CMD_SET_THROTTLE_RELEASE,
	MOT_ANALOG_USER_CMD_SET_BRAKE_RELEASE,
//	MOT_ANALOG_USER_CMD_SET_BRAKE_START,
//	MOT_ANALOG_USER_CMD_SET_THROTTLE_START,
	MOT_ANALOG_USER_CMD_SET_NEUTRAL,
	MOT_ANALOG_USER_CMD_PROC_NEUTRAL,
	MOT_ANALOG_USER_CMD_SET_DIRECTION_FORWARD,
	MOT_ANALOG_USER_CMD_SET_DIRECTION_REVERSE,
	MOT_ANALOG_USER_CMD_PROC_RELEASE,	/* No input */
}
MotAnalogUser_Cmd_T;

typedef enum
{
	MOT_ANALOG_USER_DIRECTION_NEUTRAL,
	MOT_ANALOG_USER_DIRECTION_FORWARD,
	MOT_ANALOG_USER_DIRECTION_REVERSE,
	MOT_ANALOG_USER_DIRECTION_NEUTRAL_EDGE,
	MOT_ANALOG_USER_DIRECTION_FORWARD_EDGE,
	MOT_ANALOG_USER_DIRECTION_REVERSE_EDGE,
}
MotAnalogUser_Direction_T;

typedef struct __attribute__((aligned (4U)))
{
	uint16_t ThrottleZero_ADCU;
	uint16_t ThrottleMax_ADCU;
	uint16_t BrakeZero_ADCU;
	uint16_t BrakeMax_ADCU;

	bool EnablePinThrottle;
	bool EnablePinBrake;
	bool EnablePinNeutral;
}
MotAnalogUser_Params_T;

/*
 * Activate Adc outside module
 */
typedef const struct
{
	const MotAnalogUser_Params_T * const P_PARAMS;
}
MotAnalogUser_Config_T;

typedef struct
{
	MotAnalogUser_Config_T CONFIG;
	MotAnalogUser_Params_T Params;

	Debounce_T PinForward;
	Debounce_T PinReverse;

	Debounce_T PinBrake;
	Debounce_T PinThrottle;
	Debounce_T PinNeutral;

	Linear_T UnitThrottle;
	Linear_T UnitBrake;

	uint16_t Throttle_Frac16;
	uint16_t ThrottlePrev_Frac16;
	uint16_t Brake_Frac16;
	uint16_t BrakePrev_Frac16;
}
MotAnalogUser_T;

#define MOT_ANALOG_USER_CONFIG(p_Brake_PinHal, Brake_PinId, p_Throttle_PinHal, Throttle_PinId, p_Forward_PinHal, Forward_PinId, p_Reverse_PinHal, Reverse_PinId, p_Millis, p_Params)	\
{																\
	.CONFIG =													\
	{															\
		.P_PARAMS =		p_Params,								\
	},															\
	.PinBrake 		= DEBOUNCE_CONFIG(p_Brake_PinHal, 		Brake_PinId, 		p_Millis),  	\
	.PinThrottle 	= DEBOUNCE_CONFIG(p_Throttle_PinHal, 	Throttle_PinId, 	p_Millis),  	\
	.PinForward 	= DEBOUNCE_CONFIG(p_Forward_PinHal, 	Forward_PinId, 		p_Millis),  	\
	.PinReverse 	= DEBOUNCE_CONFIG(p_Reverse_PinHal, 	Reverse_PinId, 		p_Millis),  	\
	.PinNeutral 	= DEBOUNCE_CONFIG(0, 0, 0),  	\
}

/*
 * adcu capture from outside module, or use pointer
 */
static inline void MotAnalogUser_CaptureThrottleValue(MotAnalogUser_T * p_user, uint16_t throttle_ADCU)
{
	if((p_user->Params.EnablePinThrottle == false) || (Debounce_GetState(&p_user->PinThrottle) == true))
	{
		p_user->ThrottlePrev_Frac16 = p_user->Throttle_Frac16;
		p_user->Throttle_Frac16 = Linear_ADC_CalcFractionUnsigned16(&p_user->UnitThrottle, throttle_ADCU);
		//		p_user->Throttle_Frac16 = (Linear_ADC_CalcFractionUnsigned16(&p_user->UnitThrottle, throttle_ADCU) + p_user->ThrottlePrev_Frac16) / 2U;
	}

}

static inline void MotAnalogUser_CaptureBrakeValue(MotAnalogUser_T * p_user, uint16_t brake_ADCU)
{
	if((p_user->Params.EnablePinBrake == false) || (Debounce_GetState(&p_user->PinBrake) == true))
	{
		p_user->BrakePrev_Frac16 = p_user->Brake_Frac16;
		p_user->Brake_Frac16 = Linear_ADC_CalcFractionUnsigned16(&p_user->UnitBrake, brake_ADCU);
		//		p_user->Brake_Frac16 = (Linear_ADC_CalcFractionUnsigned16(&p_user->UnitBrake, throttle_ADCU) + p_user->BrakePrev_Frac16) / 2U;
	}
}

static inline void MotAnalogUser_CaptureSwitches(MotAnalogUser_T * p_user)
{
 	if (p_user->Params.EnablePinThrottle == true)	{Debounce_CaptureState(&p_user->PinThrottle);}
	if (p_user->Params.EnablePinBrake == true)		{Debounce_CaptureState(&p_user->PinBrake);}
	Debounce_CaptureState(&p_user->PinForward);
	Debounce_CaptureState(&p_user->PinReverse);
	if (p_user->Params.EnablePinNeutral == true)	{Debounce_CaptureState(&p_user->PinNeutral);}
}

/*
 * Calling module is responsible for ADC activation and passes complete results
 */
static inline void MotAnalogUser_CaptureInput(MotAnalogUser_T * p_user, uint16_t throttle_ADCU, uint16_t brake_ADCU)
{
	MotAnalogUser_CaptureSwitches(p_user);
	MotAnalogUser_CaptureThrottleValue(p_user, throttle_ADCU);
	MotAnalogUser_CaptureBrakeValue(p_user, brake_ADCU);
}

/*
 * Optional separate Brake polling
 */
static inline bool MotAnalogUser_PollBrake(MotAnalogUser_T * p_user)
{
	Debounce_CaptureState(&p_user->PinBrake);
	return Debounce_GetState(&p_user->PinBrake);
}

static inline bool MotAnalogUser_GetThrottleSwitch(const MotAnalogUser_T * p_user) 		{return (p_user->Params.EnablePinThrottle 	== true) ? Debounce_GetState(&p_user->PinThrottle) 	: (p_user->Throttle_Frac16 	> 0U);	}
static inline bool MotAnalogUser_GetBrakeSwitch(const MotAnalogUser_T * p_user) 		{return (p_user->Params.EnablePinBrake 		== true) ? Debounce_GetState(&p_user->PinBrake) 	: (p_user->Brake_Frac16 	> 0U);	}

static inline bool MotAnalogUser_PollThrottleSwitchFalling(MotAnalogUser_T * p_user)
{
	return (p_user->Params.EnablePinThrottle == true) ? Debounce_PollFallingEdge(&p_user->PinThrottle) : ((p_user->Throttle_Frac16 == 0U) && (p_user->ThrottlePrev_Frac16 > 0U));
}

static inline bool MotAnalogUser_PollBrakeSwitchFalling(MotAnalogUser_T * p_user)
{
	return (p_user->Params.EnablePinBrake == true) ? Debounce_PollFallingEdge(&p_user->PinBrake) : ((p_user->Brake_Frac16 == 0U) && (p_user->BrakePrev_Frac16 > 0U));
}

static inline uint16_t MotAnalogUser_GetThrottleValue(const MotAnalogUser_T * p_user)	{return p_user->Throttle_Frac16;}
static inline uint16_t MotAnalogUser_GetBrakeValue(const MotAnalogUser_T * p_user)		{return p_user->Brake_Frac16;}
static inline uint16_t MotAnalogUser_GetThrottle(const MotAnalogUser_T * p_user) 		{return (MotAnalogUser_GetThrottleSwitch(p_user) == true) 	? p_user->Throttle_Frac16	: 0U; }
static inline uint16_t MotAnalogUser_GetBrake(const MotAnalogUser_T * p_user) 			{return (MotAnalogUser_GetBrakeSwitch(p_user) == true) 		? p_user->Brake_Frac16 		: 0U; }

/*
 * Early throttle release detect
 */
static inline bool MotAnalogUser_CheckThrottleRelease(const MotAnalogUser_T * p_user)
{
	return (((int32_t)p_user->ThrottlePrev_Frac16 - (int32_t)p_user->Throttle_Frac16) > (65535/100));;
}

static inline bool MotAnalogUser_GetNeutralSwitch(const MotAnalogUser_T * p_user) 		{return Debounce_GetState(&p_user->PinNeutral);}
static inline bool MotAnalogUser_GetForwardSwitch(const MotAnalogUser_T * p_user) 		{return Debounce_GetState(&p_user->PinForward);}
static inline bool MotAnalogUser_GetReverseSwitch(const MotAnalogUser_T * p_user) 		{return Debounce_GetState(&p_user->PinReverse);}

static inline bool MotAnalogUser_GetDirection(const MotAnalogUser_T * p_user)
{
	MotAnalogUser_Direction_T direction;

	if((p_user->Params.EnablePinNeutral == true) && (Debounce_GetState(&p_user->PinNeutral) == true))
	{
		direction = MOT_ANALOG_USER_DIRECTION_NEUTRAL;
	}
	else if(MotAnalogUser_GetForwardSwitch(p_user) == true)
	{
		direction = MOT_ANALOG_USER_DIRECTION_FORWARD;
	}
	else if(MotAnalogUser_GetReverseSwitch(p_user) == true)
	{
		direction = MOT_ANALOG_USER_DIRECTION_REVERSE;
	}
	else
	{
		direction = MOT_ANALOG_USER_DIRECTION_NEUTRAL;
	}

	return direction;
}

static inline MotAnalogUser_Direction_T MotAnalogUser_PollDirection(MotAnalogUser_T * p_user)
{
	MotAnalogUser_Direction_T direction;

	if((p_user->Params.EnablePinNeutral == true) && (Debounce_GetState(&p_user->PinNeutral) == true))
	{
		direction = Debounce_PollRisingEdge(&p_user->PinNeutral) ? MOT_ANALOG_USER_DIRECTION_NEUTRAL_EDGE : MOT_ANALOG_USER_DIRECTION_NEUTRAL;
	}
	else if (Debounce_GetState(&p_user->PinForward) == true)
	{
		direction = Debounce_PollRisingEdge(&p_user->PinForward) ? MOT_ANALOG_USER_DIRECTION_FORWARD_EDGE : MOT_ANALOG_USER_DIRECTION_FORWARD;
	}
	else if (Debounce_GetState(&p_user->PinReverse) == true)
	{
		direction = Debounce_PollRisingEdge(&p_user->PinReverse) ? MOT_ANALOG_USER_DIRECTION_REVERSE_EDGE : MOT_ANALOG_USER_DIRECTION_REVERSE;
	}
	else
	{
		direction = ((Debounce_PollFallingEdge(&p_user->PinForward) == true) || (Debounce_PollFallingEdge(&p_user->PinReverse) == true))
			? MOT_ANALOG_USER_DIRECTION_NEUTRAL_EDGE : MOT_ANALOG_USER_DIRECTION_NEUTRAL;
	}

	return direction;
}


static inline MotAnalogUser_Cmd_T MotAnalogUser_PollCmd(MotAnalogUser_T * p_user)
{
	MotAnalogUser_Direction_T direction = MotAnalogUser_PollDirection(p_user);
	MotAnalogUser_Cmd_T cmd = MOT_ANALOG_USER_CMD_PROC_RELEASE;

	/*
	 * Edge functions are instantaneous and can proc before brake, or check in separate function to proc in addition to brake
	 * Need to detect even during braking, for error response
	 *
	 * Direction should be able to set while brake is set.
	 * if state filters inputs,
	 */
	switch(direction)
	{
//		case MOT_ANALOG_USER_DIRECTION_NEUTRAL_EDGE: cmd = MOT_ANALOG_USER_CMD_SET_NEUTRAL;				break;
		case MOT_ANALOG_USER_DIRECTION_FORWARD_EDGE: cmd = MOT_ANALOG_USER_CMD_SET_DIRECTION_FORWARD;	break;
		case MOT_ANALOG_USER_DIRECTION_REVERSE_EDGE: cmd = MOT_ANALOG_USER_CMD_SET_DIRECTION_REVERSE;	break;
//		case MOT_ANALOG_USER_DIRECTION_NEUTRAL:
//		case MOT_ANALOG_USER_DIRECTION_REVERSE:
//		case MOT_ANALOG_USER_DIRECTION_FORWARD:
		default:
			/* Check Brake first */
			if(MotAnalogUser_GetBrakeSwitch(p_user) == true)
			{
				//need different cmd on brake edge?
				cmd = MOT_ANALOG_USER_CMD_SET_BRAKE;
			}
			else if(MotAnalogUser_PollBrakeSwitchFalling(p_user)== true)
			{
				cmd = MOT_ANALOG_USER_CMD_SET_BRAKE_RELEASE;
			}
			/* Check Direction */
			else if(direction == MOT_ANALOG_USER_DIRECTION_NEUTRAL_EDGE)
			{
				cmd = MOT_ANALOG_USER_CMD_SET_NEUTRAL;
			}
			else if(direction == MOT_ANALOG_USER_DIRECTION_NEUTRAL)
			{
				cmd = MOT_ANALOG_USER_CMD_PROC_NEUTRAL;
			}
			/* Check Throttle. Direction is non edge Forward or Reverse */
			else if(MotAnalogUser_GetThrottleSwitch(p_user) == true)
			{
				if(MotAnalogUser_CheckThrottleRelease(p_user) == true) /* repeat throttle release is okay for now, otherwise track previous cmd state */
				{
					cmd = MOT_ANALOG_USER_CMD_SET_THROTTLE_RELEASE;
				}
				else
				{
					cmd = MOT_ANALOG_USER_CMD_SET_THROTTLE;
				}
			}
			else if(MotAnalogUser_PollThrottleSwitchFalling(p_user)== true)
			{
				cmd = MOT_ANALOG_USER_CMD_SET_THROTTLE_RELEASE;
			}
			else
			{
				cmd = MOT_ANALOG_USER_CMD_PROC_RELEASE;
			}

			break;
	}

	return cmd;
}


extern void MotAnalogUser_Init(MotAnalogUser_T * p_user);
extern void MotAnalogUser_SetParams(MotAnalogUser_T * p_user, const MotAnalogUser_Params_T * p_param);

#endif
