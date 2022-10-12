/******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSourcery / The Firebrand Forge Inc

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
	@file 	MotAnalogUser.h
	@author FireSourcery
	@brief  Analog user controls. 1 instance for all motor, input
	@version V0
*/
/******************************************************************************/
#ifndef MOT_ANALOG_USER_H
#define MOT_ANALOG_USER_H

#include "Transducer/Debounce/Debounce.h"
#include "Math/Linear/Linear_ADC.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum MotAnalogUser_Cmd_Tag
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
	MOT_ANALOG_USER_CMD_PROC_ZERO,	/* No input */
}
MotAnalogUser_Cmd_T;

typedef enum MotAnalogUser_Direction_Tag
{
	MOT_ANALOG_USER_DIRECTION_NEUTRAL,
	MOT_ANALOG_USER_DIRECTION_FORWARD,
	MOT_ANALOG_USER_DIRECTION_REVERSE,
	MOT_ANALOG_USER_DIRECTION_NEUTRAL_EDGE,
	MOT_ANALOG_USER_DIRECTION_FORWARD_EDGE,
	MOT_ANALOG_USER_DIRECTION_REVERSE_EDGE,
}
MotAnalogUser_Direction_T;

typedef enum MotAnalogUser_DirectionPins_Tag
{
	MOT_ANALOG_USER_DIRECTION_PINS_FNR,
	MOT_ANALOG_USER_DIRECTION_PINS_FR,
	MOT_ANALOG_USER_DIRECTION_PINS_R,
}
MotAnalogUser_DirectionPins_T;

typedef union MotAnalogUser_InvertPins_Tag
{
	struct
	{
		uint32_t ThrottleEdge 		: 1U;
		uint32_t BrakeEdge 			: 1U;
		uint32_t Neutral 			: 1U;
		uint32_t Forward 			: 1U;
		uint32_t Reverse 			: 1U;
		uint32_t BistateBrake 		: 1U;
		uint32_t ThrottleSafety 	: 1U;
	};
	uint8_t State;
}
MotAnalogUser_InvertPins_T;

typedef struct __attribute__((aligned(4U))) MotAnalogUser_Params_Tag
{
	uint16_t ThrottleZero_Adcu;
	uint16_t ThrottleMax_Adcu;
	uint16_t BrakeZero_Adcu;
	uint16_t BrakeMax_Adcu;

	bool UseThrottleEdgePin;	 /* ADC Threshold. Pin determines THROTTLE_RELEASE mode */
	bool UseBrakeEdgePin;
	bool UseNeutralPin;
	bool UseForwardPin;

	bool UseBistateBrakePin;
	bool UseThrottleSafetyPin;	/* ADC Threshold. EXCEPT Throttle_Frac16, ThrottleZero_Adcu determines THROTTLE_RELEASE mode by default */
	uint16_t BistateBrakeValue_Frac16;

	// bool UseErrorAdcuRange; //report error if adcu read falls outside Zero_Adcu and Max_Adcu
	MotAnalogUser_InvertPins_T InvertPins;
}
MotAnalogUser_Params_T;

/*
	Activate ADC outside module
*/
typedef const struct MotAnalogUser_Config_Tag
{
	const MotAnalogUser_Params_T * const P_PARAMS;
}
MotAnalogUser_Config_T;

typedef struct MotAnalogUser_Tag
{
	MotAnalogUser_Config_T CONFIG;
	MotAnalogUser_Params_T Params;

	Debounce_T ReversePin;
	Debounce_T ForwardPin;
	Debounce_T NeutralPin;
	Debounce_T BrakeEdgePin;
	Debounce_T ThrottleEdgePin;
	Debounce_T BistateBrakePin;
	Debounce_T ThrottleSafetyPin;
	Linear_T UnitThrottle;
	Linear_T UnitBrake;

	uint16_t Throttle_Frac16;
	uint16_t ThrottlePrev_Frac16;
	uint16_t Brake_Frac16;
	uint16_t BrakePrev_Frac16;

	MotAnalogUser_Cmd_T Cmd; /* Store last active Cmd */
}
MotAnalogUser_T;

#define MOT_ANALOG_USER_INIT(p_BrakePinHal, BrakePinId, p_ThrottlePinHal, ThrottlePinId, p_ForwardPinHal, ForwardPinId, p_ReversePinHal, ReversePinId, p_BistateBrakePinHal, BistateBrakePinId, p_ThrottleSafetyPinHal, ThrottleSafetyPinId, p_Millis, p_Params)	\
{																											\
	.CONFIG 				= { .P_PARAMS = p_Params, },													\
	.BrakeEdgePin 			= DEBOUNCE_INIT(p_BrakePinHal, 			BrakePinId, 			p_Millis), 	\
	.ThrottleEdgePin 		= DEBOUNCE_INIT(p_ThrottlePinHal, 		ThrottlePinId, 			p_Millis), 	\
	.ForwardPin 			= DEBOUNCE_INIT(p_ForwardPinHal, 			ForwardPinId, 			p_Millis), 	\
	.ReversePin 			= DEBOUNCE_INIT(p_ReversePinHal, 			ReversePinId, 			p_Millis), 	\
	.BistateBrakePin		= DEBOUNCE_INIT(p_BistateBrakePinHal, 	BistateBrakePinId, 		p_Millis), 	\
	.ThrottleSafetyPin 		= DEBOUNCE_INIT(p_ThrottleSafetyPinHal, 	ThrottleSafetyPinId, 	p_Millis), 	\
	.NeutralPin 			= DEBOUNCE_INIT(0, 0, 0),  													\
}

/*
	adcu capture from outside module, or use pointer
*/
static inline void MotAnalogUser_CaptureThrottleValue(MotAnalogUser_T * p_user, uint16_t throttle_Adcu)
{
	if((p_user->Params.UseThrottleEdgePin == false) || (Debounce_GetState(&p_user->ThrottleEdgePin) == true))
	{
		p_user->ThrottlePrev_Frac16 = p_user->Throttle_Frac16;
		p_user->Throttle_Frac16 = Linear_ADC_CalcFractionUnsigned16(&p_user->UnitThrottle, throttle_Adcu);
		//		p_user->Throttle_Frac16 = (Linear_ADC_CalcFractionUnsigned16(&p_user->UnitThrottle, throttle_Adcu) + p_user->ThrottlePrev_Frac16) / 2U;
	}
}

static inline void MotAnalogUser_CaptureBrakeValue(MotAnalogUser_T * p_user, uint16_t brake_Adcu)
{
	if((p_user->Params.UseBrakeEdgePin == false) || (Debounce_GetState(&p_user->BrakeEdgePin) == true))
	{
		p_user->BrakePrev_Frac16 = p_user->Brake_Frac16;
		p_user->Brake_Frac16 = Linear_ADC_CalcFractionUnsigned16(&p_user->UnitBrake, brake_Adcu);
		//		p_user->Brake_Frac16 = (Linear_ADC_CalcFractionUnsigned16(&p_user->UnitBrake, throttle_Adcu) + p_user->BrakePrev_Frac16) / 2U;
	}
}

static inline void MotAnalogUser_CapturePins(MotAnalogUser_T * p_user)
{
	Debounce_CaptureState(&p_user->ReversePin);
	if(p_user->Params.UseForwardPin == true) 			{ Debounce_CaptureState(&p_user->ForwardPin); }
	if(p_user->Params.UseNeutralPin == true) 			{ Debounce_CaptureState(&p_user->NeutralPin); }
	if(p_user->Params.UseThrottleEdgePin == true) 		{ Debounce_CaptureState(&p_user->ThrottleEdgePin); }
	if(p_user->Params.UseBrakeEdgePin == true) 			{ Debounce_CaptureState(&p_user->BrakeEdgePin); }
	if(p_user->Params.UseBistateBrakePin == true) 		{ Debounce_CaptureState(&p_user->BistateBrakePin); }
	if(p_user->Params.UseThrottleSafetyPin == true) 	{ Debounce_CaptureState(&p_user->ThrottleSafetyPin); }
}

static inline void MotAnalogUser_CaptureBistateBrake(MotAnalogUser_T * p_user)
{
	if((p_user->Params.UseBistateBrakePin == true) && (Debounce_GetState(&p_user->BistateBrakePin) == true))
	{
		if(p_user->Brake_Frac16 < p_user->Params.BistateBrakeValue_Frac16)
		{
			p_user->Brake_Frac16 = p_user->Params.BistateBrakeValue_Frac16;
		}
	}
}

/*
	Calling module is responsible for ADC activation and passes complete results
*/
static inline void MotAnalogUser_CaptureInput(MotAnalogUser_T * p_user, uint16_t throttle_Adcu, uint16_t brake_Adcu)
{
	MotAnalogUser_CapturePins(p_user);
	MotAnalogUser_CaptureThrottleValue(p_user, throttle_Adcu);
	MotAnalogUser_CaptureBrakeValue(p_user, brake_Adcu);
	MotAnalogUser_CaptureBistateBrake(p_user);
}

/* Optional separate Brake polling */
static inline bool MotAnalogUser_PollBrakePin(MotAnalogUser_T * p_user)
{
	Debounce_CaptureState(&p_user->BrakeEdgePin);
	return Debounce_GetState(&p_user->BrakeEdgePin);
}

static inline bool MotAnalogUser_PollBrakePinRisingEdge(MotAnalogUser_T * p_user)
{
	return ((Debounce_CaptureState(&p_user->BrakeEdgePin) && Debounce_GetState(&p_user->BrakeEdgePin)) == true);
}

static inline bool _MotAnalogUser_GetIsNeutralOn(const MotAnalogUser_T * p_user) 		{ return (p_user->Params.UseNeutralPin == true) && (Debounce_GetState(&p_user->NeutralPin) == true); }
static inline bool MotAnalogUser_GetIsForwardOn(const MotAnalogUser_T * p_user) 		{ return (p_user->Params.UseForwardPin == true) ? Debounce_GetState(&p_user->ForwardPin) : !Debounce_GetState(&p_user->ReversePin); }
static inline bool MotAnalogUser_GetIsReverseOn(const MotAnalogUser_T * p_user) 		{ return Debounce_GetState(&p_user->ReversePin); }
static inline bool MotAnalogUser_GetIsThrottleOn(const MotAnalogUser_T * p_user) 		{ return (p_user->Params.UseThrottleEdgePin == true) 	? Debounce_GetState(&p_user->ThrottleEdgePin) 	: (p_user->Throttle_Frac16 > 0U); }
static inline bool MotAnalogUser_GetIsBrakeOn(const MotAnalogUser_T * p_user) 			{ return (p_user->Params.UseBrakeEdgePin == true) 		? Debounce_GetState(&p_user->BrakeEdgePin) 		: (p_user->Brake_Frac16 > 0U); }
static inline bool MotAnalogUser_GetIsBistateBrakeOn(const MotAnalogUser_T * p_user) 	{ return ((p_user->Params.UseBistateBrakePin == true) && (Debounce_GetState(&p_user->BistateBrakePin) == true)); }
static inline bool MotAnalogUser_GetIsThrottleSafetyOn(const MotAnalogUser_T * p_user) 	{ return (p_user->Params.UseThrottleSafetyPin == true) ? Debounce_GetState(&p_user->ThrottleSafetyPin) : true; }

/*
	Single thread capture and poll only. Must follow capure when using compare ThrottlePrev_Frac16
	alternatively maintain state or update prev
*/
static inline bool _MotAnalogUser_PollThrottleFallingEdge(MotAnalogUser_T * p_user)
{
	return (p_user->Params.UseThrottleEdgePin == true) ? Debounce_PollFallingEdge(&p_user->ThrottleEdgePin) : ((p_user->Throttle_Frac16 == 0U) && (p_user->ThrottlePrev_Frac16 > 0U));
}

static inline bool _MotAnalogUser_PollBrakeFallingEdge(MotAnalogUser_T * p_user)
{
	return (p_user->Params.UseBrakeEdgePin == true) ? Debounce_PollFallingEdge(&p_user->BrakeEdgePin) : ((p_user->Brake_Frac16 == 0U) && (p_user->BrakePrev_Frac16 > 0U));
}

static inline bool MotAnalogUser_PollBistateBrakeFallingEdge(MotAnalogUser_T * p_user)
{
	return ((p_user->Params.UseBistateBrakePin == true) && Debounce_PollFallingEdge(&p_user->BistateBrakePin));
}

static inline bool MotAnalogUser_PollThrottleSafetyFallingEdge(MotAnalogUser_T * p_user)
{
	return ((p_user->Params.UseThrottleSafetyPin == true) && Debounce_PollFallingEdge(&p_user->ThrottleSafetyPin));
}


/* Early throttle release detect */
static inline bool MotAnalogUser_CheckThrottleValueRelease(const MotAnalogUser_T * p_user)
{
	return (((int32_t)p_user->ThrottlePrev_Frac16 - (int32_t)p_user->Throttle_Frac16) > (65536 / 20));;
}

static inline MotAnalogUser_Direction_T MotAnalogUser_GetDirection(const MotAnalogUser_T * p_user)
{
	MotAnalogUser_Direction_T direction;

	if		(_MotAnalogUser_GetIsNeutralOn(p_user) == true) 	{ direction = MOT_ANALOG_USER_DIRECTION_NEUTRAL; }
	else if	(MotAnalogUser_GetIsForwardOn(p_user) == true) 		{ direction = MOT_ANALOG_USER_DIRECTION_FORWARD; }
	else if	(MotAnalogUser_GetIsReverseOn(p_user) == true) 		{ direction = MOT_ANALOG_USER_DIRECTION_REVERSE; }
	else 														{ direction = MOT_ANALOG_USER_DIRECTION_NEUTRAL; }

	return direction;
}

static inline MotAnalogUser_Direction_T MotAnalogUser_PollDirection(MotAnalogUser_T * p_user)
{
	MotAnalogUser_Direction_T direction;

	if(_MotAnalogUser_GetIsNeutralOn(p_user) == true)
	{
		direction = Debounce_PollRisingEdge(&p_user->NeutralPin) ? MOT_ANALOG_USER_DIRECTION_NEUTRAL_EDGE : MOT_ANALOG_USER_DIRECTION_NEUTRAL;
	}
	else if(MotAnalogUser_GetIsForwardOn(p_user) == true)
	{
		direction = Debounce_PollRisingEdge(&p_user->ForwardPin) ? MOT_ANALOG_USER_DIRECTION_FORWARD_EDGE : MOT_ANALOG_USER_DIRECTION_FORWARD;
	}
	else if(MotAnalogUser_GetIsReverseOn(p_user) == true)
	{
		direction = Debounce_PollRisingEdge(&p_user->ReversePin) ? MOT_ANALOG_USER_DIRECTION_REVERSE_EDGE : MOT_ANALOG_USER_DIRECTION_REVERSE;
	}
	else
	{
		direction = ((Debounce_PollFallingEdge(&p_user->ForwardPin) == true) || (Debounce_PollFallingEdge(&p_user->ReversePin) == true)) ?
			MOT_ANALOG_USER_DIRECTION_NEUTRAL_EDGE : MOT_ANALOG_USER_DIRECTION_NEUTRAL;
	}

	return direction;
}

/*
	If using a single return status. Forward/Reverse must be able to be detected while braking.
	Alternatively, use 2 separate return status, Direction/Cmd.

	No throttle release detect while in neutral
*/
static inline MotAnalogUser_Cmd_T MotAnalogUser_PollCmd(MotAnalogUser_T * p_user)
{
	MotAnalogUser_Direction_T direction = MotAnalogUser_PollDirection(p_user);
	MotAnalogUser_Cmd_T cmd = MOT_ANALOG_USER_CMD_PROC_ZERO;

	switch(direction)
	{
		case MOT_ANALOG_USER_DIRECTION_FORWARD_EDGE: cmd = MOT_ANALOG_USER_CMD_SET_DIRECTION_FORWARD;	break;
		case MOT_ANALOG_USER_DIRECTION_REVERSE_EDGE: cmd = MOT_ANALOG_USER_CMD_SET_DIRECTION_REVERSE;	break;
		case MOT_ANALOG_USER_DIRECTION_NEUTRAL_EDGE: cmd = MOT_ANALOG_USER_CMD_SET_NEUTRAL; 			break;
		// case MOT_ANALOG_USER_DIRECTION_NEUTRAL:
		// case MOT_ANALOG_USER_DIRECTION_REVERSE:
		// case MOT_ANALOG_USER_DIRECTION_FORWARD:
		default:
			/* Check Brake first */
			if((MotAnalogUser_GetIsBrakeOn(p_user) == true) || (MotAnalogUser_GetIsBistateBrakeOn(p_user) == true))
			{
				cmd = MOT_ANALOG_USER_CMD_SET_BRAKE;
			}
			else if((_MotAnalogUser_PollBrakeFallingEdge(p_user) == true) || (MotAnalogUser_PollBistateBrakeFallingEdge(p_user) == true))
			{
				cmd = MOT_ANALOG_USER_CMD_SET_BRAKE_RELEASE;
			}
			/* Check Direction */
			// else if(direction == MOT_ANALOG_USER_DIRECTION_NEUTRAL_EDGE)
			// {
			// 	cmd = MOT_ANALOG_USER_CMD_SET_NEUTRAL;
			// }
			else if(direction == MOT_ANALOG_USER_DIRECTION_NEUTRAL)
			{
				cmd = MOT_ANALOG_USER_CMD_PROC_NEUTRAL;
			}
			/* Check Throttle. Direction is non edge Forward or Reverse */
			else if(MotAnalogUser_GetIsThrottleSafetyOn(p_user) == true)
			{
				if(MotAnalogUser_GetIsThrottleOn(p_user) == true)
				{
					// if(MotAnalogUser_CheckThrottleValueRelease(p_user) == true) /* repeat throttle release is okay for now, otherwise track previous cmd state */
					// {
					// 	cmd = MOT_ANALOG_USER_CMD_SET_THROTTLE_RELEASE;
					// }
					// else
					{
						cmd = MOT_ANALOG_USER_CMD_SET_THROTTLE;
					}
				}
				else if(_MotAnalogUser_PollThrottleFallingEdge(p_user) == true)
				{
					cmd = MOT_ANALOG_USER_CMD_SET_THROTTLE_RELEASE;
				}
				else
				{
					cmd = MOT_ANALOG_USER_CMD_PROC_ZERO;
				}
			}
			else if(MotAnalogUser_PollThrottleSafetyFallingEdge(p_user) == true)
			{
				cmd = MOT_ANALOG_USER_CMD_SET_THROTTLE_RELEASE;
			}
			else
			{
				cmd = MOT_ANALOG_USER_CMD_PROC_ZERO; /* Direction is Forward or Reverse, no throttle or brake value */
			}
			break;
	}

	p_user->Cmd = cmd;

	return cmd;
}

static inline uint16_t MotAnalogUser_GetThrottleValue(const MotAnalogUser_T * p_user) 	{ return p_user->Throttle_Frac16; }
static inline uint16_t MotAnalogUser_GetBrakeValue(const MotAnalogUser_T * p_user) 		{ return p_user->Brake_Frac16; }
static inline uint16_t MotAnalogUser_GetThrottle(const MotAnalogUser_T * p_user) 		{ return (MotAnalogUser_GetIsThrottleOn(p_user) == true) ? p_user->Throttle_Frac16 : 0U; }
static inline uint16_t MotAnalogUser_GetBrake(const MotAnalogUser_T * p_user) 			{ return (MotAnalogUser_GetIsBrakeOn(p_user) == true) ? p_user->Brake_Frac16 : 0U; }

extern void MotAnalogUser_Init(MotAnalogUser_T * p_user);
extern void MotAnalogUser_SetParams(MotAnalogUser_T * p_user, const MotAnalogUser_Params_T * p_param);
extern void MotAnalogUser_SetBrakeRange(MotAnalogUser_T * p_user, uint16_t zero_Adcu, uint16_t max_Adcu);
extern void MotAnalogUser_SetThrottleRange(MotAnalogUser_T * p_user, uint16_t zero_Adcu, uint16_t max_Adcu);
extern void MotAnalogUser_SetBrakeAdc(MotAnalogUser_T * p_user, uint16_t zero_Adcu, uint16_t max_Adcu, bool useBrakeEdgePin);
extern void MotAnalogUser_SetThrottleAdc(MotAnalogUser_T * p_user, uint16_t zero_Adcu, uint16_t max_Adcu, bool useThrottleEdgePin);
extern void MotAnalogUser_SetBistateBrake(MotAnalogUser_T * p_user, bool useBistateBrake, uint16_t bistateBrakeIntensity_Frac16);
extern void MotAnalogUser_SetThrottleSafety(MotAnalogUser_T * p_user, bool useThrottleSafety);
extern void MotAnalogUser_SetDirectionPins(MotAnalogUser_T * p_user, MotAnalogUser_DirectionPins_T pins);
extern void MotAnalogUser_SetPinInvert(MotAnalogUser_T * p_user, MotAnalogUser_InvertPins_T invertPins);

#endif
