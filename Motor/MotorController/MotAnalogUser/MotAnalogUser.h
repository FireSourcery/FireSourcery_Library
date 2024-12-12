/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

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
    @file   MotAnalogUser.h
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

/*
    Input state/variation combined into single status
*/
typedef enum MotAnalogUser_Cmd
{
    MOT_ANALOG_USER_CMD_SET_BRAKE,
    MOT_ANALOG_USER_CMD_SET_THROTTLE,
    MOT_ANALOG_USER_CMD_SET_THROTTLE_RELEASE,
    MOT_ANALOG_USER_CMD_SET_BRAKE_RELEASE,
    MOT_ANALOG_USER_CMD_PROC_ZERO,    /* No Brake/Throttle input */
    MOT_ANALOG_USER_CMD_SET_NEUTRAL,
    MOT_ANALOG_USER_CMD_PROC_NEUTRAL,
    MOT_ANALOG_USER_CMD_SET_DIRECTION_FORWARD,
    MOT_ANALOG_USER_CMD_SET_DIRECTION_REVERSE,
}
MotAnalogUser_Cmd_T;

typedef enum MotAnalogUser_Direction
{
    MOT_ANALOG_USER_DIRECTION_NEUTRAL,
    MOT_ANALOG_USER_DIRECTION_FORWARD,
    MOT_ANALOG_USER_DIRECTION_REVERSE,
    MOT_ANALOG_USER_DIRECTION_NEUTRAL_EDGE,
    MOT_ANALOG_USER_DIRECTION_FORWARD_EDGE,
    MOT_ANALOG_USER_DIRECTION_REVERSE_EDGE,
}
MotAnalogUser_Direction_T;

typedef enum MotAnalogUser_DirectionPins
{
    MOT_ANALOG_USER_DIRECTION_PINS_FNR,
    MOT_ANALOG_USER_DIRECTION_PINS_FR,
    MOT_ANALOG_USER_DIRECTION_PINS_R,
}
MotAnalogUser_DirectionPins_T;

/* AnalogInput Sub-Module, todo move to ain */
typedef struct MotAnalogUser_AIn
{
    Debounce_T EdgePin;
    bool UseEdgePin;        /* Use digital pin to determine AIn Threshold. */
    Linear_T Units;
    uint16_t Value_Percent16;
    uint16_t ValuePrev_Percent16;
}
MotAnalogUser_AIn_T;

typedef struct MotAnalogUser_Config
{
    uint16_t ThrottleZero_Adcu;
    uint16_t ThrottleMax_Adcu;
    uint16_t BrakeZero_Adcu;
    uint16_t BrakeMax_Adcu;

    union
    {
        struct
        {
            uint8_t UseThrottleEdgePin  : 1U; /* Nv Memory instance, loaded to AIn struct */
            uint8_t UseBrakeEdgePin     : 1U;
            uint8_t UseNeutralPin       : 1U;
            uint8_t UseForwardPin       : 1U;
            uint8_t UseBistateBrakePin  : 1U;
        };
        uint8_t PinsSelect;
    };


    uint16_t BistateBrakeValue_Percent16;

    // bool UseErrorAdcuRange; //report error if adcu read falls outside Zero_Adcu and Max_Adcu
    // MotAnalogUser_InvertPins_T InvertPins;
}
MotAnalogUser_Config_T;

/*
    Activate ADC outside module
*/
typedef const struct MotAnalogUser_Const
{
    const MotAnalogUser_Config_T * const P_CONFIG;
}
MotAnalogUser_Const_T;

typedef struct MotAnalogUser
{
    MotAnalogUser_Const_T CONST;
    MotAnalogUser_Config_T Config;
    MotAnalogUser_AIn_T ThrottleAIn;
    MotAnalogUser_AIn_T BrakeAIn;
    Debounce_T ReversePin;
    Debounce_T ForwardPin;
    Debounce_T NeutralPin;
    Debounce_T BistateBrakePin;
    MotAnalogUser_Cmd_T Cmd; /* Store last active Cmd */
}
MotAnalogUser_T;


#define MOT_ANALOG_USER_INIT(p_BrakePinHal, BrakePinId, p_ThrottlePinHal, ThrottlePinId, p_ForwardPinHal, ForwardPinId, p_ReversePinHal, ReversePinId, p_BistateBrakePinHal, BistateBrakePinId, InputPinsInvert, p_Millis, p_Config)    \
{                                                                                                                       \
    .CONST              = { .P_CONFIG = p_Config, },                                                                    \
    .ThrottleAIn        = { .EdgePin = DEBOUNCE_INIT(p_ThrottlePinHal,  ThrottlePinId,  InputPinsInvert, p_Millis), },  \
    .BrakeAIn           = { .EdgePin = DEBOUNCE_INIT(p_BrakePinHal,     BrakePinId,     InputPinsInvert, p_Millis), },  \
    .ForwardPin         = DEBOUNCE_INIT(p_ForwardPinHal,        ForwardPinId,           InputPinsInvert, p_Millis ),    \
    .ReversePin         = DEBOUNCE_INIT(p_ReversePinHal,        ReversePinId,           InputPinsInvert, p_Millis ),    \
    .BistateBrakePin    = DEBOUNCE_INIT(p_BistateBrakePinHal,   BistateBrakePinId,      InputPinsInvert, p_Millis ),    \
    .NeutralPin         = DEBOUNCE_INIT(0, 0, 0, 0),                                                                    \
}

/*
    AIn Sub Module
*/
static inline void _MotAnalogUser_AIn_CaptureValue(MotAnalogUser_AIn_T * p_aIn, uint16_t value_Adcu)
{
    if(p_aIn->UseEdgePin == true) { Debounce_CaptureState(&p_aIn->EdgePin); }
    if((p_aIn->UseEdgePin == false) || (Debounce_GetState(&p_aIn->EdgePin) == true))
    {
        p_aIn->Value_Percent16 = ((uint32_t)Linear_ADC_Percent16(&p_aIn->Units, value_Adcu) + p_aIn->ValuePrev_Percent16) / 2U;
        p_aIn->ValuePrev_Percent16 = p_aIn->Value_Percent16;
    }
}

/* Edge acts as threshold - Both on for on, one off for off */
static inline bool _MotAnalogUser_AIn_GetIsOn(const MotAnalogUser_AIn_T * p_aIn)
{
    return (p_aIn->UseEdgePin == true) ? (Debounce_GetState(&p_aIn->EdgePin) && (p_aIn->Value_Percent16 > 0U)) : (p_aIn->Value_Percent16 > 0U);
}

static inline bool _MotAnalogUser_AIn_PollFallingEdge(MotAnalogUser_AIn_T * p_aIn)
{
    bool isValueFallingEdge = ((p_aIn->Value_Percent16 == 0U) && (p_aIn->ValuePrev_Percent16 > 0U));
    bool isFallingEdge;

    if(p_aIn->UseEdgePin == true)
    {
        /* Once 1 part detects falling edge, disable the other from repeat detect */
        if(Debounce_PollFallingEdge(&p_aIn->EdgePin) == true)   { p_aIn->Value_Percent16 = 0U; p_aIn->ValuePrev_Percent16 = 0U; isFallingEdge = true; }
        else if(isValueFallingEdge == true)                     { p_aIn->EdgePin.DebouncedState = false; p_aIn->EdgePin.DebouncedState = false; isFallingEdge = true; }
        else                                                    { isFallingEdge = false; }
    }
    else
    {
        isFallingEdge = isValueFallingEdge;
    }

    return isFallingEdge;
    // return (p_aIn->UseEdgePin == true) ? (Debounce_PollFallingEdge(&p_aIn->EdgePin)) : ((p_aIn->Value_U16 == 0U) && (p_aIn->ValuePrev_U16 > 0U));
}

static inline uint16_t _MotAnalogUser_AIn_GetValue(const MotAnalogUser_AIn_T * p_aIn)
{
    return (_MotAnalogUser_AIn_GetIsOn(p_aIn) == true) ? p_aIn->Value_Percent16 : 0U; /* Check IsOn again, If !IsOn Value_U16 remains prev captured value*/
}

/*
    Capture Inputs
*/
/*
    ADCU capture from outside module. Alternatively, map pointer.
*/
static inline void MotAnalogUser_CaptureThrottleValue(MotAnalogUser_T * p_user, uint16_t throttle_Adcu) { _MotAnalogUser_AIn_CaptureValue(&p_user->ThrottleAIn, throttle_Adcu); }
static inline void MotAnalogUser_CaptureBrakeValue(MotAnalogUser_T * p_user, uint16_t brake_Adcu)       { _MotAnalogUser_AIn_CaptureValue(&p_user->BrakeAIn, brake_Adcu); }

static inline void MotAnalogUser_CapturePins(MotAnalogUser_T * p_user)
{
    Debounce_CaptureState(&p_user->ReversePin);
    if(p_user->Config.UseForwardPin == true)        { Debounce_CaptureState(&p_user->ForwardPin); }
    if(p_user->Config.UseNeutralPin == true)        { Debounce_CaptureState(&p_user->NeutralPin); }
    if(p_user->Config.UseBistateBrakePin == true)   { Debounce_CaptureState(&p_user->BistateBrakePin); }
}

/*
    Calling module is responsible for ADC activation and passes complete results
*/
static inline void MotAnalogUser_CaptureInput(MotAnalogUser_T * p_user, uint16_t throttle_Adcu, uint16_t brake_Adcu)
{
    MotAnalogUser_CapturePins(p_user);
    MotAnalogUser_CaptureThrottleValue(p_user, throttle_Adcu);
    MotAnalogUser_CaptureBrakeValue(p_user, brake_Adcu);
}

/*
    BiState/Handbrake
*/
static inline bool _MotAnalogUser_PollBistateBrakeFallingEdge(MotAnalogUser_T * p_user)
{
    return ((p_user->Config.UseBistateBrakePin == true) && Debounce_PollFallingEdge(&p_user->BistateBrakePin));
}

/* Default off when not used */
static inline bool MotAnalogUser_GetIsBistateBrakeOn(const MotAnalogUser_T * p_user)
{
    return ((p_user->Config.UseBistateBrakePin == true) && (Debounce_GetState(&p_user->BistateBrakePin) == true));
}

/*
    Direction
*/
static inline bool _MotAnalogUser_GetIsNeutralOn(const MotAnalogUser_T * p_user)    { return (p_user->Config.UseNeutralPin == true) && (Debounce_GetState(&p_user->NeutralPin) == true); }
static inline bool MotAnalogUser_GetIsForwardOn(const MotAnalogUser_T * p_user)     { return (p_user->Config.UseForwardPin == true) ? Debounce_GetState(&p_user->ForwardPin) : !Debounce_GetState(&p_user->ReversePin); }
static inline bool MotAnalogUser_GetIsReverseOn(const MotAnalogUser_T * p_user)     { return Debounce_GetState(&p_user->ReversePin); }

static inline MotAnalogUser_Direction_T MotAnalogUser_PollDirection(MotAnalogUser_T * p_user)
{
    MotAnalogUser_Direction_T direction;

    if(_MotAnalogUser_GetIsNeutralOn(p_user) == true)
    {
        direction = Debounce_PollRisingEdge(&p_user->NeutralPin) ? MOT_ANALOG_USER_DIRECTION_NEUTRAL_EDGE : MOT_ANALOG_USER_DIRECTION_NEUTRAL;
    }
    else if(MotAnalogUser_GetIsReverseOn(p_user) == true)
    {
        direction = Debounce_PollRisingEdge(&p_user->ReversePin) ? MOT_ANALOG_USER_DIRECTION_REVERSE_EDGE : MOT_ANALOG_USER_DIRECTION_REVERSE;
    }
    else if(MotAnalogUser_GetIsForwardOn(p_user) == true)
    {
        direction = Debounce_PollRisingEdge(&p_user->ForwardPin) ? MOT_ANALOG_USER_DIRECTION_FORWARD_EDGE : MOT_ANALOG_USER_DIRECTION_FORWARD;
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

    No throttle release detect while in neutral.

    _MotAnalogUser_AIn_PollFallingEdge must follow capture when using compare Prev_Frac16
*/
static inline MotAnalogUser_Cmd_T MotAnalogUser_PollCmd(MotAnalogUser_T * p_user)
{
    MotAnalogUser_Direction_T direction = MotAnalogUser_PollDirection(p_user);
    MotAnalogUser_Cmd_T cmd = MOT_ANALOG_USER_CMD_PROC_ZERO;

    switch(direction)
    {
        case MOT_ANALOG_USER_DIRECTION_FORWARD_EDGE: cmd = MOT_ANALOG_USER_CMD_SET_DIRECTION_FORWARD;    break;
        case MOT_ANALOG_USER_DIRECTION_REVERSE_EDGE: cmd = MOT_ANALOG_USER_CMD_SET_DIRECTION_REVERSE;    break;
        // case MOT_ANALOG_USER_DIRECTION_NEUTRAL_EDGE: cmd = MOT_ANALOG_USER_CMD_SET_NEUTRAL;             break;
        // case MOT_ANALOG_USER_DIRECTION_NEUTRAL:
        // case MOT_ANALOG_USER_DIRECTION_REVERSE:
        // case MOT_ANALOG_USER_DIRECTION_FORWARD:
        default:
            /* Check Brake first */
            if((_MotAnalogUser_AIn_GetIsOn(&p_user->BrakeAIn) == true) || (MotAnalogUser_GetIsBistateBrakeOn(p_user) == true))
            {
                cmd = MOT_ANALOG_USER_CMD_SET_BRAKE;
                //check throttle active error
            }
            /* Both Brakes are off. possible release run twice? */
            else if((_MotAnalogUser_AIn_PollFallingEdge(&p_user->BrakeAIn) == true) || (_MotAnalogUser_PollBistateBrakeFallingEdge(p_user) == true))
            {
                cmd = MOT_ANALOG_USER_CMD_SET_BRAKE_RELEASE;
            }
            /* Check Direction */
            else if(direction == MOT_ANALOG_USER_DIRECTION_NEUTRAL_EDGE)                { cmd = MOT_ANALOG_USER_CMD_SET_NEUTRAL; }
            else if(direction == MOT_ANALOG_USER_DIRECTION_NEUTRAL)                     { cmd = MOT_ANALOG_USER_CMD_PROC_NEUTRAL; }
            /* Check Throttle. Direction is non edge Forward or Reverse */
            else if(_MotAnalogUser_AIn_GetIsOn(&p_user->ThrottleAIn) == true)           { cmd = MOT_ANALOG_USER_CMD_SET_THROTTLE; }
            else if(_MotAnalogUser_AIn_PollFallingEdge(&p_user->ThrottleAIn) == true)   { cmd = MOT_ANALOG_USER_CMD_SET_THROTTLE_RELEASE; }
            /* Direction is Forward or Reverse, no throttle or brake value */
            else                                                                        { cmd = MOT_ANALOG_USER_CMD_PROC_ZERO; }
            break;
    }

    p_user->Cmd = cmd;

    return cmd;
}

/* Optional separate Brake polling */
static inline bool MotAnalogUser_PollBrakePins(MotAnalogUser_T * p_user)
{
    if(p_user->Config.UseBrakeEdgePin == true)      { Debounce_CaptureState(&p_user->BrakeAIn.EdgePin); }
    if(p_user->Config.UseBistateBrakePin == true)   { Debounce_CaptureState(&p_user->BistateBrakePin); }
    return ((Debounce_GetState(&p_user->BrakeAIn.EdgePin) == true) || (Debounce_GetState(&p_user->BistateBrakePin) == true)) ;
}

static inline uint16_t MotAnalogUser_GetThrottle(const MotAnalogUser_T * p_user)
{
    return _MotAnalogUser_AIn_GetValue(&p_user->ThrottleAIn);
}

static inline uint16_t MotAnalogUser_GetBrake(const MotAnalogUser_T * p_user)
{
    uint16_t brakeValue = _MotAnalogUser_AIn_GetValue(&p_user->BrakeAIn);
    if((MotAnalogUser_GetIsBistateBrakeOn(p_user) == true) && (p_user->Config.BistateBrakeValue_Percent16 > brakeValue))
    {
        brakeValue = p_user->Config.BistateBrakeValue_Percent16;
    }
    return brakeValue;
}

extern void MotAnalogUser_Init(MotAnalogUser_T * p_user);
extern MotAnalogUser_Direction_T MotAnalogUser_GetDirection(const MotAnalogUser_T * p_user);
extern void MotAnalogUser_SetBrakeZero(MotAnalogUser_T * p_user, uint16_t zero_Adcu);
extern void MotAnalogUser_SetThrottleZero(MotAnalogUser_T * p_user, uint16_t zero_Adcu);
extern void MotAnalogUser_SetBrakeRange(MotAnalogUser_T * p_user, uint16_t zero_Adcu, uint16_t max_Adcu);
extern void MotAnalogUser_SetThrottleRange(MotAnalogUser_T * p_user, uint16_t zero_Adcu, uint16_t max_Adcu);
extern void MotAnalogUser_SetBrakeAIn(MotAnalogUser_T * p_user, uint16_t zero_Adcu, uint16_t max_Adcu, bool useBrakeEdgePin);
extern void MotAnalogUser_SetThrottleAIn(MotAnalogUser_T * p_user, uint16_t zero_Adcu, uint16_t max_Adcu, bool useThrottleEdgePin);
extern void MotAnalogUser_SetBistateBrake(MotAnalogUser_T * p_user, bool useBistateBrake, uint16_t bistateBrakeIntensity_Frac16);
extern void MotAnalogUser_SetDirectionPins(MotAnalogUser_T * p_user, MotAnalogUser_DirectionPins_T pins);

#endif
