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
    @file   MotAnalogUser.c
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#include "MotAnalogUser.h"
#include <string.h>

//todo move
static inline void AIn_EnableEdgePin(MotAnalogUser_AIn_T * p_aIn) { p_aIn->UseEdgePin = true; Debounce_Init(&p_aIn->EdgePin, 5U); }
static inline void ResetBrakeUnits(MotAnalogUser_T * p_user)    { Linear_ADC_Init_Scalar(&p_user->BrakeAIn.Units, p_user->Config.ThrottleZero_Adcu, p_user->Config.ThrottleMax_Adcu); }
static inline void ResetThrottleUnits(MotAnalogUser_T * p_user) { Linear_ADC_Init_Scalar(&p_user->ThrottleAIn.Units, p_user->Config.BrakeZero_Adcu, p_user->Config.BrakeMax_Adcu); }

void MotAnalogUser_Init(MotAnalogUser_T * p_user)
{
    if(p_user->CONST.P_CONFIG != NULL)
    {
        memcpy(&p_user->Config, p_user->CONST.P_CONFIG, sizeof(MotAnalogUser_Config_T));
        ResetBrakeUnits(p_user);
        ResetThrottleUnits(p_user);
    }

    Debounce_Init(&p_user->ReversePin, 5U);
    if(p_user->Config.UseForwardPin == true)            { Debounce_Init(&p_user->ForwardPin, 5U); }
    if(p_user->Config.UseNeutralPin == true)            { Debounce_Init(&p_user->NeutralPin, 5U); }
    if(p_user->Config.UseThrottleEdgePin == true)       { AIn_EnableEdgePin(&p_user->ThrottleAIn); }
    if(p_user->Config.UseBrakeEdgePin == true)          { AIn_EnableEdgePin(&p_user->BrakeAIn); }
    if(p_user->Config.UseBistateBrakePin == true)       { Debounce_Init(&p_user->BistateBrakePin, 5U); }

    p_user->ThrottleAIn.ValuePrev_Percent16 = 0U;
    p_user->ThrottleAIn.Value_Percent16 = 0U;
    p_user->BrakeAIn.Value_Percent16 = 0U;
    p_user->BrakeAIn.ValuePrev_Percent16 = 0U;
}

MotAnalogUser_Direction_T MotAnalogUser_GetDirection(const MotAnalogUser_T * p_user)
{
    MotAnalogUser_Direction_T direction;
    if      (_MotAnalogUser_GetIsNeutralOn(p_user) == true) { direction = MOT_ANALOG_USER_DIRECTION_NEUTRAL; }
    else if (MotAnalogUser_GetIsReverseOn(p_user) == true)  { direction = MOT_ANALOG_USER_DIRECTION_REVERSE; }
    else if (MotAnalogUser_GetIsForwardOn(p_user) == true)  { direction = MOT_ANALOG_USER_DIRECTION_FORWARD; }
    else                                                    { direction = MOT_ANALOG_USER_DIRECTION_NEUTRAL; }
    return direction;
}


/*
    Set Config
*/
void MotAnalogUser_SetBrakeZero(MotAnalogUser_T * p_user, uint16_t zero_Adcu)
{
    p_user->Config.BrakeZero_Adcu = zero_Adcu;
    ResetBrakeUnits(p_user);

}

void MotAnalogUser_SetThrottleZero(MotAnalogUser_T * p_user, uint16_t zero_Adcu)
{
    p_user->Config.ThrottleZero_Adcu = zero_Adcu;
    ResetThrottleUnits(p_user);
}

void MotAnalogUser_SetBrakeRange(MotAnalogUser_T * p_user, uint16_t zero_Adcu, uint16_t max_Adcu)
{
    p_user->Config.BrakeZero_Adcu = zero_Adcu;
    p_user->Config.BrakeMax_Adcu = max_Adcu;
    ResetBrakeUnits(p_user);

}

void MotAnalogUser_SetThrottleRange(MotAnalogUser_T * p_user, uint16_t zero_Adcu, uint16_t max_Adcu)
{
    p_user->Config.ThrottleZero_Adcu = zero_Adcu;
    p_user->Config.ThrottleMax_Adcu = max_Adcu;
    ResetThrottleUnits(p_user);
}

//todo range error
void MotAnalogUser_SetBrakeAIn(MotAnalogUser_T * p_user, uint16_t zero_Adcu, uint16_t max_Adcu, bool useBrakeEdgePin)
{
    MotAnalogUser_SetBrakeRange(p_user, zero_Adcu, max_Adcu);
    p_user->Config.UseBrakeEdgePin = useBrakeEdgePin;
    p_user->BrakeAIn.UseEdgePin = useBrakeEdgePin;
    if(useBrakeEdgePin == true) { AIn_EnableEdgePin(&p_user->BrakeAIn); }
}

void MotAnalogUser_SetThrottleAIn(MotAnalogUser_T * p_user, uint16_t zero_Adcu, uint16_t max_Adcu, bool useThrottleEdgePin)
{
    MotAnalogUser_SetThrottleRange(p_user, zero_Adcu, max_Adcu);
    p_user->Config.UseThrottleEdgePin = useThrottleEdgePin;
    p_user->ThrottleAIn.UseEdgePin = useThrottleEdgePin;
    if(useThrottleEdgePin == true) { AIn_EnableEdgePin(&p_user->ThrottleAIn); }
}

void MotAnalogUser_SetBistateBrake(MotAnalogUser_T * p_user, bool useBistateBrake, uint16_t bistateBrakeIntensity_Fract16)
{
    p_user->Config.UseBistateBrakePin = useBistateBrake;
    p_user->Config.BistateBrakeValue_Percent16 = bistateBrakeIntensity_Fract16;
}

void MotAnalogUser_SetDirectionPins(MotAnalogUser_T * p_user, MotAnalogUser_DirectionPins_T pins)
{
    switch(pins)
    {
        case MOT_ANALOG_USER_DIRECTION_PINS_FNR:
            p_user->Config.UseForwardPin = true;
            p_user->Config.UseNeutralPin = true;
            break;

        case MOT_ANALOG_USER_DIRECTION_PINS_FR:
            p_user->Config.UseForwardPin = true;
            p_user->Config.UseNeutralPin = false;
            break;

        case MOT_ANALOG_USER_DIRECTION_PINS_R:
            p_user->Config.UseForwardPin = false;
            p_user->Config.UseNeutralPin = false;
            break;

        default: break;
    }
}


// typedef union MotAnalogUser_InvertPins
// {
//     struct
//     {
//         uint32_t ThrottleEdge         : 1U;
//         uint32_t BrakeEdge             : 1U;
//         uint32_t Neutral             : 1U;
//         uint32_t Forward             : 1U;
//         uint32_t Reverse             : 1U;
//         uint32_t BistateBrake         : 1U;
//         uint32_t ThrottleSafety     : 1U;
//     };
//     uint8_t State;
// }
// MotAnalogUser_InvertPins_T;

// void MotAnalogUser_SetPinInvert(MotAnalogUser_T * p_user, MotAnalogUser_InvertPins_T invertPins)
// {
//     if(p_user->Config.InvertPins.State != invertPins.State)
//     {
//         p_user->Config.InvertPins.State = invertPins.State;
//         (invertPins.BistateBrake == true) ? Debounce_EnableInvert(&p_user->BistateBrakePin) : Debounce_DisableInvert(&p_user->BistateBrakePin);
//         (invertPins.Reverse == true) ? Debounce_EnableInvert(&p_user->ReversePin) : Debounce_DisableInvert(&p_user->ReversePin);
//         (invertPins.Forward == true) ? Debounce_EnableInvert(&p_user->ForwardPin) : Debounce_DisableInvert(&p_user->ForwardPin);
//         (invertPins.Neutral == true) ? Debounce_EnableInvert(&p_user->NeutralPin) : Debounce_DisableInvert(&p_user->NeutralPin);
//         (invertPins.BrakeEdge == true) ? Debounce_EnableInvert(&p_user->BrakeEdgePin) : Debounce_DisableInvert(&p_user->BrakeEdgePin);
//         (invertPins.ThrottleEdge == true) ? Debounce_EnableInvert(&p_user->ThrottleEdgePin) : Debounce_DisableInvert(&p_user->ThrottleEdgePin);
//     }
// }

