/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2026 FireSourcery

    This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).
*/
/******************************************************************************/
/******************************************************************************/
/*!
    @file   Shifter.c
    @author FireSourcery
*/
/******************************************************************************/
#include "Shifter.h"

static void _ApplyPinMode(const Shifter_T * p_shifter, Shifter_PinMode_T mode)
{
    /* Pin presence is encoded in UserDIn Mode: DISABLED pins short-circuit
       PollEdge/GetState. FNR uses all three; FR disables Neutral; R disables
       both Forward and Neutral. */
    switch (mode)
    {
        case SHIFTER_PIN_MODE_FNR:
            p_shifter->FORWARD_DIN.P_STATE->Mode = USER_DIN_MODE_NORMAL;
            p_shifter->REVERSE_DIN.P_STATE->Mode = USER_DIN_MODE_NORMAL;
            p_shifter->NEUTRAL_DIN.P_STATE->Mode = USER_DIN_MODE_NORMAL;
            break;
        case SHIFTER_PIN_MODE_FR:
            p_shifter->FORWARD_DIN.P_STATE->Mode = USER_DIN_MODE_NORMAL;
            p_shifter->REVERSE_DIN.P_STATE->Mode = USER_DIN_MODE_NORMAL;
            p_shifter->NEUTRAL_DIN.P_STATE->Mode = USER_DIN_MODE_DISABLED;
            break;
        case SHIFTER_PIN_MODE_R:
            p_shifter->FORWARD_DIN.P_STATE->Mode = USER_DIN_MODE_DISABLED;
            p_shifter->REVERSE_DIN.P_STATE->Mode = USER_DIN_MODE_NORMAL;
            p_shifter->NEUTRAL_DIN.P_STATE->Mode = USER_DIN_MODE_DISABLED;
            break;
        default: break;
    }
}

void Shifter_Init(const Shifter_T * p_shifter)
{
    if (p_shifter->P_NVM_CONFIG != NULL) { p_shifter->P_STATE->Config = *p_shifter->P_NVM_CONFIG; }
    UserDIn_Init(&p_shifter->FORWARD_DIN);
    UserDIn_Init(&p_shifter->REVERSE_DIN);
    UserDIn_Init(&p_shifter->NEUTRAL_DIN);
    _ApplyPinMode(p_shifter, p_shifter->P_STATE->Config.PinMode);
    p_shifter->P_STATE->LastDirection = SHIFTER_DIRECTION_NEUTRAL;
}

void Shifter_SetPinMode(const Shifter_T * p_shifter, Shifter_PinMode_T mode)
{
    p_shifter->P_STATE->Config.PinMode = mode;
    _ApplyPinMode(p_shifter, mode);
}

int32_t Shifter_VarId_Get(const Shifter_T * p_shifter, Shifter_VarId_T id)
{
    int32_t value = 0;
    switch (id)
    {
        case SHIFTER_VAR_DIRECTION:    value = Shifter_GetDirection(p_shifter);                   break;
        case SHIFTER_VAR_FORWARD_PIN:  value = UserDIn_GetState(&p_shifter->FORWARD_DIN);         break;
        case SHIFTER_VAR_REVERSE_PIN:  value = UserDIn_GetState(&p_shifter->REVERSE_DIN);         break;
        case SHIFTER_VAR_NEUTRAL_PIN:  value = UserDIn_GetState(&p_shifter->NEUTRAL_DIN);         break;
        default: break;
    }
    return value;
}

int32_t Shifter_ConfigId_Get(const Shifter_T * p_shifter, Shifter_ConfigId_T id)
{
    int32_t value = 0;
    switch (id)
    {
        case SHIFTER_CONFIG_PIN_MODE: value = p_shifter->P_STATE->Config.PinMode; break;
        default: break;
    }
    return value;
}

void Shifter_ConfigId_Set(const Shifter_T * p_shifter, Shifter_ConfigId_T id, int32_t value)
{
    switch (id)
    {
        case SHIFTER_CONFIG_PIN_MODE: Shifter_SetPinMode(p_shifter, (Shifter_PinMode_T)value); break;
        default: break;
    }
}
