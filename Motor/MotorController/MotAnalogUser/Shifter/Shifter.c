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
    @brief  Decode table per wiring variant [Shifter_PinMode_T].
*/
/******************************************************************************/
#include "Shifter.h"

/******************************************************************************/
/*
    Indexed by [Shifter_Pins_T].
    FNR: Forward or Reverse alone, otherwise Neutral.
    R-only: the Reverse level alone. Released is Forward; Forward and Neutral pins have no effect.
*/
/******************************************************************************/
#define _N SHIFTER_DIRECTION_NEUTRAL
#define _F SHIFTER_DIRECTION_FORWARD
#define _R SHIFTER_DIRECTION_REVERSE

static const Shifter_Direction_T DECODE_TABLE_FNR[8]    = { _N, _R, _F, _N, _N, _N, _N, _N };
static const Shifter_Direction_T DECODE_TABLE_R_ONLY[8] = { _F, _R, _F, _R, _F, _R, _F, _R };

static inline const Shifter_Direction_T * SelectDecodeTable(Shifter_PinMode_T mode)
{
    return (mode == SHIFTER_PIN_MODE_R) ? DECODE_TABLE_R_ONLY : DECODE_TABLE_FNR;
}

/******************************************************************************/
/*
    Public
*/
/******************************************************************************/
void Shifter_InitFrom(const Shifter_T * p_shifter, const Shifter_Config_T * p_config)
{
    if (p_config != NULL) { p_shifter->P_STATE->Config = *p_config; }
    UserDIn_Init(&p_shifter->FORWARD_DIN);
    UserDIn_Init(&p_shifter->REVERSE_DIN);
#if defined(SHIFTER_NEUTRAL_PIN_ENABLE)
    UserDIn_Init(&p_shifter->NEUTRAL_DIN);
#endif
    Shifter_SetPinMode(p_shifter, p_shifter->P_STATE->Config.PinMode);
    p_shifter->P_STATE->LastDirection = SHIFTER_DIRECTION_NEUTRAL;
}

// void Shifter_Init(const Shifter_T * p_shifter)
// {
//     Shifter_InitFrom(p_shifter, p_shifter->P_NVM_CONFIG);
// }

void Shifter_SetPinMode(const Shifter_T * p_shifter, Shifter_PinMode_T mode)
{
    p_shifter->P_STATE->Config.PinMode = mode;
    p_shifter->P_STATE->p_DecodeTable = SelectDecodeTable(mode);
}


/*
    VarId/ConfigId Get
*/
int32_t Shifter_VarId_Get(const Shifter_T * p_shifter, Shifter_VarId_T id)
{
    int32_t value = 0;
    switch (id)
    {
        case SHIFTER_VAR_DIRECTION:    value = Shifter_ResolveDirection(p_shifter);               break;
        case SHIFTER_VAR_FORWARD_PIN:  value = UserDIn_GetState(&p_shifter->FORWARD_DIN);         break;
        case SHIFTER_VAR_REVERSE_PIN:  value = UserDIn_GetState(&p_shifter->REVERSE_DIN);         break;
#if defined(SHIFTER_NEUTRAL_PIN_ENABLE)
        case SHIFTER_VAR_NEUTRAL_PIN:  value = UserDIn_GetState(&p_shifter->NEUTRAL_DIN);         break;
#endif
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
    }
    return value;
}
