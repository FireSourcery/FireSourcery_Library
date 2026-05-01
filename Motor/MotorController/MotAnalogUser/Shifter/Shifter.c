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
    @brief  Strategy table — one row per wiring SKU (FNR/FR/R), bundling the
            pin-state -> Direction Decoder with the per-pin UserDIn_Mode.
*/
/******************************************************************************/
#include "Shifter.h"

/******************************************************************************/
/*
    Strategy table — one row per wiring SKU. Each row is pure data:
    an 8-entry decode truth table + the per-pin UserDIn_Mode policy.

    Decode index = (Neutral<<2) | (Forward<<1) | (Reverse<<0).
    For SKUs with disabled pins (always read 0), unreachable rows are filled
    with the SKU's safe default. R-only defaults to FORWARD when REVERSE is
    not asserted (its asymmetric semantic); FNR/FR default to NEUTRAL.
*/
/******************************************************************************/
// #define _N SHIFTER_DIRECTION_NEUTRAL
// #define _F SHIFTER_DIRECTION_FORWARD
// #define _R SHIFTER_DIRECTION_REVERSE

// static const Shifter_Direction_T DECODE_TABLE_FNR[8]    = { _N, _F, _R, _N, _N, _N, _N, _N };
// static const Shifter_Direction_T DECODE_TABLE_R_ONLY[8] = { _F, _F, _R, _R, _F, _F, _R, _R };

// /******************************************************************************/
// /*
//     Apply — the Config.PinMode mask is the SKU. It gates Init (only wired
//     pins are touched, and only when their HAL is non-NULL) and selects the
//     decode table. The Forward bit is the discriminator: R-only (no Forward
//     pin) defaults to Forward; FNR/FR share a table.
// */
// /******************************************************************************/
// static inline const Shifter_Direction_T * SelectDecodeTable(Shifter_Pins_T pins)
// {
//     return pins.Forward ? DECODE_TABLE_FNR : DECODE_TABLE_R_ONLY;
// }


static void Apply(const Shifter_T * p_shifter, Shifter_Pins_T pins)
{
    // p_shifter->P_STATE->p_DecodeTable = SelectDecodeTable(pins);
    if (pins.Forward) { UserDIn_Init(&p_shifter->FORWARD_DIN); }
    if (pins.Reverse) { UserDIn_Init(&p_shifter->REVERSE_DIN); }
    if (pins.Neutral) { UserDIn_Init(&p_shifter->NEUTRAL_DIN); }
}


/******************************************************************************/
/*
    Public
*/
/******************************************************************************/
void Shifter_InitFrom(const Shifter_T * p_shifter, const Shifter_Config_T * p_config)
{
    if (p_config != NULL) { p_shifter->P_STATE->Config = *p_config; }
    Apply(p_shifter, p_shifter->P_STATE->Config.PinMode);
    p_shifter->P_STATE->LastDirection = SHIFTER_DIRECTION_NEUTRAL;
}

// void Shifter_Init(const Shifter_T * p_shifter)
// {
//     Shifter_InitFrom(p_shifter, p_shifter->P_NVM_CONFIG);
// }

void Shifter_SetPinMode(const Shifter_T * p_shifter, Shifter_PinMode_T mode)
{
    Shifter_Pins_T pins = { .Value = (uint8_t)mode };
    p_shifter->P_STATE->Config.PinMode = pins;
    Apply(p_shifter, pins);
}


/*
    VarId/ConfigId Get/Set
*/
// int32_t Shifter_VarId_Get(const Shifter_T * p_shifter, Shifter_VarId_T id)
// {
//     int32_t value = 0;
//     switch (id)
//     {
//         case SHIFTER_VAR_DIRECTION:    value = Shifter_ResolveDirection(p_shifter);                   break;
//         case SHIFTER_VAR_FORWARD_PIN:  value = UserDIn_GetState(&p_shifter->FORWARD_DIN);         break;
//         case SHIFTER_VAR_REVERSE_PIN:  value = UserDIn_GetState(&p_shifter->REVERSE_DIN);         break;
//         case SHIFTER_VAR_NEUTRAL_PIN:  value = UserDIn_GetState(&p_shifter->NEUTRAL_DIN);         break;
//         default: break;
//     }
//     return value;
// }

// int32_t Shifter_ConfigId_Get(const Shifter_T * p_shifter, Shifter_ConfigId_T id)
// {
//     int32_t value = 0;
//     switch (id)
//     {
//         case SHIFTER_CONFIG_PIN_MODE: value = p_shifter->P_STATE->Config.PinMode.Value; break;
//         default: break;
//     }
//     return value;
// }

// void Shifter_ConfigId_Set(const Shifter_T * p_shifter, Shifter_ConfigId_T id, int32_t value)
// {
//     switch (id)
//     {
//         case SHIFTER_CONFIG_PIN_MODE: Shifter_SetPinMode(p_shifter, (Shifter_PinMode_T)value); break;
//         default: break;
//     }
// }
