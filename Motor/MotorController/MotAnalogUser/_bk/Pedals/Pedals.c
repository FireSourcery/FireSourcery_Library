/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2026 FireSourcery

    This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).
*/
/******************************************************************************/
/******************************************************************************/
/*!
    @file   Pedals.c
    @author FireSourcery
*/
/******************************************************************************/
#include "Pedals.h"

void Pedals_Init(const Pedals_T * p_pedals)
{
    if (p_pedals->P_NVM_CONFIG != NULL) { p_pedals->P_STATE->Config = *p_pedals->P_NVM_CONFIG; }
    UserAIn_Init(&p_pedals->THROTTLE_AIN);
    UserAIn_Init(&p_pedals->BRAKE_AIN);
}

void Pedals_SetThrottleRange(const Pedals_T * p_pedals, uint16_t zero_Adcu, uint16_t max_Adcu)
{
    p_pedals->P_STATE->Config.ThrottleAInConfig.AdcZero = zero_Adcu;
    p_pedals->P_STATE->Config.ThrottleAInConfig.AdcMax  = max_Adcu;
    UserAIn_InitFrom(&p_pedals->THROTTLE_AIN, &p_pedals->P_STATE->Config.ThrottleAInConfig);
}

void Pedals_SetBrakeRange(const Pedals_T * p_pedals, uint16_t zero_Adcu, uint16_t max_Adcu)
{
    p_pedals->P_STATE->Config.BrakeAInConfig.AdcZero = zero_Adcu;
    p_pedals->P_STATE->Config.BrakeAInConfig.AdcMax  = max_Adcu;
    UserAIn_InitFrom(&p_pedals->BRAKE_AIN, &p_pedals->P_STATE->Config.BrakeAInConfig);
}

void Pedals_SetThrottleZero(const Pedals_T * p_pedals, uint16_t zero_Adcu)
{
    Pedals_SetThrottleRange(p_pedals, zero_Adcu, p_pedals->P_STATE->Config.ThrottleAInConfig.AdcMax);
}

void Pedals_SetBrakeZero(const Pedals_T * p_pedals, uint16_t zero_Adcu)
{
    Pedals_SetBrakeRange(p_pedals, zero_Adcu, p_pedals->P_STATE->Config.BrakeAInConfig.AdcMax);
}

int32_t Pedals_VarId_Get(const Pedals_T * p_pedals, Pedals_VarId_T id)
{
    int32_t value = 0;
    switch (id)
    {
        case PEDALS_VAR_THROTTLE:          value = Pedals_GetThrottle(p_pedals);           break;
        case PEDALS_VAR_BRAKE:             value = Pedals_GetBrake(p_pedals);              break;
        case PEDALS_VAR_THROTTLE_RAW:      value = Pedals_GetThrottleRaw(p_pedals);        break;
        case PEDALS_VAR_BRAKE_RAW:         value = Pedals_GetBrakeRaw(p_pedals);           break;
        case PEDALS_VAR_THROTTLE_EDGE_PIN: value = (p_pedals->THROTTLE_AIN.P_EDGE_PIN != NULL)
                                                 ? UserDIn_GetState(p_pedals->THROTTLE_AIN.P_EDGE_PIN) : 0; break;
        case PEDALS_VAR_BRAKE_EDGE_PIN:    value = (p_pedals->BRAKE_AIN.P_EDGE_PIN != NULL)
                                                 ? UserDIn_GetState(p_pedals->BRAKE_AIN.P_EDGE_PIN) : 0; break;
        default: break;
    }
    return value;
}

int32_t Pedals_ConfigId_Get(const Pedals_T * p_pedals, Pedals_ConfigId_T id)
{
    int32_t value = 0;
    switch (id)
    {
        case PEDALS_CONFIG_THROTTLE_ZERO_ADCU: value = p_pedals->P_STATE->Config.ThrottleAInConfig.AdcZero; break;
        case PEDALS_CONFIG_THROTTLE_MAX_ADCU:  value = p_pedals->P_STATE->Config.ThrottleAInConfig.AdcMax;  break;
        case PEDALS_CONFIG_BRAKE_ZERO_ADCU:    value = p_pedals->P_STATE->Config.BrakeAInConfig.AdcZero;    break;
        case PEDALS_CONFIG_BRAKE_MAX_ADCU:     value = p_pedals->P_STATE->Config.BrakeAInConfig.AdcMax;     break;
        default: break;
    }
    return value;
}

void Pedals_ConfigId_Set(const Pedals_T * p_pedals, Pedals_ConfigId_T id, int32_t value)
{
    uint16_t v = (uint16_t)value;
    switch (id)
    {
        case PEDALS_CONFIG_THROTTLE_ZERO_ADCU:
            Pedals_SetThrottleRange(p_pedals, v, p_pedals->P_STATE->Config.ThrottleAInConfig.AdcMax); break;
        case PEDALS_CONFIG_THROTTLE_MAX_ADCU:
            Pedals_SetThrottleRange(p_pedals, p_pedals->P_STATE->Config.ThrottleAInConfig.AdcZero, v); break;
        case PEDALS_CONFIG_BRAKE_ZERO_ADCU:
            Pedals_SetBrakeRange(p_pedals, v, p_pedals->P_STATE->Config.BrakeAInConfig.AdcMax); break;
        case PEDALS_CONFIG_BRAKE_MAX_ADCU:
            Pedals_SetBrakeRange(p_pedals, p_pedals->P_STATE->Config.BrakeAInConfig.AdcZero, v); break;
        default: break;
    }
}
