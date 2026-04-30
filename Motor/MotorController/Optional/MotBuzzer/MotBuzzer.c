/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2026 FireSourcery

    This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).
*/
/******************************************************************************/
/******************************************************************************/
/*!
    @file   MotBuzzer.c
    @author FireSourcery
*/
/******************************************************************************/
#include "MotBuzzer.h"

void MotBuzzer_Init(const MotBuzzer_T * p_buzzer)
{
    if (p_buzzer->P_NVM_CONFIG != NULL) { p_buzzer->P_STATE->Config = *p_buzzer->P_NVM_CONFIG; }
    Blinky_Init(&p_buzzer->BLINKY);
    if (p_buzzer->P_STATE->Config.IsEnabled == 0U) { Blinky_Disable(&p_buzzer->BLINKY); }
}

void MotBuzzer_SetEnabled(const MotBuzzer_T * p_buzzer, bool isEnabled)
{
    p_buzzer->P_STATE->Config.IsEnabled = isEnabled;
    if (isEnabled == false) { Blinky_Stop(&p_buzzer->BLINKY); Blinky_Disable(&p_buzzer->BLINKY); }
    else                    { Blinky_Enable(&p_buzzer->BLINKY); }
}

void MotBuzzer_SetEventFlags(const MotBuzzer_T * p_buzzer, MotBuzzer_OptionFlags_T flags)
{
    p_buzzer->P_STATE->Config.EventFlags = flags;
}

int32_t MotBuzzer_VarId_Get(const MotBuzzer_T * p_buzzer, MotBuzzer_VarId_T id)
{
    int32_t value = 0;
    switch (id)
    {
        case MOT_BUZZER_VAR_IS_ENABLED: value = MotBuzzer_IsEnabled(p_buzzer); break;
        default: break;
    }
    return value;
}

int32_t MotBuzzer_ConfigId_Get(const MotBuzzer_T * p_buzzer, MotBuzzer_ConfigId_T id)
{
    int32_t value = 0;
    switch (id)
    {
        case MOT_BUZZER_CONFIG_IS_ENABLED:      value = p_buzzer->P_STATE->Config.IsEnabled;            break;
        case MOT_BUZZER_CONFIG_EVENT_FLAGS:     value = p_buzzer->P_STATE->Config.EventFlags.Value;     break;
        default: break;
    }
    return value;
}

void MotBuzzer_ConfigId_Set(const MotBuzzer_T * p_buzzer, MotBuzzer_ConfigId_T id, int32_t value)
{
    switch (id)
    {
        case MOT_BUZZER_CONFIG_IS_ENABLED:      MotBuzzer_SetEnabled(p_buzzer, (value != 0));                       break;
        case MOT_BUZZER_CONFIG_EVENT_FLAGS:     p_buzzer->P_STATE->Config.EventFlags.Value = (uint16_t)value;       break;
        default: break;
    }
}
