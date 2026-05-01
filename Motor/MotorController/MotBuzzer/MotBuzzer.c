/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2026 FireSourcery

    This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
*/
/******************************************************************************/
/******************************************************************************/
/*!
    @file   MotBuzzer.c
    @author FireSourcery
    @brief
*/
/******************************************************************************/
#include "MotBuzzer.h"

void MotBuzzer_Init(MotBuzzer_T * p_buzzer)
{
    if (p_buzzer->P_NVM_CONFIG != NULL) { p_buzzer->P_STATE->Config = *p_buzzer->P_NVM_CONFIG; }
    Blinky_Init(&p_buzzer->BLINKY);
    if (p_buzzer->P_STATE->Config.IsEnabled == 0U) { Blinky_Disable(&p_buzzer->BLINKY); }
}

int32_t MotBuzzer_VarId_Get(MotBuzzer_T * p_buzzer, MotBuzzer_VarId_T id)
{
    int32_t value = 0;
    switch (id)
    {
        case MOT_BUZZER_VAR_IS_ENABLED: value = MotBuzzer_IsEnabled(p_buzzer); break;
        default: break;
    }
    return value;
}

int32_t MotBuzzer_ConfigId_Get(const MotBuzzer_Config_T * p_buzzer, MotBuzzer_ConfigId_T id)
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

void MotBuzzer_ConfigId_Set(MotBuzzer_Config_T * p_buzzer, MotBuzzer_ConfigId_T id, int32_t value)
{
    switch (id)
    {
        case MOT_BUZZER_CONFIG_IS_ENABLED:      MotBuzzer_SetEnabled(p_buzzer, (value != 0));                       break;
        case MOT_BUZZER_CONFIG_EVENT_FLAGS:     p_buzzer->P_STATE->Config.EventFlags.Value = (uint16_t)value;       break;
        default: break;
    }
}
