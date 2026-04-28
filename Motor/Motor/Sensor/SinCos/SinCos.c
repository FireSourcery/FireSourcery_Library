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
    @file   SinCos.c
    @author FireSourcery
*/
/******************************************************************************/
#include "SinCos.h"

#include "Peripheral/Analog/Analog_Reference.h"
#include <string.h>


void SinCos_State_ResetUnits(SinCos_State_T * p_state)
{
    Linear_Q16_Init(&p_state->UnitsAngle, p_state->Config.Min_Adcu, p_state->Config.Max_Adcu);
}

void SinCos_Init(const SinCos_T * p_sincos)
{
    if (p_sincos->P_NVM_CONFIG != NULL)
    {
        memcpy(&p_sincos->P_STATE->Config, p_sincos->P_NVM_CONFIG, sizeof(SinCos_Config_T));
    }
    SinCos_State_ResetUnits(p_sincos->P_STATE);
    p_sincos->P_STATE->Angle = 0;
}


void SinCos_Config_SetAdc(SinCos_State_T * p_state, uint16_t min_Adcu, uint16_t max_Adcu, uint16_t min_mV, uint16_t max_mV)
{
    p_state->Config.Min_Adcu = min_Adcu;
    p_state->Config.Max_Adcu = max_Adcu;
    p_state->Config.Min_MilliV = min_mV;
    p_state->Config.Max_MilliV = max_mV;
    SinCos_State_ResetUnits(p_state);
}

void SinCos_Config_SetAdc_mV(SinCos_State_T * p_state, uint16_t adcVref_mV, uint16_t min_mV, uint16_t max_mV)
{
    uint16_t min_Adcu = (uint32_t)min_mV * ANALOG_REFERENCE.ADC_MAX / adcVref_mV;
    uint16_t max_Adcu = (uint32_t)max_mV * ANALOG_REFERENCE.ADC_MAX / adcVref_mV;
    SinCos_Config_SetAdc(p_state, min_Adcu, max_Adcu, min_mV, max_mV);
}

void SinCos_Config_SetAngleRatio(SinCos_State_T * p_state, uint16_t electricalRotationsRatio)
{
    p_state->Config.ElectricalRotationsRatio = electricalRotationsRatio;
}

/*
    Capture sensor angle while rotor is held at electrical 0 — record decoded raw as offset.
*/
void SinCos_CalibrateAngleOffset(SinCos_State_T * p_state, uint16_t sin_Adcu, uint16_t cos_Adcu)
{
    p_state->Config.AngleOffset = _SinCos_DecodeAngle(p_state, sin_Adcu, cos_Adcu);
}

/*
    Call after AngleOffset, with rotor held at electrical +120° (CCW reference).
    Positive decoded angle => sensor rotates CCW-positive natively.
*/
void SinCos_CalibrateCcwPositive(SinCos_State_T * p_state, uint16_t sin_Adcu, uint16_t cos_Adcu)
{
    angle16_t angle = _SinCos_DecodeAngle(p_state, sin_Adcu, cos_Adcu) - p_state->Config.AngleOffset;
    p_state->Config.IsCcwPositive = ((int16_t)angle > 0);
}


/******************************************************************************/
/*
    Var ID
*/
/******************************************************************************/
int SinCos_ConfigId_Get(const SinCos_Config_T * p_state, SinCos_ConfigId_T id)
{
    switch (id)
    {
        case SIN_COS_CONFIG_MIN_ADCU:                  return p_state->Min_Adcu;
        case SIN_COS_CONFIG_MAX_ADCU:                  return p_state->Max_Adcu;
        case SIN_COS_CONFIG_MIN_MILLIV:                return p_state->Min_MilliV;
        case SIN_COS_CONFIG_MAX_MILLIV:                return p_state->Max_MilliV;
        case SIN_COS_CONFIG_ANGLE_OFFSET:              return (int16_t)p_state->AngleOffset;
        case SIN_COS_CONFIG_IS_CCW_POSITIVE:           return p_state->IsCcwPositive;
        case SIN_COS_CONFIG_ELECTRICAL_ROTATIONS_RATIO: return p_state->ElectricalRotationsRatio;
        default: return 0;
    }
}

void SinCos_ConfigId_Set(SinCos_Config_T * p_state, SinCos_ConfigId_T id, int value)
{
    switch (id)
    {
        case SIN_COS_CONFIG_MIN_ADCU:                  p_state->Min_Adcu = value; SinCos_State_ResetUnits(p_state); break;
        case SIN_COS_CONFIG_MAX_ADCU:                  p_state->Max_Adcu = value; SinCos_State_ResetUnits(p_state); break;
        case SIN_COS_CONFIG_MIN_MILLIV:                p_state->Min_MilliV = value; break;
        case SIN_COS_CONFIG_MAX_MILLIV:                p_state->Max_MilliV = value; break;
        case SIN_COS_CONFIG_ANGLE_OFFSET:              p_state->AngleOffset = (angle16_t)value; break;
        case SIN_COS_CONFIG_IS_CCW_POSITIVE:           p_state->IsCcwPositive = (bool)value; break;
        case SIN_COS_CONFIG_ELECTRICAL_ROTATIONS_RATIO: p_state->ElectricalRotationsRatio = value; break;
        default: break;
    }
}
