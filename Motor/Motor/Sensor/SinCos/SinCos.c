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
    @file   SinCos.h
    @author FireSourcery
    @brief

*/
/******************************************************************************/
#include "SinCos.h"
#include "Config.h"

#include "Peripheral/Analog/AnalogReference.h"
#include <string.h>

void SinCos_Init(SinCos_T * p_sincos)
{
    if(p_sincos->CONST.P_CONFIG != 0U)
    {
        memcpy(&p_sincos->Config, p_sincos->CONST.P_CONFIG, sizeof(SinCos_Config_T));
        SinCos_ResetUnitsAngle(p_sincos);
    }
}

void SinCos_ResetUnitsAngle(SinCos_T * p_sincos)
{
    Linear_Q16_Init(&p_sincos->UnitsAngle, p_sincos->Config.Min_Adcu, p_sincos->Config.Max_Adcu);
}

void SinCos_SetConfigAdc(SinCos_T * p_sincos, uint16_t min_Adcu, uint16_t max_Adcu, uint16_t min_mV, uint16_t max_mV)
{
    p_sincos->Config.Min_Adcu = min_Adcu;
    p_sincos->Config.Max_Adcu = max_Adcu;
    p_sincos->Config.Min_MilliV = min_mV;
    p_sincos->Config.Max_MilliV = max_mV;
    SinCos_ResetUnitsAngle(p_sincos);
}

void SinCos_SetConfigAdc_mV(SinCos_T * p_sincos, uint16_t adcVref_mV, uint16_t min_mV, uint16_t max_mV)
{
    uint16_t min_Adcu = (uint32_t)min_mV * ANALOG_REFERENCE.ADC_MAX / adcVref_mV;
    uint16_t max_Adcu = (uint32_t)max_mV * ANALOG_REFERENCE.ADC_MAX / adcVref_mV;
    SinCos_SetConfigAdc(p_sincos, min_Adcu, max_Adcu, min_mV, max_mV);
}

void SinCos_SetAngleRatio(SinCos_T * p_sincos, uint16_t polePairs)
{
    p_sincos->Config.ElectricalRotationsRatio = polePairs;
}

void SinCos_CalibrateAngleOffset(SinCos_T * p_sincos, uint16_t sin_Adcu, uint16_t cos_Adcu)
{
    // p_sincos->DebugAPre = _SinCos_CalcAngle(p_sincos, sin_Adcu, cos_Adcu);    //debug //check == 0

    p_sincos->Config.AngleOffet = _SinCos_CalcAngle(p_sincos, sin_Adcu, cos_Adcu);
}

/*
    Call after angle offset
*/
void SinCos_CalibrateCcwPositive(SinCos_T * p_sincos, uint16_t sin_Adcu, uint16_t cos_Adcu)
{
    // p_sincos->DebugBPre = _SinCos_CalcAngle(p_sincos, sin_Adcu, cos_Adcu);    //debug //check == 120

    p_sincos->Config.IsCcwPositive = (_SinCos_CalcAngle(p_sincos, sin_Adcu, cos_Adcu) > 0);
}

void SinCos_CalibrateA(SinCos_T * p_sincos, uint16_t sin_Adcu, uint16_t cos_Adcu)
{
    SinCos_CalibrateAngleOffset(p_sincos, sin_Adcu, cos_Adcu);
}

void SinCos_CalibrateB(SinCos_T * p_sincos, uint16_t sin_Adcu, uint16_t cos_Adcu)
{
    SinCos_CalibrateCcwPositive(p_sincos, sin_Adcu, cos_Adcu);
}
