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
    @file    Encoder_DeltaD.c
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#include "Encoder_DeltaD.h"
#include <string.h>

/*!

*/
void _Encoder_DeltaD_InitCounter(Encoder_T * p_encoder)
{
#if     defined(CONFIG_ENCODER_HW_DECODER)
    HAL_Encoder_InitCounter(p_encoder->CONST.P_HAL_ENCODER_COUNTER);
    HAL_Encoder_WriteCounterMax(p_encoder->CONST.P_HAL_ENCODER_COUNTER, p_encoder->Config.CountsPerRevolution - 1U);
#elif   defined(CONFIG_ENCODER_HW_EMULATED)
    // #ifdef CONFIG_ENCODER_QUADRATURE_MODE_ENABLE
    if(p_encoder->Config.IsQuadratureCaptureEnabled == true)
    {
        Pin_Input_Init(&p_encoder->PinA);
        Pin_Input_Init(&p_encoder->PinB);
    }
    // #endif
#endif
}

/*!

*/
void Encoder_DeltaD_Init(Encoder_T * p_encoder)
{
    if(p_encoder->CONST.P_CONFIG != 0U) { memcpy(&p_encoder->Config, p_encoder->CONST.P_CONFIG, sizeof(Encoder_Config_T)); }
    _Encoder_DeltaD_InitCounter(p_encoder);
    p_encoder->UnitT_Freq = p_encoder->CONST.SAMPLE_FREQ;
    _Encoder_ResetUnits(p_encoder);
    p_encoder->DeltaT = 1U;
    Encoder_DeltaD_SetInitial(p_encoder);
}


void Encoder_DeltaD_SetInitial(Encoder_T * p_encoder)
{
    p_encoder->DeltaD = 0U;
#if     defined(CONFIG_ENCODER_HW_DECODER)
    HAL_Encoder_ClearCounterOverflow(p_encoder->CONST.P_HAL_ENCODER_COUNTER);
    HAL_Encoder_WriteCounter(p_encoder->CONST.P_HAL_ENCODER_COUNTER, 0U);
    p_encoder->IndexCount = 0U;
#elif   defined(CONFIG_ENCODER_HW_EMULATED)
    _Encoder_ZeroPulseCount(p_encoder);
#endif
}



