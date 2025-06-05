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

*/
/******************************************************************************/
#include "Encoder_DeltaD.h"
#include <string.h>

/*!

*/
void _Encoder_DeltaD_InitCounter(const Encoder_T * p_encoder)
{
#if     defined(CONFIG_ENCODER_HW_DECODER)
    HAL_Encoder_InitCounter(p_encoder->P_HAL_ENCODER_COUNTER);
    HAL_Encoder_WriteCounterMax(p_encoder->P_HAL_ENCODER_COUNTER, p_encoder->P_STATE->Config.CountsPerRevolution - 1U);
#elif   defined(CONFIG_ENCODER_HW_EMULATED)
    // #ifdef CONFIG_ENCODER_QUADRATURE_MODE_ENABLE
    if(p_encoder->P_STATE->Config.IsQuadratureCaptureEnabled == true)
    {
        Pin_Input_Init(&p_encoder->PIN_A);
        Pin_Input_Init(&p_encoder->PIN_B);
    }
    // #endif
#endif
}

/*!

*/
void Encoder_DeltaD_Init(const Encoder_T * p_encoder)
{
    if(p_encoder->P_NVM_CONFIG != NULL) { memcpy(&p_encoder->P_STATE->Config, p_encoder->P_NVM_CONFIG, sizeof(Encoder_Config_T)); }
    _Encoder_DeltaD_InitCounter(p_encoder);
    p_encoder->P_STATE->UnitTime_Freq = p_encoder->SAMPLE_FREQ;
    _Encoder_ResetUnits(p_encoder->P_STATE);
    p_encoder->P_STATE->DeltaT = 1U;
    Encoder_DeltaD_SetInitial(p_encoder);
}


void Encoder_DeltaD_SetInitial(const Encoder_T * p_encoder)
{
#if     defined(CONFIG_ENCODER_HW_DECODER)
    HAL_Encoder_ClearCounterOverflow(p_encoder->P_HAL_ENCODER_COUNTER);
    HAL_Encoder_WriteCounter(p_encoder->P_HAL_ENCODER_COUNTER, 0U);
    p_encoder->P_STATE->IndexCount = 0U;
#elif   defined(CONFIG_ENCODER_HW_EMULATED)
#endif
    _Encoder_ZeroPulseCount(p_encoder->P_STATE);
    p_encoder->P_STATE->DeltaD = 0U;
}



