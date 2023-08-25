
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
    @file    Encoder_DeltaDT.c
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#include "Encoder_DeltaD.h"
#include "Encoder_DeltaT.h"

#include <string.h>

void Encoder_ModeDT_Init(Encoder_T * p_encoder)
{
    if(p_encoder->CONFIG.P_PARAMS != 0U) { memcpy(&p_encoder->Params, p_encoder->CONFIG.P_PARAMS, sizeof(Encoder_Params_T)); }
    _Encoder_DeltaT_InitTimer(p_encoder);
    _Encoder_DeltaD_InitCounter(p_encoder);
    p_encoder->UnitT_Freq = 1U;
    _Encoder_ResetUnits(p_encoder);
    Encoder_DeltaD_SetInitial(p_encoder);
    Encoder_DeltaT_SetInitial(p_encoder);
}

void Encoder_ModeDT_SetInitial(Encoder_T * p_encoder)
{
    Encoder_DeltaD_SetInitial(p_encoder);
    Encoder_DeltaT_SetInitial(p_encoder);
    p_encoder->DeltaTh = 0U;
    p_encoder->FreqD = 0;
    p_encoder->DirectionD = 0;
    p_encoder->TotalD = 0;
}