
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
#include "Encoder_ModeDT.h"

static void InitValues(Encoder_T * p_encoder)
{
    if (p_encoder->CONST.P_CONFIG != NULL) { memcpy(&p_encoder->Config, p_encoder->CONST.P_CONFIG, sizeof(Encoder_Config_T)); }

    p_encoder->UnitTime_Freq = 1U;
    p_encoder->DirectionComp = _Encoder_GetDirectionComp(p_encoder);
    _Encoder_ResetUnits(p_encoder);
    Encoder_DeltaD_SetInitial(p_encoder);
    Encoder_DeltaT_SetInitial(p_encoder);
}

/*
    Init function coupled with HW
*/
void Encoder_ModeDT_Init_Polling(Encoder_T * p_encoder)
{
    _Encoder_DeltaT_InitTimer(p_encoder);
    InitValues(p_encoder);
}

void Encoder_ModeDT_Init_InterruptQuadrature(Encoder_T * p_encoder)
{
    _Encoder_DeltaT_InitTimer(p_encoder);
    _Encoder_DeltaD_InitCounter(p_encoder);
    Encoder_InitInterrupts_Quadrature(p_encoder);
    InitValues(p_encoder);
    p_encoder->Config.IsQuadratureCaptureEnabled = true;
}

void Encoder_ModeDT_Init_InterruptAbc(Encoder_T * p_encoder)
{
    _Encoder_DeltaT_InitTimer(p_encoder);
    _Encoder_DeltaD_InitCounter(p_encoder);
    Encoder_InitInterrupts_ABC(p_encoder);
    InitValues(p_encoder);
}

void Encoder_ModeDT_SetInitial(Encoder_T * p_encoder)
{
    Encoder_DeltaD_SetInitial(p_encoder);
    Encoder_DeltaT_SetInitial(p_encoder);
    p_encoder->DeltaTh = p_encoder->CONST.TIMER_FREQ;
    p_encoder->FreqD = 0;
    // p_encoder->DirectionD = 0;
    // p_encoder->TotalD = 0;
}