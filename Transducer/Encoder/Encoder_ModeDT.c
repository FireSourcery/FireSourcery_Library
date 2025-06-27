
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

*/
/******************************************************************************/
#include "Encoder_DeltaD.h"
#include "Encoder_DeltaT.h"

#include <string.h>
#include "Encoder_ModeDT.h"


/*
    Capture Mode
*/
void _Encoder_ModeDT_InitPolling(const Encoder_T * p_encoder)
{
    _Encoder_DeltaT_InitTimer(p_encoder);
}

void _Encoder_ModeDT_InitInterruptQuadrature(const Encoder_T * p_encoder)
{
    _Encoder_DeltaT_InitTimer(p_encoder);
    _Encoder_DeltaD_InitCounter(p_encoder);
    Encoder_InitInterrupts_Quadrature(p_encoder);
    p_encoder->P_STATE->Config.IsQuadratureCaptureEnabled = true;
}

/* For hall sensor */
void _Encoder_ModeDT_InitInterruptAbc(const Encoder_T * p_encoder)
{
    _Encoder_DeltaT_InitTimer(p_encoder);
    _Encoder_DeltaD_InitCounter(p_encoder);
    Encoder_InitInterrupts_ABC(p_encoder);
}


void Encoder_ModeDT_InitValuesFrom(const Encoder_T * p_encoder, const Encoder_Config_T * p_config)
{
    if (p_config != NULL) { memcpy(&p_encoder->P_STATE->Config, p_config, sizeof(Encoder_Config_T)); }
    p_encoder->P_STATE->UnitTime_Freq = 1U;
    p_encoder->P_STATE->PollingFreq = p_encoder->POLLING_FREQ;
    p_encoder->P_STATE->DirectionComp = _Encoder_GetDirectionComp(p_encoder->P_STATE);
    _Encoder_ResetUnits(p_encoder->P_STATE);
    Encoder_DeltaD_SetInitial(p_encoder);
    Encoder_DeltaT_SetInitial(p_encoder);
}

/*
    Init function coupled with HWs
*/
void Encoder_ModeDT_Init_Polling(const Encoder_T * p_encoder)
{
    _Encoder_ModeDT_InitPolling(p_encoder);
    Encoder_ModeDT_InitValuesFrom(p_encoder , p_encoder->P_NVM_CONFIG);
}

void Encoder_ModeDT_Init_InterruptQuadrature(const Encoder_T * p_encoder)
{
    _Encoder_ModeDT_InitInterruptQuadrature(p_encoder);
    Encoder_ModeDT_InitValuesFrom(p_encoder, p_encoder->P_NVM_CONFIG);
}

/* For hall sensor */
void Encoder_ModeDT_Init_InterruptAbc(const Encoder_T * p_encoder)
{
    _Encoder_ModeDT_InitInterruptAbc(p_encoder);
    Encoder_ModeDT_InitValuesFrom(p_encoder, p_encoder->P_NVM_CONFIG);
}

/*
    Zero Hw Counters
*/
void Encoder_ModeDT_SetInitial(const Encoder_T * p_encoder)
{
    Encoder_DeltaD_SetInitial(p_encoder);
    Encoder_DeltaT_SetInitial(p_encoder);
    p_encoder->P_STATE->DeltaTh = p_encoder->TIMER_FREQ;
    p_encoder->P_STATE->FreqD = 0;
    // p_encoder->DirectionD = 0;
    // p_encoder->TotalD = 0;
}



/******************************************************************************/
/*!

*/
/******************************************************************************/
int32_t Enocder_ModeDT_VarId_Get(const Encoder_State_T * p_encoder, Encoder_VarId_T varId)
{
    int32_t value = 0;
    switch (varId)
    {
        case ENCODER_VAR_FREQ:            value = p_encoder->FreqD;     break;
        case ENCODER_VAR_COUNTER_D:       value = p_encoder->CounterD;  break;
        case ENCODER_VAR_RPM:             value = Encoder_ModeDT_GetRotationalSpeed_RPM(p_encoder);     break;
        case ENCODER_VAR_DELTA_T_SPEED:   value = Encoder_DeltaT_GetRotationalSpeed_RPM(p_encoder);     break;
        case ENCODER_VAR_DELTA_D_SPEED:   value = Encoder_DeltaD_GetRotationalSpeed_RPM(p_encoder);     break;
    }
    return value;
}
