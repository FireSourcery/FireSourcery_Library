
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
#include "Encoder_ModeDT.h"


/*
    Capture Mode Init
*/
void Encoder_ModeDT_InitValuesFrom(const Encoder_T * p_encoder, const Encoder_Config_T * p_config)
{
    if (p_config != NULL) { p_encoder->P_STATE->Config = *p_config; }

    AngleCounter_Config_T angleCounterConfig =
    {
        .CountsPerRevolution = p_config->CountsPerRevolution,
        .PollingFreq = p_encoder->POLLING_FREQ,
        .FractSpeedRef_Rpm = p_config->ScalarSpeedRef_Rpm,
    };

    AngleCounter_InitFrom(&p_encoder->P_STATE->AngleCounter, &angleCounterConfig);

    p_encoder->P_STATE->DirectionComp = _Encoder_GetDirectionComp(p_encoder->P_STATE);
    Encoder_ModeDT_SetInitial(p_encoder);
}

/*
    Init function coupled with HWs
*/
void Encoder_ModeDT_Init_Polling(const Encoder_T * p_encoder)
{
    PulseTimer_Init(&p_encoder->TIMER);
    Encoder_ModeDT_InitValuesFrom(p_encoder, p_encoder->P_NVM_CONFIG);
}

void Encoder_ModeDT_Init_InterruptQuadrature(const Encoder_T * p_encoder)
{
    PulseTimer_Init(&p_encoder->TIMER);
    Encoder_InitInterrupts_Quadrature(p_encoder);
    // Encoder_InitCounter(p_encoder);
    Encoder_ModeDT_InitValuesFrom(p_encoder, p_encoder->P_NVM_CONFIG);
    p_encoder->P_STATE->Config.IsQuadratureCaptureEnabled = true;
}

// void Encoder_ModeDT_Init(const Encoder_T * p_encoder)
// {

// }

/* For hall sensor */
// static void _Encoder_ModeDT_InitInterruptAbc(const Encoder_T * p_encoder)
// {
//     PulseTimer_Init(&p_encoder->TIMER);
//     Encoder_InitInterrupts_ABC(p_encoder);
// }

/* For hall sensor */
// void Encoder_ModeDT_Init_InterruptAbc(const Encoder_T * p_encoder)
// {
//     _Encoder_ModeDT_InitInterruptAbc(p_encoder);
//     Encoder_ModeDT_InitValuesFrom(p_encoder, p_encoder->P_NVM_CONFIG);
// }

/*
    Zero Hw Counters
*/
void Encoder_ModeDT_SetInitial(const Encoder_T * p_encoder)
{
    _Encoder_ZeroPulseCount(p_encoder->P_STATE);
    PulseTimer_SetInitial(&p_encoder->TIMER);
    p_encoder->P_STATE->AngleCounter.FreqD = 0;
}



/******************************************************************************/
/*!

*/
/******************************************************************************/
int32_t Encoder_ModeDT_VarId_Get(const Encoder_State_T * p_encoder, Encoder_VarId_T varId)
{
    int32_t value = 0;
    switch (varId)
    {
        case ENCODER_VAR_FREQ:            value = p_encoder->AngleCounter.FreqD;     break;
        case ENCODER_VAR_COUNTER_D:       value = p_encoder->AngleCounter.CounterD;  break;
        case ENCODER_VAR_RPM:             value = Encoder_ModeDT_GetRotationalSpeed_RPM(p_encoder);     break;
        // case ENCODER_VAR_DELTA_T_SPEED:   value = Encoder_DeltaT_GetRotationalSpeed_RPM(p_encoder);     break;
        // case ENCODER_VAR_DELTA_D_SPEED:   value = Encoder_DeltaD_GetRotationalSpeed_RPM(p_encoder);     break;
    }
    return value;
}
