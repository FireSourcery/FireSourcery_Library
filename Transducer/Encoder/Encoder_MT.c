
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
    @file   Encoder_MT.c
    @author FireSourcery
    @brief

*/
/******************************************************************************/
#include "Encoder_MT.h"


/*
    Speed units from Config
*/
void Encoder_MT_InitUnits(const Encoder_T * p_encoder)
{
    AngleCounter_InitFrom(&p_encoder->P_STATE->AngleCounter,
                          AngleCounter_Ref(p_encoder->POLLING_FREQ, p_encoder->P_STATE->Config.CountsPerRevolution, p_encoder->P_STATE->Config.AngleFreqBase));
}

/*
    Capture Mode Init
*/
void Encoder_MT_InitValuesFrom(const Encoder_T * p_encoder, const Encoder_Config_T * p_config)
{
    if (p_config != NULL) { p_encoder->P_STATE->Config = *p_config; }

    Encoder_MT_InitUnits(p_encoder);
    PulseTimer_SetExtendedWatchStop_Millis(&p_encoder->TIMER, p_encoder->P_STATE->Config.ExtendedDeltaTStop);

    p_encoder->P_STATE->DirectionComp = _Encoder_ResolveDirectionComp(p_encoder->P_STATE);
    Encoder_MT_SetInitial(p_encoder);
}

/*
    Init function coupled with HWs
*/
void Encoder_MT_Init_Polling(const Encoder_T * p_encoder)
{
    PulseTimer_Init(&p_encoder->TIMER);
    Encoder_MT_InitValuesFrom(p_encoder, p_encoder->P_NVM_CONFIG);
}

void Encoder_MT_Init_InterruptQuadrature(const Encoder_T * p_encoder)
{
    PulseTimer_Init(&p_encoder->TIMER);
    Encoder_MT_InitValuesFrom(p_encoder, p_encoder->P_NVM_CONFIG);
    p_encoder->P_STATE->Config.IsQuadratureCaptureEnabled = true;
    p_encoder->P_STATE->DirectionComp = _Encoder_ResolveDirectionComp(p_encoder->P_STATE);
#if defined(ENCODER_HW_DECODER)
    Encoder_InitCounter(p_encoder);
#elif defined(ENCODER_HW_EMULATED)
    Encoder_InitInterrupts_Quadrature(p_encoder);
#endif
}

/*
    Zero Hw Counters
*/
void Encoder_MT_SetInitial(const Encoder_T * p_encoder)
{
    _Encoder_ZeroPulseCount(p_encoder);
    PulseTimer_SetInitial(&p_encoder->TIMER);
    p_encoder->P_STATE->AngleCounter.FreqD = 0;
}


