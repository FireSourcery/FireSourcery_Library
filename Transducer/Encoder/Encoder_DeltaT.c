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
#include "Encoder_DeltaT.h"
#include <string.h>

/*!
    Protected Init Timer HAL
*/
void _Encoder_DeltaT_InitTimer(const Encoder_T * p_encoder)
{

    // if(p_encoder->P_HAL_ENCODER_TIMER_INIT) { p_encoder->P_HAL_ENCODER_TIMER_INIT(); }
    // else{}

    HAL_Encoder_InitTimer(p_encoder->P_HAL_ENCODER_TIMER);
    /*
        RPM * CPR / 60[Seconds] = CPS
        CPS = T_FREQ [Hz] / deltaT_ticks [timerticks/Count]
        RPM * CPR / 60[Seconds] = T_FREQ [Hz] / deltaT_ticks
        Error ~ 1, deltaT_ticks = 100
        => T_FREQ/CPS >= 100, (CPS/T_FREQ <= .01)
            T_FREQ /(RPM * CPR / 60) >= 100
        eg. RPM = 10000
            T_FREQ >= 100*(10000RPM * CPR / 60)
            T_FREQ >= 16666 * CPR

        Min: deltaT_ticks = 65535
            RPM = (T_FREQ / CPR) * (60 / 65535)
            => 15 ~= 16666 * (60 / 65535)
            T_FREQ = DeltaT_Ticks * (CountsPerRevolution * rpm) / 60
            T_FREQ = 65535 * (24 * 100) / 60 ~=2.6Mhz

        TIMER_FREQ ~= 10000 * CPR
            => 10000RPM error ~1%
            => RPM Min ~= 10RPM
    */
// #ifdef CONFIG_ENCODER_DYNAMIC_TIMER
//     // uint32_t timerFreq = HAL_Encoder_InitTimerFreq(p_encoder->P_HAL_ENCODER_TIMER, p_encoder->P_STATE->Config.CountsPerRevolution * 16666U);
//     uint32_t timerFreq = HAL_Encoder_InitTimerFreq(p_encoder->P_HAL_ENCODER_TIMER, p_encoder->TIMER_FREQ);
//     p_encoder->ExtendedTimerConversion = timerFreq / p_encoder->EXTENDED_TIMER_FREQ;
// #else
    /* Compile time defined TIMER_FREQ */
    HAL_Encoder_InitTimerFreq(p_encoder->P_HAL_ENCODER_TIMER, p_encoder->TIMER_FREQ);
    p_encoder->P_STATE->ExtendedTimerConversion = p_encoder->TIMER_FREQ / p_encoder->EXTENDED_TIMER_FREQ;
// #endif
}


/*!
    Uses pin ISR, or polling
*/
void Encoder_DeltaT_Init(const Encoder_T * p_encoder)
{
    if (p_encoder->P_NVM_CONFIG != NULL) { memcpy(&p_encoder->P_STATE->Config, p_encoder->P_NVM_CONFIG, sizeof(Encoder_Config_T)); }
    _Encoder_DeltaT_InitTimer(p_encoder);
    p_encoder->P_STATE->UnitTime_Freq = p_encoder->TIMER_FREQ;
    p_encoder->P_STATE->PollingFreq = p_encoder->POLLING_FREQ;
    _Encoder_ResetUnits(p_encoder->P_STATE);
    p_encoder->P_STATE->DeltaD = 1U; /* Effective for shared functions only */
    p_encoder->P_STATE->DirectionComp = 1;
    Encoder_DeltaT_SetInitial(p_encoder);
}

/*
    Set initial capture to lowest speed, DeltaT = ENCODER_TIMER_MAX
*/
void Encoder_DeltaT_SetInitial(const Encoder_T * p_encoder)
{
    HAL_Encoder_ClearTimerOverflow(p_encoder->P_HAL_ENCODER_TIMER);
    HAL_Encoder_WriteTimer(p_encoder->P_HAL_ENCODER_TIMER, 0U);

    p_encoder->P_STATE->DeltaT = p_encoder->TIMER_FREQ; /* Set as 1s 60/Cpr RPM, alternatively multiply for 1rpm */
    p_encoder->P_STATE->ExtendedTimerPrev = *p_encoder->P_EXTENDED_TIMER;
    // p_encoder->P_STATE->InterpolateAngleIndex = 0U;
    // _Encoder_ZeroPulseCount(p_encoder);
}

/******************************************************************************/
/*
    Config
*/
/******************************************************************************/
/*
    Extended timer ticks to determine capture stopped
    EXTENDED_TIMER_FREQ should be small, 1000, < 65536
    1[S] => 60/CPR[RPM]
*/
void Encoder_DeltaT_SetExtendedWatchStop_Millis(const Encoder_T * p_encoder, uint16_t effectiveStopTime_Millis)
{
    p_encoder->P_STATE->Config.ExtendedDeltaTStop = effectiveStopTime_Millis * p_encoder->EXTENDED_TIMER_FREQ / 1000U;
}

void Encoder_DeltaT_SetExtendedWatchStop_RPM(const Encoder_T * p_encoder)
{
    // 60U * p_encoder->EXTENDED_TIMER_FREQ / p_encoder->P_STATE->Config.CountsPerRevolution / rpm
    p_encoder->P_STATE->Config.ExtendedDeltaTStop = Encoder_DeltaT_OfRotationalSpeed_RPM(p_encoder->P_STATE, 1U) * p_encoder->EXTENDED_TIMER_FREQ / _Encoder_DeltaT_GetTimerFreq(p_encoder);
}

