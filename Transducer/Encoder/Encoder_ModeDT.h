#pragma once

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
    @file   Encoder_ModeDT.h
    @author FireSourcery
    @brief  Bridge to speed. Mixed Frequency Sampling
*/
/******************************************************************************/
#include "Encoder.h"

/******************************************************************************/
/*
    Speed
    at SAMPLE_FREQ ~1ms
*/
/******************************************************************************/
/*
    Capture [FreqD] Pulse Frequency
    Delegates to AngleCounter_CaptureFreq + PulseTimer_CaptureSampleTk_Freq
*/
static inline void Encoder_ModeDT_CaptureFreqD(Encoder_T * p_encoder)
{
    if (PulseTimer_IsExtendedStop(&p_encoder->TIMER) == false)
    {
        AngleCounter_CaptureFreq(&p_encoder->P_STATE->AngleCounter, PulseTimer_CaptureSampleTk_Freq(&p_encoder->TIMER));
    }
    else
    {
        p_encoder->P_STATE->AngleCounter.FreqD = 0;
    }
    // PulseTimer_CaptureFreq(&p_encoder->TIMER, &p_encoder->P_STATE->AngleCounter);
}


static inline angle16_t Encoder_ModeDT_ResolveInterpolation(Encoder_T * p_encoder)
{
    return AngleCounter_ResolveAngleDelta(&p_encoder->P_STATE->AngleCounter);
}

/******************************************************************************/
/*
    At POLLING_FREQ
*/
/******************************************************************************/
/* Write to a seperate angle or  */
static inline angle16_t _Encoder_ModeDT_InterpolateAngle(Encoder_T * p_encoder)
{
    return Encoder_GetAngle(p_encoder) ;
}

/* |DeltaD| <= 1 */
static inline angle16_t Encoder_ModeDT_InterpolateAngle(Encoder_T * p_encoder)
{
    // return (math_abs(p_encoder->P_STATE->AngleCounter.FreqD) < p_encoder->POLLING_FREQ / 2U) ?
    return Encoder_GetAngle(p_encoder);
}


/******************************************************************************/
/*
*/
/******************************************************************************/
/* Signed with capture reference */
static inline int32_t Encoder_ModeDT_GetSpeed_PerUnit(Encoder_State_T * p_encoder) { return AngleCounter_GetSpeed_Fract16(&p_encoder->AngleCounter); }


/******************************************************************************/
/*

*/
/******************************************************************************/
static inline int32_t Encoder_ModeDT_GetRotationalSpeed_RPM(const Encoder_State_T * p_encoder) { return rpm_of_count_freq(p_encoder->Config.CountsPerRevolution, p_encoder->AngleCounter.FreqD); }


/******************************************************************************/
/*
*/
/******************************************************************************/
extern void Encoder_ModeDT_Init(Encoder_T *);
extern void Encoder_ModeDT_InitValuesFrom(Encoder_T * p_encoder, const Encoder_Config_T * p_config);
extern void Encoder_ModeDT_Init_Polling(Encoder_T *);
extern void Encoder_ModeDT_Init_InterruptQuadrature(Encoder_T *);

extern void Encoder_ModeDT_SetInitial(Encoder_T *);


/******************************************************************************/
/*
*/
/******************************************************************************/

/*
    Capture [FreqD] Pulse Frequency
    Call at SAMPLE_FREQ ~1ms
*/
// static inline void _Encoder_ModeDT_CaptureFreqD(Encoder_T * p_encoder)
// {
//     // const uint32_t sampleFreq = p_encoder->SAMPLE_FREQ; /* periodTs = 1 / SAMPLE_FREQ */
//     const uint32_t timerFreq = p_encoder->TIMER_FREQ;
//     const uint32_t samplePeriod = p_encoder->SAMPLE_TIME; /* in Timer ticks */

//     // Encoder_State_T * p_state = p_encoder->P_STATE;
//     AngleCounter_T * p_state = &p_encoder->P_STATE->AngleCounter;

//     uint32_t deltaTh;
//     uint32_t periodTk;

//     Encoder_DeltaD_Capture(p_encoder);

//     if (p_state->DeltaD == 0)
//     {
//         /* Same FreqD/speed until next pulse */
//         /* Accumulate DeltaTh on overflow */
//         p_state->DeltaTh = HAL_Encoder_ReadTimerOverflow(p_encoder->P_HAL_ENCODER_TIMER) ?
//             (p_state->DeltaTh + samplePeriod) : HAL_Encoder_ReadTimer(p_encoder->P_HAL_ENCODER_TIMER);
//     }
//     else
//     {
//         /* Overflow is > samplePeriod. DeltaD == 0 occurs prior. */
//         deltaTh = HAL_Encoder_ReadTimer(p_encoder->P_HAL_ENCODER_TIMER);

//         periodTk = samplePeriod + (p_state->DeltaTh - deltaTh);
//         if (periodTk > samplePeriod / 2)
//         {
//             p_state->PeriodT = periodTk;
//             p_state->FreqD = p_state->DeltaD * (timerFreq / periodTk);
//             // p_encoder->PeriodT = (periodTk + p_encoder->PeriodT) / 2;
//             // p_encoder->FreqD = p_state->DeltaD * (timerFreq / p_encoder->PeriodT);
//         }

//         p_state->DeltaTh = deltaTh;
//     }
// }