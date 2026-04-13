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

    @brief  Mixed Frequency Sampling
*/
/******************************************************************************/
#ifndef ENCODER_MODE_DT_H
#define ENCODER_MODE_DT_H

#include "Encoder_DeltaD.h"
#include "Encoder_DeltaT.h"

/******************************************************************************/
/*

*/
/******************************************************************************/
/*
    Capture [FreqD] Pulse Frequency
    Call at SAMPLE_FREQ ~1ms
*/
static inline void _Encoder_ModeDT_CaptureFreqD(const Encoder_T * p_encoder)
{
    // const uint32_t sampleFreq = p_encoder->SAMPLE_FREQ; /* periodTs = 1 / SAMPLE_FREQ */
    const uint32_t timerFreq = p_encoder->TIMER_FREQ;
    const uint32_t samplePeriod = p_encoder->SAMPLE_TIME; /* in Timer ticks */

    // Encoder_State_T * p_state = p_encoder->P_STATE;
    AngleCounter_T * p_state = &p_encoder->P_STATE->AngleCounter;

    uint32_t deltaTh;
    uint32_t periodTk;

    Encoder_DeltaD_Capture(p_encoder);

    if (p_state->DeltaD == 0)
    {
        /* Same FreqD/speed until next pulse */
        /* Accumulate DeltaTh on overflow */
        p_state->DeltaTh = HAL_Encoder_ReadTimerOverflow(p_encoder->P_HAL_ENCODER_TIMER) ?
            (p_state->DeltaTh + samplePeriod) : HAL_Encoder_ReadTimer(p_encoder->P_HAL_ENCODER_TIMER);
    }
    else
    {
        /* Overflow is > samplePeriod. DeltaD == 0 occurs prior. */
        deltaTh = HAL_Encoder_ReadTimer(p_encoder->P_HAL_ENCODER_TIMER);

        periodTk = samplePeriod + (p_state->DeltaTh - deltaTh);
        if (periodTk > samplePeriod / 2)
        {
            p_state->PeriodT = periodTk;
            p_state->FreqD = p_state->DeltaD * (timerFreq / periodTk);
            // p_encoder->PeriodT = (periodTk + p_encoder->PeriodT) / 2;
            // p_encoder->FreqD = p_state->DeltaD * (timerFreq / p_encoder->PeriodT);
        }

        p_state->DeltaTh = deltaTh;
    }
}

/* Speed => 0 when DeltaT > 1S */
/* Speed may reach zero before checking with extended counter takes effect */
static inline void Encoder_ModeDT_CaptureFreqD(const Encoder_T * p_encoder)
{
    if (Encoder_DeltaT_IsExtendedStop(p_encoder) == false) { _Encoder_ModeDT_CaptureFreqD(p_encoder); }
    else { p_encoder->P_STATE->AngleCounter.FreqD = 0; }
}


static inline uint32_t Encoder_ModeDT_ResolveInterpolation(Encoder_State_T * p_encoder)
{
    AngleCounter_ResolveInterpolationDelta(&p_encoder->AngleCounter);
    return p_encoder->AngleCounter.Base.Delta;
}


/******************************************************************************/
/*
*/
/******************************************************************************/
/* Signed with capture reference */
/*
    Speed is signed without direction comp for quadrature, unsigned single phase
*/

/******************************************************************************/
/*
    At POLLING_FREQ
*/
/******************************************************************************/
static inline uint32_t _Encoder_ModeDT_InterpolateAngle(Encoder_State_T * p_encoder) { return Angle_Interpolate(&p_encoder->AngleCounter.Base); }

/* |DeltaD| <= 1 */
static inline uint32_t Encoder_ModeDT_InterpolateAngle(const Encoder_T * p_encoder)
{
    return (math_abs(p_encoder->P_STATE->AngleCounter.FreqD) < p_encoder->POLLING_FREQ / 2U) ? _Encoder_ModeDT_InterpolateAngle(p_encoder->P_STATE) : 0U;
}

// static inline int32_t Encoder_ModeDT_InterpolateAngularDisplacement(const Encoder_T * p_encoder)
// {
//     return Encoder_GetDirectionRef(p_encoder->P_STATE) * Encoder_ModeDT_InterpolateAngle(p_encoder);
// }

/*

*/
static inline int32_t Encoder_ModeDT_GetScalarSpeed(Encoder_State_T * p_encoder) { return AngleCounter_GetSpeed(&p_encoder->AngleCounter); }

/*
    Direction Comp signed with user reference
*/
// static inline int32_t Encoder_ModeDT_GetScalarVelocity(Encoder_State_T * p_encoder)
// {
//     return Encoder_GetDirectionRef(p_encoder) * Encoder_ModeDT_GetScalarSpeed(p_encoder);
// }


/******************************************************************************/
/*

*/
/******************************************************************************/
static inline int32_t Encoder_ModeDT_GetRotationalSpeed_RPM(const Encoder_State_T * p_encoder) { return rpm_of_count_freq(p_encoder->Config.CountsPerRevolution, p_encoder->AngleCounter.FreqD); }
// static inline int32_t Encoder_ModeDT_GetRotationalVelocity_RPM(const Encoder_State_T * p_encoder) { return Encoder_GetDirectionRef(p_encoder) * Encoder_ModeDT_GetRotationalSpeed_RPM(p_encoder); }

// static inline int32_t Encoder_ModeDT_GetRotationalSpeed_RPS(const Encoder_State_T * p_encoder) { return p_encoder->FreqD / p_encoder->Config.CountsPerRevolution; }

// /* Degs/S */
// static inline int32_t Encoder_ModeDT_GetAngularSpeed(const Encoder_State_T * p_encoder) { return p_encoder->FreqD * p_encoder->UnitAngularSpeed >> p_encoder->UnitAngularSpeedShift; }
// static inline int32_t Encoder_ModeDT_GetSurfaceSpeed(const Encoder_State_T * p_encoder) { return p_encoder->FreqD * p_encoder->UnitSurfaceSpeed >> p_encoder->UnitSurfaceSpeedShift; }

/******************************************************************************/
/*
*/
/******************************************************************************/
extern void Encoder_ModeDT_Init(const Encoder_T *);
extern void Encoder_ModeDT_InitValuesFrom(const Encoder_T * p_encoder, const Encoder_Config_T * p_config);
extern void Encoder_ModeDT_Init_Polling(const Encoder_T *);
extern void Encoder_ModeDT_Init_InterruptQuadrature(const Encoder_T *);
extern void Encoder_ModeDT_Init_InterruptAbc(const Encoder_T *);

extern void Encoder_ModeDT_SetInitial(const Encoder_T *);

/******************************************************************************/
/*
*/
/******************************************************************************/
extern int32_t Encoder_ModeDT_VarId_Get(const Encoder_State_T * p_encoder, Encoder_VarId_T varId);

#endif