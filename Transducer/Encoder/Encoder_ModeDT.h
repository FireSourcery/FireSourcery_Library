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
    @version V0
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
    In SAMPLE_FREQ ~1ms time
    Speed => 0 when DeltaT > 1S
*/
static inline void Encoder_ModeDT_CaptureFreqD(Encoder_T * p_encoder)
{
    const uint32_t sampleFreq = p_encoder->CONST.SAMPLE_FREQ; /* periodTs = 1 / SAMPLE_FREQ */
    const uint32_t timerFreq = p_encoder->CONST.TIMER_FREQ;
    // const uint32_t sampleTime = timerFreq / sampleFreq;
    const uint32_t sampleTime = p_encoder->CONST.SAMPLE_TIME;

    uint32_t deltaTh;
    uint32_t freqTk;

    Encoder_DeltaD_Capture(p_encoder);

    if (p_encoder->DeltaD == 0)
    {
        /* Assume its user direction, low speed opposite direction will be seen as aligned direction */
        // p_encoder->FreqD = p_encoder->DirectionD * (timerFreq / p_encoder->DeltaT);
        /* DeltaT to assume time/speed at init until first pulse, or FreqD no change for 0 */

        /* Set next periodTk as ~DeltaT on overflow */
        p_encoder->DeltaTh = (HAL_Encoder_ReadTimerOverflow(p_encoder->CONST.P_HAL_ENCODER_TIMER) == true) ?
            // (p_encoder->DeltaT - sampleTime) : HAL_Encoder_ReadTimer(p_encoder->CONST.P_HAL_ENCODER_TIMER);
            /* accumulating should be more precise than approximating current DeltaT ~= Prev DeltaT */
            (p_encoder->DeltaTh + sampleTime) : HAL_Encoder_ReadTimer(p_encoder->CONST.P_HAL_ENCODER_TIMER);
    }
    else
    {
        // p_encoder->DirectionD = math_sign(p_encoder->DeltaD);

        /* Overflow is > SampleTime. DeltaD == 0 occurs prior. */
        deltaTh = HAL_Encoder_ReadTimer(p_encoder->CONST.P_HAL_ENCODER_TIMER);

        /* periodTk = periodTs + DeltaThPrev/timerFreq - deltaTh/timerFreq */
        // freqTk = (timerFreq * sampleFreq) / (timerFreq + (sampleFreq * (p_encoder->DeltaTh - deltaTh)));
        freqTk = (timerFreq) / (sampleTime + (p_encoder->DeltaTh - deltaTh));

        /* if (periodTk > periodTs / 2) { freqD = (deltaD / periodTk) } */
        /* periodTk ~= periodTs */
        if (freqTk < (sampleFreq * 2U)) { p_encoder->FreqD = (p_encoder->DeltaD * freqTk); }

        p_encoder->DeltaTh = deltaTh;
    }
}

static inline void Encoder_ModeDT_CaptureVelocity(Encoder_T * p_encoder)
{
    /* Speed may reach zero before checking with extended counter takes effect */
    if (Encoder_DeltaT_IsExtendedStop(p_encoder) == false) { Encoder_ModeDT_CaptureFreqD(p_encoder); }
    else { p_encoder->FreqD = 0; }
}

/*
    Using FreqD to interpolate Angle

    AngularSpeed * AngleIndex / POLLING_FREQ
    (AngleIndex / POLLING_FREQ) * FreqD * (ENCODER_ANGLE_DEGREES / CountsPerRevolution)
    AngleIndex * FreqD * [ENCODER_ANGLE_DEGREES / CountsPerRevolution / POLLING_FREQ]

*/
// static uint32_t Encoder_ModeDT_CaptureInterpolateDelta(Encoder_T * p_encoder)
// {
//     // p_encoder->InterpolateAngleDelta = p_encoder->FreqD * p_encoder->UnitInterpolateAngle;
//     p_encoder->InterpolateAngleDelta = p_encoder->FreqD * (p_encoder->UnitAngularD * p_encoder->Config.InterpolateAngleScalar / p_encoder->CONST.POLLING_FREQ);
// }

// static inline uint32_t Encoder_ModeDT_ProcInterpolateAngle(Encoder_T * p_encoder)
// {
//     p_encoder->InterpolateAngleSum = math_limit_upper(p_encoder->InterpolateAngleSum + p_encoder->InterpolateAngleDelta, p_encoder->InterpolateAngleLimit);
//     return p_encoder->InterpolateAngleSum >> ENCODER_ANGLE_SHIFT;
// }

/* |DeltaD| <= 1 */
static inline uint32_t Encoder_ModeDT_InterpolateAngle(Encoder_T * p_encoder)
{
    return (math_abs(p_encoder->FreqD) < p_encoder->CONST.POLLING_FREQ / 2U) ? Encoder_DeltaT_ProcInterpolateAngle(p_encoder) : 0U;
    /* disabled for less than 1RPS 60RPM */
    // uint32_t freqD = math_abs(p_encoder->FreqD);
    // return ((p_encoder->Config.CountsPerRevolution) < freqD && freqD < p_encoder->CONST.POLLING_FREQ / 2U) ? Encoder_DeltaT_ProcInterpolateAngle(p_encoder) : 0U;
}

/*
    DirectionD to the last DeltaD capture
*/
static inline int32_t Encoder_ModeDT_InterpolateAngularDisplacement(Encoder_T * p_encoder)
{
    // return Encoder_GetDirectionRef(p_encoder) /* * p_encoder->DirectionD */ * Encoder_ModeDT_InterpolateAngle(p_encoder);
    /*
        quadrature signed capture applies math_sign(p_encoder->FreqD) only
        unsigned capture with decrement comp is unaffected
        disables when FreqD is 0
    */
    return Encoder_GetDirectionRef(p_encoder) * math_sign(p_encoder->FreqD) * Encoder_ModeDT_InterpolateAngle(p_encoder);
}

/******************************************************************************/
/*
*/
/******************************************************************************/
/* Signed with capture reference */
/*
    Speed is signed without direction comp for quadrature, unsigned single phase
*/
static inline int32_t Encoder_ModeDT_GetScalarSpeed(Encoder_T * p_encoder)
{
    return p_encoder->FreqD * p_encoder->UnitScalarSpeed >> p_encoder->UnitScalarSpeedShift;
}

/*
    Direction Comp signed with user reference
*/
static inline int32_t Encoder_ModeDT_GetScalarVelocity(Encoder_T * p_encoder)
{
    return Encoder_GetDirectionRef(p_encoder) * Encoder_ModeDT_GetScalarVelocity(p_encoder);
}

static inline int32_t Encoder_ModeDT_PollScalarVelocity(Encoder_T * p_encoder)
{
    Encoder_ModeDT_CaptureVelocity(p_encoder);
    return Encoder_ModeDT_GetScalarVelocity(p_encoder);
}

/******************************************************************************/
/*
*/
/******************************************************************************/
static inline int32_t Encoder_ModeDT_GetRotationalSpeed_RPM(const Encoder_T * p_encoder)
{
    return p_encoder->FreqD * 60 / p_encoder->Config.CountsPerRevolution;
}

static inline int32_t Encoder_ModeDT_GetRotationalVelocity_RPM(const Encoder_T * p_encoder)
{
    return Encoder_GetDirectionRef(p_encoder) * Encoder_ModeDT_GetRotationalSpeed_RPM(p_encoder);
}

static inline int32_t Encoder_ModeDT_GetRotationalSpeed_RPS(const Encoder_T * p_encoder)
{
    return p_encoder->FreqD / p_encoder->Config.CountsPerRevolution;
}
static inline int32_t Encoder_ModeDT_GetRotationalVelocity(const Encoder_T * p_encoder)
{

}


static inline int32_t Encoder_ModeDT_GetAngularSpeed(const Encoder_T * p_encoder)
{
    return p_encoder->FreqD * p_encoder->UnitAngularSpeed >> p_encoder->UnitAngularSpeedShift;
}

static inline int32_t Encoder_ModeDT_GetAngularVelocity(const Encoder_T * p_encoder)
{
}

static inline int32_t Encoder_ModeDT_GetSurfaceSpeed(const Encoder_T * p_encoder)
{
    return p_encoder->FreqD * p_encoder->UnitSurfaceSpeed >> p_encoder->UnitSurfaceSpeedShift;
}

static inline int32_t Encoder_ModeDT_GetSurfaceVelocity(const Encoder_T * p_encoder)
{

}

static inline int32_t Encoder_ModeDT_GetGroundVelocity_Mph(const Encoder_T * p_encoder)
{

}

static inline int32_t Encoder_ModeDT_GetGroundVelocity_Kmh(const Encoder_T * p_encoder)
{

}


/******************************************************************************/
/*
*/
/******************************************************************************/
extern void Encoder_ModeDT_Init(Encoder_T * p_encoder);
extern void Encoder_ModeDT_Init_Polling(Encoder_T * p_encoder);
extern void Encoder_ModeDT_Init_InterruptQuadrature(Encoder_T * p_encoder);
extern void Encoder_ModeDT_Init_InterruptAbc(Encoder_T * p_encoder);

extern void Encoder_ModeDT_SetInitial(Encoder_T * p_encoder);

#endif