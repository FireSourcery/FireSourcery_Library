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
    @file    Encoder_DeltaT.h
    @author FireSourcery
    @brief     Capture DeltaT per pulse count: variable DeltaT, DeltaD is fixed, 1.

*/
/******************************************************************************/
#ifndef ENCODER_DELTA_T_H
#define ENCODER_DELTA_T_H

#include "Encoder.h"

static inline uint32_t _Encoder_DeltaT_GetTimerFreq(const Encoder_T * p_encoder)
{
#ifdef CONFIG_ENCODER_DYNAMIC_TIMER
    return p_encoder->P_STATE->UnitTime_Freq;
#else
    return p_encoder->TIMER_FREQ;
#endif
}

/* Common */
/* Filter handle by upper layer */
static inline bool _Encoder_DeltaT_Capture(const Encoder_T * p_encoder)
{
    bool isValid = !HAL_Encoder_ReadTimerOverflow(p_encoder->P_HAL_ENCODER_TIMER);
    if (isValid == true) { p_encoder->P_STATE->DeltaT = HAL_Encoder_ReadTimer(p_encoder->P_HAL_ENCODER_TIMER); }
    else { HAL_Encoder_ClearTimerOverflow(p_encoder->P_HAL_ENCODER_TIMER); }
    HAL_Encoder_WriteTimer(p_encoder->P_HAL_ENCODER_TIMER, 0U);
    return isValid;
}

/******************************************************************************/
/*!
    @brief  Capture DeltaT - Pulse Edge Polling/ISR
            Interval cannot be greater than 0xFFFF [ticks] => (0xFFFF / TIMER_FREQ) [seconds]
            Low EncoderPulseFreq, < POLLING_FREQ, use interpolation.
*/
/******************************************************************************/
static inline void Encoder_DeltaT_Capture(const Encoder_T * p_encoder)
{
    if (_Encoder_DeltaT_Capture(p_encoder) == false) { p_encoder->P_STATE->DeltaT = ENCODER_TIMER_MAX; }
}

static inline bool Encoder_DeltaT_IsStop(const Encoder_T * p_encoder)
{
    return (p_encoder->P_STATE->DeltaT >= ENCODER_TIMER_MAX);
}

/******************************************************************************/
/*!
    @brief Extend base timer, 16bit timer cases to 32bit.
    Use Extended Timer to extend Low RPM range.
    EXTENDED_TIMER_FREQ = 1000Hz, ENCODER_TIMER_MAX = 0xFFFF
        209ms to 13743S for TimerFreq = 312500Hz, 3.2us period
        104ms to 6871S for TimerFreq = 625000Hz
        1.6ms to 107S for TimerFreq = 40Mhz
*/
/******************************************************************************/
static inline uint32_t _Encoder_GetExtendedTimerDelta(const Encoder_T * p_encoder)
{
    /* Millis overflow 40+ days */
#ifdef CONFIG_ENCODER_EXTENDED_TIMER_CHECK_OVERFLOW
    _Encoder_CaptureDeltaWrap(UINT32_MAX, p_encoder->ExtendedTimerPrev, *(p_encoder->P_EXTENDED_TIMER));
#else
    return *(p_encoder->P_EXTENDED_TIMER) - p_encoder->P_STATE->ExtendedTimerPrev;
#endif
}

/*
    Call on Encoder Edge.
    32 bit DeltaT overflow should be caught by CheckExtendedStop, which is a shorter period
*/
static inline void Encoder_DeltaT_CaptureExtended(const Encoder_T * p_encoder)
{
    if (_Encoder_DeltaT_Capture(p_encoder) == false) { p_encoder->P_STATE->DeltaT = _Encoder_GetExtendedTimerDelta(p_encoder) * p_encoder->P_STATE->ExtendedTimerConversion; }
    p_encoder->P_STATE->ExtendedTimerPrev = *(p_encoder->P_EXTENDED_TIMER);
}

/*
    Use Extended Timer to check 0 speed.
*/
static inline bool Encoder_DeltaT_IsExtendedStop(const Encoder_T * p_encoder)
{
    return (_Encoder_GetExtendedTimerDelta(p_encoder) > p_encoder->P_STATE->Config.ExtendedDeltaTStop);
}

/******************************************************************************/
/*
    Capture Period < 1S
    DeltaT / TimerFreq = DeltaT [Seconds]
*/
/******************************************************************************/
static inline uint32_t Encoder_DeltaT_AsFreq(const Encoder_T * p_encoder) { return p_encoder->TIMER_FREQ / p_encoder->P_STATE->DeltaT; }
static inline uint32_t Encoder_DeltaT_AsTime_Ms(const Encoder_T * p_encoder) { return p_encoder->P_STATE->DeltaT * 1000U / p_encoder->TIMER_FREQ; }

/******************************************************************************/
/*!
    @brief  Angle Interpolation Functions

    Estimate Angle each control cycle in between encoder counts

    Only when POLLING_FREQ > PulseFreq, i.e. 0 encoder counts per poll, polls per encoder count > 1
    e.g. High res break even point
        8192 CountsPerRevolution, 20Khz POLLING_FREQ => 146 RPM
*/
/******************************************************************************/
/*
    optionally shift instead of index
*/
static uint32_t Encoder_DeltaT_CapturePollingDelta(const Encoder_T * p_encoder)
{
    // p_encoder->PollingAngleDelta = p_encoder->UnitAngleD / p_encoder->DeltaT * (p_encoder->TIMER_FREQ / p_encoder->POLLING_FREQ);
    // p_encoder->PollingAngleDelta = p_encoder->UnitPollingAngle / p_encoder->DeltaT << 16;
}

/*!
    @brief InterpolateAngle
    AngleIndex * 1(DeltaD) * [UnitPollingAngle] / DeltaT
    UnitPollingAngle = [ENCODER_ANGLE_DEGREES[65536] * TIMER_FREQ / POLLING_FREQ / CountsPerRevolution]
    AngleIndex [0:InterpolationCount]
*/
static inline uint32_t Encoder_DeltaT_InterpolateAngleIndex(Encoder_State_T * p_encoder, uint32_t pollingIndex)
{
    math_limit_upper(pollingIndex * p_encoder->UnitPollingAngle / p_encoder->DeltaT, p_encoder->InterpolateAngleLimit);
}

static inline uint32_t Encoder_DeltaT_ProcInterpolateAngle(Encoder_State_T * p_encoder)
{
    // p_encoder->InterpolateAngleIndex++; /* todo replace index for sum */
    // return Encoder_DeltaT_InterpolateAngleIndex(p_encoder, p_encoder->InterpolateAngleIndex);
    // p_encoder->InterpolateAngleSum += p_encoder->PollingAngleDelta;
    // return p_encoder->InterpolateAngleSum >> 16;
}

static inline void Encoder_DeltaT_ZeroInterpolateAngle(Encoder_State_T * p_encoder)
{
    // p_encoder->InterpolateAngleIndex = 0U;
    p_encoder->InterpolateAngleSum = 0U;
}

/*
    InterpolationCount - numbers of Polls per encoder count, per DeltaT Capture, AngleIndex max
    POLLING_FREQ/EncoderPulseFreq == POLLING_FREQ / (TIMER_FREQ / DeltaT);
*/
// static inline uint32_t Encoder_DeltaT_GetInterpolationCount(const Encoder_State_T * p_encoder)
// {
//     return p_encoder->POLLING_FREQ * p_encoder->DeltaT / p_encoder->TIMER_FREQ;
// }

// /* */
// static inline uint32_t Encoder_DeltaT_InterpolationCountOfRpm(const Encoder_State_T * p_encoder, uint16_t mechRpm)
// {
//     return p_encoder->POLLING_FREQ * 60U / (p_encoder->P_STATE->Config.CountsPerRevolution * mechRpm);
// }

// static inline uint32_t Encoder_DeltaT_RpmOfInterpolationCount(const Encoder_State_T * p_encoder, uint16_t interpolationCount)
// {
//     return Encoder_DeltaT_InterpolationCountOfRpm(p_encoder, interpolationCount);
// }



/******************************************************************************/
/*!
    todo split DeltaT conversion math
*/
/******************************************************************************/
static inline uint32_t Encoder_DeltaT_OfRotationalSpeed_RPM(const Encoder_State_T * p_encoder, uint32_t rpm)
{
    // check (p_encoder->UnitAngularSpeed == 0U) || *60 > INTMAX
    // return (rpm == 0U) ? 0U : p_encoder->TIMER_FREQ * 60U / (p_encoder->Config.CountsPerRevolution * rpm);
    return (rpm == 0U) ? 0U : p_encoder->UnitTime_Freq * 60U / (p_encoder->Config.CountsPerRevolution * rpm);
}

static inline uint32_t Encoder_DeltaT_ToRotationalSpeed_RPM(const Encoder_State_T * p_encoder, uint32_t deltaT_ticks)
{
    return Encoder_DeltaT_OfRotationalSpeed_RPM(p_encoder, deltaT_ticks);
    // return  p_encoder->TIMER_FREQ / deltaT_ticks * 60U / p_encoder->P_STATE->Config.CountsPerRevolution;
}

static inline uint32_t Encoder_DeltaT_OfLinearSpeed(const Encoder_State_T * p_encoder, uint32_t speed_UnitsPerSecond)
{
    return (speed_UnitsPerSecond == 0U) ? 0U : p_encoder->UnitSurfaceSpeed / speed_UnitsPerSecond;
}

static inline uint32_t Encoder_DeltaT_ToLinearSpeed(const Encoder_State_T * p_encoder, uint32_t deltaT_ticks)
{
    return Encoder_DeltaT_OfLinearSpeed(p_encoder, deltaT_ticks);
}



/******************************************************************************/
/*!
    @brief Angular/Rotational Speed
*/
/******************************************************************************/
static inline uint32_t Encoder_DeltaT_GetRotationalSpeed_RPM(const Encoder_State_T * p_encoder)
{
    return Encoder_DeltaT_ToRotationalSpeed_RPM(p_encoder, p_encoder->DeltaT);
}

// static inline uint32_t Encoder_DeltaT_GetAngularSpeed(const Encoder_State_T * p_encoder)
// {
//     return (p_encoder->TIMER_FREQ / p_encoder->POLLING_FREQ << ENCODER_ANGLE_BITS) /
//         (p_encoder->P_STATE->Config.CountsPerRevolution * p_encoder->DeltaT) * p_encoder->POLLING_FREQ;
//     // return (Encoder_DeltaT_AsFreq(p_encoder) << ENCODER_ANGLE_BITS) / p_encoder->P_STATE->Config.CountsPerRevolution;
// }

// static inline uint32_t Encoder_DeltaT_GetAngularSpeed_PerPoll(const Encoder_State_T * p_encoder)
// {
//     return (p_encoder->TIMER_FREQ / p_encoder->POLLING_FREQ << ENCODER_ANGLE_BITS) /
//         (p_encoder->P_STATE->Config.CountsPerRevolution * p_encoder->DeltaT);
// }

/******************************************************************************/
/*!
    @brief Scalar Speed
*/
/******************************************************************************/
static inline uint32_t Encoder_DeltaT_GetScalarSpeed(const Encoder_State_T * p_encoder)
{
    return p_encoder->UnitScalarSpeed / p_encoder->DeltaT;
}

/******************************************************************************/
/*!
    @brief Linear Speed
*/
/******************************************************************************/
/* In UnitsPerSecond */
static inline uint32_t Encoder_DeltaT_GetLinearSpeed(const Encoder_State_T * p_encoder)
{
    // return Encoder_CalcLinearSpeed(p_encoder, 1U, p_encoder->DeltaT);
}

static inline uint32_t Encoder_DeltaT_GetGroundSpeed_Mph(const Encoder_State_T * p_encoder)
{
    return Encoder_GroundSpeedOf_Mph(p_encoder, 1U, p_encoder->DeltaT);
}

static inline uint32_t Encoder_DeltaT_GetGroundSpeed_Kmh(const Encoder_State_T * p_encoder)
{
    return Encoder_GroundSpeedOf_Kmh(p_encoder, 1U, p_encoder->DeltaT);
}

/******************************************************************************/
/*!
    @brief     Extern Declarations
*/
/*! @{ */
/******************************************************************************/
extern void _Encoder_DeltaT_InitTimer(const Encoder_T * p_encoder);
extern void Encoder_DeltaT_Init(const Encoder_T * p_encoder);
extern void Encoder_DeltaT_SetInitial(const Encoder_T * p_encoder);
extern void Encoder_DeltaT_SetExtendedWatchStop_Millis(const Encoder_T * p_encoder, uint16_t effectiveStopTime_Millis);
extern void Encoder_DeltaT_SetExtendedWatchStop_RPM(const Encoder_T * p_encoder);
/******************************************************************************/
/*! @} */
/******************************************************************************/

#endif

