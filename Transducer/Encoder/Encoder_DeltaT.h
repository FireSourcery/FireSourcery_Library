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
    @version V0
*/
/******************************************************************************/
#ifndef ENCODER_DELTA_T_H
#define ENCODER_DELTA_T_H

#include "Encoder.h"

static inline uint32_t _Encoder_DeltaT_GetTimerFreq(Encoder_T * p_encoder)
{
#ifdef CONFIG_ENCODER_DYNAMIC_TIMER
    return p_encoder->UnitT_Freq;
#else
    return p_encoder->CONST.TIMER_FREQ;
#endif
}

/* Common */
static inline bool _Encoder_DeltaT_Capture(Encoder_T * p_encoder)
{
    bool isValid = !HAL_Encoder_ReadTimerOverflow(p_encoder->CONST.P_HAL_ENCODER_TIMER);
    // p_encoder->DeltaT = (HAL_Encoder_ReadTimer(p_encoder->CONST.P_HAL_ENCODER_TIMER) + p_encoder->DeltaT) / 2U;
    if(isValid == true) { p_encoder->DeltaT = HAL_Encoder_ReadTimer(p_encoder->CONST.P_HAL_ENCODER_TIMER); }
    else { HAL_Encoder_ClearTimerOverflow(p_encoder->CONST.P_HAL_ENCODER_TIMER); }
    HAL_Encoder_WriteTimer(p_encoder->CONST.P_HAL_ENCODER_TIMER, 0U);
    return isValid;
}

/******************************************************************************/
/*!
    @brief     Capture DeltaT - Pulse Edge Polling/ISR
            Interval cannot be greater than 0xFFFF [ticks] => (0xFFFF / TIMER_FREQ) [seconds]
            Low EncoderPulseFreq, < POLLING_FREQ, use interpolation.
*/
/******************************************************************************/
static inline void Encoder_DeltaT_Capture(Encoder_T * p_encoder)
{
    if(_Encoder_DeltaT_Capture(p_encoder) == false) { p_encoder->DeltaT = ENCODER_TIMER_MAX; }
}

static inline bool Encoder_DeltaT_CheckStop(Encoder_T * p_encoder)
{
    return (p_encoder->DeltaT == ENCODER_TIMER_MAX);
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
static inline uint32_t _Encoder_GetExtendedTimerDelta(Encoder_T * p_encoder)
{
#ifdef CONFIG_ENCODER_EXTENDED_TIMER_CHECK_OVERFLOW
    /* Millis overflow 40+ days */
    uint32_t time = *(p_encoder->CONST.P_EXTENDED_TIMER);
    return (time < p_encoder->ExtendedDeltaTimerSaved) ?
        (UINT32_MAX - p_encoder->ExtendedDeltaTimerSaved + time + 1U) :
        *(p_encoder->CONST.P_EXTENDED_TIMER) - p_encoder->ExtendedTimerPrev;
#else
    return *(p_encoder->CONST.P_EXTENDED_TIMER) - p_encoder->ExtendedTimerPrev;
#endif
}

/*
    Call on Encoder Edge.
    32 bit DeltaT overflow should be caught by CheckExtendedStop, which is a shorter period
*/
static inline void Encoder_DeltaT_CaptureExtended(Encoder_T * p_encoder)
{
    if(_Encoder_DeltaT_Capture(p_encoder) == false) { p_encoder->DeltaT = _Encoder_GetExtendedTimerDelta(p_encoder) * p_encoder->ExtendedTimerConversion; }
    p_encoder->ExtendedTimerPrev = *(p_encoder->CONST.P_EXTENDED_TIMER);
}

/*
    Use Extended Timer to check 0 speed.
*/
static inline bool Encoder_DeltaT_CheckExtendedStop(Encoder_T * p_encoder)
{
    return (_Encoder_GetExtendedTimerDelta(p_encoder) > p_encoder->Config.ExtendedDeltaTStop);
}

/******************************************************************************/
/*!
    @brief  Angle Interpolation Functions

    Estimate Angle each control cycle in between encoder counts
    AngularSpeed * AngleIndex / POLLING_FREQ
        (AngleIndex / POLLING_FREQ) * [1(DeltaD) * UnitT_Freq / DeltaT) * (AngleSize / EncoderResolution)]
        AngleIndex * [1(DeltaD) * AngleSize * UnitT_Freq / EncoderResolution / POLLING_FREQ] / DeltaT
    AngleIndex [0:InterpolationCount]

    Only when POLLING_FREQ > EncoderPulseFreq, i.e. 0 encoder counts per poll, polls per encoder count > 1
    e.g. High res break even point
        8192 CountsPerRevolution, 20Khz POLLING_FREQ => 146 RPM
*/
/******************************************************************************/
/*!
    @brief InterpolateAngle
    AngleIndex * 1(DeltaD) * [UnitInterpolateAngle] / DeltaT
    UnitInterpolateAngle = [ENCODER_ANGLE_DEGREES[65536] * TIMER_FREQ / POLLING_FREQ / CountsPerRevolution]
*/
static inline uint32_t Encoder_DeltaT_InterpolateAngleIndex(Encoder_T * p_encoder, uint32_t pollingIndex)
{
    uint32_t angle = pollingIndex * p_encoder->UnitInterpolateAngle / p_encoder->DeltaT;
    return (angle > p_encoder->InterpolateAngleLimit) ? p_encoder->InterpolateAngleLimit : angle;
}

static inline uint32_t Encoder_DeltaT_ProcInterpolateAngle(Encoder_T * p_encoder)
{
    p_encoder->InterpolateAngleIndex++;
    return Encoder_DeltaT_InterpolateAngleIndex(p_encoder, p_encoder->InterpolateAngleIndex);
}

static inline void Encoder_DeltaT_ZeroInterpolateAngle(Encoder_T * p_encoder)
{
    p_encoder->InterpolateAngleIndex = 0U;
}

/*
    InterpolationCount - numbers of Polls per encoder count, per DeltaT Capture, AngleIndex max
    POLLING_FREQ/EncoderPulseFreq == POLLING_FREQ / (TIMER_FREQ / DeltaT);
*/
static inline uint32_t Encoder_DeltaT_GetInterpolationCount(Encoder_T * p_encoder)
{
    return p_encoder->CONST.POLLING_FREQ * p_encoder->DeltaT / p_encoder->CONST.TIMER_FREQ;
}

static inline uint32_t Encoder_DeltaT_RotationalSpeedToInterpolationCount_RPM(Encoder_T * p_encoder, uint16_t mechRpm)
{
    return p_encoder->CONST.POLLING_FREQ * 60U / (p_encoder->Config.CountsPerRevolution * mechRpm);
}

static inline uint32_t Encoder_DeltaT_InterpolationCountToRotationalSpeed_RPM(Encoder_T * p_encoder, uint16_t interpolationCount)
{
    return Encoder_DeltaT_RotationalSpeedToInterpolationCount_RPM(p_encoder, interpolationCount);
}

/******************************************************************************/
/*!
    @brief DeltaT only Speed Functions
*/
/******************************************************************************/
/******************************************************************************/
/*!
    @brief Scalar Speed
*/
/******************************************************************************/
static inline uint32_t Encoder_DeltaT_GetScalarSpeed(Encoder_T * p_encoder)
{
    return p_encoder->UnitScalarSpeed / p_encoder->DeltaT;
}

/******************************************************************************/
/*!
    @brief Angular/Rotational Speed
*/
/******************************************************************************/
static inline uint32_t Encoder_DeltaT_GetAngularSpeed(Encoder_T * p_encoder)
{
    /* p_encoder->UnitAngularSpeed set to 0U if overflow */
    // return (p_encoder->UnitAngularSpeed == 0U) ?
    //     ((p_encoder->CONST.TIMER_FREQ / p_encoder->DeltaT) << ENCODER_ANGLE_BITS) / p_encoder->Config.CountsPerRevolution : p_encoder->UnitAngularSpeed / p_encoder->DeltaT;
}

static inline uint32_t Encoder_DeltaT_GetRotationalSpeed(Encoder_T * p_encoder)
{
    // return _Encoder_CalcRotationalSpeed(p_encoder, 1U, p_encoder->DeltaT);
}

static inline uint32_t Encoder_DeltaT_GetRotationalSpeed_RPM(Encoder_T * p_encoder)
{
    // check (p_encoder->UnitRotationalSpeed60 == 0U) || *60 > INTMAX
    // return _Encoder_CalcRotationalSpeed(p_encoder, 60U, p_encoder->DeltaT);
}

static inline uint32_t Encoder_DeltaT_FromRotationalSpeed_RPM(Encoder_T * p_encoder, uint32_t rpm)
{
    // check (p_encoder->UnitAngularSpeed == 0U) || *60 > INTMAX
    return (rpm == 0U) ? 0U : p_encoder->CONST.TIMER_FREQ * 60U / (p_encoder->Config.CountsPerRevolution * rpm);
}

static inline uint32_t Encoder_DeltaT_ToRotationalSpeed_RPM(Encoder_T * p_encoder, uint32_t deltaT_ticks)
{
    return Encoder_DeltaT_FromRotationalSpeed_RPM(p_encoder, deltaT_ticks);
}

/******************************************************************************/
/*!
    @brief Linear Speed
*/
/******************************************************************************/
/* In UnitsPerSecond */
static inline uint32_t Encoder_DeltaT_GetLinearSpeed(Encoder_T * p_encoder)
{
    // return Encoder_CalcLinearSpeed(p_encoder, 1U, p_encoder->DeltaT);
}

static inline uint32_t Encoder_DeltaT_FromLinearSpeed(Encoder_T * p_encoder, uint32_t speed_UnitsPerSecond)
{
    return (speed_UnitsPerSecond == 0U) ? 0U : p_encoder->UnitSurfaceSpeed / speed_UnitsPerSecond;
}

static inline uint32_t Encoder_DeltaT_FromLinearSpeed_UnitsPerMinute(Encoder_T * p_encoder, uint32_t speed_UnitsPerMinute)
{
    return (speed_UnitsPerMinute == 0U) ? 0U : p_encoder->UnitSurfaceSpeed * 60U / speed_UnitsPerMinute;
}

static inline uint32_t Encoder_DeltaT_ToLinearSpeed(Encoder_T * p_encoder, uint32_t deltaT_ticks)
{
    // return Encoder_DeltaT_FromLinearSpeed(p_encoder, deltaT_ticks); /* Same division */
}

static inline uint32_t Encoder_DeltaT_ToLinearSpeed_UnitsPerMinute(Encoder_T * p_encoder, uint32_t deltaT_Ticks)
{
    // return Encoder_DeltaT_FromLinearSpeed_UnitsPerMinute(p_encoder, deltaT_Ticks); /* Same division */
}

static inline uint32_t Encoder_DeltaT_GetGroundSpeed_Mph(Encoder_T * p_encoder)
{
    return Encoder_CalcGroundSpeed_Mph(p_encoder, 1U, p_encoder->DeltaT);
}

static inline uint32_t Encoder_DeltaT_GetGroundSpeed_Kmh(Encoder_T * p_encoder)
{
    return Encoder_CalcGroundSpeed_Kmh(p_encoder, 1U, p_encoder->DeltaT);
}

/******************************************************************************/
/*!
    @brief     Extern Declarations
*/
/*! @{ */
/******************************************************************************/
extern void _Encoder_DeltaT_InitTimer(Encoder_T * p_encoder);
extern void Encoder_DeltaT_Init(Encoder_T * p_encoder);
extern void Encoder_DeltaT_SetInitial(Encoder_T * p_encoder);
extern void Encoder_DeltaT_SetExtendedWatchStop_Millis(Encoder_T * p_encoder, uint16_t effectiveStopTime_Millis);
extern void Encoder_DeltaT_SetExtendedWatchStop_RPM(Encoder_T * p_encoder);
extern void Encoder_DeltaT_SetInterpolateAngleScalar(Encoder_T * p_encoder, uint16_t scalar);
/******************************************************************************/
/*! @} */
/******************************************************************************/

#endif

