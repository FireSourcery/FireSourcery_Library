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
    @file    Encoder_DeltaD.h
    @author FireSourcery
    @brief     Capture DeltaD per sample time: variable DeltaD, DeltaT is fixed, 1.
    @version V0
*/
/******************************************************************************/
#ifndef ENCODER_DELTA_D_H
#define ENCODER_DELTA_D_H

#include "Encoder.h"

/*!
    @brief     Capture Increasing DeltaD between 2 samples.
*/
#if defined(CONFIG_ENCODER_HW_DECODER)
static inline uint32_t _Encoder_CaptureDeltaD(Encoder_T * p_encoder, uint32_t timerCounterMax, uint32_t timerCounterValue)
{
    uint32_t delta = (timerCounterValue < p_encoder->CounterD) ?
        (timerCounterMax - p_encoder->CounterD + timerCounterValue + 1U) :    /* TimerCounter overflow */
        (timerCounterValue - p_encoder->CounterD);                             /* Normal case */
    p_encoder->CounterD = timerCounterValue;
    return delta;
}
#endif

/******************************************************************************/
/*!
    @brief     Capture DeltaD
*/
/******************************************************************************/
static inline void Encoder_DeltaD_Capture(Encoder_T * p_encoder)
{
#if defined(CONFIG_ENCODER_HW_DECODER)
    /* For common interface functions. Emulated Capture in ISR */
    uint16_t counterD = HAL_Encoder_ReadCounter(p_encoder->CONST.P_HAL_ENCODER_COUNTER);
    p_encoder->DeltaD = _Encoder_CaptureDeltaD(p_encoder, p_encoder->Config.CountsPerRevolution - 1U, counterD);
    // p_encoder->TotalD += p_encoder->DeltaD;
    // p_encoder->Angle32 = counterD * p_encoder->UnitAngularD;
    //quadrature check overflow flag
#else
    p_encoder->DeltaD = p_encoder->CounterD;
    p_encoder->CounterD = 0;
#endif
}

/******************************************************************************/
/*!
    Displacement
*/
/******************************************************************************/
static inline int32_t  Encoder_DeltaD_Get(const Encoder_T * p_encoder)                { return p_encoder->DeltaD; }
static inline uint32_t Encoder_DeltaD_GetDeltaAngle(const Encoder_T * p_encoder)      { return Encoder_AngleOfCount(p_encoder, p_encoder->DeltaD); }
static inline uint32_t Encoder_DeltaD_GetDeltaDistance(const Encoder_T * p_encoder)   { return Encoder_DistanceOfCount(p_encoder, p_encoder->DeltaD); }


/******************************************************************************/
/*!
    @brief     DeltaD only Speed Functions
*/
/******************************************************************************/
/*
    Speed to DeltaD conversion
    1 Division for inverse conversion.
*/
static inline uint32_t Encoder_DeltaD_OfRotationalSpeed_RPM(const Encoder_T * p_encoder, uint32_t rpm)
{
    return (rpm * p_encoder->Config.CountsPerRevolution) / (p_encoder->CONST.SAMPLE_FREQ * 60U);
}

static inline uint32_t Encoder_DeltaD_ToRotationalSpeed_RPM(const Encoder_T * p_encoder, uint32_t deltaD_Ticks)
{
    return (deltaD_Ticks * p_encoder->CONST.SAMPLE_FREQ * 60U) / p_encoder->Config.CountsPerRevolution;
}

/*!
    @return DeltaD in counter ticks.
*/
static inline uint32_t Encoder_DeltaD_OfAngularSpeed(const Encoder_T * p_encoder, uint32_t angularSpeed_UserDegreesPerSecond)
{
    // return angularSpeed_UserDegreesPerSecond / p_encoder->UnitAngularSpeed;
}

static inline uint32_t Encoder_DeltaD_ToAngularSpeed(const Encoder_T * p_encoder, uint32_t deltaD_Ticks)
{
    // return _Encoder_CalcAngularSpeed(p_encoder, deltaD_Ticks, 1U);
}

static inline uint32_t Encoder_DeltaD_OfLinearSpeed(const Encoder_T * p_encoder, uint32_t speed_UnitsPerSecond)
{
    // return speed_UnitsPerSecond / p_encoder->UnitSurfaceSpeed;
}

static inline uint32_t Encoder_DeltaD_OfLinearSpeed_UnitsPerMinute(const Encoder_T * p_encoder, uint32_t speed_UnitsPerMinute)
{
    // return speed_UnitsPerMinute * 60U / p_encoder->UnitSurfaceSpeed;
}

static inline uint32_t Encoder_DeltaD_ToLinearSpeed(const Encoder_T * p_encoder, uint32_t deltaD_Ticks)
{
    // return Encoder_CalcLinearSpeed(p_encoder, deltaD_Ticks, 1U);
}

static inline uint32_t Encoder_DeltaD_ToLinearSpeed_UnitsPerMinute(const Encoder_T * p_encoder, uint32_t deltaD_Ticks)
{
    // return Encoder_CalcLinearSpeed(p_encoder, deltaD_Ticks * 60U, 1U);
}


/******************************************************************************/
/*!
    Scalar
*/
/******************************************************************************/
static inline uint32_t Encoder_DeltaD_GetScalarSpeed(const Encoder_T * p_encoder)
{
    return p_encoder->DeltaD * p_encoder->UnitScalarSpeed >> p_encoder->UnitScalarSpeedShift;
}

/******************************************************************************/
/*!
    Angular
*/
/******************************************************************************/
static inline uint32_t Encoder_DeltaD_GetAngularSpeed(const Encoder_T * p_encoder)
{
    // return _Encoder_CalcAngularSpeed(p_encoder, p_encoder->DeltaD, 1U);
    return p_encoder->DeltaD * p_encoder->UnitAngularSpeed >> p_encoder->UnitAngularSpeedShift;
}

static inline uint32_t Encoder_DeltaD_GetRotationalSpeed_RPM(const Encoder_T * p_encoder)
{
    // return _Encoder_CalcRotationalSpeed_Shift(p_encoder, p_encoder->DeltaD * 60U, 1U);
    return Encoder_DeltaD_ToRotationalSpeed_RPM(p_encoder, p_encoder->DeltaD);
}

/******************************************************************************/
/*!
    Linear
*/
/******************************************************************************/
/*!
    Overflow caution: Max DeltaD = UINT32_MAX / UnitSurfaceSpeed
*/
static inline uint32_t Encoder_DeltaD_GetLinearSpeed(const Encoder_T * p_encoder)
{
    // return Encoder_CalcLinearSpeed(p_encoder, p_encoder->DeltaD, 1U);
    return p_encoder->DeltaD * p_encoder->UnitSurfaceSpeed >> p_encoder->UnitSurfaceSpeedShift;
}

static inline uint32_t Encoder_DeltaD_GetGroundSpeed_Mph(const Encoder_T * p_encoder)
{
    return Encoder_GroundSpeedOf_Mph(p_encoder, p_encoder->DeltaD, 1U);
}

static inline uint32_t Encoder_DeltaD_GetGroundSpeed_Kmh(const Encoder_T * p_encoder)
{
    return Encoder_GroundSpeedOf_Kmh(p_encoder, p_encoder->DeltaD, 1U);
}

/******************************************************************************/
/*!
    @brief     Extern Declarations
*/
/*! @{ */
/******************************************************************************/
extern void _Encoder_DeltaD_InitCounter(Encoder_T * p_encoder);
extern void Encoder_DeltaD_Init(Encoder_T * p_encoder);
extern void Encoder_DeltaD_SetInitial(Encoder_T * p_encoder);
/******************************************************************************/
/*! @} */
/******************************************************************************/

#endif
