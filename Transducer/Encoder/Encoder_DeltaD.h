/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2021 FireSourcery / The Firebrand Forge Inc

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
    @file      Encoder_DeltaD.h
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
    uint16_t counterD = HAL_Encoder_ReadCounter(p_encoder->CONFIG.P_HAL_ENCODER_COUNTER);
    p_encoder->DeltaD = _Encoder_CaptureDeltaD(p_encoder, p_encoder->Params.CountsPerRevolution - 1U, counterD);
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
    Angle
*/
/******************************************************************************/
static inline uint32_t Encoder_DeltaD_GetDelta(Encoder_T * p_encoder)             { return p_encoder->DeltaD; }
static inline uint32_t Encoder_DeltaD_GetDeltaAngle(Encoder_T * p_encoder)         { return Encoder_ConvertCounterDToAngle(p_encoder, p_encoder->DeltaD); }
static inline uint32_t Encoder_DeltaD_GetDeltaDistance(Encoder_T * p_encoder)     { return Encoder_ConvertCounterDToDistance(p_encoder, p_encoder->DeltaD); }

/******************************************************************************/
/*!
    @brief     DeltaD only Speed Functions
*/
/******************************************************************************/
/******************************************************************************/
/*!
    Scalar
*/
/******************************************************************************/
static inline uint32_t Encoder_DeltaD_GetScalarSpeed(Encoder_T * p_encoder)
{
    return p_encoder->DeltaD * p_encoder->UnitScalarSpeed >> p_encoder->UnitScalarSpeedShift;
}

/******************************************************************************/
/*!
    Angular
*/
/******************************************************************************/
/*

*/
static inline uint32_t Encoder_DeltaD_GetAngularSpeed(Encoder_T * p_encoder)
{
    // return _Encoder_CalcAngularSpeed(p_encoder, p_encoder->DeltaD, 1U);
    return p_encoder->DeltaD * p_encoder->UnitAngularSpeed >> p_encoder->UnitAngularSpeedShift;
}

/*!
    @return DeltaD in counter ticks.
*/
static inline uint32_t Encoder_DeltaD_ConvertFromAngularSpeed(Encoder_T * p_encoder, uint32_t angularSpeed_UserDegreesPerSecond)
{
    // return angularSpeed_UserDegreesPerSecond / p_encoder->UnitAngularSpeed;
}

static inline uint32_t Encoder_DeltaD_ConvertToAngularSpeed(Encoder_T * p_encoder, uint32_t deltaD_Ticks)
{
    // return _Encoder_CalcAngularSpeed(p_encoder, deltaD_Ticks, 1U);
}

static inline uint32_t Encoder_DeltaD_GetRotationalSpeed_RPM(Encoder_T * p_encoder)
{
    // return _Encoder_CalcRotationalSpeed_Shift(p_encoder, p_encoder->DeltaD * 60U, 1U);
}

/*
    Speed to DeltaD conversion
    1 Division for inverse conversion.
*/
static inline uint32_t Encoder_DeltaD_ConvertFromRotationalSpeed_RPM(Encoder_T * p_encoder, uint32_t rpm)
{
    return (rpm * p_encoder->Params.CountsPerRevolution) / (p_encoder->CONFIG.SAMPLE_FREQ * 60U);
}

static inline uint32_t Encoder_DeltaD_ConvertToRotationalSpeed_RPM(Encoder_T * p_encoder, uint32_t deltaD_Ticks)
{
    // return _Encoder_CalcRotationalSpeed_Shift(p_encoder, deltaD_Ticks * 60U, 1U);
}

/******************************************************************************/
/*!
    Linear
*/
/******************************************************************************/
/*!
    Overflow caution: Max DeltaD = UINT32_MAX / UnitSurfaceSpeed
*/
static inline uint32_t Encoder_DeltaD_GetLinearSpeed(Encoder_T * p_encoder)
{
    // return Encoder_CalcLinearSpeed(p_encoder, p_encoder->DeltaD, 1U);
    return p_encoder->DeltaD * p_encoder->UnitSurfaceSpeed >> p_encoder->UnitSurfaceSpeedShift;
}

static inline uint32_t Encoder_DeltaD_ConvertFromLinearSpeed(Encoder_T * p_encoder, uint32_t speed_UnitsPerSecond)
{
    // return speed_UnitsPerSecond / p_encoder->UnitSurfaceSpeed;
}

static inline uint32_t Encoder_DeltaD_ConvertFromLinearSpeed_UnitsPerMinute(Encoder_T * p_encoder, uint32_t speed_UnitsPerMinute)
{
    // return speed_UnitsPerMinute * 60U / p_encoder->UnitSurfaceSpeed;
}

static inline uint32_t Encoder_DeltaD_ConvertToLinearSpeed(Encoder_T * p_encoder, uint32_t deltaD_Ticks)
{
    // return Encoder_CalcLinearSpeed(p_encoder, deltaD_Ticks, 1U);
}

static inline uint32_t Encoder_DeltaD_ConvertToLinearSpeed_UnitsPerMinute(Encoder_T * p_encoder, uint32_t deltaD_Ticks)
{
    // return Encoder_CalcLinearSpeed(p_encoder, deltaD_Ticks * 60U, 1U);
}

static inline uint32_t Encoder_DeltaD_GetGroundSpeed_Mph(Encoder_T * p_encoder)
{
    return Encoder_CalcGroundSpeed_Mph(p_encoder, p_encoder->DeltaD, 1U);
}

static inline uint32_t Encoder_DeltaD_GetGroundSpeed_Kmh(Encoder_T * p_encoder)
{
    return Encoder_CalcGroundSpeed_Kmh(p_encoder, p_encoder->DeltaD, 1U);
}

/******************************************************************************/
/*!
    @brief     Extern Declarations
*/
/*! @{ */
/******************************************************************************/
extern void _Encoder_DeltaD_Init(Encoder_T * p_encoder);
extern void Encoder_DeltaD_Init(Encoder_T * p_encoder);
extern void Encoder_DeltaD_SetInitial(Encoder_T * p_encoder);
/******************************************************************************/
/*! @} */
/******************************************************************************/

#endif
