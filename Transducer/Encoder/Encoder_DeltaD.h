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

*/
/******************************************************************************/
#ifndef ENCODER_DELTA_D_H
#define ENCODER_DELTA_D_H

#include "Encoder.h"

/******************************************************************************/
/*!
    @brief     Capture DeltaD
*/
/******************************************************************************/
static inline void _Encoder_DeltaD_Capture(const Encoder_T * p_encoder, Encoder_State_T * p_state)
{
#if defined(CONFIG_ENCODER_HW_DECODER)
    /* For common interface functions. Emulated Capture in ISR */
    uint16_t counterD = HAL_Encoder_ReadCounter(p_encoder->P_HAL_ENCODER_COUNTER);
    p_state->DeltaD = _Encoder_CaptureDeltaWrap(p_encoder->Config.CountsPerRevolution - 1U, p_state->CounterPrev, counterD);
    // p_state->TotalD += p_state->DeltaD;
    // quadrature check overflow flag
    /* Do not clear the counter as it is also the angle in this case */
#else
    /* signed if capture is signed */
    p_state->DeltaD = p_state->CounterD - p_state->CounterPrev; /* int/uint wrap preserves relationship */

    // optionally clear counter
    // p_state->DeltaD = p_state->CounterD;
    // p_state->CounterD = 0;
#endif
    p_state->CounterPrev = p_state->CounterD;
}

static inline void Encoder_DeltaD_Capture(const Encoder_T * p_encoder)
{
    _Encoder_DeltaD_Capture(p_encoder, p_encoder->P_STATE);
}

/******************************************************************************/
/*!
    Displacement
*/
/******************************************************************************/
static inline int32_t  Encoder_DeltaD(const Encoder_State_T * p_encoder) { return p_encoder->DeltaD; }
static inline uint32_t Encoder_DeltaD_AsAngle(const Encoder_State_T * p_encoder) { return _Encoder_AngleOfCount(p_encoder, p_encoder->DeltaD); }
static inline uint32_t Encoder_DeltaD_AsDistance(const Encoder_State_T * p_encoder) { return Encoder_DistanceOfCount(p_encoder, p_encoder->DeltaD); }


/******************************************************************************/
/*!
    todo split DeltaD conversion math
*/
/******************************************************************************/
static inline uint32_t  delta_d_of_rpm(uint32_t cpr, uint32_t sampleFreq, uint32_t rpm) { return (rpm * cpr) / (sampleFreq * 60U); }
static inline uint32_t  delta_d_to_rpm(uint32_t cpr, uint32_t sampleFreq, uint32_t deltaD_Ticks) { return (deltaD_Ticks * sampleFreq * 60U) / cpr; }

/*
    Speed to DeltaD conversion
    1 Division for inverse conversion.
*/
static inline uint32_t Encoder_DeltaD_OfRotationalSpeed_RPM(const Encoder_State_T * p_encoder, uint32_t rpm)
{
    // return (rpm * p_encoder->Config.CountsPerRevolution) / (p_encoder->SAMPLE_FREQ * 60U);
}

static inline uint32_t Encoder_DeltaD_ToRotationalSpeed_RPM(const Encoder_State_T * p_encoder, uint32_t deltaD_Ticks)
{
    // return (deltaD_Ticks * p_encoder->SAMPLE_FREQ * 60U) / p_encoder->Config.CountsPerRevolution;
}

/*!
    @return DeltaD in counter ticks.
*/
static inline uint32_t Encoder_DeltaD_OfAngularSpeed(const Encoder_State_T * p_encoder, uint32_t angularSpeed_degreesPerSecond)
{
    // return angularSpeed_UserDegreesPerSecond / p_encoder->UnitAngularSpeed;
}

static inline uint32_t Encoder_DeltaD_ToAngularSpeed(const Encoder_State_T * p_encoder, uint32_t deltaD_Ticks)
{
    // return _Encoder_CalcAngularSpeed(p_encoder, deltaD_Ticks, 1U);
}

static inline uint32_t Encoder_DeltaD_OfLinearSpeed(const Encoder_State_T * p_encoder, uint32_t speed_UnitsPerSecond)
{
    // return speed_UnitsPerSecond / p_encoder->UnitSurfaceSpeed;
}

static inline uint32_t Encoder_DeltaD_OfLinearSpeed_UnitsPerMinute(const Encoder_State_T * p_encoder, uint32_t speed_UnitsPerMinute)
{
    // return speed_UnitsPerMinute * 60U / p_encoder->UnitSurfaceSpeed;
}

static inline uint32_t Encoder_DeltaD_ToLinearSpeed(const Encoder_State_T * p_encoder, uint32_t deltaD_Ticks)
{
    // return Encoder_CalcLinearSpeed(p_encoder, deltaD_Ticks, 1U);
}

static inline uint32_t Encoder_DeltaD_ToLinearSpeed_UnitsPerMinute(const Encoder_State_T * p_encoder, uint32_t deltaD_Ticks)
{
    // return Encoder_CalcLinearSpeed(p_encoder, deltaD_Ticks * 60U, 1U);
}

/******************************************************************************/
/*!
    Angular
*/
/******************************************************************************/
static inline uint32_t Encoder_DeltaD_GetRotationalSpeed_RPM(const Encoder_State_T * p_encoder)
{
    // return _Encoder_CalcRotationalSpeed_Shift(p_encoder, p_encoder->DeltaD * 60U, 1U);
    return Encoder_DeltaD_ToRotationalSpeed_RPM(p_encoder, p_encoder->DeltaD);
}

static inline uint32_t Encoder_DeltaD_GetAngularSpeed(const Encoder_State_T * p_encoder)
{
    // return _Encoder_CalcAngularSpeed(p_encoder, p_encoder->DeltaD, 1U);
    return p_encoder->DeltaD * p_encoder->UnitAngularSpeed >> p_encoder->UnitAngularSpeedShift;
}

/******************************************************************************/
/*!
    Scalar
*/
/******************************************************************************/
static inline uint32_t Encoder_DeltaD_GetScalarSpeed(const Encoder_State_T * p_encoder)
{
    return p_encoder->DeltaD * p_encoder->UnitScalarSpeed >> p_encoder->UnitScalarSpeedShift;
}

/******************************************************************************/
/*!
    Linear
*/
/******************************************************************************/
/*!
    Overflow caution: Max DeltaD = UINT32_MAX / UnitSurfaceSpeed
*/
static inline uint32_t Encoder_DeltaD_GetLinearSpeed(const Encoder_State_T * p_encoder)
{
    // return Encoder_CalcLinearSpeed(p_encoder, p_encoder->DeltaD, 1U);
    return p_encoder->DeltaD * p_encoder->UnitSurfaceSpeed >> p_encoder->UnitSurfaceSpeedShift;
}

static inline uint32_t Encoder_DeltaD_GetGroundSpeed_Mph(const Encoder_State_T * p_encoder)
{
    return Encoder_GroundSpeedOf_Mph(p_encoder, p_encoder->DeltaD, 1U);
}

static inline uint32_t Encoder_DeltaD_GetGroundSpeed_Kmh(const Encoder_State_T * p_encoder)
{
    return Encoder_GroundSpeedOf_Kmh(p_encoder, p_encoder->DeltaD, 1U);
}

/******************************************************************************/
/*!
    @brief     Extern Declarations
*/
/*! @{ */
/******************************************************************************/
extern void _Encoder_DeltaD_InitCounter(const Encoder_T * p_encoder);
extern void Encoder_DeltaD_Init(const Encoder_T * p_encoder);
extern void Encoder_DeltaD_SetInitial(const Encoder_T * p_encoder);
/******************************************************************************/
/*! @} */
/******************************************************************************/

#endif
