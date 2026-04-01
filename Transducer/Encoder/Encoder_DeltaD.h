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
#include "Math/Angle/math_angle_counter.h"

/******************************************************************************/
/*!
    @brief     Capture DeltaD
*/
/******************************************************************************/
static inline void _Encoder_DeltaD_Capture(const Encoder_T * p_encoder, Encoder_State_T * p_state)
{
#if defined(ENCODER_HW_DECODER)
    /* For common interface functions. Emulated Capture in ISR */
    uint16_t counterD = HAL_Encoder_ReadCounter(p_encoder->P_HAL_ENCODER_COUNTER);
    p_state->DeltaD = _Encoder_CaptureDeltaWrap(p_encoder->Config.CountsPerRevolution - 1U, p_state->CounterPrev, counterD);
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
    Compose Angle_Coutner for conversions.
*/
/******************************************************************************/

/******************************************************************************/
/*!
    Displacement
*/
/******************************************************************************/
static inline int32_t  Encoder_DeltaD(const Encoder_State_T * p_encoder) { return p_encoder->DeltaD; }
static inline uint32_t Encoder_DeltaD_AsAngle(const Encoder_State_T * p_encoder) { /* return Angle_Counter_GetAngle(p_encoder, p_encoder->DeltaD);  */}

static inline uint32_t Encoder_DeltaD_GetAngularSpeed(const Encoder_State_T * p_encoder) { /* angle_speed_of_count( , p_encoder->Config.CountsPerRevolution, p_encoder->DeltaD); */ }
static inline uint32_t Encoder_DeltaD_GetFractSpeed(const Encoder_State_T * p_encoder) { return p_encoder->DeltaD * p_encoder->UnitScalarSpeed >> p_encoder->UnitScalarSpeedShift; }


/******************************************************************************/
/*!
    Extended
*/
/******************************************************************************/
/*!
    Linear Distance
*/
static inline uint32_t Encoder_DistanceOfCount(const Encoder_State_T * p_encoder, uint32_t counterD_Ticks) { return counterD_Ticks * p_encoder->UnitLinearD; }
static inline uint32_t Encoder_CountOfDistance(const Encoder_State_T * p_encoder, uint32_t distance_Units) { return distance_Units / p_encoder->UnitLinearD; }
static inline uint32_t Encoder_DeltaD_AsDistance(const Encoder_State_T * p_encoder) { return Encoder_DistanceOfCount(p_encoder, p_encoder->DeltaD); }

/******************************************************************************/
/*!
    Angular
*/
/******************************************************************************/
static inline uint32_t Encoder_DeltaD_GetRotationalSpeed_RPM(const Encoder_State_T * p_encoder) { return rpm_of_count(1000, p_encoder->Config.CountsPerRevolution, p_encoder->DeltaD); }
static inline uint32_t Encoder_DeltaD_GetAngularSpeed_Rads(const Encoder_State_T * p_encoder){}

/******************************************************************************/
/*!
    Linear
*/
/******************************************************************************/
/*!
    Overflow caution: Max DeltaD = UINT32_MAX / UnitSurfaceSpeed
*/
static inline uint32_t Encoder_DeltaD_GetLinearSpeed(const Encoder_State_T * p_encoder){    return p_encoder->DeltaD * p_encoder->UnitSurfaceSpeed >> p_encoder->UnitSurfaceSpeedShift;}
static inline uint32_t Encoder_DeltaD_GetGroundSpeed_Mph(const Encoder_State_T * p_encoder){    /* return Encoder_GroundSpeedOf_Mph(p_encoder, p_encoder->DeltaD, 1U); */}
static inline uint32_t Encoder_DeltaD_GetGroundSpeed_Kmh(const Encoder_State_T * p_encoder){    /* return Encoder_GroundSpeedOf_Kmh(p_encoder, p_encoder->DeltaD, 1U); */}

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

// static inline void _Encoder_DeltaD_Capture_Quadrature(Encoder_T * p_encoder, Encoder_State_T * p_state)
// {
//     uint32_t counterValue = HAL_Encoder_ReadTimerCounter(p_encoder->P_HAL_ENCODER_COUNTER);

//     /*
//         Unsigned DeltaD capture
//     */
//     if (HAL_Encoder_ReadTimerCounterOverflow(p_encoder->P_HAL_ENCODER_COUNTER) == true)
//     {
//         if (HAL_Encoder_ReadDecoderCounterOverflowIncrement(p_encoder->P_HAL_ENCODER_COUNTER) == true)
//         {
//         }
//         else if (HAL_Encoder_ReadDecoderCounterOverflowDecrement(p_encoder->P_HAL_ENCODER_COUNTER) == true)
//         {
//         }

//         HAL_Encoder_ClearTimerCounterOverflow(p_encoder->P_HAL_ENCODER_COUNTER);
//     }
//     else
//     {
//         p_state->DeltaD = counterValue - p_state->CounterPrev;
//     }
// }