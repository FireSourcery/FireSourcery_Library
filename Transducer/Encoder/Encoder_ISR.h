#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery

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
    @file   Encoder_ISR.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/

// Your code here

#include "Encoder.h"
#include "Encoder_DeltaD.h"
#include "Encoder_DeltaT.h"
#include "Encoder_ModeDT.h"

/******************************************************************************/
/*!
    @brief     SW Capture Functions -
    CONFIG_ENCODER_HW_EMULATED mode DeltaD, ModeDT; DeltaT ISR Mode
*/
/******************************************************************************/

/******************************************************************************/
/* Common  */
/******************************************************************************/
static inline Encoder_Phases_T _Encoder_ReadPins(const Encoder_T * p_encoder)
{
    return (Encoder_Phases_T) { .A = Pin_Input_ReadPhysical(&p_encoder->PIN_A), .B = Pin_Input_ReadPhysical(&p_encoder->PIN_B) };
}

static inline uint8_t _Encoder_CaptureStateOf(Encoder_State_T * p_encoder, Encoder_Phases_T phases)
{
    p_encoder->Phases.PrevA = p_encoder->Phases.A;
    p_encoder->Phases.PrevB = p_encoder->Phases.B;
    p_encoder->Phases.A = phases.A;
    p_encoder->Phases.B = phases.B;
    return p_encoder->Phases.Value;
}

static inline uint8_t _Encoder_CapturePhasesState(const Encoder_T * p_encoder)
{
    return _Encoder_CaptureStateOf(p_encoder->P_STATE, _Encoder_ReadPins(p_encoder));
}

/* count = {-2, -1, 0, +1, +2} */
static inline void _Encoder_CaptureCount(Encoder_State_T * p_encoder, int8_t count)
{
    // count = count * directionCalibration
    p_encoder->CounterD += count;
    // p_encoder->TotalD += count;
    /* instead of imitating the hw decoder case, capture a separate position */
    p_encoder->Angle32 += ((int32_t)count * p_encoder->UnitAngleD);
}


/******************************************************************************/
/*
    Quadrature, Signed Direction
    Captures as ALeadB is Positive, compensate on user Get
*/
/******************************************************************************/
#define _ENCODER_TABLE_INC_2 (2)
#define _ENCODER_TABLE_DEC_2 (-2)

static inline void _Encoder_Quadrature_CapturePulse(const Encoder_T * p_encoder)
{
    /* Phase in XXXXBABA order, ALeadB as increment */
    static const int8_t ENCODER_TABLE[] =
    {
        0, 1, -1, _ENCODER_TABLE_INC_2, /* 0011: as B rising edge, missed A rising edge */
        -1, 0, _ENCODER_TABLE_INC_2, 1, /* 0110: as A rising edge, missed B rising edge */
        1, _ENCODER_TABLE_DEC_2, 0, -1, /* 1001: as B falling edge, missed A rising edge */
        _ENCODER_TABLE_DEC_2, -1, 1, 0  /* 1100: as A falling edge, missed B rising edge */
    };

    // if (count == +2||-2) { p_encoder->ErrorCount++; }
    _Encoder_CaptureCount(p_encoder->P_STATE, ENCODER_TABLE[_Encoder_CapturePhasesState(p_encoder)]);
}

/* Alternatively, single phase signed capture or combine with B */
static inline void _Encoder_Quadrature_CapturePulse_PhaseA(const Encoder_T * p_encoder)
{
    _Encoder_CaptureCount(p_encoder->P_STATE, ((Pin_Input_ReadPhysical(&p_encoder->PIN_B) == false) ? 1 : -1));
}

static inline void _Encoder_Quadrature_CapturePulse_PhaseB(const Encoder_T * p_encoder)
{
    _Encoder_CaptureCount(p_encoder->P_STATE, ((Pin_Input_ReadPhysical(&p_encoder->PIN_A) == true) ? 1 : -1));
}

/******************************************************************************/
/* Single Phase, Non-Directional */
/******************************************************************************/
static inline void _Encoder_SinglePhase_CapturePulse(const Encoder_T * p_encoder)
{
    _Encoder_CaptureCount(p_encoder->P_STATE, 1);
}

/******************************************************************************/
/*
    User compile time implement mode
*/
/******************************************************************************/
static inline void Encoder_Quadrature_CapturePulse(const Encoder_T * p_encoder)
{
    _Encoder_Quadrature_CapturePulse(p_encoder);
    Encoder_DeltaT_Capture(p_encoder);
    // Encoder_DeltaT_CaptureExtended(p_encoder);
    Encoder_DeltaT_ZeroInterpolateAngle(p_encoder->P_STATE);
}

/* Signed capture use QuadratureEnable */
/* -1, 0, 1 */
// static inline void Encoder_CaptureCount_Polling(Encoder_State_T * p_encoder, int8_t sign)
// {
//     _Encoder_CaptureCount(p_encoder, sign);
//     Encoder_DeltaT_CaptureExtended(p_encoder);
//     Encoder_DeltaT_ZeroInterpolateAngle(p_encoder->P_STATE);
// }

static inline void Encoder_SinglePhase_CapturePulse(const Encoder_T * p_encoder)
{
    _Encoder_SinglePhase_CapturePulse(p_encoder);
    Encoder_DeltaT_CaptureExtended(p_encoder);
    Encoder_DeltaT_ZeroInterpolateAngle(p_encoder->P_STATE);
}



/******************************************************************************/
/* Default call from ISR. Quadrature and Single Phase select via IsQuadratureMode flag */
/******************************************************************************/
static inline void Encoder_CapturePulse(const Encoder_T * p_encoder)
{
    if (_Encoder_IsQuadratureCaptureEnabled(p_encoder->P_STATE) == true) { _Encoder_Quadrature_CapturePulse(p_encoder); }
    else { _Encoder_SinglePhase_CapturePulse(p_encoder); }
    // Encoder_CaptureMode_Proc(p_encoder, _Encoder_Quadrature_CapturePulse, _Encoder_SinglePhase_CapturePulse); /* Quadrature On/Off Switch */
    // Encoder_DeltaT_CaptureExtended(p_encoder);
    Encoder_DeltaT_Capture(p_encoder);
    Encoder_DeltaT_ZeroInterpolateAngle(p_encoder->P_STATE);
}

/******************************************************************************/
/* Index */
/******************************************************************************/
static inline void Encoder_CaptureIndex(Encoder_State_T * p_encoder)
{
#if defined(CONFIG_ENCODER_HW_DECODER)
    // HAL_Encoder_ClearCounter(p_encoder->P_HAL_ENCODER_COUNTER);
    // sync Angle with count
#elif defined(CONFIG_ENCODER_HW_EMULATED)
    // _Encoder_SetCounterD(p_encoder, 0);
#endif
    p_encoder->IndexAngleError = p_encoder->Angle32 - p_encoder->Config.IndexAngleRef;
    p_encoder->Angle32 = p_encoder->Config.IndexAngleRef;
    // p_encoder->Angle32 = p_encoder->IndexAngleRef;
    // p_encoder->CounterOnIndex = p_encoder->CounterD;
    p_encoder->IndexCount++;
}

/******************************************************************************/
/*!
    ISRs
*/
/*! @{ */
/******************************************************************************/
/******************************************************************************/
/*!
    Configured using IsQuadratureMode flag
*/
/******************************************************************************/
static inline void Encoder_OnPhaseA_ISR(const Encoder_T * p_encoder)
{
    HAL_Encoder_ClearPinInterrupt(p_encoder->P_HAL_PIN_A, p_encoder->PIN_A_ID);
    Encoder_CapturePulse(p_encoder);
}

static inline void Encoder_OnPhaseB_ISR(const Encoder_T * p_encoder)
{
    HAL_Encoder_ClearPinInterrupt(p_encoder->P_HAL_PIN_B, p_encoder->PIN_B_ID);
    Encoder_CapturePulse(p_encoder);
}

/* Index Pin */
static inline void Encoder_OnIndex_ISR(const Encoder_T * p_encoder)
{
    HAL_Encoder_ClearPinInterrupt(p_encoder->P_HAL_PIN_Z, p_encoder->PIN_Z_ID);
    Encoder_CaptureIndex(p_encoder->P_STATE);
}

/* Shared A, B ISR */
static inline void Encoder_OnPhaseAB_ISR(const Encoder_T * p_encoder)
{
    if         (HAL_Encoder_ReadPinInterrupt(p_encoder->P_HAL_PIN_A, p_encoder->PIN_A_ID) == true) { Encoder_OnPhaseA_ISR(p_encoder); }
    else if    (HAL_Encoder_ReadPinInterrupt(p_encoder->P_HAL_PIN_B, p_encoder->PIN_B_ID) == true) { Encoder_OnPhaseB_ISR(p_encoder); }
}

/* Shared A, B, Index ISR */
static inline void Encoder_OnPhaseABZ_ISR(const Encoder_T * p_encoder)
{
    if         (HAL_Encoder_ReadPinInterrupt(p_encoder->P_HAL_PIN_A, p_encoder->PIN_A_ID) == true) { Encoder_OnPhaseA_ISR(p_encoder); }
    else if    (HAL_Encoder_ReadPinInterrupt(p_encoder->P_HAL_PIN_B, p_encoder->PIN_B_ID) == true) { Encoder_OnPhaseB_ISR(p_encoder); }
    else if    (HAL_Encoder_ReadPinInterrupt(p_encoder->P_HAL_PIN_Z, p_encoder->PIN_Z_ID) == true) { Encoder_OnIndex_ISR(p_encoder); }
}

static inline void Encoder_OnPhaseC_Hall_ISR(const Encoder_T * p_encoder)
{
    HAL_Encoder_ClearPinInterrupt(p_encoder->P_HAL_PIN_Z, p_encoder->PIN_Z_ID);
    Encoder_CapturePulse(p_encoder);
}

/******************************************************************************/
/*!
    Upper layer configures ISRs
*/
/******************************************************************************/
static inline void _Encoder_OnPhaseA_ISR(const Encoder_T * p_encoder) { HAL_Encoder_ClearPinInterrupt(p_encoder->P_HAL_PIN_A, p_encoder->PIN_A_ID); }
static inline void _Encoder_OnPhaseB_ISR(const Encoder_T * p_encoder) { HAL_Encoder_ClearPinInterrupt(p_encoder->P_HAL_PIN_B, p_encoder->PIN_B_ID); }
static inline void _Encoder_OnPhaseZ_ISR(const Encoder_T * p_encoder) { HAL_Encoder_ClearPinInterrupt(p_encoder->P_HAL_PIN_Z, p_encoder->PIN_Z_ID); }

/* Shared A, B ISR */
static inline void _Encoder_OnPhaseAB_ISR(const Encoder_T * p_encoder)
{
    if      (HAL_Encoder_ReadPinInterrupt(p_encoder->P_HAL_PIN_A, p_encoder->PIN_A_ID) == true) { _Encoder_OnPhaseA_ISR(p_encoder); }
    else if (HAL_Encoder_ReadPinInterrupt(p_encoder->P_HAL_PIN_B, p_encoder->PIN_B_ID) == true) { _Encoder_OnPhaseB_ISR(p_encoder); }
}
/******************************************************************************/
/*! @} */
/******************************************************************************/
