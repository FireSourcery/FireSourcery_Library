/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2022 FireSourcery

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
    @file    Encoder_ISR.h
    @author FireSourcery
    @version V0
    @brief
*/
/******************************************************************************/
#ifndef ENCODER_ISR_H
#define ENCODER_ISR_H

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
/******************************************************************************/
static inline uint8_t _Encoder_CapturePhasesState(Encoder_T * p_encoder)
{
    p_encoder->Phases.PrevA = p_encoder->Phases.A;
    p_encoder->Phases.PrevB = p_encoder->Phases.B;
    p_encoder->Phases.A = Pin_Input_ReadPhysical(&p_encoder->PinA);
    p_encoder->Phases.B = Pin_Input_ReadPhysical(&p_encoder->PinB);
    return p_encoder->Phases.Value;
}

/* count = {-1, 0, +1} */
static inline void _Encoder_CaptureCount(Encoder_T * p_encoder, int8_t count)
{
    // count = count * directionCalibration
    p_encoder->CounterD += count;
    // p_encoder->TotalD += count;
    p_encoder->Angle32 += ((int32_t)count * p_encoder->UnitAngularD);
}

/******************************************************************************/
/* Quadrature */
/******************************************************************************/
static inline void _Encoder_Quadrature_CaptureCount(Encoder_T * p_encoder, int8_t count)
{
    if (count == _ENCODER_TABLE_ERROR) { p_encoder->ErrorCount++; }
    else { _Encoder_CaptureCount(p_encoder, count); }
}

/*
    Quadrature, Signed Direction
    Captures as ALeadB is Positive, compensate on user Get
*/
static inline void _Encoder_Quadrature_CapturePulse(Encoder_T * p_encoder)
{
    _Encoder_Quadrature_CaptureCount(p_encoder, _ENCODER_TABLE[_Encoder_CapturePhasesState(p_encoder)]);
}

// static inline void _Encoder_CapturePulse_QuadraturePhaseA(Encoder_T * p_encoder)
// {
//     _Encoder_Quadrature_CaptureCount(p_encoder, _ENCODER_TABLE_PHASE_A[_Encoder_CapturePhasesState(p_encoder)]);
// }

// static inline void _Encoder_CapturePulse_QuadraturePhaseARisingEdge(Encoder_T * p_encoder)
// {
//     _Encoder_Quadrature_CaptureCount(p_encoder, ((Pin_Input_ReadPhysical(&p_encoder->PinB) == false) ? 1 : -1));
// }

/******************************************************************************/
/* Single Phase, Non-Directional */
/******************************************************************************/
static inline void _Encoder_SinglePhase_CapturePulse(Encoder_T * p_encoder)
{
    _Encoder_CaptureCount(p_encoder, 1);
}

/******************************************************************************/
/*
    User compile time implement mode
*/
/******************************************************************************/
static inline void Encoder_Quadrature_CapturePulse(Encoder_T * p_encoder)
{
    _Encoder_Quadrature_CapturePulse(p_encoder);
    Encoder_DeltaT_CaptureExtended(p_encoder);
    Encoder_DeltaT_ZeroInterpolateAngle(p_encoder);
}

static inline void Encoder_SinglePhase_CapturePulse(Encoder_T * p_encoder)
{
    _Encoder_SinglePhase_CapturePulse(p_encoder);
    Encoder_DeltaT_CaptureExtended(p_encoder);
    Encoder_DeltaT_ZeroInterpolateAngle(p_encoder);
}

/*
    Quadrature and Single Phase select via IsQuadratureMode flag
*/
static inline void Encoder_CapturePulse(Encoder_T * p_encoder)
{
    Encoder_CaptureMode_Proc(p_encoder, _Encoder_Quadrature_CapturePulse, _Encoder_SinglePhase_CapturePulse); /* Quadrature On/Off Switch */
    Encoder_DeltaT_CaptureExtended(p_encoder);
    Encoder_DeltaT_ZeroInterpolateAngle(p_encoder);
}

static inline void Encoder_CaptureRef(Encoder_T * p_encoder)
{
    _Encoder_SetCounterD(p_encoder, p_encoder->IndexOffsetRef);
    p_encoder->Angle32 = ((int32_t)p_encoder->IndexOffsetRef * p_encoder->UnitAngularD);
    // Encoder_ModeDT_SetInitial(p_encoder);
}

/* External */
/* -1, 0, 1 */
static inline void Encoder_CaptureCount(Encoder_T * p_encoder, int8_t count)
{
    _Encoder_CaptureCount(p_encoder, count);
    Encoder_DeltaT_CaptureExtended(p_encoder);
    Encoder_DeltaT_ZeroInterpolateAngle(p_encoder);
}

static inline void Encoder_CaptureExternal(Encoder_T * p_encoder, int8_t prev, int8_t count)
{
    /* direction changed */
    if (prev != count) { Encoder_ModeDT_SetInitial(p_encoder); }
    else { Encoder_CaptureCount(p_encoder, count); }
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
    Superfluous capture DeltaT for Emulated DeltaD only mode
*/
/******************************************************************************/
static inline void Encoder_OnPhaseA_ISR(Encoder_T * p_encoder)
{
    HAL_Encoder_ClearPinInterrupt(p_encoder->CONST.P_HAL_PIN_A, p_encoder->CONST.PIN_A_ID);
    Encoder_CapturePulse(p_encoder);
}

static inline void Encoder_OnPhaseB_ISR(Encoder_T * p_encoder)
{
    HAL_Encoder_ClearPinInterrupt(p_encoder->CONST.P_HAL_PIN_B, p_encoder->CONST.PIN_B_ID);
    Encoder_CapturePulse(p_encoder);
}

/* Index Pin */
static inline void Encoder_OnIndex_ISR(Encoder_T * p_encoder)
{
    HAL_Encoder_ClearPinInterrupt(p_encoder->CONST.P_HAL_PIN_Z, p_encoder->CONST.PIN_Z_ID);
    p_encoder->IndexCount++;
    _Encoder_SetCounterD(p_encoder, p_encoder->IndexOffsetRef);
    // todo phase index delta
}

/* Shared A, B ISR */
static inline void Encoder_OnPhaseAB_ISR(Encoder_T * p_encoder)
{
    if         (HAL_Encoder_ReadPinInterrupt(p_encoder->CONST.P_HAL_PIN_A, p_encoder->CONST.PIN_A_ID) == true) { Encoder_OnPhaseA_ISR(p_encoder); }
    else if    (HAL_Encoder_ReadPinInterrupt(p_encoder->CONST.P_HAL_PIN_B, p_encoder->CONST.PIN_B_ID) == true) { Encoder_OnPhaseB_ISR(p_encoder); }
}

/* Shared A, B, Index ISR */
static inline void Encoder_OnPhaseABZ_ISR(Encoder_T * p_encoder)
{
    if         (HAL_Encoder_ReadPinInterrupt(p_encoder->CONST.P_HAL_PIN_A, p_encoder->CONST.PIN_A_ID) == true) { Encoder_OnPhaseA_ISR(p_encoder); }
    else if    (HAL_Encoder_ReadPinInterrupt(p_encoder->CONST.P_HAL_PIN_B, p_encoder->CONST.PIN_B_ID) == true) { Encoder_OnPhaseB_ISR(p_encoder); }
    else if    (HAL_Encoder_ReadPinInterrupt(p_encoder->CONST.P_HAL_PIN_Z, p_encoder->CONST.PIN_Z_ID) == true) { Encoder_OnIndex_ISR(p_encoder); }
}

static inline void Encoder_OnPhaseC_Hall_ISR(Encoder_T * p_encoder)
{
    HAL_Encoder_ClearPinInterrupt(p_encoder->CONST.P_HAL_PIN_Z, p_encoder->CONST.PIN_Z_ID);
    Encoder_CapturePulse(p_encoder);
}

/******************************************************************************/
/*!
    Upper layer configures ISRs
*/
/******************************************************************************/
static inline void _Encoder_OnPhaseA_ISR(Encoder_T * p_encoder)
{
    HAL_Encoder_ClearPinInterrupt(p_encoder->CONST.P_HAL_PIN_A, p_encoder->CONST.PIN_A_ID);
}

static inline void _Encoder_OnPhaseB_ISR(Encoder_T * p_encoder)
{
    HAL_Encoder_ClearPinInterrupt(p_encoder->CONST.P_HAL_PIN_B, p_encoder->CONST.PIN_B_ID);
}

static inline void _Encoder_OnPhaseZ_ISR(Encoder_T * p_encoder)
{
    HAL_Encoder_ClearPinInterrupt(p_encoder->CONST.P_HAL_PIN_Z, p_encoder->CONST.PIN_Z_ID);
}

/* Shared A, B ISR */
static inline void _Encoder_OnPhaseAB_ISR(Encoder_T * p_encoder)
{
    if      (HAL_Encoder_ReadPinInterrupt(p_encoder->CONST.P_HAL_PIN_A, p_encoder->CONST.PIN_A_ID) == true) { _Encoder_OnPhaseA_ISR(p_encoder); }
    else if (HAL_Encoder_ReadPinInterrupt(p_encoder->CONST.P_HAL_PIN_B, p_encoder->CONST.PIN_B_ID) == true) { _Encoder_OnPhaseB_ISR(p_encoder); }
}
/******************************************************************************/
/*! @} */
/******************************************************************************/
#endif
