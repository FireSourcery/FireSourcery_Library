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
/* Common  */
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

static inline void _Encoder_Quadrature_CapturePulse(Encoder_T * p_encoder)
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
    _Encoder_CaptureCount(p_encoder, ENCODER_TABLE[_Encoder_CapturePhasesState(p_encoder)]);
}

/* Alternatively, single phase signed capture or combine with B */
static inline void _Encoder_Quadrature_CapturePulse_PhaseA(Encoder_T * p_encoder)
{
    _Encoder_CaptureCount(p_encoder, ((Pin_Input_ReadPhysical(&p_encoder->PinB) == false) ? 1 : -1));
}

static inline void _Encoder_Quadrature_CapturePulse_PhaseB(Encoder_T * p_encoder)
{
    _Encoder_CaptureCount(p_encoder, ((Pin_Input_ReadPhysical(&p_encoder->PinA) == true) ? 1 : -1));
}

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
    Encoder_DeltaT_Capture(p_encoder);
    // Encoder_DeltaT_CaptureExtended(p_encoder);
    // Encoder_DeltaT_ZeroInterpolateAngle(p_encoder);
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
    // Encoder_DeltaT_Capture(p_encoder);
    Encoder_DeltaT_ZeroInterpolateAngle(p_encoder);
}

/* External */
/* -1, 0, 1 */
static inline void Encoder_CaptureCount_Polling(Encoder_T * p_encoder, int8_t count)
{
    _Encoder_CaptureCount(p_encoder, count);
    Encoder_DeltaT_CaptureExtended(p_encoder);
    Encoder_DeltaT_ZeroInterpolateAngle(p_encoder);
}

/******************************************************************************/
/* Index */
/******************************************************************************/
static inline void Encoder_CaptureIndex(Encoder_T * p_encoder)
{
#if defined(CONFIG_ENCODER_HW_DECODER)
    // HAL_Encoder_ClearCounter(p_encoder->CONST.P_HAL_ENCODER_COUNTER);
    // sync Angle with count
#elif defined(CONFIG_ENCODER_HW_EMULATED)
    // _Encoder_SetCounterD(p_encoder, 0);
#endif
    p_encoder->Angle32 = p_encoder->Config.IndexAngleRef;
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
    Encoder_CaptureIndex(p_encoder);
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
