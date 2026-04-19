#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2026 FireSourcery

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
    @file   Encoder.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/

// Your code here

#include "HAL_Encoder.h"
#include "Peripheral/Pin/Pin.h"

#include "Math/Angle/AngleCounter.h"
#include "Math/Fixed/fixed.h"
#include "Math/math_general.h"

#include "Transducer/Pulse/PulseTimer.h"

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*!
*/
/******************************************************************************/
/*
    Compile time define if chip supports decoder/counter.
*/
#if     defined(ENCODER_HW_DECODER)
#elif   defined(ENCODER_HW_EMULATED)
#else
// #define ENCODER_HW_DECODER
#endif

#if     defined(ENCODER_QUADRATURE_MODE_ENABLE) /* Emulated and Decoder */
#elif   defined(ENCODER_QUADRATURE_MODE_DISABLE)
#else
#define ENCODER_QUADRATURE_MODE_ENABLE
#endif


/******************************************************************************/
/*!
*/
/******************************************************************************/

#ifndef ENCODER_ANGLE_BITS
#define ENCODER_ANGLE_BITS      (16U)
#define ENCODER_ANGLE_DEGREES   ((uint32_t)1UL << ENCODER_ANGLE_BITS)
#define ENCODER_ANGLE_SHIFT     (32U - ENCODER_ANGLE_BITS)
#endif

typedef union Encoder_Phases
{
    struct
    {
        uint8_t A       : 1U;
        uint8_t B       : 1U;
        uint8_t PrevA   : 1U;
        uint8_t PrevB   : 1U;
        uint8_t Resv    : 4U;
    };
    uint8_t Value;
}
Encoder_Phases_T;


// typedef enum Encoder_PositionRef
// {
//     ENCODER_REF_STATE_NONE,
//     ENCODER_REF_STATE_ALIGN,
//     ENCODER_REF_STATE_INDEX_HOME,
// };

typedef enum Encoder_Align
{
    ENCODER_ALIGN_NONE,
    ENCODER_ALIGN_PHASE,    /* Calibrate Aligned */
    ENCODER_ALIGN_ABSOLUTE,
}
Encoder_Align_T;


typedef struct Encoder_Config
{
    uint16_t CountsPerRevolution;        /* Derive Angular Units. */
    uint32_t ScalarSpeedRef_Rpm;         /* Derive Fract16 Units. */

    uint32_t IndexAngleRef;             /* Virtual Index - Index, VirtualIndexOffset */
    uint32_t AlignOffsetRef;            /* Align - Index */

    uint16_t ExtendedDeltaTStop;         /* ExtendedTimer time read as deltaT stopped, default as 1s */
    // move to extension
    // uint16_t SurfaceDiameter;            /* Derive Linear Units. */
    // uint16_t GearRatioInput;             /* Derivce Surface Units. DistancePerRevolution_Divider */
    // uint16_t GearRatioOutput;            /* DistancePerRevolution_Factor */

#if defined(ENCODER_QUADRATURE_MODE_ENABLE)
    bool IsQuadratureCaptureEnabled;    /* Quadrature Mode - enable hardware/emulated quadrature speed capture */
    bool IsALeadBPositive;              /* User runtime calibration for encoder install direction. Accounts for LUT calibration */
#endif
}
Encoder_Config_T;


typedef struct Encoder_State
{
    Encoder_Config_T Config;
// #if defined(ENCODER_HW_EMULATED)
    Encoder_Phases_T Phases; /* Prev State */
    uint32_t ErrorCount;
// #endif

    AngleCounter_T AngleCounter;
    PulseTimer_State_T TimerState;

    int32_t DirectionComp;

    /* Homing */
    uint32_t IndexCount;
    uint32_t IndexAngleRef; /* 32 */
    uint32_t IndexAngleError;
    Encoder_Align_T Align;
    uint32_t AlignOffsetRef;
    uint32_t AlignAngle; /* angle at last align */
    // int32_t AbsoluteOffset;
    bool IsHomed;
}
Encoder_State_T;

// #define ENCODER_STATE_ALLOC() (&(Encoder_State_T){0})

typedef const struct Encoder
{
#if     defined(ENCODER_HW_DECODER)
    HAL_Encoder_Counter_T * P_HAL_ENCODER_COUNTER; /*!< Pulse Counter */
#elif   defined(ENCODER_HW_EMULATED)
    /* HAL_Encoder_Pin_T configures settings not included in Pin_T, case of interrupt support */
    HAL_Encoder_Pin_T * P_HAL_PIN_A; uint32_t PIN_A_ID; /* Encoder_Pin_T */
    HAL_Encoder_Pin_T * P_HAL_PIN_B; uint32_t PIN_B_ID;
    HAL_Encoder_Pin_T * P_HAL_PIN_Z; uint32_t PIN_Z_ID;
    Pin_T PIN_A;
    Pin_T PIN_B;
#endif
    PulseTimer_T TIMER;
    uint32_t POLLING_FREQ;        /*!< Angle Sample Freq. DeltaT Interpolation Freq. */
    Encoder_State_T * P_STATE;    /*!< Pointer to Encoder State. */
    const Encoder_Config_T * P_NVM_CONFIG;
}
Encoder_T;

/*
    TIMER_FREQ time, 16-bit Timer, SAMPLE_FREQ 1000Hz (1ms)
    0xFFFF/50[Mhz] = 1.31[ms]
    0xFFFFFFFF/50[Mhz] = 85.899[ms]
    TIMER_FREQ * 60 < UINT32_MAX for RPM calc
*/
#if defined(ENCODER_HW_EMULATED)
    #define _ENCODER_INIT_HW_COUNTER(p_CounterHal, p_PhaseAHal, PhaseAId, p_PhaseBHal, PhaseBId, p_PhaseZHal, PhaseZId)  \
        .P_HAL_PIN_A = p_PhaseAHal, .PIN_A_ID = PhaseAId, .P_HAL_PIN_B = p_PhaseBHal, .PIN_B_ID = PhaseBId, .P_HAL_PIN_Z = p_PhaseZHal, .PIN_Z_ID = PhaseZId,
    #define _ENCODER_INIT_HW_PINS(p_PinAHal, PinAId, p_PinBHal, PinBId) \
        .PIN_A = PIN_INIT(p_PinAHal, PinAId), .PIN_B = PIN_INIT(p_PinBHal, PinBId),
#else
    #define _ENCODER_INIT_HW_COUNTER(p_CounterHal, p_PhaseAHal, PhaseAId, p_PhaseBHal, PhaseBId, p_PhaseZHal, PhaseZId) .P_HAL_ENCODER_COUNTER  = p_CounterHal,
    #define _ENCODER_INIT_HW_PINS(p_PinAHal, PinAId, p_PinBHal, PinBId)
#endif

#define ENCODER_INIT(p_CounterHal, p_PhaseAHal, PhaseAId, p_PinAHal, PinAId, p_PhaseBHal, PhaseBId, p_PinBHal, PinBId, p_PhaseZHal, PhaseZId, p_TimerHal, TimerFreq, PollingFreq, SpeedSampleFreq, p_ExtendedTimer, ExtendedTimerFreq, p_State, p_Config)    \
{                                                                                                               \
    _ENCODER_INIT_HW_COUNTER(p_CounterHal, p_PhaseAHal, PhaseAId, p_PhaseBHal, PhaseBId, p_PhaseZHal, PhaseZId) \
    _ENCODER_INIT_HW_PINS(p_PinAHal, PinAId, p_PinBHal, PinBId)                                                 \
    .TIMER = PULSE_TIMER_INIT_EXTENDED(p_TimerHal, TimerFreq, SpeedSampleFreq, &((p_State)->TimerState), p_ExtendedTimer, ExtendedTimerFreq), \
    .POLLING_FREQ           = PollingFreq,                                  \
    .P_STATE                = (p_State),                                      \
    .P_NVM_CONFIG           = p_Config,                                      \
}



/******************************************************************************/
/*!
    @brief Hal Abstraction
*/
/******************************************************************************/
/*
    Does not wrap in HW_EMULATED case
        = p_encoder->Angle32 / p_encoder->UnitAngleD;
*/
static inline int32_t Encoder_GetCounterD(const Encoder_State_T * p_encoder)
{
#if     defined(ENCODER_HW_DECODER)
    return HAL_Encoder_ReadCounter(p_encoder->P_HAL_ENCODER_COUNTER);
#elif   defined(ENCODER_HW_EMULATED)
    // return p_encoder->CounterD;
    return p_encoder->AngleCounter.CounterD;
#endif
}

static inline void _Encoder_SetCounterD(Encoder_State_T * p_encoder, int32_t counterD)
{
#if     defined(ENCODER_HW_DECODER)
    HAL_Encoder_WriteCounter(p_encoder->P_HAL_ENCODER_COUNTER, counterD);
#elif   defined(ENCODER_HW_EMULATED)
    // p_encoder->CounterD = counterD;
    p_encoder->AngleCounter.CounterD = counterD;
#endif
}

static inline uint32_t _Encoder_GetAngle32(const Encoder_State_T * p_encoder)
{
#if     defined(ENCODER_HW_DECODER)
    return HAL_Encoder_ReadCounter(p_encoder->P_HAL_ENCODER_COUNTER) * (uint32_t)p_encoder->AngleCounter.Ref.Angle32PerCount;
#elif   defined(ENCODER_HW_EMULATED)
    return p_encoder->AngleCounter.Base.Angle;
#endif
}

// keep different factor
static inline uint16_t _Encoder_GetAngle(const Encoder_State_T * p_encoder)
{
#if     defined(ENCODER_HW_DECODER)
    return HAL_Encoder_ReadCounter(p_encoder->P_HAL_ENCODER_COUNTER) * (uint32_t)p_encoder->AngleCounter.Ref.AnglePerCount;
#elif   defined(ENCODER_HW_EMULATED)
    return Angle_Value(&p_encoder->AngleCounter.Base);
#endif
}

static inline uint16_t Encoder_GetAngle(const Encoder_State_T * p_encoder) { return _Encoder_GetAngle32(p_encoder) >> ENCODER_ANGLE_SHIFT; }

/*  */
static inline void _Encoder_ZeroPulseCount(Encoder_State_T * p_encoder)
{
    AngleCounter_Zero(&p_encoder->AngleCounter);
    Angle_ZeroCaptureState(&p_encoder->AngleCounter.Base);
    p_encoder->IndexCount = 0U;
#if     defined(ENCODER_HW_DECODER)
    HAL_Encoder_WriteCounter(p_encoder->P_HAL_ENCODER_COUNTER, 0);
    HAL_Encoder_ClearCounterOverflow(p_encoder->P_HAL_ENCODER_COUNTER);
#endif
}

static inline void _Encoder_CaptureDeltaD(const Encoder_T * p_encoder, Encoder_State_T * p_state)
{
#if defined(ENCODER_HW_DECODER)
    /* For common interface functions. Emulated Capture in ISR */
    uint16_t counterD = HAL_Encoder_ReadCounter(p_encoder->P_HAL_ENCODER_COUNTER);
    p_state->DeltaD = _Encoder_CaptureDeltaWrap(p_encoder->Config.CountsPerRevolution - 1U, p_state->CounterD, counterD);
    // quadrature check overflow flag
    /* Do not clear the counter as it is also the angle in this case */
#else
    (void)p_encoder; (void)p_state;
    // p_state->DeltaD = AngleCounter_ResolveDeltaD(&p_state->AngleCounter);
#endif
}


/******************************************************************************/
/*!
    @brief Direction
*/
/******************************************************************************/
/* Direction set by outer module */
/* SinglePhase Capture is always positive. assign direction comp */
static inline void Encoder_SinglePhase_CaptureDirection(Encoder_State_T * p_encoder, int8_t direction) { p_encoder->DirectionComp = direction; }
/* 0 until set */
static inline int32_t _Encoder_SinglePhase_GetDirection(const Encoder_State_T * p_encoder) { return p_encoder->DirectionComp; }

static inline bool _Encoder_IsQuadratureCaptureEnabled(const Encoder_State_T * p_encoder)
{
#if     defined(ENCODER_QUADRATURE_MODE_ENABLE)
    return (p_encoder->Config.IsQuadratureCaptureEnabled == true);
#else
    return false;
#endif
}
/*
    Convert signed capture to user reference. Captured as ALeadB is positive by default
*/
static inline int32_t _Encoder_Quadrature_GetDirection(const Encoder_State_T * p_encoder) { return (p_encoder->Config.IsALeadBPositive == true) ? 1 : -1; }

/* On Init */
/* Select using IsQuadratureCaptureEnabled */
static inline int32_t _Encoder_GetDirectionComp(const Encoder_State_T * p_encoder)
{
    return (_Encoder_IsQuadratureCaptureEnabled(p_encoder)) ? _Encoder_Quadrature_GetDirection(p_encoder) : _Encoder_SinglePhase_GetDirection(p_encoder);
}

/* Query for Comp */
/* return value assigned at Init */
static inline int32_t Encoder_GetDirectionRef(const Encoder_State_T * p_encoder) { return p_encoder->DirectionComp; }



/******************************************************************************/
/*!
    @brief
*/
/******************************************************************************/
static inline void Encoder_ZeroInterpolateAngle(Encoder_State_T * p_encoder) { (void)p_encoder; /* Angle_ZeroAngle(&p_encoder->AngleCounter.Base); */ }

/******************************************************************************/
/*!
    todo align state
*/
/******************************************************************************/
static inline bool Encoder_IsAligned(const Encoder_State_T * p_encoder)
{
    return (p_encoder->Align == ENCODER_ALIGN_ABSOLUTE) || (p_encoder->Align == ENCODER_ALIGN_PHASE);
}

static inline bool Encoder_IsPositionRefSet(Encoder_State_T * p_encoder) { return (p_encoder->IsHomed || Encoder_IsAligned(p_encoder)); }


/******************************************************************************/
/*!
    @brief inline config
*/
/******************************************************************************/
static inline uint16_t Encoder_GetIndexZeroRef(const Encoder_State_T * p_encoder) { return p_encoder->Config.IndexAngleRef >> ENCODER_ANGLE_SHIFT; }
static inline void Encoder_SetIndexZeroRef(Encoder_State_T * p_encoder, uint16_t angle) { p_encoder->Config.IndexAngleRef = angle << ENCODER_ANGLE_SHIFT; }
static inline void Encoder_ClearIndexZeroRef(Encoder_State_T * p_encoder) { p_encoder->Config.IndexAngleRef = 0U; }


/******************************************************************************/
/*!
    @brief Extern Declarations
*/
/******************************************************************************/
extern void Encoder_InitInterrupts_Quadrature(const Encoder_T * p_encoder);
extern void Encoder_InitInterrupts_ABC(const Encoder_T * p_encoder);

void Encoder_StartHoming(Encoder_State_T * p_encoder);
uint16_t Encoder_GetHomingAngle(const Encoder_State_T * p_encoder);
bool Encoder_IsHomingIndexFound(const Encoder_State_T * p_encoder);
bool Encoder_IsHomingIndexError(const Encoder_State_T * p_encoder);
bool Encoder_PollHomingComplete(Encoder_State_T * p_encoder);
void Encoder_CalibrateIndexZeroRef(Encoder_State_T * p_encoder);
void Encoder_ClearIndexZeroRef(Encoder_State_T * p_encoder);

extern void Encoder_CheckAlignRef(Encoder_State_T * p_encoder);
extern void Encoder_CaptureAlignZero(Encoder_State_T * p_encoder);
extern uint16_t Encoder_GetAngleAligned(const Encoder_State_T * p_encoder);
extern bool Encoder_ProcAlignValidate(Encoder_State_T * p_encoder);
extern void Encoder_CompleteAlignValidate(Encoder_State_T * p_encoder);
extern void Encoder_ClearAlign(Encoder_State_T * p_encoder);

#if defined(ENCODER_QUADRATURE_MODE_ENABLE)
extern void Encoder_SetQuadratureMode(Encoder_State_T * p_encoder, bool isEnabled);
extern void Encoder_EnableQuadratureMode(Encoder_State_T * p_encoder);
extern void Encoder_SetQuadratureDirection(Encoder_State_T * p_encoder, bool isALeadBPositive);
extern void Encoder_CaptureQuadratureReference(Encoder_State_T * p_encoder);
extern void Encoder_CalibrateQuadraturePositive(Encoder_State_T * p_encoder);
#endif

extern void Encoder_SetCountsPerRevolution(Encoder_State_T * p_encoder, uint16_t countsPerRevolution);
extern void Encoder_SetScalarSpeedRef(Encoder_State_T * p_encoder, uint16_t speedRef);
// extern void Encoder_SetDistancePerRevolution(Encoder_State_T * p_encoder, uint16_t distance, uint16_t rotation);
// extern void Encoder_SetSurfaceRatio(Encoder_State_T * p_encoder, uint32_t surfaceDiameter, uint32_t gearRatioSurface, uint32_t gearRatioDrive);
// extern void Encoder_SetGroundRatio_US(Encoder_State_T * p_encoder, uint32_t wheelDiameter_Inch10, uint32_t wheelRatio, uint32_t motorRatio);
// extern void Encoder_SetGroundRatio_Metric(Encoder_State_T * p_encoder, uint32_t wheelDiameter_Mm, uint32_t wheelRatio, uint32_t motorRatio);

/******************************************************************************/
/*!
*/
/******************************************************************************/
typedef enum Encoder_VarId
{
    ENCODER_VAR_FREQ,
    ENCODER_VAR_RPM,
    ENCODER_VAR_DELTA_T_SPEED,
    ENCODER_VAR_DELTA_D_SPEED,
    ENCODER_VAR_COUNTER_D,
}
Encoder_VarId_T;

/*

*/
typedef enum Encoder_ConfigId
{
    ENCODER_CONFIG_COUNTS_PER_REVOLUTION,
    ENCODER_CONFIG_EXTENDED_TIMER_DELTA_T_STOP,
    ENCODER_CONFIG_INTERPOLATE_ANGLE_SCALAR,
    ENCODER_CONFIG_IS_QUADRATURE_CAPTURE_ENABLED,
    ENCODER_CONFIG_IS_A_LEAD_B_POSITIVE,

    ENCODER_CONFIG_INDEX_ZERO_REF,
    ENCODER_CONFIG_CALIBRATE_ZERO_REF,

    ENCODER_CONFIG_RUN_HOMING,
}
Encoder_ConfigId_T;

extern int32_t _Encoder_ConfigId_Get(const Encoder_State_T * p_encoder, Encoder_ConfigId_T varId);
extern void _Encoder_ConfigId_Set(Encoder_State_T * p_encoder, Encoder_ConfigId_T varId, int32_t value);

extern void Encoder_ConfigId_Set(const Encoder_T * p_encoder, Encoder_ConfigId_T varId, int32_t varValue);

