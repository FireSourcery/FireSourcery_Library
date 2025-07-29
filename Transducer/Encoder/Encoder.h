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
    @file    Encoder.h
    @author  FireSourcery
    @brief     Encoder Speed, Position
            Encoder common file contains unit conversions.

*/
/******************************************************************************/
#ifndef ENCODER_H
#define ENCODER_H

#include "HAL_Encoder.h"
#include "Config.h"
#include "Peripheral/Pin/Pin.h"

#include "Math/Fixed/fixed.h"
#include "Math/math_general.h"

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*!
*/
/******************************************************************************/
#ifndef ENCODER_TIMER_MAX
#define ENCODER_TIMER_MAX (0xFFFFU)
#endif

#ifndef ENCODER_ANGLE_BITS
#define ENCODER_ANGLE_BITS      (16U)
#define ENCODER_ANGLE_SHIFT     (32U - ENCODER_ANGLE_BITS)
#define ENCODER_ANGLE_DEGREES   ((uint32_t)1UL << ENCODER_ANGLE_BITS)
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

//todo
typedef enum Encoder_Align
{
    ENCODER_ALIGN_NONE,
    ENCODER_ALIGN_PHASE,    /* Calibrate Aligned */
    ENCODER_ALIGN_ABSOLUTE,
}
Encoder_Align_T;

// typedef enum Encoder_PositionRef
// {
//     ENCODER_REF_STATE_NONE,
//     ENCODER_REF_STATE_ALIGN,
//     ENCODER_REF_STATE_INDEX_HOME,
// };

typedef struct Encoder_Config
{
    uint16_t CountsPerRevolution;        /* Derive Angular Units. */
    uint32_t ScalarSpeedRef_Rpm;         /* Derive Fract16 Units. */
    uint16_t SurfaceDiameter;            /* Derive Linear Units. */
    uint16_t GearRatioInput;             /* Derivce Surface Units. DistancePerRevolution_Divider */
    uint16_t GearRatioOutput;            /* DistancePerRevolution_Factor */
    uint16_t ExtendedDeltaTStop;         /* ExtendedTimer time read as deltaT stopped, default as 1s */
    uint32_t PartitionsPerRevolution;    /* Sets Interpolate Angle - UnitPollingAngle and InterpolateAngleLimit. e.g electrical angle conversion */
    // uint32_t AngularSpeedScalar;

    uint32_t IndexAngleRef;             /* Virtual Index - Index, VirtualIndexOffset */
    uint32_t AlignOffsetRef;            /* Align - Index */

#if defined(CONFIG_ENCODER_QUADRATURE_MODE_ENABLE)
    bool IsQuadratureCaptureEnabled;    /* Quadrature Mode - enable hardware/emulated quadrature speed capture */
    bool IsALeadBPositive;              /* User runtime calibration for encoder install direction. Accounts for LUT calibration */
    /* Optionally combine with compile time defined QUADRATURE_A_LEAD_B_INCREMENT */
#endif
}
Encoder_Config_T;

//todo split math component as PulseEncoder
typedef struct Encoder_State
{
    Encoder_Config_T Config;
// #if defined(CONFIG_ENCODER_HW_EMULATED)
    Encoder_Phases_T Phases; /* Save Prev State */
    uint32_t CounterD;
    uint32_t Angle32;
    uint32_t ErrorCount;
// #endif

    int32_t DirectionComp;
    uint32_t CounterPrev; /* for DeltaD */
    uint32_t ExtendedTimerPrev; /* for DeltaT */
    uint32_t ExtendedTimerConversion; /* Extended Timer to Short Timer */

    int32_t DeltaD;         /*!< Counter counts (of distance) between 2 samples. Units in raw counter ticks */
    uint32_t DeltaT;        /*!< Timer counts between 2 pulse counts. Units in raw timer ticks */
    uint32_t DeltaTh;       /*!< ModeDT */
    int32_t FreqD;          /*!< ModeDT. EncoderPulse Freq. DeltaD 1 Second [Hz] */
    uint32_t PeriodT;       /*!< ModeDT */
    uint32_t PollingAngleDelta; /*!< UnitPollingAngle / DeltaT */

    /*  */
    // uint32_t InterpolateAngleIndex; /*!< ModeT/DT */
    uint32_t InterpolateAngleSum;
    uint32_t InterpolateAngleLimit; /* UnitPartitionAngle */

    /* Homing */
    uint32_t IndexCount;
    uint32_t IndexAngleRef; /* 32 */
    uint32_t IndexAngleError;
    bool IsHomed;
    Encoder_Align_T Align;
    uint32_t AlignOffsetRef;
    uint32_t AlignAngle; /* angle at last align */
    // int32_t AbsoluteOffset;

    /* Experimental */
    // int32_t TotalD;          /* Integral Capture */
    // uint32_t TotalT;         /*!< Timer ticks, persistent through DeltaT captures */
    // uint32_t SpeedPrev;      /*!< Most recently calculated speed, use for acceleration */
    // uint32_t DeltaSpeed;     /*!< Save for acceleration calc */

    /*
        Unit conversion. derived on init from NvMem Config
    */
    uint32_t UnitTime_Freq;                 /*!< Common Units propagating set depending on mode. T seconds conversion factor. */
    uint32_t PollingFreq;                   /* cache to simplify unit reset */
    /* with shift */
    uint32_t UnitAngleD;                    /*!< [(UINT32_MAX+1)/CountsPerRevolution] => Angle = PulseCounter * UnitAngleD >> DEGREES_SHIFT */
    uint32_t UnitPollingAngle;              /*!< [(1 << DEGREES_BITS) * UnitTime_Freq / POLLING_FREQ / CountsPerRevolution] */
    uint32_t UnitLinearD;                   /*!< Linear D unit conversion factor. Units per TimerCounter tick, using Capture DeltaD (DeltaT = 1). Units per DeltaT capture, using Capture DeltaT (DeltaD = 1).*/

    uint32_t UnitScalarSpeed;               /*!< Percentage Speed of ScalarSpeedRef_Rpm, given max speed */
    uint32_t UnitAngularSpeed;              /*!< [(1 << DEGREES_BITS) * UnitTime_Freq / CountsPerRevolution] => AngularSpeed = DeltaD * UnitAngularSpeed / DeltaT */
    uint32_t UnitSurfaceSpeed;              /*!< [UnitD * UnitTime_Freq] => Speed = DeltaD * UnitSpeed / DeltaT */
    uint8_t UnitScalarSpeedShift;
    uint8_t UnitSurfaceSpeedShift;          /* Shifts applicable to ModeD/DT */
    uint8_t UnitAngularSpeedShift;
}
Encoder_State_T;

#define ENCODER_STATE_ALLOC() (&(Encoder_State_T){0})

typedef const struct Encoder
{
#if     defined(CONFIG_ENCODER_HW_DECODER)
    HAL_Encoder_Counter_T * P_HAL_ENCODER_COUNTER; /*!< Pulse Counter */
#elif   defined(CONFIG_ENCODER_HW_EMULATED)
    /* HAL_Encoder_Pin_T configures settings not included in Pin_T, case of interrupt support */
    HAL_Encoder_Pin_T * P_HAL_PIN_A; uint32_t PIN_A_ID; /* Encoder_Pin_T */
    HAL_Encoder_Pin_T * P_HAL_PIN_B; uint32_t PIN_B_ID;
    HAL_Encoder_Pin_T * P_HAL_PIN_Z; uint32_t PIN_Z_ID;
    Pin_T PIN_A;
    Pin_T PIN_B;
#endif
    /*  Timers used to represent time. */
    HAL_Encoder_Timer_T * P_HAL_ENCODER_TIMER;    /*!< DeltaT Timer. */
    uint32_t TIMER_FREQ;                          /*!< DeltaT Timer Freq. Divisible by SAMPLE_FREQ */

    const volatile uint32_t * P_EXTENDED_TIMER;   /* 32-bit extension + WatchStop */
    uint32_t EXTENDED_TIMER_FREQ;

    /* Alternatively as macros for common across instances */
    uint32_t POLLING_FREQ;        /*!< Angle Sample Freq. DeltaT Interpolation Freq. */
    // POLLING_TIME
    uint32_t SAMPLE_FREQ;         /*!< Speed Sample Freq. DeltaD Sample Freq. */
    uint32_t SAMPLE_TIME;         /*!< Compile time TIMER_FREQ/SAMPLE_FREQ */

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
#if defined(CONFIG_ENCODER_HW_EMULATED)
    #define _ENCODER_INIT_HW_COUNTER(p_CounterHal, p_PhaseAHal, PhaseAId, p_PhaseBHal, PhaseBId, p_PhaseZHal, PhaseZId)  \
        .P_HAL_PIN_A = p_PhaseAHal, .PIN_A_ID = PhaseAId, .P_HAL_PIN_B = p_PhaseBHal, .PIN_B_ID = PhaseBId, .P_HAL_PIN_Z = p_PhaseZHal, .PIN_Z_ID = PhaseZId,
    #define _ENCODER_INIT_HW_PINS(p_PinAHal, PinAId, p_PinBHal, PinBId) \
        .PIN_A = PIN_INIT(p_PinAHal, PinAId), .PIN_B = PIN_INIT(p_PinBHal, PinBId),
#else
    #define _ENCODER_INIT_HW_COUNTER(p_CounterHal, p_PhaseAHal, PhaseAId, p_PhaseBHal, PhaseBId, p_PhaseZHal, PhaseZId) .P_HAL_ENCODER_COUNTER  = p_CounterHal,
    #define _ENCODER_INIT_HW_PINS(p_PinAHal, PinAId, p_PinBHal, PinBId)
#endif

#define _ENCODER_INIT_TIMER(p_TimerHal, TimerFreq, p_ExtendedTimer, ExtendedTimerFreq) \
    .P_HAL_ENCODER_TIMER = p_TimerHal, .TIMER_FREQ = TimerFreq, .P_EXTENDED_TIMER = p_ExtendedTimer, .EXTENDED_TIMER_FREQ = ExtendedTimerFreq,

#define ENCODER_INIT(p_CounterHal, p_PhaseAHal, PhaseAId, p_PinAHal, PinAId, p_PhaseBHal, PhaseBId, p_PinBHal, PinBId, p_PhaseZHal, PhaseZId, p_TimerHal, TimerFreq, PollingFreq, SpeedSampleFreq, p_ExtendedTimer, ExtendedTimerFreq, p_State, p_Config)    \
{                                                                                                               \
    _ENCODER_INIT_HW_COUNTER(p_CounterHal, p_PhaseAHal, PhaseAId, p_PhaseBHal, PhaseBId, p_PhaseZHal, PhaseZId) \
    _ENCODER_INIT_HW_PINS(p_PinAHal, PinAId, p_PinBHal, PinBId)                                                 \
    _ENCODER_INIT_TIMER(p_TimerHal, TimerFreq, p_ExtendedTimer, ExtendedTimerFreq)                              \
    .POLLING_FREQ           = PollingFreq,                                  \
    .SAMPLE_FREQ            = SpeedSampleFreq,                              \
    .SAMPLE_TIME            = TimerFreq/SpeedSampleFreq,                    \
    .P_STATE                = p_State,                                      \
    .P_NVM_CONFIG           = p_Config,                                     \
}

// #define ENCODER_ALLOC(p_CounterHal, p_PhaseAHal, PhaseAId, p_PinAHal, PinAId, p_PhaseBHal, PhaseBId, p_PinBHal, PinBId, p_PhaseZHal, PhaseZId, p_TimerHal, TimerFreq, PollingFreq, SpeedSampleFreq, p_ExtendedTimer, ExtendedTimerFreq, p_Config)    \
//     ENCODER_INIT(p_CounterHal, p_PhaseAHal, PhaseAId, p_PinAHal, PinAId, p_PhaseBHal, PhaseBId, p_PinBHal, PinBId, p_PhaseZHal, PhaseZId, p_TimerHal, TimerFreq, PollingFreq, SpeedSampleFreq, p_ExtendedTimer, ExtendedTimerFreq, &(Encoder_State_T){0}, p_Config)

/******************************************************************************/
/*!
*/
/******************************************************************************/
// typedef enum Encoder_StateField
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

/******************************************************************************/
/*!

*/
/******************************************************************************/

static inline uint32_t _Encoder_CaptureDeltaWrap(uint32_t tcMax, uint32_t tcPrev, uint32_t tcValue)
{
    return (tcValue < tcPrev) ? (tcMax + 1U + tcValue - tcPrev) : (tcValue - tcPrev);
}

/******************************************************************************/
/*!
    @brief  Function
*/
/******************************************************************************/
static inline bool _Encoder_IsQuadratureCaptureEnabled(const Encoder_State_T * p_encoder)
{
#if     defined(CONFIG_ENCODER_QUADRATURE_MODE_ENABLE)
    return (p_encoder->Config.IsQuadratureCaptureEnabled == true);
#else
    return false;
#endif
}

typedef void (*Encoder_Proc_T)(Encoder_State_T * p_encoder);

static inline void Encoder_CaptureMode_Proc(Encoder_State_T * p_encoder, Encoder_Proc_T quadratureFunction, Encoder_Proc_T singlePhaseFunction)
{
#if     defined(CONFIG_ENCODER_QUADRATURE_MODE_ENABLE)
    if (p_encoder->Config.IsQuadratureCaptureEnabled == true) { quadratureFunction(p_encoder); } else { singlePhaseFunction(p_encoder); }
#else
    (void)quadratureFunction; singlePhaseFunction(p_encoder);
#endif
}

typedef int32_t(*Encoder_Get_T)(const Encoder_State_T * p_encoder);

static inline int32_t Encoder_CaptureMode_Get(const Encoder_State_T * p_encoder, Encoder_Get_T quadratureFunction, Encoder_Get_T singlePhaseFunction)
{
#if defined(CONFIG_ENCODER_QUADRATURE_MODE_ENABLE)
    return (p_encoder->Config.IsQuadratureCaptureEnabled == true) ? quadratureFunction(p_encoder) : singlePhaseFunction(p_encoder);
#else
    (void)quadratureFunction; return singlePhaseFunction(p_encoder);
#endif
}

/******************************************************************************/
/*!
    @brief Hal Abstraction
*/
/******************************************************************************/
/*
    Does not wrap in HW_EMULATED case
*/
static inline int32_t Encoder_GetCounterD(const Encoder_State_T * p_encoder)
{
#if     defined(CONFIG_ENCODER_HW_DECODER)
    return HAL_Encoder_ReadCounter(p_encoder->P_HAL_ENCODER_COUNTER);
#elif   defined(CONFIG_ENCODER_HW_EMULATED)
    return p_encoder->CounterD;
    // return p_encoder->Angle32 / p_encoder->UnitAngleD;
#endif
}

static inline void _Encoder_SetCounterD(Encoder_State_T * p_encoder, int32_t counterD)
{
#if     defined(CONFIG_ENCODER_HW_DECODER)
    HAL_Encoder_WriteCounter(p_encoder->P_HAL_ENCODER_COUNTER, counterD);
#elif   defined(CONFIG_ENCODER_HW_EMULATED)
    p_encoder->CounterD = counterD;
#endif
}

static inline uint32_t _Encoder_GetAngle32(const Encoder_State_T * p_encoder)
{
#if     defined(CONFIG_ENCODER_HW_DECODER)
    return HAL_Encoder_ReadCounter(p_encoder->P_HAL_ENCODER_COUNTER) * (uint32_t)p_encoder->UnitAngleD;
#elif   defined(CONFIG_ENCODER_HW_EMULATED)
    return p_encoder->Angle32;
#endif
}

// static inline uint16_t _Encoder_GetAngle(const Encoder_State_T * p_encoder)
// {
// #if     defined(CONFIG_ENCODER_HW_DECODER)
//     return HAL_Encoder_ReadCounter(p_encoder->P_HAL_ENCODER_COUNTER) * (uint32_t)p_encoder->UnitAnglePerCount;
// #elif   defined(CONFIG_ENCODER_HW_EMULATED)
//     return p_encoder->Angle32 >> ENCODER_ANGLE_SHIFT;
// #endif
// }

static inline void _Encoder_ZeroPulseCount(Encoder_State_T * p_encoder)
{
    p_encoder->CounterD = 0U;
    p_encoder->CounterPrev = 0U;
    p_encoder->IndexCount = 0U;
#if     defined(CONFIG_ENCODER_HW_DECODER)
    p_encoder->CounterPrev = HAL_Encoder_ReadCounter(p_encoder->P_HAL_ENCODER_COUNTER);
    HAL_Encoder_WriteCounter(p_encoder->P_HAL_ENCODER_COUNTER, 0);
    HAL_Encoder_ClearCounterOverflow(p_encoder->P_HAL_ENCODER_COUNTER);
#elif   defined(CONFIG_ENCODER_HW_EMULATED)
#endif
}

/******************************************************************************/
/*!
    @brief Direction
*/
/******************************************************************************/
/* Direction set by outer module */
/* 0 until set */
static inline int32_t _Encoder_SinglePhase_GetDirection(const Encoder_State_T * p_encoder) { return p_encoder->DirectionComp; }
// static inline int32_t _Encoder_SinglePhase_GetDirection(const Encoder_State_T * p_encoder) { return (p_encoder->IsSinglePhasePositive == true) ? 1 : -1; }

/*
    Convert signed capture to user reference. Captured as ALeadB is positive by default
*/
static inline int32_t _Encoder_Quadrature_GetDirection(const Encoder_State_T * p_encoder) { return (p_encoder->Config.IsALeadBPositive == true) ? 1 : -1; }

/* Select using IsQuadratureCaptureEnabled */
static inline int32_t _Encoder_GetDirectionComp(const Encoder_State_T * p_encoder)
{
    return Encoder_CaptureMode_Get(p_encoder, _Encoder_Quadrature_GetDirection, _Encoder_SinglePhase_GetDirection);
}
// static inline int32_t Encoder_GetUserDirection(const Encoder_State_T * p_encoder) { return  p_encoder->DirectionD * _Encoder_GetDirectionComp(p_encoder->P_STATE); }

/* return value assigned at Init */
static inline int32_t Encoder_GetDirectionRef(const Encoder_State_T * p_encoder) { return p_encoder->DirectionComp; }

/*
    Capture Direction on each Count or accumulated during Speed
*/
// static inline void Encoder_CaptureDirection(Encoder_State_T * p_encoder, int8_t sign){}

/* SinglePhase assign direction */
static inline void Encoder_SinglePhase_SetDirection(Encoder_State_T * p_encoder, int8_t direction) { p_encoder->DirectionComp = direction; }


/******************************************************************************/
/*!
    @brief
*/
/******************************************************************************/
static inline uint16_t _Encoder_GetAngle(const Encoder_State_T * p_encoder) { return _Encoder_GetAngle32(p_encoder) >> ENCODER_ANGLE_SHIFT; }
// static inline uint16_t _Encoder_GetPartitionAngle(const Encoder_State_T * p_encoder) { return  *(uint32_t)p_encoder->UnitPartitionAnglePerCount; }

static inline uint16_t Encoder_GetAngle(const Encoder_State_T * p_encoder) { return Encoder_GetDirectionRef(p_encoder) * _Encoder_GetAngle(p_encoder); }

static inline uint16_t Encoder_GetPartitionAngle(const Encoder_State_T * p_encoder)
{
    return Encoder_GetDirectionRef(p_encoder) * (((_Encoder_GetAngle32(p_encoder) >> 8U) * p_encoder->Config.PartitionsPerRevolution) >> 8U);
}

static inline void Encoder_ZeroInterpolateAngle(Encoder_State_T * p_encoder)
{
    // p_encoder->InterpolateAngleIndex = 0U;
    p_encoder->InterpolateAngleSum = 0U;
}


/******************************************************************************/
/*!
    @brief
*/
/******************************************************************************/

static inline bool Encoder_IsAligned(const Encoder_State_T * p_encoder)
{
    return (p_encoder->Align == ENCODER_ALIGN_ABSOLUTE) || (p_encoder->Align == ENCODER_ALIGN_PHASE);
}

static inline bool _Encoder_IsAligned(const Encoder_State_T * p_encoder)
{
    // return (p_encoder->Align == ENCODER_ALIGN_PHASE) || (p_encoder->IsHomed && Encoder_IsAlignOffsetOn(p_encoder));
}

static inline bool Encoder_IsPositionRefSet(Encoder_State_T * p_encoder) { return (p_encoder->IsHomed || Encoder_IsAligned(p_encoder)); }


/******************************************************************************/
/*!
    @brief inline config
*/
/******************************************************************************/
static inline uint16_t Encoder_GetIndexZeroRef(const Encoder_State_T * p_encoder) { return p_encoder->Config.IndexAngleRef >> ENCODER_ANGLE_SHIFT; }
static inline void Encoder_SetIndexZeroRef(Encoder_State_T * p_encoder, uint16_t angle) { p_encoder->Config.IndexAngleRef = angle << ENCODER_ANGLE_SHIFT; }

/* Clears the value to config index to set as 0 */
static inline void Encoder_ClearIndexZeroRef(Encoder_State_T * p_encoder) { p_encoder->Config.IndexAngleRef = 0U; }


/******************************************************************************/
/*!
    @brief Pulse CounterD conversions.
*/
/******************************************************************************/
/*!
    Angle
    input [0:CountsPerRevolution], wrapped counter value
*/
/* counter */
static inline uint32_t _Encoder_AngleOfCount(const Encoder_State_T * p_encoder, uint32_t counterD_Ticks)
{
    return ((counterD_Ticks * p_encoder->UnitAngleD) >> ENCODER_ANGLE_SHIFT);
}

/* delta */
// static inline uint32_t Encoder_AngleOfCount(const Encoder_State_T * p_encoder, uint32_t counterD_Ticks)
// {
//     return (counterD_Ticks < p_encoder->Config.CountsPerRevolution) ?
//         ((counterD_Ticks * p_encoder->UnitAngleD) >> ENCODER_ANGLE_SHIFT) :
//         ((counterD_Ticks << ENCODER_ANGLE_BITS) / p_encoder->Config.CountsPerRevolution);

//     return ((counterD_Ticks << ENCODER_ANGLE_BITS) / p_encoder->Config.CountsPerRevolution);
// }

static inline uint32_t _Encoder_CountOfAngle(const Encoder_State_T * p_encoder, uint16_t angle_UserDegrees)
{
    return (angle_UserDegrees << ENCODER_ANGLE_SHIFT) / p_encoder->UnitAngleD;
}

/******************************************************************************/
/*!

*/
/******************************************************************************/
/*!
    Linear Distance
*/
static inline uint32_t Encoder_DistanceOfCount(const Encoder_State_T * p_encoder, uint32_t counterD_Ticks) { return counterD_Ticks * p_encoder->UnitLinearD; }
static inline uint32_t Encoder_CountOfDistance(const Encoder_State_T * p_encoder, uint32_t distance_Units) { return distance_Units / p_encoder->UnitLinearD; }


// static inline uint32_t Encoder_GetAngularSpeed_PerPoll(const Encoder_State_T * p_encoder, uint32_t deltaD_Ticks, uint32_t deltaT_Ticks)
// {
//     return (deltaD_Ticks * p_encoder->UnitTime_Freq / p_encoder->POLLING_FREQ << ENCODER_ANGLE_BITS) / (p_encoder->Config.CountsPerRevolution * deltaT_Ticks);
// }

/*
    Only When base units in mm, as set via SetGroundRatio function.
*/
static inline uint32_t Encoder_GroundSpeedOf_Mph(const Encoder_State_T * p_encoder, uint32_t deltaD_Ticks, uint32_t deltaT_Ticks)
{
    return deltaD_Ticks * p_encoder->UnitSurfaceSpeed * 60U * 60U / (deltaT_Ticks * 1609344U);
}

static inline uint32_t Encoder_GroundSpeedOf_Kmh(const Encoder_State_T * p_encoder, uint32_t deltaD_Ticks, uint32_t deltaT_Ticks)
{
    return deltaD_Ticks * p_encoder->UnitSurfaceSpeed * 60U * 60U / (deltaT_Ticks * 1000000U);
}




/******************************************************************************/
/*!
    @brief Extern Declarations
*/
/*! @{ */
/******************************************************************************/
extern void Encoder_InitInterrupts_Quadrature(const Encoder_T * p_encoder);
extern void Encoder_InitInterrupts_ABC(const Encoder_T * p_encoder);



void Encoder_StartHoming(Encoder_State_T * p_encoder);
uint16_t Encoder_GetHomingAngle(const Encoder_State_T * p_encoder);
bool Encoder_PollHomingComplete(Encoder_State_T * p_encoder);
void Encoder_CalibrateIndexZeroRef(Encoder_State_T * p_encoder);
void Encoder_ClearIndexZeroRef(Encoder_State_T * p_encoder);

extern void Encoder_CaptureAlignZero(Encoder_State_T * p_encoder);
extern void Encoder_CompleteAlignValidate(Encoder_State_T * p_encoder);

#if defined(CONFIG_ENCODER_QUADRATURE_MODE_ENABLE)
extern void Encoder_SetQuadratureMode(Encoder_State_T * p_encoder, bool isEnabled);
extern void Encoder_EnableQuadratureMode(Encoder_State_T * p_encoder);
extern void Encoder_SetQuadratureDirection(Encoder_State_T * p_encoder, bool isALeadBPositive);
extern void Encoder_CaptureQuadratureReference(Encoder_State_T * p_encoder);
extern void Encoder_CalibrateQuadraturePositive(Encoder_State_T * p_encoder);
#endif

extern int32_t _Encoder_ConfigId_Get(const Encoder_State_T * p_encoder, Encoder_ConfigId_T varId);
extern void _Encoder_ConfigId_Set(Encoder_State_T * p_encoder, Encoder_ConfigId_T varId, int32_t value);

extern void Encoder_ConfigId_Set(const Encoder_T * p_encoder, Encoder_ConfigId_T varId, int32_t varValue);


extern void _Encoder_ResetUnits(Encoder_State_T * p_encoder);
extern void _Encoder_ResetUnitsAngle(Encoder_State_T * p_encoder);
extern void _Encoder_ResetUnitsPollingAngle(Encoder_State_T * p_encoder);
extern void _Encoder_ResetUnitsAngularSpeed(Encoder_State_T * p_encoder);
extern void _Encoder_ResetUnitsLinearSpeed(Encoder_State_T * p_encoder);
extern void _Encoder_ResetUnitsScalarSpeed(Encoder_State_T * p_encoder);
extern void Encoder_SetCountsPerRevolution(Encoder_State_T * p_encoder, uint16_t countsPerRevolution);
extern void Encoder_SetPartitionsPerRevolution(Encoder_State_T * p_encoder, uint16_t count);
extern void Encoder_SetScalarSpeedRef(Encoder_State_T * p_encoder, uint16_t speedRef);


// extern void Encoder_SetDistancePerRevolution(Encoder_State_T * p_encoder, uint16_t distance, uint16_t rotation);
extern void Encoder_SetSurfaceRatio(Encoder_State_T * p_encoder, uint32_t surfaceDiameter, uint32_t gearRatioSurface, uint32_t gearRatioDrive);
extern void Encoder_SetGroundRatio_US(Encoder_State_T * p_encoder, uint32_t wheelDiameter_Inch10, uint32_t wheelRatio, uint32_t motorRatio);
extern void Encoder_SetGroundRatio_Metric(Encoder_State_T * p_encoder, uint32_t wheelDiameter_Mm, uint32_t wheelRatio, uint32_t motorRatio);
/******************************************************************************/
/*! @} */
/******************************************************************************/


#endif
