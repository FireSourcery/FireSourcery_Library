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
    @author FireSourcery
    @brief     Encoder Speed, Position
            Encoder common file contains unit conversions.
    @version V0
*/
/******************************************************************************/
#ifndef ENCODER_H
#define ENCODER_H

#include "HAL_Encoder.h"
#include "Config.h"
#include "Peripheral/Pin/Pin.h"

#include "Math/Q/Q.h"
#include "Math/math_general.h"

#include <stdint.h>
#include <stdbool.h>

#ifndef ENCODER_TIMER_MAX
#define ENCODER_TIMER_MAX (0xFFFFU)
#endif

#ifndef ENCODER_ANGLE_DEGREES
#define ENCODER_ANGLE_BITS      (16U)
#define ENCODER_ANGLE_SHIFT     (32U - ENCODER_ANGLE_BITS)
#define ENCODER_ANGLE_DEGREES   ((uint32_t)1UL << ENCODER_ANGLE_BITS)
#endif

#define _ENCODER_TABLE_ERROR (2U)
#define _ENCODER_TABLE_LENGTH (16U)
extern const int8_t _ENCODER_TABLE[_ENCODER_TABLE_LENGTH];
extern const int8_t _ENCODER_TABLE_PHASE_A[_ENCODER_TABLE_LENGTH];

typedef union Encoder_Phases
{
    struct
    {
        uint8_t B : 1U;
        uint8_t A : 1U;
        uint8_t PrevB : 1U;
        uint8_t PrevA : 1U;
        uint8_t Resv : 4U;
    };
    uint8_t State;
}
Encoder_Phases_T;

typedef enum Encoder_Align
{
    ENCODER_ALIGN_NO,
    ENCODER_ALIGN_PHASE,
    ENCODER_ALIGN_ABSOLUTE,
}
Encoder_Align_T;

typedef struct Encoder_Config
{
    uint16_t CountsPerRevolution;         /* Derive Angular Units. Max for counting AngularD, CaptureDeltaT mode need 2nd TimerCounterMax */
    uint16_t ScalarSpeedRef_Rpm;         /* Derive Scalar16 Units. */
    uint16_t SurfaceDiameter;            /* Derive Linear Units. */
    uint16_t GearRatio_Factor;            /* Derive Linear Units. Surface:Encoder Ratio */
    uint16_t GearRatio_Divisor;            /* Derive Linear Units. */
    /* DistancePerRevolution_Factor, DistancePerRevolution_Factor_Divider */
    uint16_t ExtendedDeltaTStop;        /* ExtendedTimer time read as deltaT stopped, default as 1s or .5rpm */
    uint32_t InterpolateAngleScalar;    /* Sets UnitInterpolateAngle Scalar and InterpolateAngleLimit. e.g electrical angle conversion */
    uint32_t ZeroIndexOffset;
#if defined(CONFIG_ENCODER_QUADRATURE_MODE_ENABLE)
    bool IsQuadratureCaptureEnabled;     /* Quadrature Mode - enable hardware/emulated quadrature speed capture */
    bool IsALeadBPositive;                 /* User runtime calibration for encoder install direction, combine with compile time defined QUADRATURE_A_LEAD_B_INCREMENT */
#endif
}
Encoder_Config_T;

typedef const struct Encoder_Const
{
    const Encoder_Config_T * const P_CONFIG;
#if     defined(CONFIG_ENCODER_HW_DECODER)
    HAL_Encoder_Counter_T * const P_HAL_ENCODER_COUNTER; /*!< Pulse Counter */
#elif   defined(CONFIG_ENCODER_HW_EMULATED)
    /* HAL_Encoder_Pin_T configures settings not included in Pin_T, case of interrupt support */
    HAL_Encoder_Pin_T * const P_HAL_PIN_A; const uint32_t PIN_A_ID;
    HAL_Encoder_Pin_T * const P_HAL_PIN_B; const uint32_t PIN_B_ID;
    HAL_Encoder_Pin_T * const P_HAL_PIN_Z; const uint32_t PIN_Z_ID;
#endif
    HAL_Encoder_Timer_T * const P_HAL_ENCODER_TIMER;    /*!< DeltaT Timer. */
    const uint32_t TIMER_FREQ;                          /*!< DeltaT Timer Freq */
    const volatile uint32_t * const P_EXTENDED_TIMER;   /* Polling DeltaT stop */
    const uint32_t EXTENDED_TIMER_FREQ;
    const uint32_t POLLING_FREQ;        /*!< DeltaT Interpolation Freq. */
    const uint32_t SAMPLE_FREQ;         /*!< DeltaD Speed Sample Freq. */
}
Encoder_Const_T;

typedef struct Encoder
{
    const Encoder_Const_T CONST;
    Encoder_Config_T Config;
#if defined(CONFIG_ENCODER_HW_EMULATED)
    Pin_T PinA;
    Pin_T PinB;
#endif

#if defined(CONFIG_ENCODER_HW_EMULATED)
    Encoder_Phases_T Phases; /* Save Prev State */
#endif
    int32_t CounterD;
    uint32_t Angle32;
    int32_t DeltaD;         /*!< Counter counts (of distance) between 2 samples. Units in raw counter ticks */
    uint32_t DeltaT;        /*!< Timer counts between 2 pulse counts. Units in raw timer ticks */
    uint32_t DeltaTh;       /*!< ModeDT */
    int32_t FreqD;          /*!< EncoderPulseFreq ModeDT. DeltaD 1 Second */
    uint32_t InterpolateAngleIndex; /*!< ModeDT */
    int32_t DirectionD;     /*!< previous DeltaD sign, when DeltaD == 0 */
    uint32_t ErrorCount;
    uint32_t IndexCount;
    // int32_t IndexCounterDOffset;
    uint32_t ExtendedTimerPrev;
    uint32_t ExtendedTimerConversion;    /* Extended Timer to Short Timer */

    bool IsSinglePhasePositive;
    Encoder_Align_T Align;
    int32_t AbsoluteOffset;

    int32_t _FreqDMax;

    /* Experimental */
    int32_t TotalD;             /* Integral Capture */
    // uint32_t TotalT;         /*!< Extended? Timer ticks, persistent through DeltaT captures */
    // uint32_t SpeedPrev;      /*!< Most recently calculated speed, use for acceleration */
    // uint32_t DeltaSpeed;     /*!< Save for acceleration calc */

    /*
        Unit conversion. derived on init from Nv Config
    */
    uint32_t UnitT_Freq;                    /*!< Common Units propagating set depending on mode. T unit(seconds) conversion factor. */
    uint32_t UnitAngularD;                     /*!< [0xFFFFFFFFU/CountsPerRevolution + 1]         => Angle = PulseCounter * UnitAngle_Factor >> (32 - DEGREES_BITS) */
    uint32_t UnitLinearD;                    /*!< Linear D unit conversion factor. Units per TimerCounter tick, using Capture DeltaD (DeltaT is 1). Units per DeltaT capture, using Capture DeltaT (DeltaD is 1).*/
    uint32_t UnitInterpolateAngle;             /*!< [UnitT_Freq << DEGREES_BITS / POLLING_FREQ / CountsPerRevolution] */
    uint32_t InterpolateAngleLimit;
    uint32_t UnitAngularSpeed;                 /*!< [(1 << DEGREES_BITS) * UnitT_Freq / CountsPerRevolution]     => AngularSpeed = DeltaD * UnitAngularSpeed / DeltaT */
    uint32_t UnitSurfaceSpeed;                /*!< [UnitD * UnitT_Freq]                         => Speed = DeltaD * UnitSpeed / DeltaT */
    uint32_t UnitScalarSpeed;                /*!< Percentage Speed of ScalarSpeedRef_Rpm, given max speed, as Fraction16 */
    uint8_t UnitSurfaceSpeedShift;            /* Shifts applicable to ModeD/DT */
    uint8_t UnitAngularSpeedShift;
    uint8_t UnitScalarSpeedShift;
}
Encoder_T;

/*
    TIMER_FREQ time, 16-bit Timer, SAMPLE_FREQ 1000Hz (1ms)
    0xFFFF/50[Mhz]*1000 = 1.31[ms]
    0xFFFF/65.536[Mhz]*1000 = 1[ms]

    Mode DeltaT,
    Min Speed for Interpolation,

    UnitAngularSpeed Shift Divide
    TIMER_FREQ < UINT32_MAX * CountsPerRevolution / (1 << ENCODER_ANGLE_BITS) ~= 1Mhz-4Mhz for Unsigned Speed
    TIMER_FREQ * 60 < UINT32_MAX for RPM calc
*/
#if defined(CONFIG_ENCODER_HW_EMULATED)
    #define _ENCODER_INIT_HW_COUNTER(p_CounterHal, p_PhaseAHal, PhaseAId, p_PhaseBHal, PhaseBId, p_PhaseZHal, PhaseZId)  \
        .P_HAL_PIN_A = p_PhaseAHal, .PIN_A_ID = PhaseAId,    \
        .P_HAL_PIN_B = p_PhaseBHal, .PIN_B_ID = PhaseBId,    \
        .P_HAL_PIN_Z = p_PhaseZHal, .PIN_Z_ID = PhaseZId,
    #define _ENCODER_INIT_HW_PINS(p_PinAHal, PinAId, p_PinBHal, PinBId)   \
        .PinA = PIN_INIT(p_PinAHal, PinAId),                               \
        .PinB = PIN_INIT(p_PinBHal, PinBId),
#else
    #define _ENCODER_INIT_HW_COUNTER(p_CounterHal, p_PhaseAHal, PhaseAId, p_PhaseBHal, PhaseBId, p_PhaseZHal, PhaseZId)  \
        .P_HAL_ENCODER_COUNTER  = p_CounterHal,
    #define _ENCODER_INIT_HW_PINS(p_PinAHal, PinAId, p_PinBHal, PinBId)
#endif

#define ENCODER_INIT(p_CounterHal, p_PhaseAHal, PhaseAId, p_PinAHal, PinAId, p_PhaseBHal, PhaseBId, p_PinBHal, PinBId, p_PhaseZHal, PhaseZId, p_TimerHal, TimerFreq, PollingFreq, SpeedSampleFreq, p_ExtendedTimer, ExtendedTimerFreq, p_Config)    \
{                                                                                                                           \
    .CONST =                                                                                                               \
    {                                                                                                                       \
        _ENCODER_INIT_HW_COUNTER(p_CounterHal, p_PhaseAHal, PhaseAId, p_PhaseBHal, PhaseBId, p_PhaseZHal, PhaseZId)         \
        .P_HAL_ENCODER_TIMER    = p_TimerHal,            .TIMER_FREQ           = TimerFreq,                                 \
        .P_EXTENDED_TIMER       = p_ExtendedTimer,       .EXTENDED_TIMER_FREQ  = ExtendedTimerFreq,                         \
        .POLLING_FREQ           = PollingFreq,           .SAMPLE_FREQ          = SpeedSampleFreq,                           \
        .P_CONFIG               = p_Config,                                                                                 \
    },                                                                                                                      \
    _ENCODER_INIT_HW_PINS(p_PinAHal, PinAId, p_PinBHal, PinBId)                                                             \
}

/******************************************************************************/
/*!
    @brief  Function Templates
*/
/******************************************************************************/
typedef void(*Encoder_Proc_T)(Encoder_T * p_encoder);
typedef int32_t(*Encoder_Get_T)(const Encoder_T * p_encoder);

static inline void Encoder_CaptureMode_Proc(Encoder_T * p_encoder, Encoder_Proc_T quadratureFunction, Encoder_Proc_T singlePhaseFunction)
{
#if     defined(CONFIG_ENCODER_QUADRATURE_MODE_ENABLE)
    if(p_encoder->Config.IsQuadratureCaptureEnabled == true)     { quadratureFunction(p_encoder); }
    else                                                         { singlePhaseFunction(p_encoder); }
#else
    (void)quadratureFunction; singlePhaseFunction(p_encoder);
#endif
}

static inline int32_t Encoder_CaptureMode_Get(const Encoder_T * p_encoder, Encoder_Get_T quadratureFunction, Encoder_Get_T singlePhaseFunction)
{
#if defined(CONFIG_ENCODER_QUADRATURE_MODE_ENABLE)
    return (p_encoder->Config.IsQuadratureCaptureEnabled == true) ? quadratureFunction(p_encoder) : singlePhaseFunction(p_encoder);
#else
    (void)quadratureFunction; return singlePhaseFunction(p_encoder);
#endif
}

/******************************************************************************/
/*!
    @brief
*/
/******************************************************************************/
static inline void _Encoder_ZeroPulseCount(Encoder_T * p_encoder)
{
    p_encoder->CounterD = 0U;
    p_encoder->IndexCount = 0U;
}

static inline int32_t Encoder_GetCounterD(const Encoder_T * p_encoder)
{
#if     defined(CONFIG_ENCODER_HW_DECODER)
    return HAL_Encoder_ReadCounter(p_encoder->CONST.P_HAL_ENCODER_COUNTER);
#elif   defined(CONFIG_ENCODER_HW_EMULATED)
    return p_encoder->CounterD;
#endif
}

static inline uint32_t _Encoder_GetAngle32(const Encoder_T * p_encoder)
{
#if     defined(CONFIG_ENCODER_HW_DECODER)
    return HAL_Encoder_ReadCounter(p_encoder->CONST.P_HAL_ENCODER_COUNTER) * (int32_t)p_encoder->UnitAngularD;
#elif   defined(CONFIG_ENCODER_HW_EMULATED)
    return p_encoder->Angle32;
#endif
}

static inline uint16_t _Encoder_GetAngle(const Encoder_T * p_encoder) { return _Encoder_GetAngle32(p_encoder) >> ENCODER_ANGLE_SHIFT; }

/* Direction set by outer module */
static inline int32_t Encoder_GetDirection_SinglePhase(const Encoder_T * p_encoder) { return (p_encoder->IsSinglePhasePositive == true) ? 1 : -1; }
static inline void Encoder_SetSinglePhaseDirection(Encoder_T * p_encoder, bool isPositive) { p_encoder->IsSinglePhasePositive = isPositive; }
static inline void Encoder_SetSinglePhaseDirectionPositive(Encoder_T * p_encoder) { p_encoder->IsSinglePhasePositive = true; }
static inline void Encoder_SetSinglePhaseDirectionNegative(Encoder_T * p_encoder) { p_encoder->IsSinglePhasePositive = false; }
static inline uint16_t Encoder_GetAngle_SinglePhase(Encoder_T * p_encoder) { return Encoder_GetDirection_SinglePhase(p_encoder) * _Encoder_GetAngle(p_encoder); }

/*
    Convert signed capture to user reference
    Captured as ALeadB is positive by default
*/
static inline int32_t Encoder_GetDirection_Quadrature(const Encoder_T * p_encoder) { return (p_encoder->Config.IsALeadBPositive == true) ? 1 : -1; }
static inline uint16_t Encoder_GetAngle_Quadrature(Encoder_T * p_encoder) { return Encoder_GetDirection_Quadrature(p_encoder) * _Encoder_GetAngle(p_encoder); }


/* Select using IsQuadratureCaptureEnabled */
static inline int32_t Encoder_GetDirection(const Encoder_T * p_encoder) { return Encoder_CaptureMode_Get(p_encoder, Encoder_GetDirection_Quadrature, Encoder_GetDirection_SinglePhase); }
static inline uint16_t Encoder_GetAngle(const Encoder_T * p_encoder) { return Encoder_GetDirection(p_encoder) * _Encoder_GetAngle(p_encoder); }
/* scalar < 256 */ //ENCODER_ANGLE_SHIFT/2
static inline uint16_t Encoder_GetAngle_Scalar(const Encoder_T * p_encoder, uint8_t scalar) { return Encoder_GetDirection(p_encoder) * (((_Encoder_GetAngle32(p_encoder) >> 8U) * scalar) >> 8U); }


/******************************************************************************/
/*!
    @brief      PulseCounter conversions.
*/
/******************************************************************************/
/*!
    Angle - Base unit in ENCODER_ANGLE_BITS
*/
    /* Overflow: counterD > CountsPerRevolution, UnitAngularD == UINT32_MAX / CountsPerRevolution */
static inline uint32_t Encoder_AngleFromCounterD(Encoder_T * p_encoder, uint32_t counterD_Ticks)
{
    return (counterD_Ticks < p_encoder->Config.CountsPerRevolution) ?
        ((counterD_Ticks * p_encoder->UnitAngularD) >> ENCODER_ANGLE_SHIFT) :
        ((counterD_Ticks << ENCODER_ANGLE_BITS) / p_encoder->Config.CountsPerRevolution);
}

static inline uint32_t Encoder_CounterDFromAngle(Encoder_T * p_encoder, uint16_t angle_UserDegrees)
{
    return (angle_UserDegrees << ENCODER_ANGLE_SHIFT) / p_encoder->UnitAngularD;
}

/*!
    Linear Distance
*/
static inline uint32_t Encoder_DistanceFromCounterD(Encoder_T * p_encoder, uint32_t counterD_Ticks) { return counterD_Ticks * p_encoder->UnitLinearD; }
static inline uint32_t Encoder_CounterDFromDistance(Encoder_T * p_encoder, uint32_t distance_Units) { return distance_Units / p_encoder->UnitLinearD; }

/******************************************************************************/
/*!

*/
/******************************************************************************/
/*
    Only When base units in mm, as set via SetGroundRatio function.
*/
static inline uint32_t Encoder_CalcGroundSpeed_Mph(Encoder_T * p_encoder, uint32_t deltaD_Ticks, uint32_t deltaT_Ticks)
{
    return deltaD_Ticks * p_encoder->UnitSurfaceSpeed * 60U * 60U / (deltaT_Ticks * 1609344U);
}

static inline uint32_t Encoder_CalcGroundSpeed_Kmh(Encoder_T * p_encoder, uint32_t deltaD_Ticks, uint32_t deltaT_Ticks)
{
    return deltaD_Ticks * p_encoder->UnitSurfaceSpeed * 60U * 60U / (deltaT_Ticks * 1000000U);
}

/******************************************************************************/
/*!
    @brief Extern Declarations
*/
/*! @{ */
/******************************************************************************/
extern void Encoder_InitInterrupts_Quadrature(Encoder_T * p_encoder);
extern void Encoder_InitInterrupts_ABC(Encoder_T * p_encoder);

extern void Encoder_CalibrateAlignZero(Encoder_T * p_encoder);
extern void Encoder_CalibrateAlignValidate(Encoder_T * p_encoder);

#if defined(CONFIG_ENCODER_QUADRATURE_MODE_ENABLE)
extern void Encoder_SetQuadratureMode(Encoder_T * p_encoder, bool isEnabled);
extern void Encoder_EnableQuadratureMode(Encoder_T * p_encoder);
extern void Encoder_SetQuadratureDirectionCalibration(Encoder_T * p_encoder, bool isALeadBPositive);
extern void Encoder_CalibrateQuadratureReference(Encoder_T * p_encoder);
extern void Encoder_CalibrateQuadraturePositive(Encoder_T * p_encoder);
#endif

extern void _Encoder_ResetUnits(Encoder_T * p_encoder);
extern void _Encoder_ResetUnitsAngle(Encoder_T * p_encoder);
extern void _Encoder_ResetUnitsInterpolateAngle(Encoder_T * p_encoder);
extern void _Encoder_ResetUnitsAngularSpeed(Encoder_T * p_encoder);
extern void _Encoder_ResetUnitsLinearSpeed(Encoder_T * p_encoder);
extern void _Encoder_ResetUnitsScalarSpeed(Encoder_T * p_encoder);
extern void Encoder_SetCountsPerRevolution(Encoder_T * p_encoder, uint16_t countsPerRevolution);
extern void Encoder_SetDistancePerRevolution(Encoder_T * p_encoder, uint16_t distancePerCount);
extern void Encoder_SetScalarSpeedRef(Encoder_T * p_encoder, uint16_t speedRef);
extern void Encoder_SetSurfaceRatio(Encoder_T * p_encoder, uint32_t surfaceDiameter, uint32_t gearRatio_Factor, uint32_t gearRatio_Divisor);
extern void Encoder_SetGroundRatio_US(Encoder_T * p_encoder, uint32_t wheelDiameter_Inch10, uint32_t wheelToMotorRatio_Factor, uint32_t wheelToMotorRatio_Divisor);
extern void Encoder_SetGroundRatio_Metric(Encoder_T * p_encoder, uint32_t wheelDiameter_Mm, uint32_t wheelToMotorRatio_Factor, uint32_t wheelToMotorRatio_Divisor);
/******************************************************************************/
/*! @} */
/******************************************************************************/

/******************************************************************************/
/*!
    @brief
*/
/******************************************************************************/
/* todo testing */
static inline bool Encoder_GetIsAligned(const Encoder_T * p_encoder)
{
    return (p_encoder->Align == ENCODER_ALIGN_ABSOLUTE) || (p_encoder->Align == ENCODER_ALIGN_PHASE);
}

#endif
