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

#include "Math/Fixed/fixed.h"
#include "Math/math_general.h"

#include <stdint.h>
#include <stdbool.h>

#ifndef ENCODER_TIMER_MAX
#define ENCODER_TIMER_MAX (0xFFFFU)
#endif

#ifndef ENCODER_ANGLE_DEGREES
#define ENCODER_ANGLE_BITS      (16U)
#define ENCODER_ANGLE_SHIFT     (32U - ENCODER_ANGLE_BITS)
#define ENCODER_ANGLE_DEGREES   ((uint32_t)1UL << ENCODER_ANGLE_BITS) // ENCODER_ANGLE_WRAP
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
    ENCODER_ALIGN_NO,
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
    uint16_t CountsPerRevolution;       /* Derive Angular Units. */
    uint16_t ScalarSpeedRef_Rpm;        /* Derive Percent16 Units. */
    uint16_t SurfaceDiameter;           /* Derive Linear Units. */
    uint16_t GearRatioInput;            /* DistancePerRevolution_Divider */
    uint16_t GearRatioOutput;           /* DistancePerRevolution_Factor */
    uint16_t ExtendedDeltaTStop;        /* ExtendedTimer time read as deltaT stopped, default as 1s */
    uint32_t InterpolateAngleScalar;    /* Sets UnitInterpolateAngle Scalar and InterpolateAngleLimit. e.g electrical angle conversion */

    uint32_t IndexAngleRef;             /* Virtual Index - Index, VirtualIndexOffset */
    uint32_t AlignOffsetRef;            /* Align - Index */

#if defined(CONFIG_ENCODER_QUADRATURE_MODE_ENABLE)
    bool IsQuadratureCaptureEnabled;    /* Quadrature Mode - enable hardware/emulated quadrature speed capture */
    bool IsALeadBPositive;              /* User runtime calibration for encoder install direction. Accounts for LUT calibration */
    /* Optionally combine with compile time defined QUADRATURE_A_LEAD_B_INCREMENT */
#endif
}
Encoder_Config_T;

typedef const struct Encoder_Const
{
#if     defined(CONFIG_ENCODER_HW_DECODER)
    HAL_Encoder_Counter_T * const P_HAL_ENCODER_COUNTER; /*!< Pulse Counter */
#elif   defined(CONFIG_ENCODER_HW_EMULATED)
    /* HAL_Encoder_Pin_T configures settings not included in Pin_T, case of interrupt support */
    HAL_Encoder_Pin_T * const P_HAL_PIN_A; const uint32_t PIN_A_ID;
    HAL_Encoder_Pin_T * const P_HAL_PIN_B; const uint32_t PIN_B_ID;
    HAL_Encoder_Pin_T * const P_HAL_PIN_Z; const uint32_t PIN_Z_ID;
#endif
    /*  Timers used to represent time. */
    HAL_Encoder_Timer_T * const P_HAL_ENCODER_TIMER;    /*!< DeltaT Timer. */
    const uint32_t TIMER_FREQ;                          /*!< DeltaT Timer Freq */
    const volatile uint32_t * const P_EXTENDED_TIMER;   /* 32-bit extension + WatchStop */
    const uint32_t EXTENDED_TIMER_FREQ;

    const uint32_t POLLING_FREQ;        /*!< Angle Sample Freq. DeltaT Interpolation Freq. */
    const uint32_t SAMPLE_FREQ;         /*!< Speed Sample Freq. DeltaD Sample Freq. */
    const uint32_t SAMPLE_TIME;         /* derived from TIMER_FREQ/SAMPLE_TIME */
    const Encoder_Config_T * const P_CONFIG;
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

// #if defined(CONFIG_ENCODER_HW_EMULATED)
    Encoder_Phases_T Phases; /* Save Prev State */
    uint32_t CounterD;
    uint32_t Angle32;

    // int32_t DirectionD;     /*!< CounterD Direction without user compensation. previous DeltaD sign, when DeltaD == 0 */
    bool IsSinglePhasePositive;
    // int32_t DirectionComp;   /* todo */

    uint32_t CounterPrev; /* for DeltaD */
    uint32_t ExtendedTimerPrev; /* for DeltaT */
    uint32_t ExtendedTimerConversion;    /* Extended Timer to Short Timer */

    int32_t DeltaD;         /*!< Counter counts (of distance) between 2 samples. Units in raw counter ticks */
    uint32_t DeltaT;        /*!< Timer counts between 2 pulse counts. Units in raw timer ticks */
    uint32_t DeltaTh;       /*!< ModeDT */
    int32_t FreqD;          /*!< EncoderPulseFreq ModeDT. DeltaD 1 Second [Hz] */

    uint32_t InterpolateAngleDelta; /*!< UnitInterpolateAngle / DeltaT */
    uint32_t InterpolateAngleIndex; /*!< ModeT/DT */
    uint32_t InterpolateAngleSum;
    uint32_t InterpolateAngleLimit;

    uint32_t ErrorCount;

    uint32_t IndexCount;
    uint32_t IndexAngleRef; /* 32 */
    uint32_t IndexAngleError;
    // uint32_t CounterOnIndex;
    // uint32_t CounterOnIndexPrev;
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
        Unit conversion. derived on init from Nv Config
    */
    uint32_t UnitTime_Freq;                 /*!< Common Units propagating set depending on mode. T seconds conversion factor. */
    uint32_t UnitAngleD;                    /*!< [(UINT32_MAX+1)/CountsPerRevolution] => Angle = PulseCounter * UnitAngleD >> DEGREES_SHIFT */
    uint32_t UnitLinearD;                   /*!< Linear D unit conversion factor. Units per TimerCounter tick, using Capture DeltaD (DeltaT = 1). Units per DeltaT capture, using Capture DeltaT (DeltaD = 1).*/
    uint32_t UnitInterpolateAngle;          /*!< [UnitTime_Freq << DEGREES_BITS / POLLING_FREQ / CountsPerRevolution] */

    uint32_t UnitScalarSpeed;               /*!< Percentage Speed of ScalarSpeedRef_Rpm, given max speed, as 65535 */
    uint32_t UnitAngularSpeed;              /*!< [(1 << DEGREES_BITS) * UnitTime_Freq / CountsPerRevolution] => AngularSpeed = DeltaD * UnitAngularSpeed / DeltaT */
    uint32_t UnitSurfaceSpeed;              /*!< [UnitD * UnitTime_Freq] => Speed = DeltaD * UnitSpeed / DeltaT */
    uint8_t UnitScalarSpeedShift;
    uint8_t UnitSurfaceSpeedShift;          /* Shifts applicable to ModeD/DT */
    uint8_t UnitAngularSpeedShift;
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
    .CONST =                                                                                                                \
    {                                                                                                                       \
        _ENCODER_INIT_HW_COUNTER(p_CounterHal, p_PhaseAHal, PhaseAId, p_PhaseBHal, PhaseBId, p_PhaseZHal, PhaseZId)         \
        .P_HAL_ENCODER_TIMER    = p_TimerHal,            .TIMER_FREQ           = TimerFreq,                                 \
        .P_EXTENDED_TIMER       = p_ExtendedTimer,       .EXTENDED_TIMER_FREQ  = ExtendedTimerFreq,                         \
        .POLLING_FREQ           = PollingFreq,           .SAMPLE_FREQ          = SpeedSampleFreq,                           \
        .P_CONFIG               = p_Config,                                                                                 \
        .SAMPLE_TIME = TimerFreq/SpeedSampleFreq,                                                                           \
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
    if (p_encoder->Config.IsQuadratureCaptureEnabled == true) { quadratureFunction(p_encoder); } else { singlePhaseFunction(p_encoder); }
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
    @brief Hal Abstraction
*/
/******************************************************************************/
static inline uint32_t _Encoder_CaptureDeltaWrap(uint32_t tcMax, uint32_t tcPrev, uint32_t tcValue)
{
    return (tcValue < tcPrev) ? (tcMax + 1U + tcValue - tcPrev) : (tcValue - tcPrev);
}


static inline int32_t Encoder_GetCounterD(const Encoder_T * p_encoder)
{
#if     defined(CONFIG_ENCODER_HW_DECODER)
    return HAL_Encoder_ReadCounter(p_encoder->CONST.P_HAL_ENCODER_COUNTER);
#elif   defined(CONFIG_ENCODER_HW_EMULATED)
    return p_encoder->CounterD;
#endif
}

static inline void _Encoder_SetCounterD(Encoder_T * p_encoder, int32_t counterD)
{
#if     defined(CONFIG_ENCODER_HW_DECODER)
    HAL_Encoder_WriteCounter(p_encoder->CONST.P_HAL_ENCODER_COUNTER, counterD);
#elif   defined(CONFIG_ENCODER_HW_EMULATED)
    p_encoder->CounterD = counterD;
#endif
}

static inline uint32_t _Encoder_GetAngle32(const Encoder_T * p_encoder)
{
#if     defined(CONFIG_ENCODER_HW_DECODER)
    return HAL_Encoder_ReadCounter(p_encoder->CONST.P_HAL_ENCODER_COUNTER) * (uint32_t)p_encoder->UnitAngleD;
#elif   defined(CONFIG_ENCODER_HW_EMULATED)
    return p_encoder->Angle32;
#endif
}


/******************************************************************************/
/*!
    @brief Direction
*/
/******************************************************************************/
/* Direction set by outer module */
static inline int32_t _Encoder_SinglePhase_GetDirection(const Encoder_T * p_encoder) { return (p_encoder->IsSinglePhasePositive == true) ? 1 : -1; }

/*
    Convert signed capture to user reference. Captured as ALeadB is positive by default
*/
static inline int32_t _Encoder_Quadrature_GetDirection(const Encoder_T * p_encoder) { return (p_encoder->Config.IsALeadBPositive == true) ? 1 : -1; }

/* Select using IsQuadratureCaptureEnabled */
static inline int32_t _Encoder_GetDirectionComp(const Encoder_T * p_encoder)
{
    return Encoder_CaptureMode_Get(p_encoder, _Encoder_Quadrature_GetDirection, _Encoder_SinglePhase_GetDirection);
}

static inline int32_t Encoder_GetDirectionRef(const Encoder_T * p_encoder) { return _Encoder_GetDirectionComp(p_encoder); }

/*
    Capture Direction on each Count or accumulated during Speed
*/
static inline void Encoder_CaptureDirection(Encoder_T * p_encoder, int8_t sign)
{
    // if (p_encoder->DirectionD != sign) {  } /* comp tick , reset T if needed */
    /*
        Store the direction for when CounterD = 0, with compensator
    */
    //     p_encoder->DirectionD = math_sign(p_encoder->CounterD) * _Encoder_GetDirectionComp(p_encoder);
    //     return p_encoder->DirectionD;
    // p_encoder->DirectionD = sign; // * _Encoder_GetDirectionComp(p_encoder);
}

// static inline int32_t Encoder_GetUserDirection(const Encoder_T * p_encoder) { return  p_encoder->DirectionD * _Encoder_GetDirectionComp(p_encoder); }

/******************************************************************************/
/*!
    @brief
*/
/******************************************************************************/
static inline void _Encoder_ZeroPulseCount(Encoder_T * p_encoder)
{
    p_encoder->CounterD = 0U;
    p_encoder->CounterPrev = 0U;
    p_encoder->IndexCount = 0U;

#if     defined(CONFIG_ENCODER_HW_DECODER)
    p_encoder->CounterPrev = HAL_Encoder_ReadCounter(p_encoder->CONST.P_HAL_ENCODER_COUNTER);
    HAL_Encoder_WriteCounter(p_encoder->CONST.P_HAL_ENCODER_COUNTER, 0);
    HAL_Encoder_ClearCounterOverflow(p_encoder->CONST.P_HAL_ENCODER_COUNTER);
#endif
}

static inline uint16_t _Encoder_GetAngle(const Encoder_T * p_encoder) { return _Encoder_GetAngle32(p_encoder) >> ENCODER_ANGLE_SHIFT; }

static inline uint16_t Encoder_GetAngle(const Encoder_T * p_encoder) { return Encoder_GetDirectionRef(p_encoder) * _Encoder_GetAngle(p_encoder); }
/* scalar < 256 */
static inline uint16_t Encoder_GetAngle_Scalar(const Encoder_T * p_encoder, uint8_t scalar) { return Encoder_GetDirectionRef(p_encoder) * (((_Encoder_GetAngle32(p_encoder) >> 8U) * scalar) >> 8U); }

static inline uint16_t Encoder_GetElectricalAngle(const Encoder_T * p_encoder, uint8_t polePairs)
{
    // return Encoder_GetDirectionRef(p_encoder) * (((_Encoder_GetAngle32(p_encoder) >> 8U) * polePairs) >> 8U) + p_encoder->AlignOffsetRef;

    // p_encoder->AlignOffsetRef = 0 if aligned to phase
    // indexangle-algnedangle

}


/* SinglePhase assign direction */
static inline void Encoder_SinglePhase_SetDirectionPositive(Encoder_T * p_encoder) { p_encoder->IsSinglePhasePositive = true; /*  DirectionComp = 1 */}
static inline void Encoder_SinglePhase_SetDirectionNegative(Encoder_T * p_encoder) { p_encoder->IsSinglePhasePositive = false; /*  DirectionComp = -1 */ }


/******************************************************************************/
/*!
    @brief
*/
/******************************************************************************/

static inline bool Encoder_IsAligned(const Encoder_T * p_encoder)
{
    return (p_encoder->Align == ENCODER_ALIGN_ABSOLUTE) || (p_encoder->Align == ENCODER_ALIGN_PHASE);
}

static inline bool _Encoder_IsAligned(const Encoder_T * p_encoder)
{
    // return (p_encoder->Align == ENCODER_ALIGN_PHASE) || (p_encoder->IsHomed && Encoder_IsAlignOffsetOn(p_encoder));
}

static inline bool Encoder_IsPositionRefSet(Encoder_T * p_encoder) { return (p_encoder->IsHomed || Encoder_IsAligned(p_encoder)); }


/******************************************************************************/
/*!
    @brief inline config
*/
/******************************************************************************/
static inline uint16_t Encoder_GetIndexZeroRef(const Encoder_T * p_encoder) { return p_encoder->Config.IndexAngleRef >> ENCODER_ANGLE_SHIFT; }
static inline void Encoder_SetIndexZeroRef(Encoder_T * p_encoder, uint16_t angle) { p_encoder->Config.IndexAngleRef = angle << ENCODER_ANGLE_SHIFT; }

/* Clears the value to config index to set as 0 */
static inline void Encoder_ClearIndexZeroRef(Encoder_T * p_encoder) { p_encoder->Config.IndexAngleRef = 0U; }

/******************************************************************************/
/*!
    @brief Pulse CounterD conversions.
*/
/******************************************************************************/
/*!
    Angle - Base unit in ENCODER_ANGLE_BITS
    input [0:CountsPerRevolution], UnitAngleD == UINT32_MAX / CountsPerRevolution
*/
static inline uint32_t _Encoder_AngleOfCount(const Encoder_T * p_encoder, uint32_t counterD_Ticks)
{
    return ((counterD_Ticks * p_encoder->UnitAngleD) >> ENCODER_ANGLE_SHIFT);
}

// static inline uint32_t Encoder_AngleOfDeltaD(const Encoder_T * p_encoder, uint32_t counterD_Ticks)
// {
//     return (counterD_Ticks < p_encoder->Config.CountsPerRevolution) ?
//         ((counterD_Ticks * p_encoder->UnitAngleD) >> ENCODER_ANGLE_SHIFT) :
//         ((counterD_Ticks << ENCODER_ANGLE_BITS) / p_encoder->Config.CountsPerRevolution);
// }

static inline uint32_t _Encoder_CountOfAngle(const Encoder_T * p_encoder, uint16_t angle_UserDegrees)
{
    return (angle_UserDegrees << ENCODER_ANGLE_SHIFT) / p_encoder->UnitAngleD;
}

/*!
    Linear Distance
*/
static inline uint32_t Encoder_DistanceOfCount(const Encoder_T * p_encoder, uint32_t counterD_Ticks) { return counterD_Ticks * p_encoder->UnitLinearD; }
static inline uint32_t Encoder_CountOfDistance(const Encoder_T * p_encoder, uint32_t distance_Units) { return distance_Units / p_encoder->UnitLinearD; }

/******************************************************************************/
/*!

*/
/******************************************************************************/

/*
    Only When base units in mm, as set via SetGroundRatio function.
*/
static inline uint32_t Encoder_GroundSpeedOf_Mph(const Encoder_T * p_encoder, uint32_t deltaD_Ticks, uint32_t deltaT_Ticks)
{
    return deltaD_Ticks * p_encoder->UnitSurfaceSpeed * 60U * 60U / (deltaT_Ticks * 1609344U);
}

static inline uint32_t Encoder_GroundSpeedOf_Kmh(const Encoder_T * p_encoder, uint32_t deltaD_Ticks, uint32_t deltaT_Ticks)
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

extern void Encoder_CaptureAlignZero(Encoder_T * p_encoder);
extern void Encoder_CompleteAlignValidate(Encoder_T * p_encoder);

void Encoder_StartHoming(Encoder_T * p_encoder);
uint16_t Encoder_GetHomingAngle(const Encoder_T * p_encoder);
bool Encoder_PollHomingComplete(Encoder_T * p_encoder);
void Encoder_CalibrateIndexZeroRef(Encoder_T * p_encoder);
void Encoder_ClearIndexZeroRef(Encoder_T * p_encoder);

#if defined(CONFIG_ENCODER_QUADRATURE_MODE_ENABLE)
extern void Encoder_SetQuadratureMode(Encoder_T * p_encoder, bool isEnabled);
extern void Encoder_EnableQuadratureMode(Encoder_T * p_encoder);
extern void Encoder_SetQuadratureDirection(Encoder_T * p_encoder, bool isALeadBPositive);
extern void Encoder_CaptureQuadratureReference(Encoder_T * p_encoder);
extern void Encoder_CalibrateQuadraturePositive(Encoder_T * p_encoder);
#endif

extern void _Encoder_ResetUnits(Encoder_T * p_encoder);
extern void _Encoder_ResetUnitsAngle(Encoder_T * p_encoder);
extern void _Encoder_ResetUnitsInterpolateAngle(Encoder_T * p_encoder);
extern void _Encoder_ResetUnitsAngularSpeed(Encoder_T * p_encoder);
extern void _Encoder_ResetUnitsLinearSpeed(Encoder_T * p_encoder);
extern void _Encoder_ResetUnitsScalarSpeed(Encoder_T * p_encoder);
extern void Encoder_SetCountsPerRevolution(Encoder_T * p_encoder, uint16_t countsPerRevolution);
extern void Encoder_SetScalarSpeedRef(Encoder_T * p_encoder, uint16_t speedRef);

// extern void Encoder_SetDistancePerRevolution(Encoder_T * p_encoder, uint16_t distance, uint16_t rotation);
extern void Encoder_SetSurfaceRatio(Encoder_T * p_encoder, uint32_t surfaceDiameter, uint32_t gearRatioSurface, uint32_t gearRatioDrive);
extern void Encoder_SetGroundRatio_US(Encoder_T * p_encoder, uint32_t wheelDiameter_Inch10, uint32_t wheelRatio, uint32_t motorRatio);
extern void Encoder_SetGroundRatio_Metric(Encoder_T * p_encoder, uint32_t wheelDiameter_Mm, uint32_t wheelRatio, uint32_t motorRatio);
/******************************************************************************/
/*! @} */
/******************************************************************************/


#endif
