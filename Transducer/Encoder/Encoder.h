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
#if (!defined(ENCODER_HW_DECODER) && !defined(ENCODER_HW_EMULATED))
#define ENCODER_HW_EMULATED
#endif

/* Emulated and Decoder */
#if !defined(ENCODER_QUADRATURE_MODE_ENABLE) && !defined(ENCODER_QUADRATURE_MODE_DISABLE)
#define ENCODER_QUADRATURE_MODE_ENABLE true
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


/******************************************************************************/
/*!
*/
/******************************************************************************/
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


/*
    Monotonic. Promotion is unconditional, only fault demotes.
    Written by the polling thread only - never from an ISR, so a fault demote cannot be lost.
    PositionRef
    struct { uint8_t IsAligned: 1U; uint8_t IsHomed: 1U; };
*/
typedef enum Encoder_RefState
{
    ENCODER_REF_STATE_NONE,      /* delta only. speed valid, angle arbitrary */
    ENCODER_REF_STATE_ALIGNED,   /* theta_el valid. theta_mech unknown when PolePairs > 1 */
    ENCODER_REF_STATE_HOMED,     /* theta_mech and theta_el valid */
}
Encoder_RefState_T;

typedef enum Encoder_HomingStatus
{
    ENCODER_HOMING_SEARCHING,
    ENCODER_HOMING_FOUND,
    ENCODER_HOMING_TIMEOUT,     /* travel budget spent without an index edge */
}
Encoder_HomingStatus_T;

/*
    Reserved value of IndexAngleCapture. Armed by the polling thread, overwritten by the Index ISR.
    The ISR writes this only when the pre-snap angle lands exactly a half revolution from
    IndexAngleOffset - 1 in 2^32, costing one extra search revolution, then self-correcting.
*/
#define ENCODER_INDEX_NOT_CAPTURED (INT32_MIN)


/*
    Reference Frames - stacked, fixed order. Base.Angle accumulates in the d-axis frame.
    Aligned:
        d-axis  : zero at the rotor pole.  Set by Align.                        Read by commutation.
    Index Homed:
        Index   : zero at the Z marker.    Config.IndexAngleOffset from d-axis. The bridge.
        User    : zero at the application. Config.VirtualHomeOffset from Z.     Read by position.

    Two independent constants, and neither can do the other's job. IndexAngleOffset is fixed by how
    the encoder was bolted to the motor - measured by calibration, never chosen. VirtualHomeOffset is
    chosen by the application - arbitrary, settable at any time. Folding them loses a frame: keep only
    the first and there is no application zero, keep only the second and commutation no longer knows
    where the d-axis is.

    Base.Angle stays in the d-axis frame so commutation reads it with no offset at 20kHz; the
    application frame costs one subtraction of a derived constant on the position read path.
*/
typedef struct Encoder_Config
{
    uint16_t CountsPerRevolution;           /* Derive Angular Units. */
    uint32_t SpeedPerUnitRef_Rpm;           /* Derive Fract16 Units. */
    uint32_t IndexAngleOffset;              /* AngleZ. Commutation offset - d-axis to Z. Calibrated, not user set. */
    uint32_t VirtualHomeOffset;             /* Home offset - Z to application zero. User set, arbitrary. */
    uint16_t ExtendedDeltaTStop;            /* ExtendedTimer time read as deltaT stopped, default as 1s */
    bool IsIndexCalibrated;                 /* IndexAngleOffset determined by a calibration pass. Angle 0 is a legal value. */
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
    // Angle_T InterpolationAngle;
    // PulseTimer_State_T TimerState;
    int32_t DirectionComp;

    /* Position Reference */
    int32_t IndexAngleCapture;      /* Base.Angle at the last index edge, raw. ISR writes. ENCODER_INDEX_NOT_CAPTURED when armed. */
    uint32_t UserZeroAngle;         /* Derived: IndexAngleOffset + VirtualHomeOffset. Application zero in the d-axis frame. */
    uint32_t HomingStepsRemaining;  /* Search travel budget, counted down by Encoder_ProcHoming */
    Encoder_RefState_T RefState;
}
Encoder_State_T;

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


/******************************************************************************/
/*!
    @brief Hal Abstraction
    Handle the AngleCounter
*/
/******************************************************************************/
/*
    Does not wrap in HW_EMULATED case
        = p_encoder->Angle32 / p_encoder->UnitAngleD;
*/
static inline int32_t Encoder_GetCounterD(Encoder_T * p_encoder)
{
#if     defined(ENCODER_HW_DECODER)
    return HAL_Encoder_ReadCounter(p_encoder->P_HAL_ENCODER_COUNTER);
#else
    return p_encoder->P_STATE->AngleCounter.CounterD;
#endif
}

static inline void _Encoder_SetCounterD(Encoder_T * p_encoder, int32_t counterD)
{
    p_encoder->P_STATE->AngleCounter.CounterD = counterD;
#if     defined(ENCODER_HW_DECODER)
    HAL_Encoder_WriteCounter(p_encoder->P_HAL_ENCODER_COUNTER, counterD);
#endif
}

static inline uint32_t _Encoder_GetAngle32(Encoder_T * p_encoder)
{
#if     defined(ENCODER_HW_DECODER)
    return HAL_Encoder_ReadCounter(p_encoder->P_HAL_ENCODER_COUNTER) * (uint32_t)p_encoder->P_STATE->AngleCounter.Ref.Angle32PerCount;
#else
    return p_encoder->P_STATE->AngleCounter.Base.Angle;
#endif
}

/*
    Reset pulse accumulation and speed state. Retains Base.Angle - the position datum is
    owned by Align and Index, not by a counter reset.
*/
static inline void _Encoder_ZeroPulseCount(Encoder_T * p_encoder)
{
    AngleCounter_ZeroCount(&p_encoder->P_STATE->AngleCounter);
    Angle_StopDelta(&p_encoder->P_STATE->AngleCounter.Base);
#if     defined(ENCODER_HW_DECODER)
    HAL_Encoder_WriteCounter(p_encoder->P_HAL_ENCODER_COUNTER, 0);
    HAL_Encoder_ClearCounterOverflow(p_encoder->P_HAL_ENCODER_COUNTER);
#endif
}

/******************************************************************************/
/*!
    @brief Angle by frame. Base.Angle is the d-axis frame.
*/
/******************************************************************************/
static inline uint16_t Encoder_GetAngle(Encoder_T * p_encoder) { return _Encoder_GetAngle32(p_encoder) >> ENCODER_ANGLE_SHIFT; }

/* Application frame - zero at virtual home. Valid when Encoder_IsHomed. */
static inline uint16_t Encoder_GetAngle_User(const Encoder_State_T * p_encoder) { return (p_encoder->AngleCounter.Base.Angle - p_encoder->UserZeroAngle) >> ENCODER_ANGLE_SHIFT; }

/******************************************************************************/
/*
    Index
*/
/******************************************************************************/
/*
    Assert theta := IndexAngleOffset. Two stores, no branch, no mode dispatch.

    Captures raw and interprets nothing - the polling thread subtracts IndexAngleOffset to get
    either the calibration delta or the per revolution count drift, and promotes RefState.

    IndexAngleCapture doubles as the armed sentinel, so no third field is written here:
    two writes carry two values, and a latch would be a third.
*/
static inline void Encoder_CaptureIndex(Encoder_State_T * p_encoder)
{
// #if defined(ENCODER_HW_DECODER)
//     // HAL_Encoder_ClearCounter(p_encoder->P_HAL_ENCODER_COUNTER);
// #endif
    // p_encoder->IndexAngleError = p_encoder->AngleCounter.Base.Angle - p_encoder->Config.IndexAngleOffset;
    p_encoder->IndexAngleCapture = p_encoder->AngleCounter.Base.Angle;
    p_encoder->AngleCounter.Base.Angle = p_encoder->Config.IndexAngleOffset; /* Snap to Z */
}

/* Signed count drift over the last revolution. Calibration delta while IndexAngleOffset is uncalibrated. */
static inline int32_t _Encoder_GetIndexAngleError(const Encoder_State_T * p_encoder) { return p_encoder->IndexAngleCapture - (int32_t)p_encoder->Config.IndexAngleOffset; }
static inline int32_t Encoder_GetIndexAngleError(const Encoder_State_T * p_encoder) { return _Encoder_GetIndexAngleError(p_encoder) >> ENCODER_ANGLE_SHIFT; }

/* An index edge has landed since the capture was last armed. IndexAngleError is meaningless until then. */
static inline bool Encoder_IsIndexCaptured(const Encoder_State_T * p_encoder) { return (p_encoder->IndexAngleCapture != ENCODER_INDEX_NOT_CAPTURED); }

/*
    Call at the sensor poll rate, on every path - not only while searching.
    An index edge reached during an ordinary open loop start up promotes the same way a
    deliberate homing sweep does; only the travel budget belongs to the search.
*/
static inline void Encoder_PollIndexCapture(Encoder_State_T * p_encoder)
{
    if (Encoder_IsIndexCaptured(p_encoder) == true) { p_encoder->RefState = ENCODER_REF_STATE_HOMED; }
}

/******************************************************************************/
/*!
    @brief Position Reference State
*/
/******************************************************************************/
static inline Encoder_RefState_T Encoder_GetRefState(const Encoder_State_T * p_encoder) { return p_encoder->RefState; }

/* theta_e valid - commutation available */
static inline bool Encoder_IsAligned(const Encoder_State_T * p_encoder) { return (p_encoder->RefState >= ENCODER_REF_STATE_ALIGNED); }
/* theta_m valid - absolute position available */
static inline bool Encoder_IsHomed(const Encoder_State_T * p_encoder) { return (p_encoder->RefState >= ENCODER_REF_STATE_HOMED); }

static inline bool Encoder_IsPositionRefSet(const Encoder_State_T * p_encoder) { return Encoder_IsAligned(p_encoder); }


/******************************************************************************/
/*!
    Align to the rotor d-axis. Electrical datum only.
*/
/******************************************************************************/
/*
    Assert theta := 0 at the d-axis. Call after energizing phase A and waiting for the shaft to settle.
    Skipped once homed - a mechanical datum is strictly stronger and must not be demoted to an
    electrical one, which would also discard the calibrated frame on every ordinary start up.

    Re-arms the capture: the frame restarts here, so only an index edge seen after this align
    may promote. A stale capture from a previous run refers to a frame that no longer exists.
*/
static inline void Encoder_CaptureAlignZero(Encoder_State_T * p_encoder)
{
    if (Encoder_IsHomed(p_encoder) == false)
    {
        p_encoder->AngleCounter.Base.Angle = 0;
        p_encoder->IndexAngleCapture = ENCODER_INDEX_NOT_CAPTURED;
        p_encoder->RefState = ENCODER_REF_STATE_ALIGNED;
    }
}

static inline void Encoder_CompleteAlignValidate(const Encoder_T * p_encoder)
{
    _Encoder_ZeroPulseCount(p_encoder);
}


/******************************************************************************/
/*!
    @brief Direction
*/
/******************************************************************************/
static inline bool _Encoder_IsQuadratureCaptureEnabled(const Encoder_State_T * p_encoder)
{
#if     defined(ENCODER_QUADRATURE_MODE_ENABLE)
    return (p_encoder->Config.IsQuadratureCaptureEnabled == true);
#else
    return false;
#endif
}

/* 0 until set */
static inline int32_t _Encoder_SinglePhase_GetDirection(const Encoder_State_T * p_encoder) { return p_encoder->DirectionComp; }
/* Direction set by outer module */
/* SinglePhase Capture is always positive. assign direction comp */
static inline void Encoder_SinglePhase_CaptureDirection(Encoder_State_T * p_encoder, int8_t direction) { p_encoder->DirectionComp = direction; }

/*
    Convert signed capture to user reference. Captured as ALeadB is positive by default
*/
static inline int32_t _Encoder_Quadrature_GetDirection(const Encoder_State_T * p_encoder) { return (p_encoder->Config.IsALeadBPositive == true) ? 1 : -1; }

/* On Init */
/* Select using IsQuadratureCaptureEnabled */
static inline int32_t _Encoder_ResolveDirectionComp(const Encoder_State_T * p_encoder)
{
    return _Encoder_IsQuadratureCaptureEnabled(p_encoder) ? _Encoder_Quadrature_GetDirection(p_encoder) : _Encoder_SinglePhase_GetDirection(p_encoder);
}

/* Query for Comp */
/* return value assigned at Init */
static inline int32_t Encoder_GetDirectionRef(const Encoder_State_T * p_encoder) { return p_encoder->DirectionComp; }


/******************************************************************************/
/*!
    @brief Extern Declarations
*/
/******************************************************************************/
extern void Encoder_InitCounter(Encoder_T * p_encoder);

#if defined(ENCODER_HW_EMULATED)
extern void Encoder_InitInterrupts_Quadrature(Encoder_T * p_encoder);
#endif

extern void Encoder_PollIndexCapture(Encoder_State_T * p_encoder);

/* Homing - acquire the mechanical datum from the index */
extern uint16_t Encoder_GetHomingDelta(const Encoder_State_T * p_encoder);
extern void Encoder_StartHoming(Encoder_State_T * p_encoder);
extern void Encoder_ProcHoming(Encoder_State_T * p_encoder);
extern Encoder_HomingStatus_T Encoder_GetHomingStatus(const Encoder_State_T * p_encoder);
extern Encoder_HomingStatus_T Encoder_PollHoming(Encoder_State_T * p_encoder);

extern void Encoder_CalibrateIndexAngleOffset(Encoder_State_T * p_encoder);
extern void Encoder_CalibrateVirtualHomeOffset(Encoder_State_T * p_encoder);

/* Align - acquire the electrical datum from the rotor d-axis */
// extern void Encoder_CaptureAlignZero(Encoder_State_T * p_encoder);
// extern void Encoder_CompleteAlignValidate(const Encoder_T * p_encoder);

extern void Encoder_ClearPositionRef(Encoder_State_T * p_encoder);
extern void Encoder_ResolveUserZero(Encoder_State_T * p_encoder);

#if defined(ENCODER_QUADRATURE_MODE_ENABLE)
extern void Encoder_SetQuadratureMode(Encoder_State_T * p_encoder, bool isEnabled);
extern void Encoder_EnableQuadratureMode(Encoder_State_T * p_encoder);
extern void Encoder_SetQuadratureDirection(Encoder_State_T * p_encoder, bool isALeadBPositive);
extern void Encoder_CaptureQuadratureReference(Encoder_T * p_encoder);
extern void Encoder_CalibrateQuadraturePositive(Encoder_T * p_encoder);
#endif

extern void Encoder_SetCountsPerRevolution(Encoder_State_T * p_encoder, uint16_t countsPerRevolution);
extern void Encoder_SetSpeedPerUnitRef(Encoder_State_T * p_encoder, uint16_t speedRef);

/******************************************************************************/
/*!
*/
/******************************************************************************/
/*
    Read-only diagnostics. Everything here is derived from existing state - no field exists for
    the sake of the interface.
*/
typedef enum Encoder_VarId
{
    /* Speed */
    ENCODER_VAR_FREQ,
    ENCODER_VAR_RPM,
    ENCODER_VAR_DELTA_T_SPEED,
    ENCODER_VAR_DELTA_D_SPEED,
    ENCODER_VAR_COUNTER_D,

    /* Position - the same shaft in each of the two readable frames */
    ENCODER_VAR_ANGLE,              /* angle16, d-axis frame. What commutation reads. */
    ENCODER_VAR_ANGLE_USER,         /* angle16, application frame. Meaningful once HOMED. */

    /* Reference acquisition - answers "why will it not run" and "is position trustworthy" */
    ENCODER_VAR_REF_STATE,          /* Encoder_RefState_T: NONE / ALIGNED / HOMED */
    ENCODER_VAR_HOMING_STATUS,      /* Encoder_HomingStatus_T: SEARCHING / FOUND / TIMEOUT */
    ENCODER_VAR_IS_INDEX_CAPTURED,  /* an index edge has landed since the capture was armed */

    /* Signal integrity */
    ENCODER_VAR_INDEX_ANGLE_ERROR,  /* signed angle16 drift over the last revolution. 0 when no counts lost. */
    ENCODER_VAR_ERROR_COUNT,        /* illegal quadrature transitions */
    ENCODER_VAR_DIRECTION_COMP,     /* resolved install direction, +1 / -1 / 0 before calibration */
    ENCODER_VAR_PHASES,             /* raw A/B state, XXXXBABA. Wiring check at standstill. */
}
Encoder_VarId_T;

/*

*/
typedef enum Encoder_ConfigId
{
    ENCODER_CONFIG_COUNTS_PER_REVOLUTION,
    ENCODER_CONFIG_EXTENDED_TIMER_DELTA_T_STOP,
    ENCODER_CONFIG_IS_QUADRATURE_CAPTURE_ENABLED,
    ENCODER_CONFIG_IS_A_LEAD_B_POSITIVE,
    ENCODER_CONFIG_INDEX_ANGLE_OFFSET,      /* Commutation offset - d-axis to Z */
    ENCODER_CONFIG_VIRTUAL_HOME_OFFSET,     /* Home offset - Z to application zero */
    ENCODER_CONFIG_IS_INDEX_CALIBRATED,
}
Encoder_ConfigId_T;

extern int32_t _Encoder_ConfigId_Get(const Encoder_Config_T * p_encoder, Encoder_ConfigId_T varId);
extern void _Encoder_ConfigId_Set(Encoder_Config_T * p_encoder, Encoder_ConfigId_T varId, int32_t value);

extern void Encoder_ConfigId_Set(Encoder_T * p_encoder, Encoder_ConfigId_T varId, int32_t varValue);

extern int32_t Encoder_VarId_Get(const Encoder_State_T * p_encoder, Encoder_VarId_T varId);

