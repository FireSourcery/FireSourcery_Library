#pragma once
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
    @file   Motor.h
    @author FireSourcery
    @brief  Per Motor State Control.
*/
/******************************************************************************/
#include "Phase/Phase_VOut.h"
#include "Phase_Input/Phase_Input.h"
#include "Phase_Input/Phase_Analog.h"
#include "Phase_Input/Phase_Calibration.h"
#include "VBus/VBus.h"
#include "VBus/VBus_Monitor.h"

#include "Sensor/RotorSensor_Table.h"
#include "Sensor/RotorSensor.h"

#include "Math/FOC.h"
// #ifdef
#include "Math/FOC_Sensorless.h"

#include "Peripheral/ADC/ADC_Conversion.h"
#include "Peripheral/ADC/Linear_ADC.h"

#include "Transducer/Encoder/Encoder_ModeDT.h"
#include "Transducer/Encoder/Encoder_ISR.h"
#include "Transducer/Monitor/Heat/HeatMonitor.h"

#include "Framework/StateMachine/StateMachine.h"
#include "Framework/StateMachine/_StateMachine.h" /* Include the private header to contain StateMachine_Active_T within Motor_Context_T */
#include "Framework/Timer/Timer.h"

#include "Math/Fixed/fixed.h"
#include "Math/Angle/Angle.h"
#include "Math/Angle/Angle_SpeedPu.h"
#include "Math/Linear/Linear.h"
#include "Math/Accumulator/Accumulator.h"
#include "Math/Ramp/Ramp.h"
#include "Math/PID/PID.h"

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

/* Static Def */
#include "Motor_ControlFreq.h"
#include "Motor_Electrical.h"


/******************************************************************************/
/*
*/
/******************************************************************************/
#if !defined(MOTOR_V_MATCH_SENSOR) && !defined(MOTOR_V_MATCH_SPEED)
#define MOTOR_V_MATCH_SENSOR
#endif

#if !defined(MOTOR_UNIT_CONVERSION_LOCAL)
#define MOTOR_UNIT_CONVERSION_HOST
#endif

#if !defined(MOTOR_EXTERN_CONTROL_ENABLE)
#define MOTOR_EXTERN_CONTROL_DISABLE
#endif

#if !defined(MOTOR_OPEN_LOOP_RUN_ENABLE) && !defined(MOTOR_OPEN_LOOP_RUN_DISABLE)
#define MOTOR_OPEN_LOOP_RUN_ENABLE
#endif

#if !defined(MOTOR_CALIBRATION_ELECTRICAL_ENABLE) && !defined(MOTOR_CALIBRATION_ELECTRICAL_DISABLE)
#define MOTOR_CALIBRATION_ELECTRICAL_ENABLE
#endif

// #if !defined(MOTOR_CALIBRATION_HOMING_ENABLE) && !defined(MOTOR_CALIBRATION_HOMING_DISABLE)
// #define MOTOR_CALIBRATION_HOMING_ENABLE
// #endif

#if !defined(MOTOR_FOC_FIELD_WEAKENING_ENABLE) && !defined(MOTOR_FOC_FIELD_WEAKENING_DISABLE)
#define MOTOR_FOC_FIELD_WEAKENING_ENABLE
#endif


#ifndef MOTOR_CALIBRATION_BUFFER_SIZE
#define MOTOR_CALIBRATION_BUFFER_SIZE 128U
#endif

/******************************************************************************/
/*
*/
/******************************************************************************/
typedef enum Motor_CommutationMode
{
    MOTOR_COMMUTATION_MODE_FOC,
    MOTOR_COMMUTATION_MODE_SIX_STEP,
}
Motor_CommutationMode_T;

/*
    Direction Run SubState
*/
typedef enum Motor_Direction
{
    MOTOR_DIRECTION_CW = -1,
    MOTOR_DIRECTION_NULL = 0,
    MOTOR_DIRECTION_CCW = 1,
    MOTOR_DIRECTION_REVERSE = MOTOR_DIRECTION_CW,
    MOTOR_DIRECTION_FORWARD = MOTOR_DIRECTION_CCW,
}
Motor_Direction_T;

static inline Motor_Direction_T Motor_Direction_Cast(int direction)
{
    switch ((Motor_Direction_T)direction)
    {
        case MOTOR_DIRECTION_CW:
        case MOTOR_DIRECTION_CCW:
        case MOTOR_DIRECTION_NULL:
            return (Motor_Direction_T)direction;
        default: return MOTOR_DIRECTION_NULL; /* Invalid direction */
    }
}


/******************************************************************************/
/*
    Feedback Mode
*/
/******************************************************************************/
typedef union Motor_FeedbackMode
{
    struct
    {
        uint8_t OpenLoop   : 1U;
        uint8_t Current    : 1U;   /* 0 -> Voltage, 1-> Current */
        uint8_t Speed      : 1U;   /* 0 -> Voltage or Current only, 1 -> Speed feedback */
        uint8_t Position   : 1U;
    };
    uint8_t Value; /* Id */
}
Motor_FeedbackMode_T;

/* Defined as const bit-fields rather than enum. in line with bit as conditional, not all combination need to be defined */
/* OpenLoop handled with separate State Branch */
/* Run state may change to substate handling - Current/Speed same branch */
static const Motor_FeedbackMode_T MOTOR_FEEDBACK_MODE_VOLTAGE              = { .Speed = 0U, .Current = 0U, };
static const Motor_FeedbackMode_T MOTOR_FEEDBACK_MODE_CURRENT              = { .Speed = 0U, .Current = 1U, };
static const Motor_FeedbackMode_T MOTOR_FEEDBACK_MODE_SPEED_VOLTAGE        = { .Speed = 1U, .Current = 0U, };
static const Motor_FeedbackMode_T MOTOR_FEEDBACK_MODE_SPEED_CURRENT        = { .Speed = 1U, .Current = 1U, };

static inline Motor_FeedbackMode_T Motor_FeedbackMode_Cast(int value) { return (Motor_FeedbackMode_T) { .Value = value }; }


/******************************************************************************/
/*
    Fault State Flags
*/
/******************************************************************************/
typedef union Motor_FaultFlags
{
    struct
    {
        uint16_t Overheat       : 1U;
        uint16_t PositionSensor : 1U;
        uint16_t InitCheck      : 1U;
    };
    uint16_t Value;
}
Motor_FaultFlags_T;

static const Motor_FaultFlags_T MOTOR_FAULT_OVERHEAT         = { .Overheat       = 1U };
static const Motor_FaultFlags_T MOTOR_FAULT_POSITION_SENSOR  = { .PositionSensor = 1U };
static const Motor_FaultFlags_T MOTOR_FAULT_INIT_CHECK       = { .InitCheck      = 1U };



//todo
// real_t, fract_t

/*!
    @brief Motor Config - Runtime variable configuration, settings. Load from non volatile memory.
*/
typedef struct Motor_Config
{
    /*
        Calibration parameters
    */
    RotorSensor_Id_T SensorMode;
    Motor_Direction_T DirectionForward;         /* CCW/CW Assigned positive direction. Effectively Direction actual CCW */
    Motor_ElectricalSpeedRating_T SpeedRating;  /* PolePairs, Kv, SpeedRated_Rpm */
    Phase_Triplet_T IabcZeroRef_Adcu;

    /*
        Persistent Base Limits.
        Limits of Input Ramps
        User view as unsigned. Physical frame.
        Use same units as Req/Feedback
    */
    uint16_t SpeedLimitForward_Fract16;     /* [0:32767] = [0:100%] of SpeedTypeMax */
    uint16_t SpeedLimitReverse_Fract16;
    uint16_t ILimitMotoring_Fract16;        /* [0:32767] = [0:100%] of I_TYPE_MAX_AMPS. */
    uint16_t ILimitGenerating_Fract16;

    /*
        Acceleration as slope:
        Pu per tick with additional ACCUMULATOR_SHIFT.
    */
    uint32_t SpeedRampSlope_Accum32;
    uint32_t TorqueRampSlope_Accum32;

    PID_Config_T PidSpeed;  /* Speed Control */
    PID_Config_T PidI;      /* Idq Control */

    /*
        OpenLoop
    */
    /* All OpenLoop Modes - UserCmd, Align */
    /* optionally keep precompile limit only, or hide on view side without using a seperate region of memory */
    uint16_t OpenLoopLimitRatio;    /* Limit of rated. as scalar [0:1.0F] [0:32768]. V/I Align_Fract16 < OpenLoopLimitRatio * V/I RATED */

    /* Calibration and Jog Align */
    uint16_t IAlign_Fract16;                 /* OpenLoop/Calibration Align Current, as fract16 of I_TYPE_MAX_AMPS. */
    uint16_t VAlign_Fract16;                 /* OpenLoop/Calibration Align Voltage, as fract16 of V_TYPE_MAX_VOLTS. */
    uint32_t AlignTime_Cycles;               /* Ramp time and step duration */

    /* OpenLoop Run/StartUp Preset */
// #if defined(MOTOR_OPEN_LOOP_ENABLE)
    uint16_t OpenLoopRampSpeedFinal_Fract16;
    uint32_t OpenLoopRampSpeedTime_Cycles;      /* Time to reach OpenLoopSpeed. Ticks at CONTROL_FREQ */
    uint16_t OpenLoopRampIFinal_Fract16;
    uint32_t OpenLoopRampITime_Cycles;          /* Time to reach OpenLoopI */
    // uint16_t OpenLoopGain_VHz;
// #endif

    Motor_CommutationMode_T CommutationMode; /* optional for runtime selection */

#if defined(MOTOR_SIX_STEP_ENABLE)
    Phase_Polar_Mode_T PhasePwmMode;     /* Only 1 nvm param for phase module. */
#endif
}
Motor_Config_T;


/*
    Motor Run-time variable "state". Referred to as context to avoid naming collision with StateMachine State_T
    "Procedural composition over a passive aggregate." Cohesion of StateMachine NvmConfig
*/
typedef struct Motor_Context
{
    /*
        State and SubStates
    */
    StateMachine_Active_T StateMachine;     /* Compile time mapped address */
    uint32_t ControlTimerBase;              /* Control Freq ~ 20kHz, state counter. Overflow 20Khz: 59 hours */

    /* Effectively Substates StateMachine Controlled */
    Motor_Direction_T Direction;            /* Direction of applied/cmd V. now shadows FOC.VLimit */
    Motor_FeedbackMode_T FeedbackMode;      /* Active FeedbackMode, Control/Run SubState Flags */
    Motor_FaultFlags_T FaultFlags;          /* Fault SubState */

    /*
        Position Sensor
    */
    const RotorSensor_T * p_ActiveSensor;   /* Pointer to entry in SENSOR_TABLE */
    RotorSensor_State_T SensorState;        /* Compile time configured address. Sensor State includes [Angle_T] */

    /*
        Speed Feedback
    */
    Ramp_T SpeedRamp;                   /* { Target, Output, Limit, Coefficient } — full speed setpoint contract */
    PID_T PidSpeed;                     /* Input PidSpeed(RampCmd - Speed_Fract16), Output => VPwm, Vq, Iq. */
    // PID_T PidPosition;

    volatile Phase_Input_T PhaseInput;
    // Phase_Triplet_T VOut; /* output buffer */

    /*
        FOC
    */
    Ramp_T TorqueRamp;                      /* { Target, Output, Limit, Coefficient } — full torque setpoint contract */
    FOC_T Foc;                              /* d-q vectors AND inner-loop PIDs (Foc.PidIq, Foc.PidId) */
    // PID_T PidIPhase; /* Align, or use getter */
    // Ramp_T VRamp;    /* Optional VRamp */

    /*
        Active Limit inputs. Unsigned user frame. Ramp.Limits holds the materialized [Cw:Ccw] output.
        Value:  per-motor physical cap in PU, single user channel (user/OptDin/protocol overwrite). FRACT16_MAX => no cap.
        Derate: system derate as pushed by the upper layer arbitration. FRACT16_MAX => none.
            Held because Direction/FeedbackMode/Value changes re-resolve without the upper layer,
            and Ramp.Limits cannot be read back — [0:0] on Direction NULL, V limits on TorqueRamp in voltage mode.
    */
    struct { uint16_t Motoring; uint16_t Generating; ufract16_t Derate; } ILimit;
    struct { uint16_t Forward; uint16_t Reverse; ufract16_t Derate; } SpeedLimit;

    /* OpenLoop Preset, StartUp. No boundary checking */
    Ramp_T OpenLoopSpeedRamp;       /* Preset Speed Ramp */
    Ramp_T OpenLoopIRamp;           /* Preset I Ramp */
    // Ramp_T OpenLoopTorqueRamp;   /* Preset V/I Ramp */
    Angle_T OpenLoopAngle;
    Angle_SpeedUnitRef_T OpenLoopSpeedRef;

    /*  */
    HeatMonitor_State_T HeatMonitorState;

    /*
        Storable Config
    */
    Motor_Config_T Config;

    uint8_t CalibrationBuffer[MOTOR_CALIBRATION_BUFFER_SIZE]; /* Opaque buffer for one-shot calibration procedures. */


    /* Jog */
    // uint32_t JogIndex;

    /*
        Six-Step
    */
#if defined(MOTOR_SIX_STEP_ENABLE)
    /* MotorSixStep_T */
    BEMF_T Bemf;
    Phase_Id_T NextPhase;
    Phase_Id_T CommutationPhase;
    uint32_t CommutationTimeRef;
    uint32_t IBus_Fract16;
    uint16_t VPwm;     /* Six-Step Control Variable */
    uint16_t VBemfPeak_Adcu;
    uint16_t VBemfPeakTemp_Adcu;
    uint16_t IPhasePeak_Adcu;
    uint16_t IPhasePeakTemp_Adcu;
    uint32_t OpenLoopCommutationPeriod;
#endif

#if  defined(MOTOR_DEBUG_ENABLE) && !defined(NDEBUG)
    // volatile uint32_t MicrosRef;
    volatile uint32_t DebugCounter;
#endif
    volatile uint32_t DebugBuffer[16];
}
Motor_Context_T;


/*!
    @brief Motor Compile-time const configuration instance.
    program, meta, unrelated to end user config.
    Full Context for Thread and StateMachine.
*/
typedef const struct Motor
{
    Motor_Context_T * P_MOTOR;
    const VBus_T * P_VBUS; /* Read-only. static instance. */
    Phase_VOut_T PHASE;
    Phase_Analog_T PHASE_ANALOG;
    // RotorSensor_T SENSOR; /* Compile time default */
    RotorSensor_Table_T SENSOR_TABLE; /* Runtime selection. Init macros in Motor_Sensor.h */
    StateMachine_T STATE_MACHINE;
    TimerT_T CONTROL_TIMER;     /* State Timer. Map to ControlTimerBase */
    TimerT_T SPEED_TIMER;       /* Outer Speed Loop Timer. Millis */
    const Motor_Config_T * P_NVM_CONFIG;
    const FOC_Config_T * P_FOC_NVM_CONFIG; /* config for the FOC struct with a nested config field, without including a 3rd copy in Motor_Config */
    /*  */
    HeatMonitor_T HEAT_MONITOR;
    ADC_Conversion_T HEAT_MONITOR_CONVERSION;
    void * P_EXTENSION;
}
Motor_T;

// #define MOTOR_CONTROL_TIMER_INIT(p_MotorContext, MotorState) TIMER_T_ALLOC(

static inline Motor_Config_T * Motor_Config(Motor_T * p_motor)
{
#ifdef MOTOR_CONFIG_NVM_ONLY
    return p_motor->P_NVM_CONFIG;
#else
    return &p_motor->P_MOTOR->Config;
#endif
}

/******************************************************************************/
/*
    virtualized getters
*/
/******************************************************************************/
static inline Phase_VOut_T * Motor_PhaseVOut(Motor_T * p_motor) { return &p_motor->PHASE; }
/* handle single selection case */
static inline RotorSensor_T * Motor_RotorSensor(Motor_T * p_motor) { return p_motor->P_MOTOR->p_ActiveSensor; }

static inline const Angle_T * Motor_AngleSpeed(Motor_T * p_motor) { return &p_motor->P_MOTOR->SensorState.AngleSpeed; }

static inline Phase_VOutMode_T Motor_GetPhaseState(Motor_T * p_const) { return Phase_ReadVOut(&p_const->PHASE); }

/* getter for runtime configurable or compile time fixed */
static inline uint16_t Motor_SpeedTypeMax_Rpm(Motor_T * p_motor) { return _Motor_GetSpeedTypeMax_Rpm(&Motor_Config(p_motor)->SpeedRating); }
static inline uint16_t Motor_SpeedTypeMax_Rads(Motor_T * p_motor) { return _Motor_GetSpeedTypeMax_Rads(&Motor_Config(p_motor)->SpeedRating); }

/*
    Speed VBus Ref
    [SpeedRated] via Kv * VBusNominal
*/
static inline uint16_t Motor_GetSpeedVNominalRef_Rpm(Motor_T * p_motor) { return Motor_Config(p_motor)->SpeedRating.Kv * VBus_VSupplyNominal_V(&p_motor->P_VBUS->Config); }
static inline uint16_t Motor_GetSpeedVNominalRef_Angle(Motor_T * p_motor) { return _Motor_AngleOfRpm(&Motor_Config(p_motor)->SpeedRating, Motor_GetSpeedVNominalRef_Rpm(p_motor)); }

/*
    Uniform interface for parameter variations.
*/
static inline uint16_t Motor_SpeedRated_Rpm(Motor_T * p_motor) { return Motor_GetSpeedVNominalRef_Rpm(p_motor); }
/* Alternatively store to config with VBus value */
// static inline void Motor_ResolveSpeedRated(Motor_T * p_motor) { p_motor->P_MOTOR->Config.SpeedRating.SpeedRated_Rpm = Motor_GetSpeedVNominalRef_Rpm(p_motor); }

static inline uint16_t Motor_GetSpeedVNominalRef_Fract16(Motor_T * p_motor) { return VBus_VNominal_Fract16(&p_motor->P_VBUS->Config); } /* VBus handles sync V / VPu */
static inline uint16_t Motor_SpeedRated_Fract16(Motor_T * p_motor) { return Motor_GetSpeedVNominalRef_Fract16(p_motor); }

static inline interval_t Motor_GetVLimitsAntiPlugging(Motor_T * p_motor) { return interval_half_plane((sign_t)p_motor->P_MOTOR->Direction, VBus_GetVPhaseRefSvpwm(p_motor->P_VBUS)); }
static inline interval_t Motor_GetVLimitsSymmetric(Motor_T * p_motor) { return interval_symmetric(0, VBus_GetVPhaseRefSvpwm(p_motor->P_VBUS)); }

/******************************************************************************/
/*
    Resolve Limits - Materialize virtual fields for hot path access
*/
/*
    Trace:
    [Value, Derate, Config] => Motor_*Limit{Motoring,Generating,Forward,Reverse}() comparison
        => Motor_Get*Limits() direction resolve => Motor_Resolve*Limits() => *Ramp.Limits
        Ramp.Limits is enforced by saturation on every Ramp_ProcNext()
*/
/*
    Ramp -> Feedback State
    Ramp as the speed/torque setpoint contract. Hold entire commanded trajectory
    Resolve writes the canonical pair into the Ramp itself.
    Ramp.Target is user input
    Ramp holds Cached directional limits — single materialized layer for hot-path PID/Ramp clamps
*/
/******************************************************************************/
/*
    Inline local-derate compose — function IS the spec for which local sources exist.
*/
/* Effective derate — pushed system derate, local sources */
// return math_min(HeatMonitor_GetDerate_Fract16(&p_motor->HEAT_MONITOR), Motor_GetILimitStall(p_motor->P_MOTOR));
static inline ufract16_t Motor_GetIDerate(const Motor_Context_T * p_motor) { return  (p_motor->ILimit.Derate); }
static inline ufract16_t Motor_GetSpeedDerate(const Motor_Context_T * p_motor) { return  (p_motor->SpeedLimit.Derate); }

/*
    The comparison. Unsigned user frame, direction independent.
        limit = min(Value, Derate * Config)
    Derate scales the Config ceiling, Value caps absolutely. Neither is converted into the other.
*/
static inline ufract16_t Motor_ILimitMotoring(const Motor_Context_T * p_motor) { return math_min(p_motor->ILimit.Motoring, fract16_mul(Motor_GetIDerate(p_motor), p_motor->Config.ILimitMotoring_Fract16)); }
static inline ufract16_t Motor_ILimitGenerating(const Motor_Context_T * p_motor) { return math_min(p_motor->ILimit.Generating, fract16_mul(Motor_GetIDerate(p_motor), p_motor->Config.ILimitGenerating_Fract16)); }
static inline ufract16_t Motor_SpeedLimitForward(const Motor_Context_T * p_motor) { return math_min(p_motor->SpeedLimit.Forward, fract16_mul(Motor_GetSpeedDerate(p_motor), p_motor->Config.SpeedLimitForward_Fract16)); }
static inline ufract16_t Motor_SpeedLimitReverse(const Motor_Context_T * p_motor) { return math_min(p_motor->SpeedLimit.Reverse, fract16_mul(Motor_GetSpeedDerate(p_motor), p_motor->Config.SpeedLimitReverse_Fract16)); }

/*
    Direction resolve. I: Motoring aligns with Direction, NULL collapses to [0:0]. Speed: Forward aligns with Config.DirectionForward.
*/
static inline interval_t Motor_GetILimits(const Motor_Context_T * p_motor) { return interval_of_sign_pair((sign_t)p_motor->Direction, Motor_ILimitMotoring(p_motor), Motor_ILimitGenerating(p_motor)); }
static inline interval_t Motor_GetSpeedLimits(const Motor_Context_T * p_motor) { return interval_of_sign_pair((sign_t)p_motor->Config.DirectionForward, Motor_SpeedLimitForward(p_motor), Motor_SpeedLimitReverse(p_motor)); }

/*
    Materialize — the single write point into the hot path. Ccw/Cw signed interval is what Ramp/PID consume.
    TorqueRamp carries V limits in voltage mode; Motor_SetFeedbackMode resolves I on entry to current mode.
*/
static inline void _Motor_ApplyILimits(Motor_Context_T * p_motor, interval_t iLimits)
{
    if (p_motor->FeedbackMode.Current == 1)
    {
        Ramp_SetLimits(&p_motor->TorqueRamp, iLimits.low, iLimits.high);
        if (p_motor->FeedbackMode.Speed == 1) { PID_SetOutputLimits(&p_motor->PidSpeed, iLimits.low, iLimits.high); } /* SpeedPid Output is I */
    }
}

static inline void _Motor_ApplySpeedLimits(Motor_Context_T * p_motor, interval_t sLimits)
{
    Ramp_SetLimits(&p_motor->SpeedRamp, sLimits.low, sLimits.high);
}

/*
    Resolve = [Value, Derate, Config, Direction] → write Ccw/Cw → flush PID/Ramp.
    Idempotent; call after any input change. Cold path.
*/
static inline void Motor_ResolveILimits(Motor_Context_T * p_motor) { _Motor_ApplyILimits(p_motor, Motor_GetILimits(p_motor)); }
static inline void Motor_ResolveSpeedLimits(Motor_Context_T * p_motor) { _Motor_ApplySpeedLimits(p_motor, Motor_GetSpeedLimits(p_motor)); }


/******************************************************************************/
/*
    Resolved Ramp Limits
*/
/******************************************************************************/
static inline fract16_t Motor_ILimitCcw(const Motor_Context_T * p_motor) { return Ramp_GetLimitUpper(&p_motor->TorqueRamp); }
static inline fract16_t Motor_ILimitCw(const Motor_Context_T * p_motor) { return Ramp_GetLimitLower(&p_motor->TorqueRamp); }
static inline fract16_t Motor_SpeedLimitCcw(const Motor_Context_T * p_motor) { return Ramp_GetLimitUpper(&p_motor->SpeedRamp); }
static inline fract16_t Motor_SpeedLimitCw(const Motor_Context_T * p_motor) { return Ramp_GetLimitLower(&p_motor->SpeedRamp); }

/*
    Applied pair — the materialized [Cw:Ccw] as read back from the Ramp.
    interval_aligned/interval_opposed invert the direction resolve back to unsigned magnitudes:
        I:      aligned with Direction(Speed) => Motoring,          opposed => Generating
        Speed:  aligned with DirectionForward => Forward,    opposed => Reverse
*/
static inline interval_t Motor_RampILimits(const Motor_Context_T * p_motor) { return (interval_t) { .low = Motor_ILimitCw(p_motor), .high = Motor_ILimitCcw(p_motor) }; }
static inline interval_t Motor_RampSpeedLimits(const Motor_Context_T * p_motor) { return (interval_t) { .low = Motor_SpeedLimitCw(p_motor), .high = Motor_SpeedLimitCcw(p_motor) }; }


/******************************************************************************/
/*
    Outer Control Feedback State
*/
/******************************************************************************/
/* Result of Capture */
/* Feedback Speed interface getter */
/* move to Motor_T in case of compile time single sensor defined. alternatively, sensor state is always mapped into Motor_Context_T */
static inline accum32_t Motor_GetSpeedFeedback(const Motor_Context_T * p_motor) { return RotorSensor_GetSpeed_Fract16(p_motor->p_ActiveSensor); }
/* State transition checks */
static inline bool Motor_IsSpeedZero(const Motor_Context_T * p_motor) { return (Motor_GetSpeedFeedback(p_motor) == 0); }

static inline Motor_Direction_T Motor_GetDirectionFeedback(const Motor_Context_T * p_motor) { return (Motor_Direction_T)RotorSensor_GetFeedbackDirection(p_motor->p_ActiveSensor); }

/*!
    V [Phase] approximation via Speed
    ψ_pu ~ .5
*/
static inline int32_t _Motor_GetVSpeed_Fract16(Motor_T * p_motor) { return Motor_GetSpeedFeedback(p_motor->P_MOTOR) / 2; }
static inline int32_t Motor_GetVSpeed_Fract16(Motor_T * p_motor) { return fract16_mul(_Motor_GetVSpeed_Fract16(p_motor), p_motor->P_MOTOR->Config.SpeedRating.VSpeedAdjustment); }


static inline uint16_t Motor_GetSpeedFreewheelLimit_UFract16(Motor_T * p_motor) { return Motor_SpeedRated_Fract16(p_motor); }
static inline bool Motor_IsSpeedFreewheelLimitRange(Motor_T * p_motor) { return (math_abs(Motor_GetSpeedFeedback(p_motor->P_MOTOR)) < Motor_GetSpeedFreewheelLimit_UFract16(p_motor)); }

/*
    bus ratio IS the no-FW speed ceiling, VBus_GetSpeedDerate
*/
/* SpeedRated_pu = VNominal_pu */
static inline ufract16_t Motor_SpeedVLimit_Fract16(Motor_T * p_motor) { return VBus_Fract16(p_motor->P_VBUS); }
static inline bool Motor_IsSpeedVLimited(Motor_T * p_motor) { return (math_abs(Motor_GetSpeedFeedback(p_motor->P_MOTOR)) >= Motor_SpeedVLimit_Fract16(p_motor)); }


/******************************************************************************/
/*
    Speed Feedback Loop
    Ramp input ~100Hz,
    SpeedFeedback update 1000Hz - SpeedRamp, SpeedPid
    Cascade: SpeedPID output drives TorqueRamp target.
*/
/******************************************************************************/
/*
    Map an alternative input, while User/Drive still controls RampTarget
    alternatively StateMachine guards Ramp_SetTarget(&p_motor->SpeedRamp, speedReq)
*/
static inline fract16_t Motor_ProcSpeedControlOf(Motor_Context_T * p_motor, int16_t speedReq)
{
    // PID_CaptureOutputLimits(&p_motor->PidSpeed, Ramp_GetLimitLower(&p_motor->SpeedRamp), Ramp_GetLimitUpper(&p_motor->SpeedRamp));
    return PID_ProcPI(&p_motor->PidSpeed, Motor_GetSpeedFeedback(p_motor), Ramp_ProcNextOf(&p_motor->SpeedRamp, speedReq));
}

/*
    Stored-target form: PID(feedback, Ramp_ProcNext(SpeedRamp)). Reads SpeedRamp.Target.
*/
static inline fract16_t Motor_ProcSpeedControl(Motor_Context_T * p_motor)
{
    return PID_ProcPI(&p_motor->PidSpeed, Motor_GetSpeedFeedback(p_motor), Ramp_ProcNext(&p_motor->SpeedRamp));
}

/*
    Pid State on FeedbackMode/Resume
*/
static inline void Motor_MatchSpeedTorqueState(Motor_Context_T * p_motor, int16_t torqueState)
{
    Ramp_SetOutputState(&p_motor->SpeedRamp, Motor_GetSpeedFeedback(p_motor));
    PID_SetOutputState(&p_motor->PidSpeed, torqueState);
}


/******************************************************************************/
/*
    Direction - User/Unsigned Config

    [p_motor->Config.DirectionForward * p_motor->Direction]
    Physical rotor rotation — CW or CCW as viewed from shaft end
    User/application direction — "Forward"/"Reverse" is application-defined (a vehicle, a pump, a spindle)

    [p_motor->Direction]
    Electrical commutation direction — determines torque sign
    Feedback sign conventions — speed, current, voltage must be consistently signed
*/
/******************************************************************************/
/*
    One direction clamp
    Ccw: [0:value]
    Cw: [value:0]
*/
static inline int32_t _Motor_MotoringOnly(const Motor_Context_T * p_motor, int32_t value) { return (p_motor->Direction == (Motor_Direction_T)math_sign(value)) * value; }
static inline int32_t _Motor_GeneratingOnly(const Motor_Context_T * p_motor, int32_t value) { return (p_motor->Direction != (Motor_Direction_T)math_sign(value)) * value; }

static inline Motor_Direction_T Motor_GetDirectionForward(const Motor_Context_T * p_motor) { return p_motor->Config.DirectionForward; }
static inline Motor_Direction_T Motor_GetDirectionReverse(const Motor_Context_T * p_motor) { return (p_motor->Config.DirectionForward * -1); }
/* User reference Motoring. Interpret as CCW/CW or Positive */
/* Motoring Ccw / Forward */
static inline Motor_Direction_T Motor_GetUserDirection(const Motor_Context_T * p_motor) { return p_motor->Config.DirectionForward * p_motor->Direction; }
/* IsMotoringForward */
static inline bool Motor_IsDirectionForward(const Motor_Context_T * p_motor) { return (Motor_GetUserDirection(p_motor) == 1); } /* DirectionForward == Direction, excluding null case */
static inline bool Motor_IsDirectionReverse(const Motor_Context_T * p_motor) { return (Motor_GetUserDirection(p_motor) == -1); }
static inline bool Motor_IsDirectionStopped(const Motor_Context_T * p_motor) { return (p_motor->Direction == MOTOR_DIRECTION_NULL); }



/******************************************************************************/
/*
    runtime accessor
    decouple config implementation
    config applies policy on set
    minimal or no runtime processing
*/
/******************************************************************************/
/* OpenLoop UserCmd. maybe > IAlign */
static inline uint16_t _Motor_OpenLoopILimit(const Motor_Context_T * p_motor) { return fract16_mul(p_motor->Config.OpenLoopLimitRatio, p_motor->Config.ILimitMotoring_Fract16); }
static inline uint16_t _Motor_GetIAlign(const Motor_Context_T * p_motor) { return p_motor->Config.IAlign_Fract16; }

static inline uint16_t _Motor_GetVAlign(const Motor_Context_T * p_motor) { return p_motor->Config.VAlign_Fract16; }

/* VAlign = (2/3) * duty * VBus. scales Ratio to 1/2 VBus */
static inline uint16_t _Motor_GetVAlign_Duty(const Motor_Context_T * p_motor) { return (uint32_t)p_motor->Config.OpenLoopLimitRatio * 3 / 4; }

/* resolved against the live vbus state */
/* duty = (3/2) · VAlign / VBus  =  (3/4) · VAlign / (VBus/2) */
// static inline uint16_t _Motor_GetVAlign_Duty(const Motor_Context_T * p_motor, uint32_t vBusInv_accum32)
//  { return math_min(svpwm_norm_vbus_inv(vBusInv_accum32, _Motor_GetVAlign(p_motor)) * 3 / 2, FRACT16_MAX); }

/******************************************************************************/
/*!
    Extern — public Motor_T outer-handle API.
    Layer 3: composes via p_motor->P_MOTOR to reach component structs.
*/
/******************************************************************************/
// extern void Motor_InitFrom(Motor_T * p_dev, const Motor_Config_T * p_config);
extern void Motor_Init(Motor_T * p_dev);
extern void Motor_Reset(Motor_Context_T * p_motor);
extern void Motor_Reinit(Motor_T * p_motor);
extern void Motor_InitUnits(Motor_Context_T * p_motor);
extern bool Motor_IsConfigValid(Motor_T * p_motor);
extern void Motor_ValidateConfig(Motor_T * p_motor);

extern void Motor_SetFeedbackMode(Motor_T * p_motor, Motor_FeedbackMode_T mode);
extern void Motor_SetDirection(Motor_T * p_motor, Motor_Direction_T direction);

extern void Motor_ResetSpeedPid(Motor_Context_T * p_motor);
extern void Motor_ResetIPid(Motor_Context_T * p_motor);
extern void _Motor_ResetTuning(Motor_T * p_motor);
/* Maintain consistency between runtime and Nvm */
extern void _Motor_Tuning_SetSpeedKp(Motor_Context_T * p_state, uint32_t value);
extern void _Motor_Tuning_SetSpeedKi(Motor_Context_T * p_state, uint32_t value);
extern void _Motor_Tuning_SetIKp(Motor_Context_T * p_state, uint32_t value);
extern void _Motor_Tuning_SetIKi(Motor_Context_T * p_state, uint32_t value);

/*
    Active Limits — push interface. Unsigned magnitudes in, direction-resolved Ramp limits out.
    Set*Limit*: value channel, physical PU. Set*LimitDerate: system derate, ratio of Config.
*/
extern void Motor_SetILimits(Motor_Context_T * p_motor, uint16_t motoring_ufract16, uint16_t generating_ufract16);
extern void Motor_SetILimitMotoring(Motor_Context_T * p_motor, uint16_t motoring_ufract16);
extern void Motor_SetILimitGenerating(Motor_Context_T * p_motor, uint16_t generating_ufract16);
extern void Motor_SetILimit(Motor_Context_T * p_motor, uint16_t i_ufract16);
extern void Motor_SetILimitDerate(Motor_Context_T * p_motor, uint16_t scalar_ufract16);
extern void Motor_ResetILimit(Motor_Context_T * p_motor);

extern void Motor_SetSpeedLimits(Motor_Context_T * p_motor, uint16_t forward_ufract16, uint16_t reverse_ufract16);
extern void Motor_SetSpeedLimitForward(Motor_Context_T * p_motor, uint16_t forward_ufract16);
extern void Motor_SetSpeedLimitReverse(Motor_Context_T * p_motor, uint16_t reverse_ufract16);
extern void Motor_SetSpeedLimit(Motor_Context_T * p_motor, uint16_t speed_ufract16);
extern void Motor_SetSpeedLimitDerate(Motor_Context_T * p_motor, uint16_t scalar_ufract16);
extern void Motor_ResetSpeedLimit(Motor_Context_T * p_motor);
