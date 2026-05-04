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
#ifndef MOTOR_H
#define MOTOR_H

#include "Phase/Phase.h"
#include "Phase_Input/Phase_Input.h"
#include "Phase_Input/Phase_VBus.h"
#include "Phase_Input/Phase_Analog.h"
#include "Phase_Input/Phase_Calibration.h"
#include "VBus/VBus.h"
#include "VBus/VBus_Monitor.h"

#include "Sensor/RotorSensor_Table.h"
#include "Sensor/RotorSensor.h"

#include "Math/FOC.h"

#include "Peripheral/Analog/Analog.h"
#include "Peripheral/Analog/Linear_ADC.h"

#include "Transducer/Encoder/Encoder_ModeDT.h"
#include "Transducer/Encoder/Encoder_ISR.h"
#include "Transducer/Monitor/Heat/HeatMonitor.h"

#include "Framework/StateMachine/StateMachine.h"
#include "Framework/StateMachine/_StateMachine.h" /* Include the private header to contain StateMachine_Active_T within Motor_State_T */
#include "Framework/Timer/Timer.h"
#include "Framework/LimitArray/LimitArray.h"

#include "Math/Fixed/fixed.h"
#include "Math/Angle/Angle.h"
#include "Math/Linear/Linear.h"
#include "Math/Accumulator/Accumulator.h"
#include "Math/Ramp/Ramp.h"
#include "Math/PID/PID.h"

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

// #include "_Motor_Config.h"
/* Static Def */
#include "Motor_ControlFreq.h"

/*
    Parts - with dependency on Motor_T forward declared
    This way sub modules contain handling details
*/
// #include "Motor_Var.h"
// #include "Analog/Motor_Analog.h"
// #include "Sensor/Motor_Sensor.h"
// #include "Motor_Config.h"

#if     defined(MOTOR_OPEN_LOOP_ENABLE)
#elif   defined(MOTOR_OPEN_LOOP_DISABLE)
#else
#define MOTOR_OPEN_LOOP_ENABLE
#endif

#if     defined(MOTOR_UNIT_CONVERSION_LOCAL)
#if defined(MOTOR_SURFACE_SPEED_ENABLE)
#endif
#elif   defined(MOTOR_UNIT_CONVERSION_HOST)
#else
#define MOTOR_UNIT_CONVERSION_HOST
#endif

#if     defined(MOTOR_EXTERN_CONTROL_ENABLE)
#elif   defined(MOTOR_EXTERN_CONTROL_DISABLE)
#else
#define MOTOR_EXTERN_CONTROL_DISABLE
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


/*!
    @brief Motor Config - Runtime variable configuration, settings. Load from non volatile memory.
*/
typedef struct Motor_Config
{
    /*
        Calibration parameters
    */
    RotorSensor_Id_T SensorMode;
    Motor_Direction_T DirectionForward; /* CCW/CW Assigned positive direction */
    uint8_t PolePairs;                  /* Motor Pole Pairs. Use to derive Mech/Electrical speed calibration */
    uint16_t Kv;                        /* [RpmPerVolt] Motor Constant. Use to derive SpeedVRef. Optionally sets SpeedRated */
    uint16_t SpeedRated_Rpm;            /* [Rpm] for same units as kv. Speed at nominal VSource. Clamp or scale limits. Derives Angle and Fract16 */

    uint16_t VSpeedScalar_Fract16;      /* Additional adjustment for VBemf match. ensure resume control at lower speed. */
    // Motor_ResumeMode_T ResumeMode;   // option Scale to VSpeed or VBemf on resume

    Phase_Triplet_T IabcZeroRef_Adcu;

    /*
        Persistent Base Limits.
        Limits of Input Ramps
        User view as unsigned. Physical frame.
        Use same units as Req/Feedback
    */
    uint16_t SpeedLimitForward_Fract16;     /* [0:32767] = [0:100%] of SpeedTypeMax */
    uint16_t SpeedLimitReverse_Fract16;
    uint16_t ILimitMotoring_Fract16;        /* [0:32767] = [0:100%] of I_CALIB_AMPS. */
    uint16_t ILimitGenerating_Fract16;

    //todo,
    // uint32_t SpeedRampSlope_Accum32; /* Fract16 per tick << 15 */
    // uint32_t TorqueRampSlope_Accum32;

    /* Acceleration as time */
    uint32_t SpeedRampTime_Cycles;      /* Time to reach SpeedRated */
    uint32_t TorqueRampTime_Cycles;     /* Time to reach I_RATED */

    PID_Config_T PidSpeed;  /* Speed Control */
    PID_Config_T PidI;      /* Idq Control */

    /*
        OpenLoop
    */
    /* All OpenLoop Modes - UserCmd, Align */
    uint16_t OpenLoopLimitScalar_Fract16;   /* Limit of rated. as scalar [0:1.0F] [0:32768]. */

    /* Calibration and Jog Align */
    uint16_t AlignScalar_Fract16;           /* Applies V or I. as scalar [0:1.0F] [0:32768]. */
    uint32_t AlignTime_Cycles;              /* Ramp time and step duration */

    /* OpenLoop Run/StartUp Preset */
// #if defined(MOTOR_OPEN_LOOP_ENABLE)
    uint16_t OpenLoopRampSpeedFinal_Fract16;
    uint32_t OpenLoopRampSpeedTime_Cycles;      /* Time to reach OpenLoopSpeed */
    uint16_t OpenLoopRampIFinal_Fract16;
    uint32_t OpenLoopRampITime_Cycles;          /* Time to reach OpenLoopI */
    // uint16_t OpenLoopGain_VHz;
// #endif

//     /*
//         Identified electrical parameters. User-readable SI units.
//         Populated by CALIBRATION_STATE_ELECTRICAL (Motor_Calibration_Electrical.c).
//         Rs_Fract16 is the normalized form: Rs / R_REF where R_REF = V_MAX_VOLTS / I_MAX_AMPS.
//         Ld/Lq fract16 in-loop forms are KLd_Fract16 / KLq_Fract16 below (auto-derived at commit).
//     */
//     uint16_t  Rs_MilliOhms;     /* [mOhm] stator resistance per phase */
//     uint16_t  Ld_MicroHenries;  /* [uH]  d-axis inductance */
//     uint16_t  Lq_MicroHenries;  /* [uH]  q-axis inductance */
//     fract16_t Rs_Fract16;       /* Rs / (V_MAX_VOLTS / I_MAX_AMPS) as fract16 */

// #if defined(MOTOR_DECOUPLE_ENABLE)
//     /*
//         dq cross-coupling decoupling coefficients.
//         Applied as: omega_L = fract16_mul(ElectricalDelta_angle16, K_Fract16)
//         Tune K_ such that omega_L lands in the same fract16 voltage basis as the PI output.
//         Precomputed fract16 in-loop form of Ld / Lq / psi_f; recomputed at electrical-cal commit.
//     */
//     fract16_t KLd_Fract16;
//     fract16_t KLq_Fract16;
//     fract16_t KPsi_Fract16;
// #endif
    Motor_CommutationMode_T CommutationMode; /* optional for runtime selection */
#if defined(MOTOR_SIX_STEP_ENABLE)
    Phase_Polar_Mode_T PhasePwmMode;     /* Only 1 nvm param for phase module. */
#endif
#if defined(MOTOR_UNIT_CONVERSION_LOCAL) && defined(MOTOR_SURFACE_SPEED_ENABLE)
    uint16_t SurfaceDiameter;
    uint16_t GearRatioOutput;
    uint16_t GearRatioInput;
#endif
}
Motor_Config_T;

#include "_Motor_Config.h"

/*
    Motor State - Runtime variable state.
*/
/* Motor_Runtime_T */
typedef struct Motor_State
{
    /*
        State and SubStates
    */
    StateMachine_Active_T StateMachine;
    uint32_t ControlTimerBase;              /* Control Freq ~ 20kHz, state counter. Overflow 20Khz: 59 hours */

    /* Effectively Substates StateMachine Controlled */
    Motor_Direction_T Direction;            /* Direction of applied/cmd V. Calibration correction applied on get/set access. DirectionMotoring. Applies to: Motoring/Generating Cmd Active Limits, Sensor Interpolation */
    Motor_FeedbackMode_T FeedbackMode;      /* Active FeedbackMode, Control/Run SubState Flags */
    Motor_FaultFlags_T FaultFlags;          /* Fault SubState */
    uint8_t CalibrationStateIndex;

    /*
        Position Sensor
    */
    const RotorSensor_T * p_ActiveSensor;   /* Pointer to entry in SENSOR_TABLE */
    RotorSensor_State_T SensorState;        /* Compile time configured address. Sensor State includes [Angle_T] */

    const VBus_T * p_VBus; /* BackPointer for now */

    /*
        Ramp -> Feedback State
    */
    /*
        Speed Feedback
    */
    volatile bool SpeedUpdateFlag;      /* Speed capture sync Feedback update */
    Ramp_T SpeedRamp;                   /* { Target, Output, Limit, Coefficient } — full speed setpoint contract */
    PID_T PidSpeed;                     /* Input PidSpeed(RampCmd - Speed_Fract16), Output => VPwm, Vq, Iq. */
    // PID_T PidPosition;

    /*
        FOC
    */
    /*
        System limits derive intermediate Motoring/Generating
        Ramp holds Cached directional limits — single materialized layer for hot-path PID/Ramp clamps
    */
    Ramp_T TorqueRamp;                      /* { Target, Output, Limit, Coefficient } — full torque setpoint contract */
    volatile Phase_Input_T PhaseInput;
    FOC_T Foc;                              /* d-q vectors AND inner-loop PIDs (Foc.PidIq, Foc.PidId) */
    // PID_T PidIPhase;         /* Align, or use getter */
    Ramp_T VRamp; /* Optional VRamp */


    /* OpenLoop Preset, StartUp. No boundary checking */
    Ramp_T OpenLoopSpeedRamp;       /* Preset Speed Ramp */
    Ramp_T OpenLoopIRamp;           /* Preset I Ramp */
    // Ramp_T OpenLoopTorqueRamp;   /* Preset V/I Ramp */
    Angle_T OpenLoopAngle;
    Angle_SpeedFractRef_T OpenLoopSpeedRef;

    /*  */
    HeatMonitor_State_T HeatMonitorState;

    /*
        Storable Config
    */
    Motor_Config_T Config;

    Accumulator_T FilterA;           /* Calibration use */
    Accumulator_T FilterB;
    Accumulator_T FilterC;

    /*
        Electrical parameter identification scratch. Used by CALIBRATION_STATE_ELECTRICAL.
    */
    // struct Motor_ParamId
    // {
    //     uint8_t  Step;              /* Motor_ParamId_Step_T */
    //     uint32_t CycleCount;        /* cycles within current step */
    //     fract16_t IdBias;           /* aligned Id setpoint */
    //     /* accumulators */
    //     int64_t  VdAccum;
    //     int64_t  IdAccum;
    //     uint32_t AccumN;
    //     /* Ld step */
    //     fract16_t VdBias;
    //     fract16_t VdStep;
    //     int16_t   IdSteady;
    //     uint32_t  IdTauCycles;
    //     /* Lq HFI */
    //     angle16_t HfiPhase;
    //     angle16_t HfiDelta;
    //     int64_t   IqSumI;
    //     int64_t   IqSumQ;
    //     uint16_t  VhfAmpFract16;
    //     uint16_t  HfFreqHz;
    //     /* interim results */
    //     uint16_t  Rs_MilliOhms;
    //     uint16_t  Ld_MicroHenries;
    //     uint16_t  Lq_MicroHenries;
    //     fract16_t Rs_Fract16;
    // }
    // ParamId;

#if defined(MOTOR_LOCAL_UNIT_CONVERSION_ENABLE)
    /*
        Local Unit Conversion
    */
    // Linear_T UnitsIa;           /* Fract16 and Amps */
    // Linear_T UnitsIb;
    // Linear_T UnitsIc;
    // Linear_T UnitsVabc;         /* Vbemf/VPhase Fract16 conversion of adcu */
    // Linear_T UnitsVSpeed;       /* VbemfSpeed. Vbemf_Fract16 of Speed_Fract16. Resume voltage, calculated Vbemf */
#endif
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
}
Motor_State_T;


/*!
    @brief Motor Const Handle - Compile time const configuration instance.
    program, meta, parameters. unrelated to end user config.
    Context of Thread and StateMachine.
*/
/*
    const context dependency of StateMachine: PHASE, TIMER
    alternatively, reduce StateMachine context, handle with pointer from state
*/
typedef const struct Motor
{
    Motor_State_T * P_MOTOR;
    const VBus_T * P_VBUS; /* Read-only */
    Phase_T PHASE;
    Phase_Analog_T PHASE_ANALOG;
    RotorSensor_Table_T SENSOR_TABLE; /* Runtime selection. Init macros in Motor_Sensor.h */
    HeatMonitor_T HEAT_MONITOR;
    Analog_Conversion_T HEAT_MONITOR_CONVERSION;
    StateMachine_T STATE_MACHINE;
    TimerT_T CONTROL_TIMER;     /* State Timer. Map to ControlTimerBase */
    TimerT_T SPEED_TIMER;       /* Outer Speed Loop Timer. Millis */
    const Motor_Config_T * P_NVM_CONFIG;
    /*
        System-scope arbitration handles. Mediator pattern (cf. P_VBUS).
        Pointers to LimitArray_Augments_T (cached aggregate) only — Motor reads derate state.
    */
    const LimitArray_Augments_T * P_SYSTEM_I_LIMIT;
    const LimitArray_Augments_T * P_SYSTEM_SPEED_LIMIT;
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
    Types for generic accessor
*/
/******************************************************************************/
typedef int motor_value_t;
// typedef register_t motor_value_t;

typedef void(*Motor_Proc_T)(Motor_State_T * p_motor);

typedef motor_value_t(*Motor_Get_T)(const Motor_State_T * p_motor);
typedef void(*Motor_Set_T)(Motor_State_T * p_motor, motor_value_t value);

typedef bool(*Motor_State_Test_T)(const Motor_State_T * p_motor);
typedef bool(*Motor_State_TryProc_T)(Motor_State_T * p_motor);
typedef bool(*Motor_State_TrySet_T)(Motor_State_T * p_motor, motor_value_t value);
typedef bool(*Motor_State_TryValue_T)(const Motor_State_T * p_motor, motor_value_t value);


/******************************************************************************/
/*
    Motor Module
    Cohesion of StateMachine NvmConfig
    refactor sub contexts and  Motor_State_T * p_  should be absorbed by Motor_T API
*/
/******************************************************************************/

/******************************************************************************/
/*
    Number formats

    [Fract16]           [-1:1) <=> [-32768:32767] in Q1.15
    [UFract16]          [0:2) <=> [0:65535] in Q1.15
    [Accum32]           [-2:2] <=> [-65536:65536] in Q17.15     Max [INT32_MIN:INT32_MAX]
    [UQ16]              [0:1) <=> [0:65535] in Q0.16
    [Fixed16]           [-1:1] <=> [-256:256] in Q8.8           Max [-32768:32767]
    [Fixed32]           [-1:1] <=> [-65536:65536] in Q16.16     Max [INT32_MIN:INT32_MAX]
*/
/******************************************************************************/
// typedef fract16_t quantity_t;
// typedef fract16_t magnitude_t;


/******************************************************************************/
/*
    Resolve Limits - Materialize virtual fields for hot path access
*/
/******************************************************************************/
/*
    Inline local-derate compose — function IS the spec for which local sources exist.
*/
// return math_min(HeatMonitor_GetDerate_Fract16(&p_motor->HEAT_MONITOR), Motor_GetILimitStall(p_motor->P_MOTOR));
// static inline ufract16_t Motor_GetILocalDerate(Motor_T * p_motor) { return HeatMonitor_GetDerate_Fract16(&p_motor->HEAT_MONITOR); }
static inline ufract16_t Motor_GetILocalDerate(Motor_T * p_motor) { return FRACT16_MAX; } /* temporary placeholder */
/* No per-motor speed derate sources currently. Identity = no constraint. */
static inline ufract16_t Motor_GetSpeedLocalDerate(Motor_T * p_motor) { (void)p_motor; return FRACT16_MAX; }

/* with system derate */
/* Handle remaining comparison not handled by arbitration array */
static inline ufract16_t Motor_GetIDerate(Motor_T * p_motor) { return math_min(Motor_GetILocalDerate(p_motor), _LimitArray_Upper(p_motor->P_SYSTEM_I_LIMIT)); }
static inline ufract16_t Motor_GetSpeedDerate(Motor_T * p_motor) { return math_min(Motor_GetSpeedLocalDerate(p_motor), _LimitArray_Upper(p_motor->P_SYSTEM_SPEED_LIMIT)); }

/*
    Virtual fields, user config frames
    Use to derive directional Ramp/Feedback limits.
    State interface independent of direction
    no resync on direction change
    effective as sentinel comparison
*/
/*
    provide traceability from a parameter object
    Config.SpeedLimitForward_Fract16 is read by Motor_SpeedLimitForward(),
        which feeds Motor_GetSpeedLimits(),
        which feeds Motor_ResolveSpeedLimits(),
        which writes SpeedRamp.LimitUpper,
        which is enforced by Accumulator saturation on every Ramp_ProcNext()
*/
static inline ufract16_t Motor_ILimitMotoring(Motor_T * p_motor) { return fract16_mul(Motor_GetIDerate(p_motor), p_motor->P_MOTOR->Config.ILimitMotoring_Fract16); }
static inline ufract16_t Motor_ILimitGenerating(Motor_T * p_motor) { return fract16_mul(Motor_GetIDerate(p_motor), p_motor->P_MOTOR->Config.ILimitGenerating_Fract16); }
static inline ufract16_t Motor_SpeedLimitForward(Motor_T * p_motor) { return fract16_mul(Motor_GetSpeedDerate(p_motor), p_motor->P_MOTOR->Config.SpeedLimitForward_Fract16); }
static inline ufract16_t Motor_SpeedLimitReverse(Motor_T * p_motor) { return fract16_mul(Motor_GetSpeedDerate(p_motor), p_motor->P_MOTOR->Config.SpeedLimitReverse_Fract16); }
// static inline ufract16_t _Motor_ILimitAs(Motor_State_T * p_motor, Motor_Direction_T select) { return (p_motor->Direction == select) ? p_motor->ILimitMotoring_Fract16 : p_motor->ILimitGenerating_Fract16; }
// static inline ufract16_t _Motor_SpeedLimitAs(Motor_State_T * p_motor, Motor_Direction_T select) { return (p_motor->Config.DirectionForward == select) ? p_motor->SpeedLimitForward_Fract16 : p_motor->SpeedLimitReverse_Fract16; }


/*
    Single materialized layer: Ccw/Cw is what the hot path consumes.
*/
static inline interval_t Motor_GetILimits(Motor_T * p_motor) { return interval_of_sign_pair((sign_t)p_motor->P_MOTOR->Direction, Motor_ILimitMotoring(p_motor), Motor_ILimitGenerating(p_motor)); }
static inline interval_t Motor_GetSpeedLimits(Motor_T * p_motor) { return interval_of_sign_pair((sign_t)p_motor->P_MOTOR->Config.DirectionForward, Motor_SpeedLimitForward(p_motor), Motor_SpeedLimitReverse(p_motor)); }

/*
    Resolve = re-pull canonical (LimitArray + Config + Direction) → write Ccw/Cw → flush PID/Ramp.
    Idempotent under steady state; safe to call after any direction or arbitration change.
*/
static void Motor_ResolveILimits(Motor_T * p_motor)
{
    Motor_State_T * p_state = p_motor->P_MOTOR;
    interval_t iLimits = Motor_GetILimits(p_motor);
    Ramp_SetOutputLimit(&p_state->TorqueRamp, iLimits.low, iLimits.high);
    /* Optionally handle on pull instead. */
    if ((p_state->FeedbackMode.Speed == 1) && (p_state->FeedbackMode.Current == 1))  /* SpeedPid Output is I */
    {
        PID_SetOutputLimits(&p_state->PidSpeed, iLimits.low, iLimits.high);
    }
}

static void Motor_ResolveSpeedLimits(Motor_T * p_motor)
{
    interval_t sLimits = Motor_GetSpeedLimits(p_motor);
    Ramp_SetOutputLimit(&p_motor->P_MOTOR->SpeedRamp, sLimits.low, sLimits.high);
}

/* Getters read through to the Ramp — no shadow fields. */
static inline fract16_t Motor_ILimitCcw(const Motor_State_T * p_motor) { return Ramp_GetLimitUpper(&p_motor->TorqueRamp); }
static inline fract16_t Motor_ILimitCw(const Motor_State_T * p_motor) { return Ramp_GetLimitLower(&p_motor->TorqueRamp); }
static inline fract16_t Motor_SpeedLimitCcw(const Motor_State_T * p_motor) { return Ramp_GetLimitUpper(&p_motor->SpeedRamp); }
static inline fract16_t Motor_SpeedLimitCw(const Motor_State_T * p_motor) { return Ramp_GetLimitLower(&p_motor->SpeedRamp); }

// static inline ufract16_t Motor_SpeedLimitMotoring(const Motor_State_T * p_motor) { return _Motor_SpeedLimitAs(p_motor, p_motor->Direction); }
// static inline ufract16_t Motor_SpeedLimitGenerating(const Motor_State_T * p_motor) { return _Motor_SpeedLimitAs(p_motor, p_motor->Direction * -1); }
// static inline ufract16_t Motor_ILimitForward(const Motor_State_T * p_motor) { return _Motor_ILimitAs(p_motor, p_motor->Config.DirectionForward); }
// static inline ufract16_t Motor_ILimitReverse(const Motor_State_T * p_motor) { return _Motor_ILimitAs(p_motor, p_motor->Config.DirectionForward * -1); }
// static inline fract16_t Motor_SpeedLimitMotoringReq(const Motor_State_T * p_motor, int16_t speedReq) { return math_clamp(speedReq, (int32_t)0 - Motor_SpeedLimitGenerating(p_motor), Motor_SpeedLimitMotoring(p_motor)); }
// static inline fract16_t Motor_ILimitMotoringReq(const Motor_State_T * p_motor, int16_t iReq) { return math_clamp(iReq, (int32_t)0 - Motor_ILimitGenerating(p_motor), Motor_ILimitMotoring(p_motor)); }

/******************************************************************************/
/*
    Ramp
*/
/******************************************************************************/
/*
    Ramp as the speed/torque setpoint contract. Hold entire commanded trajectory
    Resolve writes the canonical pair into the Ramp itself.
    Ramp.Target is user input
*/
/* Internal limits as input limits, updated  */
static inline fract16_t Motor_IRampOf(Motor_State_T * p_motor, int16_t req) { return Ramp_ProcNextOnInputOf(&p_motor->TorqueRamp, req); }
static inline fract16_t Motor_SpeedRampOf(Motor_State_T * p_motor, int16_t speedReq) { return Ramp_ProcNextOnInputOf(&p_motor->SpeedRamp, speedReq); }

/* user input path */
// static inline void Motor_SetSpeedReq(Motor_State_T * p_motor, int16_t req) { Ramp_SetTarget(&p_motor->SpeedRamp, req); }
// static inline void Motor_SetTorqueReq(Motor_State_T * p_motor, int16_t req) { Ramp_SetTarget(&p_motor->TorqueRamp, req); }


/* I/V rate mismatch, okay if voltage mode is for testing only */
/* Reads stored target; anti-plug applied per tick without persisting on Target. */
static inline fract16_t Motor_VRamp(Motor_State_T * p_motor)
{
    interval_t v = VBus_AntiPluggingLimits(p_motor->p_VBus, (sign_t)p_motor->Direction);
    return _Ramp_ProcNextOnInputOf(&p_motor->TorqueRamp, math_clamp(Ramp_GetTarget(&p_motor->TorqueRamp), v.low, v.high));
    // return _Ramp_ProcNextOnInputOf(&p_motor->VRamp, math_clamp(Ramp_GetTarget(&p_motor->VRamp), v.low, v.high));
}

/*
    Openloop by user cmd
*/
static inline fract16_t Motor_OpenLoopTorqueRampOf(Motor_State_T * p_motor, int16_t req) { return Ramp_ProcNextOf(&p_motor->TorqueRamp, Motor_OpenLoopILimitOf(&p_motor->Config, req)); }

/******************************************************************************/
/*
    Outer Control Feedback State
*/
/******************************************************************************/
/*
    Capture State
*/
static inline void Motor_CaptureSensor(Motor_T * p_motor)
{
    Motor_State_T * p_state = p_motor->P_MOTOR;

    RotorSensor_CaptureAngle(p_state->p_ActiveSensor);
    if (TimerT_Periodic_Poll(&p_motor->SPEED_TIMER) == true)
    {
        RotorSensor_CaptureSpeed(p_state->p_ActiveSensor);
        p_state->SpeedUpdateFlag = true; /* Set flag to forward to statemachine proc. alternatively call StateMachine input directly */
    }
}

/* Result of Capture */
static inline const Angle_T * Motor_GetAngleSpeedState(Motor_T * p_motor) { return &p_motor->P_MOTOR->SensorState.AngleSpeed; }

/* Feedback Speed interface getter */
static inline accum32_t Motor_GetSpeedFeedback(const Motor_State_T * p_motor) { return RotorSensor_GetSpeed_Fract16(p_motor->p_ActiveSensor); }
static inline Motor_Direction_T Motor_GetDirectionFeedback(const Motor_State_T * p_motor) { return (Motor_Direction_T)RotorSensor_GetFeedbackDirection(p_motor->p_ActiveSensor); }

static inline bool Motor_IsSpeedZero(const Motor_State_T * p_motor) { return (Motor_GetSpeedFeedback(p_motor) == 0); }



/******************************************************************************/
/*
   Proc FeedbackMode Flags
   check onces per execution path
*/
/******************************************************************************/
/*
    Stateless cascade override: run SpeedPID against an alternate per-tick speedReq
    without touching SpeedRamp.Target. Used by intervention (drives PID toward an
    alternate setpoint while preserving the user's stored speed target for resume).
*/
static inline fract16_t Motor_SpeedControlOf(Motor_State_T * p_motor, int16_t speedReq)
{
    return PID_ProcPI(&p_motor->PidSpeed, Motor_GetSpeedFeedback(p_motor), Motor_SpeedRampOf(p_motor, speedReq));
}

/*
    Stored-target form: PID(feedback, Ramp_ProcNext(SpeedRamp)). Reads SpeedRamp.Target.
*/
static inline fract16_t Motor_ProcSpeedControl(Motor_State_T * p_motor)
{
    return PID_ProcPI(&p_motor->PidSpeed, Motor_GetSpeedFeedback(p_motor), Ramp_ProcNext(&p_motor->SpeedRamp));
}

/*
    Speed Feedback Loop
    Ramp input ~100Hz,
    SpeedFeedback update 1000Hz - SpeedRamp, SpeedPid
    Cascade: SpeedPID output drives TorqueRamp target.
*/
static inline void Motor_ProcSpeedFeedback(Motor_State_T * p_motor)
{
    if (p_motor->FeedbackMode.Speed == 1U) { Ramp_SetTarget(&p_motor->TorqueRamp, Motor_ProcSpeedControl(p_motor)); }
}

static inline void Motor_ProcOuterFeedback(Motor_State_T * p_motor)
{
    if (p_motor->SpeedUpdateFlag == true)
    {
        p_motor->SpeedUpdateFlag = false;
        Motor_ProcSpeedFeedback(p_motor);
    }
}

/*
    Pid State on FeedbackMode/Resume
*/
static inline void Motor_MatchSpeedTorqueState(Motor_State_T * p_motor, int16_t torqueState)
{
    if (p_motor->FeedbackMode.Speed == 1U)
    {
        Ramp_SetOutputState(&p_motor->SpeedRamp, Motor_GetSpeedFeedback(p_motor));
        PID_SetOutputState(&p_motor->PidSpeed, torqueState);
    }
}

/******************************************************************************/
/*
    From Config
*/
/******************************************************************************/
/*
   Calibration values derive with system config
*/
// static inline uint16_t Motor_GetSpeedFreewheelLimit_UFract16(const Motor_State_T * p_motor) { return fract16_mul(Motor_GetSpeedRated_Fract16(&p_motor->Config), FRACT16_2_DIV_SQRT3); }
static inline uint16_t Motor_GetSpeedFreewheelLimit_UFract16(const Motor_State_T * p_motor) { return Motor_GetSpeedRated_Fract16(&p_motor->Config); }
static inline bool Motor_IsSpeedFreewheelLimitRange(const Motor_State_T * p_motor) { return (math_abs(Motor_GetSpeedFeedback(p_motor)) < Motor_GetSpeedFreewheelLimit_UFract16(p_motor)); }


/*!
    V <=> Speed conversion based on Kv.
    Speed/SpeedRated => V/VBusRef
    @return V_Fract16 - VBusRef of Speed Kv
*/
static inline accum32_t Motor_VBusOfSpeed_Fract16(const Motor_State_T * p_motor, accum32_t speed_fract16) { return Phase_VBus_GetVNominal() * speed_fract16 / Motor_GetSpeedRated_Fract16(&p_motor->Config); }
static inline accum32_t Motor_SpeedOfVBus_Fract16(const Motor_State_T * p_motor, accum32_t v_fract16) { return v_fract16 * Motor_GetSpeedRated_Fract16(&p_motor->Config) / Phase_VBus_GetVNominal(); }

/* Phase peak. */
static inline accum32_t Motor_VPhaseOfSpeed_Fract16(const Motor_State_T * p_motor, accum32_t speed_fract16) { return Motor_VBusOfSpeed_Fract16(p_motor, speed_fract16) / 2; }
/* when SPEED_MAX = Kv * VNominal * 2 */
// static inline int32_t _Motor_GetVSpeed_Fract16(const Motor_State_T * p_motor) { return fract16_mul(Phase_VBus_GetVNominal(), Motor_GetSpeedFeedback(p_motor)); }

/*!
    VPhase approximation via Speed
*/
static inline int32_t _Motor_GetVSpeed_Fract16(const Motor_State_T * p_motor) { return Motor_VPhaseOfSpeed_Fract16(p_motor, Motor_GetSpeedFeedback(p_motor)); }

static inline int32_t Motor_GetVSpeed_Fract16(const Motor_State_T * p_motor) { return fract16_mul(_Motor_GetVSpeed_Fract16(p_motor), p_motor->Config.VSpeedScalar_Fract16); }



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
static inline int32_t _Motor_MotoringOnly(const Motor_State_T * p_motor, int32_t value) { return (p_motor->Direction == math_sign(value)) * value; }
static inline int32_t _Motor_GeneratingOnly(const Motor_State_T * p_motor, int32_t value) { return (p_motor->Direction != math_sign(value)) * value; }

static inline Motor_Direction_T Motor_GetDirectionForward(const Motor_State_T * p_motor) { return p_motor->Config.DirectionForward; }
static inline Motor_Direction_T Motor_GetDirectionReverse(const Motor_State_T * p_motor) { return (p_motor->Config.DirectionForward * -1); }
/* User reference Motoring. Interpret as CCW/CW or Positive */
/* Motoring Ccw / Forward */
static inline Motor_Direction_T Motor_GetUserDirection(const Motor_State_T * p_motor) { return p_motor->Config.DirectionForward * p_motor->Direction; }
/* IsMotoringForward */
static inline bool Motor_IsDirectionForward(const Motor_State_T * p_motor) { return (Motor_GetUserDirection(p_motor) == 1); } /* DirectionForward == Direction, excluding null case */
static inline bool Motor_IsDirectionReverse(const Motor_State_T * p_motor) { return (Motor_GetUserDirection(p_motor) == -1); }
static inline bool Motor_IsDirectionStopped(const Motor_State_T * p_motor) { return (p_motor->Direction == MOTOR_DIRECTION_NULL); }




/******************************************************************************/
/*!
    Extern
*/
/******************************************************************************/
// extern void Motor_InitFrom(const Motor_T * p_dev, const Motor_Config_T * p_config);
extern void Motor_Init(const Motor_T * p_dev);
extern void Motor_Reset(Motor_State_T * p_motor);

extern void Motor_Reinit(Motor_T * p_motor);

extern void Motor_ReinitSensor(Motor_State_T * p_motor);
extern void Motor_InitUnits(Motor_State_T * p_motor);

extern void Motor_InitSpeedRamp(Motor_State_T * p_motor);
extern void Motor_InitTorqueRamp(Motor_State_T * p_motor);
extern void Motor_ResetSpeedPid(Motor_State_T * p_motor);
extern void Motor_ResetIPid(Motor_State_T * p_motor);
void _Motor_ResetTuning(Motor_T * p_motor);

extern void Motor_SetFeedbackMode(Motor_State_T * p_motor, Motor_FeedbackMode_T mode);
extern void Motor_SetDirection(Motor_T * p_motor, Motor_Direction_T direction);

extern void Motor_ResetSpeedLimit(Motor_State_T * p_motor);
extern void Motor_ResetILimit(Motor_State_T * p_motor);
extern void Motor_SetSpeedLimits(Motor_State_T * p_motor, uint16_t speed_ufract16);
extern void Motor_SetILimits(Motor_State_T * p_motor, uint16_t i_fract16);

extern void Motor_SetSpeedLimitForward(Motor_State_T * p_motor, uint16_t speed_ufract16);
extern void Motor_SetSpeedLimitReverse(Motor_State_T * p_motor, uint16_t speed_ufract16);
extern void Motor_SetSpeedLimit_Scalar(Motor_State_T * p_motor, uint16_t scalar_ufract16);
extern void Motor_SetILimitMotoring(Motor_State_T * p_motor, uint16_t i_ufract16);
extern void Motor_SetILimitGenerating(Motor_State_T * p_motor, uint16_t i_ufract16);
extern void Motor_SetILimit_Scalar(Motor_State_T * p_motor, uint16_t scalar_ufract16);

#endif


/******************************************************************************/
/*!
    Angle Speed Wrap
*/
/******************************************************************************/
// static inline void Motor_SetElAngleFeedforward(Motor_State_T * p_motor, angle16_t angle)
// {
//     _Motor_SetFeedforwardDelta(p_motor, angle - p_motor->SensorState.ElectricalAngle);
// }

// static inline void Motor_SetElSpeedFeedforward(Motor_State_T * p_motor, angle16_t elSpeed_degPerCycle)
// {
//     _Motor_SetFeedforwardDelta(p_motor, elSpeed_degPerCycle);
// }

// static inline void Motor_SetMechAngleFeedforward(Motor_State_T * p_motor, angle16_t angle)
// {
//     Motor_SetElAngleFeedforward(p_motor, angle * p_motor->Config.PolePairs);
// }


