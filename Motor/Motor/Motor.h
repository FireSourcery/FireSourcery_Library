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

#include "Phase/Phase_VOut.h"
#include "Phase_Input/Phase_Input.h"
#include "Phase_Input/Phase_VBus.h"
#include "Phase_Input/Phase_Analog.h"
#include "Phase_Input/Phase_Calibration.h"
#include "VBus/VBus.h"
#include "VBus/VBus_Monitor.h"

#include "Sensor/RotorSensor_Table.h"
#include "Sensor/RotorSensor.h"

#include "Math/FOC.h"
// #ifdef
#include "Math/FOC_Sensorless.h"

#include "Peripheral/Analog/Analog.h"
#include "Peripheral/Analog/Linear_ADC.h"

#include "Transducer/Encoder/Encoder_ModeDT.h"
#include "Transducer/Encoder/Encoder_ISR.h"
#include "Transducer/Monitor/Heat/HeatMonitor.h"

#include "Framework/StateMachine/StateMachine.h"
#include "Framework/StateMachine/_StateMachine.h" /* Include the private header to contain StateMachine_Active_T within Motor_Context_T */
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
#include "Types/Motor_Electrical.h"

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

/* Caller define to largest used */
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
#ifndef MOTOR_FLOATING_POINT
// typedef fract16_t quantity_t;
// typedef fract16_t magnitude_t;
typedef fract16_t scalar_t;
#else
typedef float scalar_t;
#endif



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

    Motor_ElectricalSpeedRating_T SpeedRating;    /* PolePairs, Kv, SpeedRated_Rpm */
    // Motor_RL_T ElectricalRsLs;

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
    uint16_t ILimitMotoring_Fract16;        /* [0:32767] = [0:100%] of I_TYPE_MAX_AMPS. */
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
    uint16_t OpenLoopLimitScalar_Fract16;   /* Limit of rated. as scalar [0:1.0F] [0:32768]. V/I Align_Fract16 < OpenLoopLimitScalar_Fract16 * V/I RATED */

    /* Calibration and Jog Align */
    uint16_t IAlign_Fract16;                 /* OpenLoop/Calibration Align Current, as fract16 of I_TYPE_MAX_AMPS. */
    uint16_t VAlign_Fract16;                 /* OpenLoop/Calibration Align Voltage, as fract16 of V_TYPE_MAX_VOLTS. */
    uint32_t AlignTime_Cycles;               /* Ramp time and step duration */

    /* OpenLoop Run/StartUp Preset */
// #if defined(MOTOR_OPEN_LOOP_ENABLE)
    uint16_t OpenLoopRampSpeedFinal_Fract16;
    uint32_t OpenLoopRampSpeedTime_Cycles;      /* Time to reach OpenLoopSpeed */
    uint16_t OpenLoopRampIFinal_Fract16;
    uint32_t OpenLoopRampITime_Cycles;          /* Time to reach OpenLoopI */
    // uint16_t OpenLoopGain_VHz;
// #endif

    FOC_Electrical_T ElectricalParams_Si;  /* optional Motor Electrical Parameters. Si units */
    FOC_Electrical_T ElectricalParams_Pu;  /* VdqDecoupling */
    // FOC_Electrical_T ElectricalParams_Si_Test;
    // FOC_Electrical_T ElectricalParams_Pu_Test;

    bool IsFieldWeakeningEnabled; /* Optional Field Weakening Enable, otherwise handled with limits. enfoce id = 0 when disabled. */
    FOC_FieldWeakeningConfig_T FieldWeakening; /* Field Weakening Parameters. Tune for max speed or voltage match. */
    // FOC_Config_T FocConfig;              /* FOC control parameters, limits. */
    FOC_SensorlessConfig_T SensorlessConfig; /* Sensorless Observer parameters. */

    Motor_CommutationMode_T CommutationMode; /* optional for runtime selection */

#if defined(MOTOR_SIX_STEP_ENABLE)
    Phase_Polar_Mode_T PhasePwmMode;     /* Only 1 nvm param for phase module. */
#endif
}
Motor_Config_T;


/*
    Motor Runtime variable "state". Refered to as context for avoid naming collision with StateMachine State_T
    "Procedural composition over a passive aggregate." Cohesion of StateMachine NvmConfig
*/
typedef struct Motor_Context
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

    // Motor_ElectricalSpeedRef_T ElectricalSpeedRef; /* Derived from SpeedRated and Ke. Cached for runtime access. */

    /*
        Position Sensor
    */
    const RotorSensor_T * p_ActiveSensor;   /* Pointer to entry in SENSOR_TABLE */
    RotorSensor_State_T SensorState;        /* Compile time configured address. Sensor State includes [Angle_T] */
    FOC_Sensorless_T FocSensorless;         /* Sensorless observer state. Single instance shared by Sensorless_Sensor adapter (when active) and debug Var readers. */

    /*
        Ramp -> Feedback State
        Ramp as the speed/torque setpoint contract. Hold entire commanded trajectory
        Resolve writes the canonical pair into the Ramp itself.
        Ramp.Target is user input
    */
    /*
        Speed Feedback
    */
    // volatile bool SpeedUpdateFlag;      /* Speed capture sync Feedback update */
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

    // Phase_Triplet_T VOut; /* output buffer */

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

    uint8_t CalibrationBuffer[MOTOR_CALIBRATION_BUFFER_SIZE]; /* Opaque buffer for calibration procedures. */

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
    volatile uint32_t DebugBuffer[16];
}
Motor_Context_T;


/*!
    @brief Motor Const Handle - Compile time const configuration instance.
    program, meta, unrelated to end user config.
    Context for Thread and StateMachine.
*/
/*
    const context dependency of StateMachine: PHASE, TIMER
    alternatively, reduce StateMachine context to mutable only, handle with pointer from state
*/
typedef const struct Motor
{
    Motor_Context_T * P_MOTOR;
    const VBus_T * P_VBUS; /* Read-only */
    Phase_VOut_T PHASE;
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
    virtualized getters
*/
/******************************************************************************/
static inline Phase_VOut_T * Motor_PhaseVOut(Motor_T * p_motor) { return &p_motor->PHASE; }
static inline RotorSensor_T * Motor_RotorSensor(Motor_T * p_motor) { return p_motor->P_MOTOR->p_ActiveSensor; }
static inline const Angle_T * Motor_AngleSpeed(Motor_T * p_motor) { return &p_motor->P_MOTOR->SensorState.AngleSpeed; }

/* Motor_GetKvVBus_Rpm */
static inline uint16_t Motor_GetSpeedVNominalRef_Rpm(Motor_T * p_motor) { return Motor_Config(p_motor)->SpeedRating.Kv * VBus_VSupplyNominal_V(&p_motor->P_VBUS->Config); }
static inline uint16_t Motor_GetSpeedVNominalRef_Angle(Motor_T * p_motor) { return _Motor_AngleOfRpm(&Motor_Config(p_motor)->SpeedRating, Motor_GetSpeedVNominalRef_Rpm(p_motor)); }
static inline uint16_t Motor_GetSpeedVNominalRef_Fract16(Motor_T * p_motor) { return Motor_Speed_Fract16OfRpm(&Motor_Config(p_motor)->SpeedRating, Motor_GetSpeedVNominalRef_Rpm(p_motor)); }


/*
    when SpeedRated is derived from V_Nominal, and SpeedTypeMax is derived from 2x SpeedRated
    SpeedTypeMax = SpeedRated * 2 = Kv * V_Nominal * 2
    ψ_pu = Kv * V_Nominal * 2 / (Kv * 2 * V_max) = V_Nominal / V_max  = V_Nominal_pu

    fract16_mul(psi, Speed_fract16) == fract16_mul(VBus_VNominal_Fract16, Speed_fract16)
*/
static inline uint16_t Motor_SpeedTypeMax_Rpm(Motor_T * p_motor) { return VBus_VSupplyNominal_V(&p_motor->P_VBUS->Config) * Motor_Config(p_motor)->SpeedRating.Kv * 2; }
static inline uint16_t Motor_SpeedRated_Rpm(Motor_T * p_motor) { return VBus_VSupplyNominal_V(&p_motor->P_VBUS->Config) * Motor_Config(p_motor)->SpeedRating.Kv; }
// static inline accum32_t Motor_Psi_Fract16(Motor_T * p_motor) { return VBus_VNominal_Fract16(&p_motor->P_VBUS->Config); }
// static inline accum32_t Motor_Ke_Fract16(Motor_T * p_motor) { return VBus_VNominal_Fract16(&p_motor->P_VBUS->Config) * 2; }

/*
    Uniform interface for parameter variations.
*/
// static inline uint16_t _Motor_GetSpeedTypeMax_Rpm(const Motor_ElectricalSpeedRating_T * p_config) { return p_config->SpeedRated_Rpm * 2; }
// static inline uint16_t _Motor_GetSpeedRated_Rpm(const Motor_ElectricalSpeedRating_T * p_config) { return p_config->SpeedRated_Rpm; }
static inline accum32_t Motor_Psi_Fract16(Motor_T * p_motor) { return p_motor->P_MOTOR->Config.ElectricalParams_Pu.Psi; }

/* alternatively inverter max as 1.0, high end limited field weakening */
// static inline uint16_t Motor_SpeedTypeMax_Rpm(Motor_T * p_motor) { return Phase_Calibration_GetVMaxVolts() * Motor_Config(p_motor)->SpeedRating.Kv; }
// static inline accum32_t Motor_Ke_Fract16(Motor_T * p_motor) { return ke_pu_rpm_of_kv(Phase_Calibration_GetVMaxVolts(), Motor_SpeedTypeMax_Rpm(p_motor), Motor_Config(p_motor)->SpeedRating.Kv); }

/*
    Speed VBus Ref
    Generally [SpeedRated] via Kv * V
*/
static inline void Motor_ResolveSpeedRated(Motor_T * p_motor) { p_motor->P_MOTOR->Config.SpeedRating.SpeedRated_Rpm = Motor_GetSpeedVNominalRef_Rpm(p_motor); }



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

/*
    Single source of Derate owned by system. System is the writer. No additional cached state in Motor_Context_T.
    Handle remaining comparison not handled by system arbitration array
*/
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
    Config * Derate => virtualized Field,
        => Motor_Get*Limits() => Motor_Resolve*Limits() => *Ramp.Limits
        Ramp.Limits is enforced by saturation on every Ramp_ProcNext()
*/
static inline ufract16_t Motor_ILimitMotoring(Motor_T * p_motor) { return fract16_mul(Motor_GetIDerate(p_motor), p_motor->P_MOTOR->Config.ILimitMotoring_Fract16); }
static inline ufract16_t Motor_ILimitGenerating(Motor_T * p_motor) { return fract16_mul(Motor_GetIDerate(p_motor), p_motor->P_MOTOR->Config.ILimitGenerating_Fract16); }
static inline ufract16_t Motor_SpeedLimitForward(Motor_T * p_motor) { return fract16_mul(Motor_GetSpeedDerate(p_motor), p_motor->P_MOTOR->Config.SpeedLimitForward_Fract16); }
static inline ufract16_t Motor_SpeedLimitReverse(Motor_T * p_motor) { return fract16_mul(Motor_GetSpeedDerate(p_motor), p_motor->P_MOTOR->Config.SpeedLimitReverse_Fract16); }

static inline interval_t Motor_ILimitsAs(Motor_T * p_motor, Motor_Direction_T motoring) { return interval_of_sign_pair((sign_t)motoring, Motor_ILimitMotoring(p_motor), Motor_ILimitGenerating(p_motor)); }
static inline interval_t Motor_SpeedLimitsAs(Motor_T * p_motor, Motor_Direction_T forward) { return interval_of_sign_pair((sign_t)forward, Motor_SpeedLimitForward(p_motor), Motor_SpeedLimitReverse(p_motor)); }

/*
    Single materialized layer: Ccw/Cw is what the hot path consumes.
*/
static inline interval_t Motor_GetILimits(Motor_T * p_motor) { return interval_of_sign_pair((sign_t)p_motor->P_MOTOR->Direction, Motor_ILimitMotoring(p_motor), Motor_ILimitGenerating(p_motor)); }
static inline interval_t Motor_GetSpeedLimits(Motor_T * p_motor) { return interval_of_sign_pair((sign_t)p_motor->P_MOTOR->Config.DirectionForward, Motor_SpeedLimitForward(p_motor), Motor_SpeedLimitReverse(p_motor)); }
// static inline interval_t Motor_GetVLimits(Motor_T * p_motor) { return interval_of_half_plane((sign_t)p_motor->P_MOTOR->Direction, Motor_SpeedLimitForward(p_motor), Motor_SpeedLimitReverse(p_motor)); }

/*
    Resolve = re-pull canonical (LimitArray + Config + Direction) → write Ccw/Cw → flush PID/Ramp.
    Idempotent under steady state; safe to call after any direction or arbitration change.
*/
static void Motor_ResolveILimits(Motor_T * p_motor)
{
    Motor_Context_T * p_state = p_motor->P_MOTOR;
    interval_t iLimits = Motor_GetILimits(p_motor);
    Ramp_SetLimits(&p_state->TorqueRamp, iLimits.low, iLimits.high);

    /* Optionally handle on pull instead. */
    if ((p_state->FeedbackMode.Speed == 1) && (p_state->FeedbackMode.Current == 1))  /* SpeedPid Output is I */
    {
        PID_SetOutputLimits(&p_state->PidSpeed, iLimits.low, iLimits.high);
    }
}

static void Motor_ResolveSpeedLimits(Motor_T * p_motor)
{
    interval_t sLimits = Motor_GetSpeedLimits(p_motor);
    Ramp_SetLimits(&p_motor->P_MOTOR->SpeedRamp, sLimits.low, sLimits.high);
}

/*
    Keep forwards as virtual getters, in case implementation changes.
*/
/* Getters read through to the Ramp — no shadow fields. */
static inline fract16_t Motor_ILimitCcw(const Motor_Context_T * p_motor) { return Ramp_GetLimitUpper(&p_motor->TorqueRamp); }
static inline fract16_t Motor_ILimitCw(const Motor_Context_T * p_motor) { return Ramp_GetLimitLower(&p_motor->TorqueRamp); }
static inline fract16_t Motor_SpeedLimitCcw(const Motor_Context_T * p_motor) { return Ramp_GetLimitUpper(&p_motor->SpeedRamp); }
static inline fract16_t Motor_SpeedLimitCw(const Motor_Context_T * p_motor) { return Ramp_GetLimitLower(&p_motor->SpeedRamp); }

// static inline ufract16_t Motor_SpeedLimitMotoring(const Motor_Context_T * p_motor) { return _Motor_SpeedLimitAs(p_motor, p_motor->Direction); }
// static inline ufract16_t Motor_SpeedLimitGenerating(const Motor_Context_T * p_motor) { return _Motor_SpeedLimitAs(p_motor, p_motor->Direction * -1); }
// static inline ufract16_t Motor_ILimitForward(const Motor_Context_T * p_motor) { return _Motor_ILimitAs(p_motor, p_motor->Config.DirectionForward); }
// static inline ufract16_t Motor_ILimitReverse(const Motor_Context_T * p_motor) { return _Motor_ILimitAs(p_motor, p_motor->Config.DirectionForward * -1); }
// static inline fract16_t Motor_SpeedLimitMotoringReq(const Motor_Context_T * p_motor, int16_t speedReq) { return math_clamp(speedReq, (int32_t)0 - Motor_SpeedLimitGenerating(p_motor), Motor_SpeedLimitMotoring(p_motor)); }
// static inline fract16_t Motor_ILimitMotoringReq(const Motor_Context_T * p_motor, int16_t iReq) { return math_clamp(iReq, (int32_t)0 - Motor_ILimitGenerating(p_motor), Motor_ILimitMotoring(p_motor)); }


/******************************************************************************/
/*
    Outer Control Feedback State
*/
/******************************************************************************/
/* Result of Capture */
/* Feedback Speed interface getter */
/* move to Motor_T in case of compile time single sensor defined. atlernatively, sensor state is always mapped into Motor_Context_T */
static inline accum32_t Motor_GetSpeedFeedback(const Motor_Context_T * p_motor) { return RotorSensor_GetSpeed_Fract16(p_motor->p_ActiveSensor); }
static inline Motor_Direction_T Motor_GetDirectionFeedback(const Motor_Context_T * p_motor) { return (Motor_Direction_T)RotorSensor_GetFeedbackDirection(p_motor->p_ActiveSensor); }

/*
    Decoupling-basis ω input for FOC_CaptureSpeed.
        rpm/Q15      ω_pu (fract16, 32768 = ω_base)
        angle16 dir  el_delta_angle16 (raw step, implicit Q15 of ω_max = π·Fs)
    Pairs with the L_pu / ψ_pu encoding selected by MOTOR_PU_BASIS_ANGLE16.
*/
#if defined(MOTOR_PU_BASIS_ANGLE16)
static inline accum32_t Motor_GetDecouplingOmega(const Motor_Context_T * p_motor) { return RotorSensor_GetElectricalDelta(p_motor->p_ActiveSensor); }
#else
static inline accum32_t Motor_GetDecouplingOmega(const Motor_Context_T * p_motor) { return RotorSensor_GetSpeed_Fract16(p_motor->p_ActiveSensor); }
#endif

static inline bool Motor_IsSpeedZero(const Motor_Context_T * p_motor) { return (Motor_GetSpeedFeedback(p_motor) == 0); }

static inline uint16_t Motor_GetSpeedFreewheelLimit_UFract16(const Motor_Config_T * p_config) { return Motor_GetSpeedRated_Fract16(&p_config->SpeedRating); }
static inline bool Motor_IsSpeedFreewheelLimitRange(const Motor_Context_T * p_motor) { return (math_abs(Motor_GetSpeedFeedback(p_motor)) < Motor_GetSpeedFreewheelLimit_UFract16(&p_motor->Config)); }

/*!
    V [Phase] approximation via Speed
    1/2 VBus Nominal at SpeedRated
*/
/* when SPEED_MAX = SpeedRated * 2 = Kv * VNominal * 2 */
static inline int32_t _Motor_GetVSpeed_Fract16(Motor_T * p_motor) { return fract16_mul(Motor_Psi_Fract16(p_motor), Motor_GetSpeedFeedback(p_motor->P_MOTOR)); }
static inline int32_t Motor_GetVSpeed_Fract16(Motor_T * p_motor) { return fract16_mul(_Motor_GetVSpeed_Fract16(p_motor), p_motor->P_MOTOR->Config.SpeedRating.VSpeedScalar_Fract16); }


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
static inline int32_t _Motor_MotoringOnly(const Motor_Context_T * p_motor, int32_t value) { return (p_motor->Direction == math_sign(value)) * value; }
static inline int32_t _Motor_GeneratingOnly(const Motor_Context_T * p_motor, int32_t value) { return (p_motor->Direction != math_sign(value)) * value; }

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
    config applies policy on set
    minimal or no runtime processing
*/
/******************************************************************************/
/* OpenLoopICmd */
static inline uint16_t Motor_OpenLoopILimit(const Motor_Config_T * p_motor) { return fract16_mul(p_motor->OpenLoopLimitScalar_Fract16, p_motor->ILimitMotoring_Fract16); }
/* alternatively seperate IAlign and Vduty */
static inline uint16_t Motor_GetIAlign(const Motor_Config_T * p_motor) { return p_motor->IAlign_Fract16; }
/* resolves against the live vbus state */
static inline uint16_t Motor_GetVAlign(const Motor_Config_T * p_motor) { return p_motor->VAlign_Fract16; }
/* fract16_div((uint32_t)Motor_GetVAlign(p_motor) * 2 / 3, Phase_VBus_GetVRef()) */
/* VAlign_Fract16 * VBus_Inv_Fract32  * 3 / 4; */
static inline uint16_t Motor_GetVAlign_Duty(const Motor_Config_T * p_motor) { return (uint32_t)p_motor->OpenLoopLimitScalar_Fract16 * 3 / 4; }


/* Virtual getters. if implementation expected to change*/
// static inline uint16_t Motor_IAlign(Motor_T * p_motor) { return p_motor->P_MOTOR->Config.IAlign_Fract16; }
// static inline uint16_t Motor_VAlign(Motor_T * p_motor) { return p_motor->P_MOTOR->Config.VAlign_Fract16; }

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

extern void Motor_ReinitSensor(Motor_Context_T * p_motor);
extern void Motor_InitUnits(Motor_Context_T * p_motor);

void Motor_InitDecouplingCoeffs(Motor_Config_T * p_config);

extern void Motor_InitSpeedRamp(Motor_Context_T * p_motor);
extern void Motor_InitTorqueRamp(Motor_Context_T * p_motor);
extern void Motor_ResetSpeedPid(Motor_Context_T * p_motor);
extern void Motor_ResetIPid(Motor_Context_T * p_motor);
extern void _Motor_ResetTuning(Motor_T * p_motor);

extern void Motor_SetFeedbackMode(Motor_T * p_motor, Motor_FeedbackMode_T mode);
extern void Motor_SetDirection(Motor_T * p_motor, Motor_Direction_T direction);

extern void Motor_ResetSpeedLimit(Motor_Context_T * p_motor);
extern void Motor_ResetILimit(Motor_Context_T * p_motor);
extern void Motor_SetSpeedLimits(Motor_Context_T * p_motor, uint16_t speed_ufract16);
extern void Motor_SetILimits(Motor_Context_T * p_motor, uint16_t i_fract16);

extern void Motor_SetSpeedLimitForward(Motor_Context_T * p_motor, uint16_t speed_ufract16);
extern void Motor_SetSpeedLimitReverse(Motor_Context_T * p_motor, uint16_t speed_ufract16);
extern void Motor_SetSpeedLimit_Scalar(Motor_Context_T * p_motor, uint16_t scalar_ufract16);
extern void Motor_SetILimitMotoring(Motor_Context_T * p_motor, uint16_t i_ufract16);
extern void Motor_SetILimitGenerating(Motor_Context_T * p_motor, uint16_t i_ufract16);
extern void Motor_SetILimit_Scalar(Motor_Context_T * p_motor, uint16_t scalar_ufract16);

#endif


/******************************************************************************/
/*!
    Angle Speed Wrap
*/
/******************************************************************************/
// static inline void Motor_SetElAngleFeedforward(Motor_Context_T * p_motor, angle16_t angle)
// {
//     _Motor_SetFeedforwardDelta(p_motor, angle - p_motor->SensorState.ElectricalAngle);
// }

// static inline void Motor_SetElSpeedFeedforward(Motor_Context_T * p_motor, angle16_t elSpeed_degPerCycle)
// {
//     _Motor_SetFeedforwardDelta(p_motor, elSpeed_degPerCycle);
// }

// static inline void Motor_SetMechAngleFeedforward(Motor_Context_T * p_motor, angle16_t angle)
// {
//     Motor_SetElAngleFeedforward(p_motor, angle * p_motor->Config.PolePairs);
// }

// static inline ufract16_t _Motor_GetIDerate(Motor_Context_T * p_motor, LimitArray_Augments_T * p_limits) { return math_min(Motor_GetILocalDerate(p_motor), _LimitArray_Upper(p_limits)); }
// static inline ufract16_t _Motor_GetSpeedDerate(Motor_Context_T * p_motor, LimitArray_Augments_T * p_limits) { return math_min(Motor_GetSpeedLocalDerate(p_motor), _LimitArray_Upper(p_limits)); }

/* Alternate base selection */
/*

typedef struct
{
    int32_t Ke_SpeedFract16;
}
Motor_ElectricalSpeedRef_T;
*/
/* ω_base => 2 * Kv * v_nominal = 1.0f */
/* L_base = V_base / (ω_base * I_base) */
// static inline uint16_t Motor_SpeedTypeMax_Rpm(Motor_T * p_motor) { return VBus_VSupplyNominal_V(&p_motor->P_VBUS->Config) * Motor_Config(p_motor)->SpeedRating.Kv; }
// static inline uint16_t Motor_VTypeMax_Volts(Motor_T * p_motor) { return VBus_VSupplyNominal_V(&p_motor->P_VBUS->Config) or Phase_Calibration_GetVMaxVolts(); }
// static inline uint16_t Motor_ITypeMax_Amps(Motor_T * p_motor) { return Phase_Calibration_GetIRatedPeak_Fract16(); }
// static inline uint16_t Motor_LTypeMax_Henries(Motor_T * p_motor) { return  Motor_VTypeMax_Volts(p_motor) / (2 * M_PI * Motor_SpeedTypeMax_Rpm(p_motor) / 60 * Motor_ITypeMax_Amps(p_motor)); }