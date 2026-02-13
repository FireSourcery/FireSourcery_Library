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

#include "Config.h"

#include "Phase/Phase.h"
#include "Phase_Input/Phase_Input.h"
#include "Phase_Input/Phase_VBus.h"
#include "Phase_Input/Phase_Analog.h"
#include "Phase_Input/Phase_Calibration.h"

#include "Sensor/RotorSensor_Table.h"
#include "Sensor/RotorSensor.h"

#include "Math/FOC.h"

#include "Peripheral/Analog/Analog.h"
#include "Peripheral/Analog/Linear_ADC.h"

#include "Transducer/Encoder/Encoder_ModeDT.h"
#include "Transducer/Encoder/Encoder_ISR.h"
#include "Transducer/Monitor/Heat/HeatMonitor.h"

#include "Utility/StateMachine/StateMachine.h"
#include "Utility/StateMachine/_StateMachine.h" /* Include the private header to contain StateMachine_Active_T within Motor_State_T */
#include "Utility/Timer/Timer.h"
#include "Utility/LimitArray/LimitArray.h"

#include "Math/Fixed/fixed.h"
#include "Math/Angle/Angle.h"
#include "Math/Linear/Linear.h"
#include "Math/Ramp/Ramp.h"
#include "Math/PID/PID.h"
#include "Math/Filter/Filter.h"

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

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
    MOTOR_DIRECTION_REVERSE = -1,
    MOTOR_DIRECTION_FORWARD = 1,
}
Motor_Direction_T;


/******************************************************************************/
/*
    Feedback Mode
*/
/******************************************************************************/
typedef union Motor_FeedbackMode
{
    struct
    {
        uint8_t OpenLoop   : 1U;   /* 0 -> Angle Sensor feedback, 1 -> OpenLoop */ /* Affective as control flag to set OpenLoop State */
        uint8_t Current    : 1U;   /* 0 -> Voltage, 1-> Current */
        uint8_t Speed      : 1U;   /* 0 -> Voltage or Current only, 1 -> Speed feedback */
        uint8_t Position   : 1U;
    };
    uint8_t Value; /* Id */
}
Motor_FeedbackMode_T;

/* Defined as const bit-fields over enum. in line with bit as conditional, not all combination need to be defined */
static const Motor_FeedbackMode_T MOTOR_FEEDBACK_MODE_OPEN_LOOP_SCALAR     = { .OpenLoop = 1U, .Speed = 0U, .Current = 0U, };
static const Motor_FeedbackMode_T MOTOR_FEEDBACK_MODE_OPEN_LOOP_CURRENT    = { .OpenLoop = 1U, .Speed = 0U, .Current = 1U, };
static const Motor_FeedbackMode_T MOTOR_FEEDBACK_MODE_VOLTAGE              = { .OpenLoop = 0U, .Speed = 0U, .Current = 0U, };
static const Motor_FeedbackMode_T MOTOR_FEEDBACK_MODE_CURRENT              = { .OpenLoop = 0U, .Speed = 0U, .Current = 1U, };
static const Motor_FeedbackMode_T MOTOR_FEEDBACK_MODE_SPEED_VOLTAGE        = { .OpenLoop = 0U, .Speed = 1U, .Current = 0U, };
static const Motor_FeedbackMode_T MOTOR_FEEDBACK_MODE_SPEED_CURRENT        = { .OpenLoop = 0U, .Speed = 1U, .Current = 1U, };

static inline Motor_FeedbackMode_T Motor_FeedbackMode_Cast(int value) { return (Motor_FeedbackMode_T) { .Value = value }; }


/******************************************************************************/
/*
    Fault State Flags
*/
/******************************************************************************/
typedef union
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

/*!
    @brief Motor Config - Runtime variable configuration, settings. Load from non volatile memory.
*/
typedef struct Motor_Config
{
    Motor_CommutationMode_T CommutationMode;
    RotorSensor_Id_T SensorMode;
    Motor_Direction_T DirectionForward; /* CCW/CW Assigned positive direction */
    // Motor_ResumeMode_T ResumeMode; // option Scale to VSpeed or VBemf on resume

    /*
        Calibration parameters
    */
    /* Config stored in Electrical Degrees need to sync with pole pairs */
    uint8_t PolePairs;                  /* Motor Pole Pairs. Use to derive Mech/Electrical speed calibration */
    uint16_t Kv;                        /* [RpmPerVolt] Motor Constant. Use to derive SpeedVRef. Optionally sets SpeedRated */
    uint16_t SpeedRated_Rpm;            /* [Rpm] for same units as kv. Speed at nominal VSource. Clamp or scale limits. */

    /*
        Derived Parameters during initialization or from Host
        alternatively store as control domain units
    */
    // uint16_t Kv_DegPerCyclePerVFract16;     /* Kv in control domain units */
    // uint16_t SpeedRated_DegPerCycle;        /* electrical degrees per control cycle */

    uint16_t VSpeedScalar_Fract16;      /* Additional adjustment for VBemf match. [0.7:1.3] typical. ensure resume control at lower speed. */

    Phase_Triplet_T IabcZeroRef_Adcu;

    /*
        Persistent Base Limits.
        Limits of Input Ramps
        User view as unsigned. Physical frame.
        Use same units as Req/Feedback
    */
    uint16_t SpeedLimitForward_Fract16;     /* [0:32767] = [0:100%] of SpeedRated */
    uint16_t SpeedLimitReverse_Fract16;
    uint16_t ILimitMotoring_Fract16;        /* [0:32767] = [0:100%] of I_CALIB_AMPS. */
    uint16_t ILimitGenerating_Fract16;

    /* Acceleration as time */
    uint32_t SpeedRampTime_Cycles;      /* Time to reach SpeedRated */
    uint32_t TorqueRampTime_Cycles;     /* Time to reach I_RATED */

    PID_Config_T PidSpeed;  /* Speed Control */
    PID_Config_T PidI;      /* Idq Control */

    /*

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


/*
    Motor State - Runtime variable state.
*/
typedef struct Motor_State
{
    /*
        State and SubStates
    */
    StateMachine_Active_T StateMachine;
    uint32_t ControlTimerBase;              /* Control Freq ~ 20kHz, calibration, commutation, angle control. Overflow 20Khz: 59 hours */ //ControlCounter

    /* Effectively Substates StateMachine Controlled */
    Motor_Direction_T Direction;            /* Direction of applied/cmd V. Calibration correction applied on get/set access. DirectionMotoring. Applies to: Motoring/Generating Cmd Active Limits, Sensor Interpolation */
    Motor_FeedbackMode_T FeedbackMode;      /* Active FeedbackMode, Control/Run SubState Flags */
    /*  */
    Motor_FaultFlags_T FaultFlags;          /* Fault SubState */
    uint8_t CalibrationStateIndex;

    /*
        Position Sensor
    */
    const RotorSensor_T * p_ActiveSensor;   /* Pointer to entry in SENSOR_TABLE */
    RotorSensor_State_T SensorState;        /* Compile time configured address. Sensor State includes [Angle_T] */

    /*
        User Input Ramps
        UserCmdValue as RampTarget. Updated without StateMachine check
    */
    /*
        Active Limits
        Ramp Setpoint Clamp

        Use to derive directional Ramp/Feedback limits.
        State interface independent of direction
        no resync on direction change
        effective as sentinel comparison
    */
    /*
        Speed Feedback
    */
    Ramp_T SpeedRamp;                   /* Output [-32767:32767] Setpoint => SpeedReq */
    // uint16_t SpeedLimit_Fract16;     /* handle update on direction change */
    uint16_t SpeedLimitForward_Fract16;
    uint16_t SpeedLimitReverse_Fract16;
    volatile bool SpeedUpdateFlag;      /* Speed capture sync Feedback update */
    PID_T PidSpeed;                     /* Input PidSpeed(RampCmd - Speed_Fract16), Output => VPwm, Vq, Iq. */
    // PID_T PidPosition;

    /*
        FOC
    */
    Ramp_T TorqueRamp;                      /* Output [-32767:32767] Setpoint => IReq/VReq */
    uint16_t ILimitMotoring_Fract16;
    uint16_t ILimitGenerating_Fract16;
    /* Cached directional limits - on feedback */ /* getters 20kz */
    int16_t ILimitCcw_Fract16;      /* + */
    int16_t ILimitCw_Fract16;       /* - */

    volatile Phase_Input_T PhaseInput;
    FOC_T Foc;
    PID_T PidIq;                /* Input (IqReq - IqFeedback), Output Vq. Sign as CCW/CW direction */
    PID_T PidId;
    // PID_T PidIPhase;         /* Align, or use getter */

    /* OpenLoop Preset, StartUp. No boundary checking */
    Ramp_T OpenLoopSpeedRamp;       /* Preset Speed Ramp */
    Ramp_T OpenLoopIRamp;           /* Preset I Ramp */
    // Ramp_T OpenLoopTorqueRamp;   /* Preset V/I Ramp */
    Angle_T OpenLoopAngle;

    /*  */
    HeatMonitor_T HeatMonitorState;

    /*
        Storable Config
    */
    Motor_Config_T Config;

    Filter_T FilterA;           /* Calibration use */
    Filter_T FilterB;
    Filter_T FilterC;

    /*
        Local Unit Conversion
    */
    // Linear_T UnitsIa;           /* Fract16 and Amps */
    // Linear_T UnitsIb;
    // Linear_T UnitsIc;
    // Linear_T UnitsVabc;         /* Vbemf/VPhase Fract16 conversion of adcu */
    // Linear_T UnitsVSpeed;       /* VbemfSpeed. Vbemf_Fract16 of Speed_Fract16. Resume voltage, calculated Vbemf */

    /* Jog */
    // uint32_t JogIndex;

    /*
        Six-Step
    */
#if defined(MOTOR_SIX_STEP_ENABLE)
    // PID_T PidIBus;
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
    volatile uint32_t MicrosRef;
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
    Motor_State_T * P_MOTOR_STATE;

    Phase_T PHASE;
    Phase_Analog_T PHASE_ANALOG;
    RotorSensor_Table_T SENSOR_TABLE; /* Runtime selection. Init macros in Motor_Sensor.h */
    HeatMonitor_Context_T HEAT_MONITOR_CONTEXT;
    // Analog_Conversion_T HEAT_MONITOR_ANALOG;
    StateMachine_T STATE_MACHINE;
    TimerT_T CONTROL_TIMER;     /* State Timer. Map to ControlTimerBase */
    TimerT_T SPEED_TIMER;       /* Outer Speed Loop Timer. Millis */
    const Motor_Config_T * P_NVM_CONFIG;
    // Motor_VarAccess_T VAR_ACCESS;
}
Motor_T;

// #define MOTOR_CONTROL_TIMER_INIT(p_MotorContext, MotorState) TIMER_T_ALLOC(

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
    Number formats

    [Fract16]           [-1:1) <=> [-32768:32767] in Q1.15
    [UFract16]          [0:2) <=> [0:65535] in Q1.15
    [Accum32]           [-2:2] <=> [-65536:65536] in Q17.15     Max [INT32_MIN:INT32_MAX]
    [Percent16/Unit16]  [0:1) <=> [0:65535] in Q0.16
    [Fixed16]           [-1:1] <=> [-256:256] in Q8.8           Max [-32768:32767]
    [Fixed32]           [-1:1] <=> [-65536:65536] in Q16.16     Max [INT32_MIN:INT32_MAX]
*/
/******************************************************************************/

/******************************************************************************/
/*
    Speed Ref / Conversion
    Derived From Config
*/
/******************************************************************************/
// static inline uint16_t motor_rpm_of_kv(int kv, int v, uint16_t v_fract16) { return fract16_mul(v_fract16, kv * v); }
static inline int16_t _Motor_RpmOfDeg(const Motor_State_T * p_motor, accum32_t speed_rpm)          { return el_angle_of_mech_rpm(MOTOR_CONTROL_FREQ, p_motor->Config.PolePairs, speed_rpm); }
static inline int16_t _Motor_DegOfRpm(const Motor_State_T * p_motor, accum32_t speed_degPerCycle)  { return mech_rpm_of_el_angle(MOTOR_CONTROL_FREQ, p_motor->Config.PolePairs, speed_degPerCycle); }

/*
    Speed/V relation based on Kv.
    Rpm of Kv * V
*/
static inline uint16_t Motor_RpmOfKv(const Motor_State_T * p_motor, uint16_t v_fract16) { return fract16_mul(v_fract16, (int32_t)p_motor->Config.Kv * Phase_Calibration_GetVMaxVolts()); }
static inline uint16_t Motor_VFract16OfKv(const Motor_State_T * p_motor, uint16_t rpm)  { return fract16_div(rpm, (int32_t)p_motor->Config.Kv * Phase_Calibration_GetVMaxVolts()); }
// static inline uint16_t Motor_GetKt(const Motor_State_T * p_motor) { 60 / (2 * pi * kV);  }

/*
   Calibration values derive with system config
*/
/*
    Speed VBus Ref
    Generally [SpeedRated] via Kv * V
    run-time dependent VBus or VNominal
*/
static inline uint16_t Motor_GetSpeedVBusRef_Rpm(const Motor_State_T * p_motor) { return Motor_RpmOfKv(p_motor, Phase_VBus_Fract16()); } /* { return (int32_t)p_motor->Config.Kv * Phase_VBus_Volts; } */
static inline uint16_t Motor_GetSpeedVBusRef_DegPerCycle(const Motor_State_T * p_motor) { return el_angle_of_mech_rpm(MOTOR_CONTROL_FREQ, p_motor->Config.PolePairs, Motor_GetSpeedVBusRef_Rpm(p_motor)); }

static inline uint16_t Motor_GetSpeedVNominalRef_Rpm(const Motor_State_T * p_motor) { return Motor_RpmOfKv(p_motor, Phase_VBus_GetVNominal()); }
static inline uint16_t Motor_GetSpeedVNominalRef_DegPerCycle(const Motor_State_T * p_motor) { return el_angle_of_mech_rpm(MOTOR_CONTROL_FREQ, p_motor->Config.PolePairs, Motor_GetSpeedVNominalRef_Rpm(p_motor)); }

/* [SpeedRated] via direct Parameter */
/* can constrain Set with Kv on config */
static inline uint16_t Motor_GetSpeedRated_Rpm(const Motor_State_T * p_motor) { return p_motor->Config.SpeedRated_Rpm; }
static inline uint32_t Motor_GetSpeedRated_ERpm(const Motor_State_T * p_motor) { return (uint32_t)p_motor->Config.SpeedRated_Rpm * p_motor->Config.PolePairs; }
static inline uint16_t Motor_GetSpeedRated_DegPerCycle(const Motor_State_T * p_motor) { return el_angle_of_mech_rpm(MOTOR_CONTROL_FREQ, p_motor->Config.PolePairs, p_motor->Config.SpeedRated_Rpm); }

/* Defined as VBusRef without config adjustment */
// static inline uint16_t Motor_GetSpeedRated_Rpm(const Motor_State_T * p_motor) { return Motor_GetSpeedVBusRef_Rpm(p_motor); }

/* Base in electrical domain */
// static inline uint32_t Motor_GetSpeedRated_DegPerCycle(const Motor_State_T * p_motor) { return p_motor->Config.SpeedRated_DegPerCycle; }
// static inline uint32_t Motor_GetSpeedRated_ERpm(const Motor_State_T * p_motor) { return rpm_of_angle(MOTOR_CONTROL_FREQ, p_motor->Config.SpeedRated_DegPerCycle); }
// static inline uint16_t Motor_GetSpeedRated_Rpm(const Motor_State_T * p_motor) { return mech_rpm_of_el_angle(MOTOR_CONTROL_FREQ, p_motor->Config.PolePairs, p_motor->Config.SpeedRated_DegPerCycle); }

/*
    Num Max
*/
/*
    Numerical max as ~2x kv*V, ~2x[SpeedRated_Rpm]
    2x as simple limit
    account for 1.15 factor and field weakening range.
*/
#ifndef MOTOR_SPEED_TYPE_MAX_RPM
#define MOTOR_SPEED_TYPE_MAX_RPM(...) ((uint32_t)Motor_GetSpeedRated_Rpm(__VA_ARGS__)*2)
#endif

static inline uint16_t Motor_GetSpeedTypeMax_Rpm(const Motor_State_T * p_motor) { return MOTOR_SPEED_TYPE_MAX_RPM(p_motor); }
static inline uint16_t Motor_GetSpeedTypeMax_DegPerCycle(const Motor_State_T * p_motor) { return el_angle_of_mech_rpm(MOTOR_CONTROL_FREQ, p_motor->Config.PolePairs, Motor_GetSpeedTypeMax_Rpm(p_motor)); }

/* Local Unit Conversion */
static inline accum32_t Motor_Speed_Fract16OfRpm(const Motor_State_T * p_motor, int16_t speed_rpm)      { return speed_rpm * INT16_MAX / Motor_GetSpeedTypeMax_Rpm(p_motor); }
static inline int16_t Motor_Speed_RpmOfFract16(const Motor_State_T * p_motor, accum32_t speed_fract16)  { return speed_fract16 * Motor_GetSpeedTypeMax_Rpm(p_motor) / 32768; }

/*
*/
/* [SpeedRated_Rpm] = [SpeedTypeMax_Rpm] / 2  */
static inline uint16_t Motor_GetSpeedRated_Fract16(const Motor_State_T * p_motor) { return INT16_MAX / 2; }

/******************************************************************************/
/*
    Proc Outer Feedback State
*/
/******************************************************************************/
/******************************************************************************/
/*
    Run/Feedback State Limits
    getters for abstraction, implementation may change
    Null returns 0 or lower abs value
*/
/******************************************************************************/
/* Cw [-:0], Ccw [0:+] */
static inline int32_t _Motor_CcwClampOf(const Motor_State_T * p_motor, int32_t value) { return math_max(value, 0); }
static inline int32_t _Motor_CwClampOf(const Motor_State_T * p_motor, int32_t value) { return math_min(value, 0); }

/*
    Clamp plugging - One direction clamp
    Cw: [value:0]
    Ccw: [0:value]
*/
static inline int32_t _Motor_VClampLimitOf(const Motor_State_T * p_motor, Motor_Direction_T select, int32_t value) { return (p_motor->Direction == select) ? value : 0; }
static inline int32_t _Motor_VClampCcwOf(const Motor_State_T * p_motor, int32_t value) { return _Motor_VClampLimitOf(p_motor, MOTOR_DIRECTION_CCW, value); }
static inline int32_t _Motor_VClampCwOf(const Motor_State_T * p_motor, int32_t value) { return _Motor_VClampLimitOf(p_motor, MOTOR_DIRECTION_CW, value); }

/* clamp using ~1/2 VBus */
/* No plugging limit */
static inline ufract16_t _Motor_VLimitOf(const Motor_State_T * p_motor, Motor_Direction_T select) { return _Motor_VClampLimitOf(p_motor, select, Phase_VBus_GetVRefSvpwm()); }
static inline fract16_t _Motor_GetVLimitCcw(const Motor_State_T * p_motor) { return _Motor_VLimitOf(p_motor, MOTOR_DIRECTION_CCW); }
static inline fract16_t _Motor_GetVLimitCw(const Motor_State_T * p_motor) { return (0 - _Motor_VLimitOf(p_motor, MOTOR_DIRECTION_CW)); }
// static inline int32_t Motor_VClampCcwOf(const Motor_State_T * p_motor, int32_t value) { return (p_motor->IsClampPlugging) ? _Motor_VClampLimitOf(p_motor, MOTOR_DIRECTION_CCW, value) : value; }
// static inline int32_t Motor_VClampCwOf(const Motor_State_T * p_motor, int32_t value) { return (p_motor->IsClampPlugging) ? _Motor_VClampLimitOf(p_motor, MOTOR_DIRECTION_CW, value) : value; }

/*
    Call ccw/cw using getters.
    clamp with limit. On Req, same units
    Ramp target and user input
*/
/*
    Limits of [Iq]
    Current limits: keyed by runtime applied [Direction]
    This SWAPS when [Direction] changes
*/
/* alternatively directional limits on trigger, direction and control limits */
static inline ufract16_t _Motor_ILimitOf(const Motor_State_T * p_motor, Motor_Direction_T select) { return (p_motor->Direction == select) ? p_motor->ILimitMotoring_Fract16 : p_motor->ILimitGenerating_Fract16; }
static inline fract16_t _Motor_GetILimitCcw(const Motor_State_T * p_motor) { return _Motor_ILimitOf(p_motor, MOTOR_DIRECTION_CCW); }
static inline fract16_t _Motor_GetILimitCw(const Motor_State_T * p_motor) { return (0 - _Motor_ILimitOf(p_motor, MOTOR_DIRECTION_CW)); }

/*
    Limits of [Speed_Fract16]
    Speed limits: keyed by calibration parameter [Config.DirectionForward] (which CW/CCW is "forward")
    This is FIXED — doesn't change when [Direction] changes
    Positive userCmd aligns to configured positive direction. independent of electrical direction.
*/
static inline ufract16_t _Motor_SpeedLimitOf(const Motor_State_T * p_motor, Motor_Direction_T select) { return (p_motor->Config.DirectionForward == select) ? p_motor->SpeedLimitForward_Fract16 : p_motor->SpeedLimitReverse_Fract16; }
static inline fract16_t _Motor_GetSpeedLimitCcw(const Motor_State_T * p_motor) { return _Motor_SpeedLimitOf(p_motor, MOTOR_DIRECTION_CCW); }
static inline fract16_t _Motor_GetSpeedLimitCw(const Motor_State_T * p_motor) { return (0 - _Motor_SpeedLimitOf(p_motor, MOTOR_DIRECTION_CW)); }

// static inline fract16_t Motor_GetSpeedLimitCcw(const Motor_State_T * p_motor) { return _Motor_GetSpeedLimitCcw }
// static inline fract16_t Motor_GetSpeedLimitCw(const Motor_State_T * p_motor) { return _Motor_GetSpeedLimitCw }


/*
    OpenLoop
*/
/* OpenLoop limit is always as aligned to the user */
static inline uint16_t Motor_OpenLoopILimit(const Motor_State_T * p_motor) { return fract16_mul(p_motor->Config.OpenLoopLimitScalar_Fract16, Phase_Calibration_GetIRatedPeak_Fract16()); }
static inline int16_t Motor_OpenLoopILimitOf(const Motor_State_T * p_motor, int16_t iReq) { return math_clamp(iReq, (int32_t)0 - Motor_OpenLoopILimit(p_motor), Motor_OpenLoopILimit(p_motor)); }

/*  */
static inline uint16_t Motor_OpenLoopVLimit(const Motor_State_T * p_motor) { return fract16_mul(p_motor->Config.OpenLoopLimitScalar_Fract16, Phase_VBus_GetVRef()); }
static inline int16_t Motor_OpenLoopVLimitOf(const Motor_State_T * p_motor, int16_t vReq) { return math_clamp(vReq, (int32_t)0 - Motor_OpenLoopVLimit(p_motor), Motor_OpenLoopVLimit(p_motor)); }

/*  */
static inline uint16_t Motor_GetIAlign(const Motor_State_T * p_motor) { return fract16_mul(p_motor->Config.AlignScalar_Fract16, Phase_Calibration_GetIRatedPeak_Fract16()); }
static inline uint16_t Motor_GetVAlign(const Motor_State_T * p_motor) { return fract16_mul(p_motor->Config.AlignScalar_Fract16, Phase_VBus_GetVRef()); }

/* fract16_div((uint32_t)Motor_GetVAlign(p_motor) * 2 / 3, Phase_VBus_GetVRef()) */
static inline uint16_t Motor_GetVAlign_Duty(const Motor_State_T * p_motor) { return (uint32_t)p_motor->Config.AlignScalar_Fract16 * 3 / 4; }

/******************************************************************************/
/*
    Ramp
*/
/******************************************************************************/
/* VLimitClamp */
/* optionally move to ramp */
static inline fract16_t Motor_VReqLimitOf(const Motor_State_T * p_motor, int16_t vReq) { return math_clamp(vReq, _Motor_GetVLimitCw(p_motor), _Motor_GetVLimitCcw(p_motor)); }

static inline fract16_t Motor_IReqLimitOf(const Motor_State_T * p_motor, int16_t iReq) { return math_clamp(iReq, _Motor_GetILimitCw(p_motor), _Motor_GetILimitCcw(p_motor)); }

/* optionally cache 20khz getters */
/* optionally move target outside */
static inline fract16_t Motor_ProcTorqueRampI(Motor_State_T * p_motor)   { return Ramp_ProcNextOf(&p_motor->TorqueRamp, Motor_IReqLimitOf(p_motor, Ramp_GetTarget(&p_motor->TorqueRamp))); }
static inline fract16_t Motor_ProcTorqueRampV(Motor_State_T * p_motor)  { return Ramp_ProcNextOf(&p_motor->TorqueRamp, Motor_VReqLimitOf(p_motor, Ramp_GetTarget(&p_motor->TorqueRamp))); }


static inline fract16_t Motor_ProcTorqueRamp(Motor_State_T * p_motor)
{
    return (p_motor->FeedbackMode.Current == 1U) ? Motor_ProcTorqueRampI(p_motor) : Motor_ProcTorqueRampV(p_motor);
}

/* user input Openloop mode limits */
static inline fract16_t Motor_ProcTorqueRampOpenLoop(Motor_State_T * p_motor)
{
    // Apply limit on Ramp state
    fract16_t req = (p_motor->FeedbackMode.Current == 1U) ?
        Motor_OpenLoopILimitOf(p_motor, Ramp_GetTarget(&p_motor->TorqueRamp)) :
        Motor_OpenLoopVLimitOf(p_motor, Ramp_GetTarget(&p_motor->TorqueRamp));

    return Ramp_ProcNextOf(&p_motor->TorqueRamp, req);
}


/******************************************************************************/
/*
    Outer Control Feedback State
*/
/******************************************************************************/
/*
    Capture State
*/
/* inline for state machine */
static inline void Motor_CaptureSensor(const Motor_T * p_motor)
{
    Motor_State_T * p_state = p_motor->P_MOTOR_STATE;
    if (TimerT_Periodic_Poll(&p_motor->SPEED_TIMER) == true)
    {
        RotorSensor_CaptureSpeed(p_state->p_ActiveSensor);
        p_state->SpeedUpdateFlag = true; /* Set flag to update speed in outer loop */ /* alternatively StateMachine input capture speed */
    }
    RotorSensor_CaptureAngle(p_state->p_ActiveSensor);

    // if caller handles rotor direction mismatch
    // if (Motor_GetRotorDirection(p_motor) != p_state->Direction)
    // {
    //     // Runtime reversal detected – update applied direction for control
    // }
}


/* alternatively clamp with single active value */
static inline fract16_t Motor_SpeedReqLimitOf(const Motor_State_T * p_motor, int16_t speedReq) { return math_clamp(speedReq, _Motor_GetSpeedLimitCw(p_motor), _Motor_GetSpeedLimitCcw(p_motor)); }


/* Feedback Speed interface getter */
static inline accum32_t Motor_GetSpeedFeedback(const Motor_State_T * p_motor) { return RotorSensor_GetSpeed_Fract16(p_motor->p_ActiveSensor); }
static inline bool Motor_IsSpeedZero(const Motor_State_T * p_motor) { return (Motor_GetSpeedFeedback(p_motor) == 0); }
static inline bool Motor_IsSpeedLimitReached(const Motor_State_T * p_motor) { return !math_is_in_range(Motor_GetSpeedFeedback(p_motor), _Motor_GetSpeedLimitCw(p_motor), _Motor_GetSpeedLimitCcw(p_motor)) ; }


/* Limits are applied to the target. Ramp smoothing applies on limit update */
static inline fract16_t Motor_ProcSpeedRamp(Motor_State_T * p_motor) { return Ramp_ProcNextOf(&p_motor->SpeedRamp, Motor_SpeedReqLimitOf(p_motor, Ramp_GetTarget(&p_motor->SpeedRamp))); }

/*
    Speed Feedback Loop

    Ramp input ~100Hz,
    SpeedFeedback update 1000Hz - SpeedRamp, SpeedPid
*/
static inline fract16_t Motor_ProcSpeedFeedback(Motor_State_T * p_motor)
{
    if (p_motor->FeedbackMode.Speed == 1U)
    {
        // alternatively pass limits on proc to remove limits sync dependency
        // PID_CaptureOutputLimits(&p_motor->PidSpeed, _Motor_GetILimitCw(p_motor), _Motor_GetILimitCcw(p_motor));
        PID_ProcPI(&p_motor->PidSpeed, Motor_GetSpeedFeedback(p_motor), Motor_ProcSpeedRamp(p_motor));
        Ramp_SetTarget(&p_motor->TorqueRamp, PID_GetOutput(&p_motor->PidSpeed)); /* Set TorqueRamp for unified interface. 20 Ticks */
    }
    // else /* Voltage/Current Mode */
    // {
        // on sign change
        // PID_ProcPI(&p_motor->PidSpeed, Motor_GetSpeedFeedback(p_motor), (int32_t)Motor_GetSpeedLimitActive(p_motor) * p_motor->Direction);
        // if (Motor_IsSpeedLimitReached(p_motor) == true) { Ramp_SetOutput(&p_motor->TorqueRamp, PID_GetOutput(&p_motor->PidSpeed)); }
    // }
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
// Motor_MatchOuterFeedbackState
static inline void Motor_MatchSpeedTorqueState(Motor_State_T * p_motor, int16_t torqueState)
{
    if (p_motor->FeedbackMode.Speed == 1U)
    {
        PID_SetOutputState(&p_motor->PidSpeed, torqueState);
        Ramp_SetOutputState(&p_motor->SpeedRamp, Motor_GetSpeedFeedback(p_motor)); // Ramp_SetTarget
    }
    Ramp_SetOutputState(&p_motor->TorqueRamp, torqueState);
}

// ApplySpeedLimit
// static inline void Motor_MatchSpeedILimits(Motor_State_T * p_motor, int32_t limit)
// {
//     // p_motor->ILimitMotoring_Fract16 = limit;
//     // p_motor->ILimitGenerating_Fract16
// }


/*!
    V <=> Speed conversion based on Kv.
    Speed/SpeedRated => V/VBusRef
    @return V_Fract16 - VBusRef of Speed Kv
*/
static inline accum32_t Motor_VBusOfSpeed_Fract16(const Motor_State_T * p_motor, accum32_t speed_fract16) { return Phase_VBus_Fract16() * speed_fract16 / Motor_GetSpeedRated_Fract16(p_motor); }
static inline accum32_t Motor_SpeedOfVBus_Fract16(const Motor_State_T * p_motor, accum32_t v_fract16) { return v_fract16 * Motor_GetSpeedRated_Fract16(p_motor) / Phase_VBus_Fract16(); }

/*  Phase peak. */
static inline accum32_t Motor_VPhaseOfSpeed_Fract16(const Motor_State_T * p_motor, accum32_t speed_fract16) { return Motor_VBusOfSpeed_Fract16(p_motor, speed_fract16) / 2; }

/*!
    VPhase approximation via Speed
*/
static inline int32_t Motor_GetVSpeed_Fract16(const Motor_State_T * p_motor)
{
    return fract16_mul(Motor_VPhaseOfSpeed_Fract16(p_motor, Motor_GetSpeedFeedback(p_motor)), p_motor->Config.VSpeedScalar_Fract16);
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
// static inline Motor_Direction_T Motor_GetRotorDirection(const Motor_State_T * p_motor) { return RotorSensor_GetDirection(p_motor->p_ActiveSensor); }

/*!
    Convert between a user reference direction to virtual CCW/CW direction
    @param[in] userCmd [-65536:65536] fract16 or percent16. positive as the direction set at config
    @return [-65536:65536]
    @note caller clamp. Over saturated if input is -32768. cast may result in overflow.
*/
// static inline int32_t Motor_DirectionalValueOf(const Motor_State_T * p_motor, int32_t userCmd) { return (p_motor->Config.DirectionForward * userCmd); }
/* Positive as the active appliedV/motoring direction.   */
// static inline int32_t Motor_ElectricalDirectionalValueOf(const Motor_State_T * p_motor, int32_t userCmd) { return (p_motor->Direction * userCmd); }
/*  */
static inline Motor_Direction_T Motor_GetVirtualDirection(const Motor_State_T * p_motor) { return p_motor->Direction; }

/* Calibrated direction that is User positive. Interpret as CCW or Positive */
static inline Motor_Direction_T Motor_GetUserDirection(const Motor_State_T * p_motor) { return p_motor->Config.DirectionForward * p_motor->Direction; }
/* Same calibration value  */
static inline int Motor_GetDirectionSign(const Motor_State_T * p_motor) { return (int)Motor_GetUserDirection(p_motor); }

static inline bool Motor_IsDirectionForward(const Motor_State_T * p_motor) { return (Motor_GetDirectionSign(p_motor) == 1); }
static inline bool Motor_IsDirectionReverse(const Motor_State_T * p_motor) { return (Motor_GetDirectionSign(p_motor) == -1); }
static inline bool Motor_IsDirectionStopped(const Motor_State_T * p_motor) { return (p_motor->Direction == MOTOR_DIRECTION_NULL); }

/* The virtual direction of sign */
static inline Motor_Direction_T Motor_GetDirectionForward(const Motor_State_T * p_motor) { return p_motor->Config.DirectionForward; }
static inline Motor_Direction_T Motor_GetDirectionReverse(const Motor_State_T * p_motor) { return (p_motor->Config.DirectionForward * -1); }


// static inline int Motor_GetDirectionFeedback(const Motor_State_T * p_motor) { return math_sign(Motor_GetSpeedFeedback(p_motor)); }

/******************************************************************************/
/*!
    Extern
*/
/******************************************************************************/
// extern void Motor_InitFrom(const Motor_T * p_context, const Motor_Config_T * p_config);
extern void Motor_Init(const Motor_T * p_context);
extern void Motor_Reset(Motor_State_T * p_motor);

extern void Motor_ReinitSensor(Motor_State_T * p_motor);
extern void Motor_ResetUnits(Motor_State_T * p_motor);

extern void Motor_ResetSpeedRamp(Motor_State_T * p_motor);
extern void Motor_ResetTorqueRamp(Motor_State_T * p_motor);

extern void Motor_SetFeedbackMode(Motor_State_T * p_motor, Motor_FeedbackMode_T mode);
extern void Motor_SetFeedbackMode_Cast(Motor_State_T * p_motor, int modeValue);

extern void Motor_SetDirection(Motor_State_T * p_motor, Motor_Direction_T direction);
extern void Motor_SetDirectionForward(Motor_State_T * p_motor);
extern void Motor_SetDirectionReverse(Motor_State_T * p_motor);

extern void Motor_ResetSpeedLimitActive(Motor_State_T * p_motor);
extern void Motor_ResetILimitActive(Motor_State_T * p_motor);

extern void Motor_SetSpeedLimitForward(Motor_State_T * p_motor, uint16_t speed_ufract16);
extern void Motor_SetSpeedLimitReverse(Motor_State_T * p_motor, uint16_t speed_ufract16);
extern void Motor_SetSpeedLimit_Scalar(Motor_State_T * p_motor, uint16_t scalar_ufract16);
extern void Motor_ClearSpeedLimit(Motor_State_T * p_motor);

extern void Motor_SetILimitMotoring(Motor_State_T * p_motor, uint16_t i_ufract16);
extern void Motor_SetILimitGenerating(Motor_State_T * p_motor, uint16_t i_ufract16);
extern void Motor_SetILimit_Scalar(Motor_State_T * p_motor, uint16_t scalar_ufract16);
extern void Motor_ClearILimit(Motor_State_T * p_motor);

#endif
