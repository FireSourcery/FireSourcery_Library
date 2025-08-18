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

#include "MotorTimeRef.h"
#include "Analog/MotorAnalog.h"
#include "Analog/MotorAnalogRef.h"

#include "Phase/Phase.h"
#include "Sensor/RotorSensor_Table.h"
#include "Sensor/RotorSensor.h"

#include "Math/math_speed.h"
#include "Math/FOC.h"

#include "Peripheral/Analog/Analog.h"
#include "Peripheral/Analog/Linear_ADC.h"

#include "Transducer/Encoder/Encoder_ModeDT.h"
#include "Transducer/Encoder/Encoder_ISR.h"
#include "Transducer/Monitor/Heat/HeatMonitor.h"

// #include "Utility/Var/VarAccess.h"
#include "Utility/StateMachine/StateMachine.h"
#include "Utility/StateMachine/_StateMachine.h" /* Include the private header to contain StateMachine_Active_T within Motor_State_T */
#include "Utility/Timer/Timer.h"
#include "Utility/LimitArray/LimitArray.h"

#include "Math/Fixed/fixed.h"
#include "Math/Linear/Linear.h"
#include "Math/Ramp/Ramp.h"
#include "Math/PID/PID.h"
#include "Math/Filter/Filter.h"

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>


/*
    Parts - with dependency on Motor_T forward declared
    This way sub modules contain handling details
*/
// #include "Motor_Var.h"
#include "Analog/Motor_Analog.h"
#include "Sensor/Motor_Sensor.h"
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
        uint8_t OpenLoop   : 1U;   /* 0 -> Angle Sensor feedback, 1 -> OpenLoop */
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

/******************************************************************************/
/*
    Number formats

    [Fract16]           [-1:1) <=> [-32768:32767] in Q1.15
    [UFract16]          [0:2) <=> [0:65535] in Q1.15
    [Accum32]           [-2:2] <=> [-65536:65536] in Q1.15      Max [INT32_MIN:INT32_MAX]
    [Percent16/Unit16]  [0:1) <=> [0:65535] in Q0.16
    [Fixed16]           [-1:1] <=> [-256:256] in Q8.8           Max [-32768:32767]
    [Fixed32]           [-1:1] <=> [-65536:65536] in Q16.16     Max [INT32_MIN:INT32_MAX]
*/
/******************************************************************************/

/*!
    @brief Motor Config - Runtime variable configuration. Load from non volatile memory.
*/
typedef struct Motor_Config
{
    Motor_CommutationMode_T CommutationMode;
    Motor_Direction_T DirectionForward;
    RotorSensor_Id_T SensorMode;

    /*
        Calibration parameters
    */
    /* Config stored in Electrical Degrees need to sync with pole pairs */
    uint8_t PolePairs;                  /* Motor Pole Pairs. Use to derive Mech/Electrical speed calibration */
    uint16_t Kv;                        /* Motor Constant. Use to derive SpeedVRef. Optionally sets SpeedRated */
    uint16_t SpeedRated_DegPerCycle;    /* Speed at nominal VSource. Clamp or scale limits. Mech >= VSource*Kv */
                                        /* option keep as rpm for same units as kv */
                                        // uint16_t SpeedRated_Digital;
    uint16_t VSpeedScalar_Fract16;  /* Additional adjustment for Bemf match scalar SpeedVMatchRef, ensure resume control at lower speed. */
    // Motor_ResumeMode_T ResumeMode; // option Scale to VSpeed or VBemf on resume

    uint16_t IaZeroRef_Adcu;
    uint16_t IbZeroRef_Adcu;
    uint16_t IcZeroRef_Adcu;

    /*
        Persistent Base Limits. Same units as control vars
        Limits of Input Ramps
    */
    /* Use same units as Feedback */
    uint16_t SpeedLimitForward_Fract16;     /* of SpeedRated */
    uint16_t SpeedLimitReverse_Fract16;
    // uint16_t SpeedLimitForward_DegPerCycle;
    // uint16_t SpeedLimitReverse_DegPerCycle;
    uint16_t ILimitMotoring_Fract16;        /* of I_CALIB_AMPS. */
    uint16_t ILimitGenerating_Fract16;

    /* Acceleration as time */
    uint32_t SpeedRampTime_Cycles;      /* Time to reach SpeedRated */
    uint32_t TorqueRampTime_Cycles;     /* Time to reach I_RATED */

    PID_Config_T PidSpeed;  /* Speed Control */
    PID_Config_T PidI;      /* Iq Control */

    /* All OpenLoop Modes */
    uint16_t OpenLoopScalarLimit_Fract16;   /* Limit of rated. as scalar [0:1.0F] [0:32768]. */

    /* Calibration and Jog Align */
    uint16_t AlignScalar_Fract16;           /* Applies V or I. as scalar [0:1.0F] [0:32768]. */
    uint32_t AlignTime_Cycles;              /* Ramp time and step duration */

    /* OpenLoop Run/StartUp */
// #if defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
    // uint16_t OpenLoopRampSpeedFinal_Fract16;
    uint16_t OpenLoopRampSpeedFinal_DegPerCycle;
    uint32_t OpenLoopRampSpeedTime_Cycles;      /* Time to reach OpenLoopSpeed */
    uint16_t OpenLoopRampIFinal_Fract16;
    uint32_t OpenLoopRampITime_Cycles;          /* Time to reach OpenLoopI */
    // uint16_t OpenLoopGain_VHz;
// #endif

#if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
    Phase_Polar_Mode_T PhasePwmMode;     /* Only 1 nvm param for phase module. */
#endif

#if defined(CONFIG_MOTOR_UNIT_CONVERSION_LOCAL) && defined(CONFIG_MOTOR_SURFACE_SPEED_ENABLE)
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
    uint32_t ControlTimerBase;                  /* Control Freq ~ 20kHz, calibration, commutation, angle control. overflow 20Khz: 59 hours*/
    Motor_FaultFlags_T FaultFlags;              /* Fault SubState */
    uint8_t CalibrationStateIndex;

    /* Interface buffers */
    volatile MotorAnalog_State_T AnalogState;
    volatile Phase_Bits_T IBatch;
    volatile Phase_Bits_T VBatch;
    volatile bool SpeedUpdateFlag;              /* Sync Feedback update */
    HeatMonitor_T HeatMonitorState;

    /*
        Feedback State
    */
    /*
        Position Sensor
    */
    const RotorSensor_T * p_ActiveSensor;
    RotorSensor_State_T SensorState;        /* Compile time configured address. Sensor State */
    // Angle_T Angle;                       /* Outer/Speed Feedback State. */

    /* StateMachine Controlled */
    Motor_Direction_T Direction;            /* Applied V direction. */
    Motor_FeedbackMode_T FeedbackMode;      /* Active FeedbackMode, Control/Run SubState */

    /*
        Active Limits
        Ramp SetPoint Clamp

        Use to derive directional Ramp/Feedback limits.
        State interface independent of direction
        no resync on direction change
        effective as sentinel comparison
    */
    uint16_t SpeedLimitForward_Fract16; /* May over saturate */
    uint16_t SpeedLimitReverse_Fract16;
    // uint16_t SpeedLimitForward_DegPerCycle;     /* As ElectricalSpeed */
    // uint16_t SpeedLimitReverse_DegPerCycle;     /* As ElectricalSpeed */
    // uint16_t SpeedLimit_DegPerCycle; /* handle on direction switch */

    uint16_t ILimitMotoring_Fract16;
    uint16_t ILimitGenerating_Fract16;

    /* Alternatively, store as runtime cw/ccw  interface apply to both directions */
    /* Cached directional limits - on feedback */
    /* Speed direction  */
    // int16_t SpeedLimitCcw_Fract16;  /* + */
    // int16_t SpeedLimitCw_Fract16;   /* - */
    /* alternatively store in torqueramp in current mode */
    /* getters 20kz */
    // int16_t ILimitCcw_Fract16;      /* + */
    // int16_t ILimitCw_Fract16;       /* - */

    /*
        User Input Ramps
        UserCmdValue as RampTarget. Updated without StateMachine check
    */
    // fract16_t UserCmd; /* altneratively, as it is on a seperate thread */
    Ramp_T SpeedRamp;               /* Output [-32767:32767] SetPoint => SpeedReq */
    Ramp_T TorqueRamp;              /* Output [-32767:32767] SetPoint => IReq/VReq */

    /* OpenLoop Preset, StartUp */
    Ramp_T OpenLoopSpeedRamp;       /* Preset Speed Ramp */
    Ramp_T OpenLoopIRamp;           /* Preset I Ramp */
    // Ramp_T OpenLoopTorqueRamp;   /* Preset V/I Ramp */

    /*
        Speed Feedback
    */
    PID_T PidSpeed;                 /* Input PidSpeed(RampCmd - Speed_Fract16), Output => VPwm, Vq, Iq. */
    // PID_T PidPosition;

    /*
        FOC
    */
    FOC_T Foc;
    PID_T PidIq;                /* Input (IqReq - IqFeedback), Output Vq. Sign as CCW/CW direction */
    PID_T PidId;

    // PID_T PidIPhase; /* Align */

    /*
        Storable Config
    */
    Motor_Config_T Config;

    /*
        Unit Conversions
    */
    // Linear_T UnitsIa;           /* Fract16 and Amps */
    // Linear_T UnitsIb;
    // Linear_T UnitsIc;
    // Linear_T UnitsVabc;         /* Vbemf/VPhase Fract16 conversion of adcu */
    // Linear_T UnitsVSpeed;       /* VbemfSpeed. Vbemf_Fract16 of Speed_Fract16. Resume voltage, calculated Vbemf */

    Filter_T FilterA;           /* Calibration use */
    Filter_T FilterB;
    Filter_T FilterC;

    /* Jog */
    // uint32_t JogIndex;
    /*
        Six-Step
    */
    // PID_T PidIBus;
#if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
    /* MotorSixStep_T */
    BEMF_T Bemf;
    Motor_SectorId_T NextPhase;
    Motor_SectorId_T CommutationPhase;
    uint32_t CommutationTimeRef;
    uint32_t IBus_Fract16;
    uint16_t VPwm;     /* Six-Step Control Variable */
    uint16_t VBemfPeak_Adcu;
    uint16_t VBemfPeakTemp_Adcu;
    uint16_t IPhasePeak_Adcu;
    uint16_t IPhasePeakTemp_Adcu;
    uint32_t OpenLoopCommutationPeriod;
#endif

#if  defined(CONFIG_MOTOR_DEBUG_ENABLE) && !defined(NDEBUG)
    volatile uint32_t MicrosRef;
    // volatile bool DebugFlag;
    // volatile uint32_t DebugError;
    volatile uint32_t DebugTime[10U];
    volatile uint32_t DebugCounter;
#endif

    // VarAccess_State_T VarAccessInputState;   /* Input and I/O Input */
    // VarAccess_State_T VarAccessPidTunningState; /*  */
    /* Optionally */
    // VarAccess_State_T VarAccessConfigState;  /* use StateMachine */
    // VarAccess_State_T VarAccessOuputState;
}
Motor_State_T;


/*!
    @brief Motor Const Handle - Compile time const configuration instance.
    program, meta, parameters. unrelated to end user config.
    Context of Thread and StateMachine.
*/
typedef const struct Motor
{
    Motor_State_T * P_MOTOR_STATE;

    /*
        const context dependency of StateMachine: PHASE, TIMER
        alternatively, reduce StateMachine context, handle with pointer from state
    */
    Phase_T PHASE;
    Motor_Analog_T ANALOG; // PhaseAnalog_T PHASE_ANALOG;
    RotorSensor_Table_T SENSOR_TABLE;
    HeatMonitor_Context_T HEAT_MONITOR_CONTEXT;
    StateMachine_T STATE_MACHINE;
    TimerT_T CONTROL_TIMER;  /* State Timer. Map to ControlTimerBase */
    TimerT_T SPEED_TIMER;   /* Outer Speed Loop Timer. Millis */
    // Motor_VarAccess_T VAR_ACCESS;
    const Motor_Config_T * P_NVM_CONFIG;
}
Motor_T;

// #define MOTOR_CONTROL_TIMER_INIT(p_MotorContext, MotorState) TIMER_T_ALLOC(


/*
    Types
*/
typedef int motor_value_t;
// typedef register_t motor_value_t;

typedef void(*Motor_Proc_T)(Motor_State_T * p_motor);

typedef motor_value_t(*Motor_Get_T)(const Motor_State_T * p_motor);
typedef void(*Motor_Set_T)(Motor_State_T * p_motor, motor_value_t value);

typedef bool(*Motor_State_Test_T)(const Motor_State_T * p_motor);
typedef bool(*Motor_State_TryProc_T)(Motor_State_T * p_motor);
typedef bool(*Motor_State_TrySet_T)(Motor_State_T * p_motor, motor_value_t value);
typedef bool(*Motor_State_TryValue_T)(const Motor_State_T * p_motor, motor_value_t value);


// static inline RotorSensor_T * _Motor_GetSensor(const Motor_T * p_motor) { return RotorSensor_Of(&p_motor->SENSOR_TABLE, p_motor->P_MOTOR_STATE->Config.SensorMode); }


/******************************************************************************/
/*
    Derived From Config
    Speed Ref / Conversion
*/
/******************************************************************************/
/******************************************************************************/
/*
*/
/******************************************************************************/
/*
    V of Speed
*/
static inline uint16_t Motor_RpmOfKv(const Motor_State_T * p_motor, uint16_t v_fract16) { return fract16_mul(v_fract16, (int32_t)p_motor->Config.Kv * MotorAnalogRef_GetVMaxVolts()); }
static inline uint16_t Motor_VFract16OfKv(const Motor_State_T * p_motor, uint16_t rpm) { return fract16_div(rpm, (int32_t)p_motor->Config.Kv * MotorAnalogRef_GetVMaxVolts()); }

/* SpeedRated via Kv */
/* run-time dependent GetVSource */
static inline uint16_t Motor_GetSpeedVRef_Rpm(const Motor_State_T * p_motor) { return Motor_RpmOfKv(p_motor, MotorAnalog_GetVSource_Fract16()); }
static inline uint16_t Motor_GetSpeedVRef_DegPerCycle(const Motor_State_T * p_motor) { return speed_el_angle16_of_mech_rpm(MOTOR_CONTROL_FREQ, p_motor->Config.PolePairs, Motor_GetSpeedVRef_Rpm(p_motor)); }

static inline uint16_t Motor_GetSpeedVSvpwmRef_Rpm(const Motor_State_T * p_motor) { return fract16_mul(Motor_GetSpeedVRef_Rpm(p_motor), FRACT16_2_DIV_SQRT3); }
static inline uint16_t Motor_GetSpeedVSvpwmRef_DegPerCycle(const Motor_State_T * p_motor) { return fract16_mul(Motor_GetSpeedVRef_DegPerCycle(p_motor), FRACT16_2_DIV_SQRT3); }

/* SpeedRated via direct Parameter */
static inline uint32_t Motor_GetSpeedRatedRef_DegPerCycle(const Motor_State_T * p_motor) { return p_motor->Config.SpeedRated_DegPerCycle; }
static inline uint32_t Motor_GetSpeedRatedRef_ERpm(const Motor_State_T * p_motor) { return speed_rpm_of_angle16(MOTOR_CONTROL_FREQ, p_motor->Config.SpeedRated_DegPerCycle); }
static inline uint16_t Motor_GetSpeedRatedRef_Rpm(const Motor_State_T * p_motor) { return speed_mech_rpm_of_el_angle16(MOTOR_CONTROL_FREQ, p_motor->Config.PolePairs, p_motor->Config.SpeedRated_DegPerCycle); }

/* V of SpeedRated. debug */
static inline int32_t Motor_GetVSpeedRated_Fract16(const Motor_State_T * p_motor) { return Motor_VFract16OfKv(p_motor, Motor_GetSpeedRatedRef_Rpm(p_motor)); }


/*!
    @return V_Fract16 - VBus using Kv
    caller handle phase adjustment
*/
/* alternatively convert kv to electrical domain */
// static inline accum32_t Motor_VSpeed_Fract16OfDegPerCycle(const Motor_State_T * p_motor, int32_t elSpeed_degPerCycle) { return elSpeed_degPerCycle * MotorAnalog_GetVSource_Fract16() / Motor_GetSpeedVRef_DegPerCycle(p_motor); }
    // { return Motor_VFract16OfKv(p_motor, speed_mech_rpm_of_el_angle16(MOTOR_CONTROL_FREQ, p_motor->Config.PolePairs, elSpeed_degPerCycle) ); }
    // { return (elSpeed_degPerCycle * MOTOR_CONTROL_FREQ / (65536U / 60U) * 32768U) * p_motor->Config.PolePairs / (p_motor->Config.Kv * MotorAnalogRef_GetVMaxVolts()); }

/* SpeedOfV return DegPerCycle */
// static inline int32_t Motor_SpeedV_DegPerCycleOfFract16(const Motor_State_T * p_motor, accum32_t v_fract16) { return v_fract16 * Motor_GetSpeedVRef_DegPerCycle(p_motor) / MotorAnalog_GetVSource_Fract16(); }

/* When Speed is scaled to VSource */
/* SpeedRated ~= SpeedVRef */
// static inline accum32_t Motor_VOfSpeed_Fract16(accum32_t speed_fract16) { return fract16_mul(speed_fract16, MotorAnalog_GetVSource_Fract16()); }
// static inline accum32_t Motor_SpeedOfV_Fract16(accum32_t v_fract16)     { return fract16_div(v_fract16, MotorAnalog_GetVSource_Fract16()); }

/* Local Unit Conversion */
// static inline accum32_t Motor_Speed_Fract16OfRpm(const Motor_State_T * p_motor, int16_t speed_rpm)      { return speed_rpm * INT16_MAX / Motor_GetSpeedRatedRef_Rpm(p_motor); }
// static inline int16_t Motor_Speed_RpmOfFract16(const Motor_State_T * p_motor, accum32_t speed_fract16)  { return speed_fract16 * Motor_GetSpeedRatedRef_Rpm(p_motor) / 32768; }
// static inline int16_t Motor_Speed_RpmOfDeg(const Motor_State_T * p_motor, accum32_t speed_degPerCycle)  { return speed_mech_rpm_of_el_angle16(MOTOR_CONTROL_FREQ, p_motor->Config.PolePairs, speed_degPerCycle); }

/******************************************************************************/
/*
    Capture State
*/
/******************************************************************************/
/* inline for state machine */
static inline void Motor_CaptureSensor(const Motor_T * p_motor)
{
    Motor_State_T * p_state = p_motor->P_MOTOR_STATE;
    if (TimerT_Periodic_Poll(&p_motor->SPEED_TIMER) == true)
    {
        RotorSensor_CaptureSpeed(p_state->p_ActiveSensor);
        p_state->SpeedUpdateFlag = true; /* Set flag to update speed in outer loop */
    }
    RotorSensor_CaptureAngle(p_state->p_ActiveSensor);
}

/*
    Interface Wrap
*/
/* Feedback Speed */
static inline int32_t Motor_GetDigitalSpeed(const Motor_State_T * p_motor) { return RotorSensor_GetElectricalSpeed(p_motor->p_ActiveSensor); }
static inline int32_t Motor_GetMechSpeed_Fract16(const Motor_State_T * p_motor) { return RotorSensor_GetSpeed_Fract16(p_motor->p_ActiveSensor); }

/* interface getter, optionally set el speed */
/* As normalized Mech Speed */
static inline int32_t Motor_GetSpeedFeedback(const Motor_State_T * p_motor) { return RotorSensor_GetSpeed_Fract16(p_motor->p_ActiveSensor); }


/******************************************************************************/
/*
    Proc Outer Feeback State
*/
/******************************************************************************/
/******************************************************************************/
/*
    Run/Feeback State Limits
    Signed with CW/CCW direction.
    getters for abstraction, implementaiton may change
*/
/******************************************************************************/
// static inline int32_t Motor_GetVPhaseRefSvpwm(void) { return fract16_mul(MotorAnalog_GetVSource_Fract16(), FRACT16_1_DIV_SQRT3); }
// static inline int32_t Motor_GetVPhaseRef(void) { return MotorAnalog_GetVSource_Fract16() / 2; }

/* No plugging limit */
/* Null returns 0 or lower value */
static inline ufract16_t _Motor_VLimitOf(const Motor_State_T * p_motor, Motor_Direction_T select) { return (p_motor->Direction == select) ? MotorAnalog_GetVSource_Fract16() / 2 : 0; }
static inline int16_t _Motor_GetVLimitCcw(const Motor_State_T * p_motor) { return _Motor_VLimitOf(p_motor, MOTOR_DIRECTION_CCW); }
static inline int16_t _Motor_GetVLimitCw(const Motor_State_T * p_motor) { return ((int32_t)0 - _Motor_VLimitOf(p_motor, MOTOR_DIRECTION_CW)); }

/* Limits of [Iq]. */
static inline ufract16_t _Motor_ILimitOf(const Motor_State_T * p_motor, Motor_Direction_T select) { return (p_motor->Direction == select) ? p_motor->ILimitMotoring_Fract16 : p_motor->ILimitGenerating_Fract16; }
static inline fract16_t _Motor_GetILimitCcw(const Motor_State_T * p_motor) { return _Motor_ILimitOf(p_motor, MOTOR_DIRECTION_CCW); }
static inline fract16_t _Motor_GetILimitCw(const Motor_State_T * p_motor) { return ((int32_t)0 - _Motor_ILimitOf(p_motor, MOTOR_DIRECTION_CW)); }
// static inline pair16_t _Motor_GetILimits(const Motor_State_T * p_motor) { return (pair16_t) { .x = _Motor_GetILimitCw(p_motor), .y = _Motor_GetILimitCcw(p_motor) }; }

/* As normalized Mech Speed */
/* Limits of [Speed_Fract16] */
static inline ufract16_t _Motor_SpeedLimitOf(const Motor_State_T * p_motor, Motor_Direction_T select) { return (p_motor->Config.DirectionForward == select) ? p_motor->SpeedLimitForward_Fract16 : p_motor->SpeedLimitReverse_Fract16; }
static inline fract16_t _Motor_GetSpeedLimitCcw(const Motor_State_T * p_motor) { return _Motor_SpeedLimitOf(p_motor, MOTOR_DIRECTION_CCW); }
static inline fract16_t _Motor_GetSpeedLimitCw(const Motor_State_T * p_motor) { return ((int32_t)0 - _Motor_SpeedLimitOf(p_motor, MOTOR_DIRECTION_CW)); }

/* clamp with limit. On Req, same units */
/* Call ccw/cw using getters. Implementation may change */
static inline fract16_t Motor_VReqLimitOf(const Motor_State_T * p_motor, int16_t vReq) { return math_clamp(vReq, _Motor_GetVLimitCw(p_motor), _Motor_GetVLimitCcw(p_motor)); }
static inline fract16_t Motor_IReqLimitOf(const Motor_State_T * p_motor, int16_t iReq) { return math_clamp(iReq, _Motor_GetILimitCw(p_motor), _Motor_GetILimitCcw(p_motor)); }
static inline fract16_t Motor_SpeedReqLimitOf(const Motor_State_T * p_motor, int16_t speedReq) { return math_clamp(speedReq, _Motor_GetSpeedLimitCw(p_motor), _Motor_GetSpeedLimitCcw(p_motor)); }

/*
    Non Directional
*/
/* Speed limit in [Direction] selected. Forward relative to the user */
static inline uint16_t Motor_GetSpeedLimitActive(const Motor_State_T * p_motor) { return _Motor_SpeedLimitOf(p_motor, p_motor->Direction); }
// static inline bool Motor_IsSpeedLimitReached(const Motor_State_T * p_motor) { return (math_abs(Motor_GetSpeedFeedback(p_motor)) > Motor_GetSpeedLimitActive(p_motor)); }

// static inline uint16_t Motor_GetILimitMotoringActive(const Motor_State_T * p_motor) { return p_motor->ILimitMotoring_Fract16; }
// static inline uint16_t Motor_GetILimitGeneratingActive(const Motor_State_T * p_motor) { return p_motor->ILimitGenerating_Fract16; }

/*
    OpenLoop
*/
/* OpenLoop limit is always as aligned to the user */
static inline uint16_t Motor_OpenLoopILimit(const Motor_State_T * p_motor) { return fract16_mul(p_motor->Config.OpenLoopScalarLimit_Fract16, MotorAnalogRef_GetIRatedPeak_Fract16()); }
static inline int16_t Motor_OpenLoopILimitOf(const Motor_State_T * p_motor, int16_t iReq) { return math_clamp(iReq, (int32_t)0 - Motor_OpenLoopILimit(p_motor), Motor_OpenLoopILimit(p_motor)); }
// static inline uint16_t Motor_OpenLoopILimitOf(Motor_State_T * p_motor, uint16_t iReq) { return math_min(iReq, Motor_OpenLoopILimit(p_motor)); }

/*  */
static inline uint16_t Motor_OpenLoopVLimit(const Motor_State_T * p_motor) { return fract16_mul(p_motor->Config.OpenLoopScalarLimit_Fract16, MotorAnalog_GetVSource_Fract16() / 2); }
static inline int16_t Motor_OpenLoopVLimitOf(const Motor_State_T * p_motor, int16_t vReq) { return math_clamp(vReq, (int32_t)0 - Motor_OpenLoopVLimit(p_motor), Motor_OpenLoopVLimit(p_motor)); }

static inline uint16_t Motor_GetIAlign(const Motor_State_T * p_motor) { return fract16_mul(p_motor->Config.AlignScalar_Fract16, MotorAnalogRef_GetIRatedPeak_Fract16()); }
static inline uint16_t Motor_GetVAlign(const Motor_State_T * p_motor) { return fract16_mul(p_motor->Config.AlignScalar_Fract16, MotorAnalog_GetVSource_Fract16() / 2); }

/* fract16_div((uint32_t)Motor_GetVAlign(p_motor) * 2 / 3, MotorAnalog_GetVSource_Fract16() / 2) */
static inline uint16_t Motor_GetVAlign_Duty(const Motor_State_T * p_motor) { return (uint32_t)p_motor->Config.AlignScalar_Fract16 * 3 / 4; }




/******************************************************************************/
/*
    Ramp
*/
/******************************************************************************/
/* optionally cache 20khz getters */
/* optionally move target outside */
static inline fract16_t Motor_ProcTorqueRamp(Motor_State_T * p_motor)           { return Ramp_ProcNextOf(&p_motor->TorqueRamp, Motor_IReqLimitOf(p_motor, Ramp_GetTarget(&p_motor->TorqueRamp))); }
static inline fract16_t Motor_ProcTorqueRampAsVoltage(Motor_State_T * p_motor)  { return Ramp_ProcNextOf(&p_motor->TorqueRamp, Motor_VReqLimitOf(p_motor, Ramp_GetTarget(&p_motor->TorqueRamp))); }

static inline fract16_t Motor_ProcTorqueRampOpenLoop(Motor_State_T * p_motor)
{
    // return (p_motor->FeedbackMode.Current == 1U) ?
    //     Motor_OpenLoopILimitOf(p_motor, Ramp_ProcOutput(&p_motor->TorqueRamp)) :
    //     Motor_OpenLoopVLimitOf(p_motor, Ramp_ProcOutput(&p_motor->TorqueRamp));

    fract16_t req = (p_motor->FeedbackMode.Current == 1U) ?
        Motor_OpenLoopILimitOf(p_motor, Ramp_GetTarget(&p_motor->TorqueRamp)) :
        Motor_OpenLoopVLimitOf(p_motor, Ramp_GetTarget(&p_motor->TorqueRamp));

    return Ramp_ProcNextOf(&p_motor->TorqueRamp, req);
}

/* ramp down on limit update this way */
static inline fract16_t Motor_ProcSpeedRamp(Motor_State_T * p_motor) { return Ramp_ProcNextOf(&p_motor->SpeedRamp, Motor_SpeedReqLimitOf(p_motor, Ramp_GetTarget(&p_motor->SpeedRamp))); }


/******************************************************************************/
/*
    Outer Control Feedback State
*/
/******************************************************************************/
/*
    Speed Feedback Loop

    Ramp input ~100Hz,
    SpeedFeedback update 1000Hz - SpeedRamp, SpeedPid
*/
static inline fract16_t Motor_ProcSpeedFeedback(Motor_State_T * p_motor)
{
    if (p_motor->FeedbackMode.Speed == 1U)
    {
        // pass limits on proc to remove limits sync dependency
        // _PID_SetOutputLimits(&p_motor->PidSpeed, _Motor_GetILimitCw(p_motor), _Motor_GetILimitCcw(p_motor)); todo switch on region
        PID_ProcPI(&p_motor->PidSpeed, Motor_GetSpeedFeedback(p_motor), Motor_ProcSpeedRamp(p_motor));
    }
    // else /* Voltage/Current Mode */
    // {
        // todo on sign change
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
//   Motor_MatchOuterFeedbackState
static inline void Motor_MatchSpeedTorqueState(Motor_State_T * p_motor, int32_t torqueState)
{
    if (p_motor->FeedbackMode.Speed == 1U)
    {
        PID_SetOutputState(&p_motor->PidSpeed, torqueState);
        Ramp_SetOutputState(&p_motor->SpeedRamp, Motor_GetSpeedFeedback(p_motor));
        // Ramp_SetTarget(&p_motor->SpeedRamp, Motor_GetSpeedFeedback(p_motor)); /* altenertiavel split by input thread */
    }
    else
    {
        Ramp_SetOutputState(&p_motor->TorqueRamp, torqueState);
        // Ramp_SetTarget(&p_motor->TorqueRamp, torqueState);
    }
}

//    ApplySpeedLimit
// static inline void Motor_MatchSpeedILimits(Motor_State_T * p_motor, int32_t limit)
// {
//     // p_motor->ILimitMotoring_Fract16 = limit;
//     // p_motor->ILimitGenerating_Fract16
// }


/******************************************************************************/
/*!
    Query
*/
/******************************************************************************/
/******************************************************************************/
/*!
    VPhase approximation via Speed
    @return
*/
/******************************************************************************/
static inline int32_t Motor_GetVSpeed_Fract16(const Motor_State_T * p_motor)
{
    // int32_t result = Motor_VSpeed_Fract16OfDegPerCycle(p_motor, RotorSensor_GetElectricalSpeed(p_motor->p_ActiveSensor));
    int32_t vmax = fract16_mul(MotorAnalog_GetVSource_Fract16(), FRACT16_1_DIV_SQRT3); // if svpwm
    int32_t result = RotorSensor_GetElectricalSpeed(p_motor->p_ActiveSensor) * vmax / p_motor->Config.SpeedRated_DegPerCycle; /* SpeedRated ~= SpeedVRef */
    return fract16_mul(result, p_motor->Config.VSpeedScalar_Fract16);
}


/******************************************************************************/
/*
    Direction - User/Unsigned Config
*/
/******************************************************************************/
/*!
    Convert between a user reference direction or non directional, to CCW/CW direction
    @param[in] userCmd int32_t[-65536:65536] fract16 or percent16. positive is forward relative to the user.
    @return int32_t[-65536:65536],
    @note Over saturated if input is -32768, caller call clamp. cast may resilt in overflow.

*/
static inline int32_t Motor_DirectionalValueOf(const Motor_State_T * p_motor, int32_t userCmd) { return (p_motor->Direction * userCmd); }

/* */
static inline Motor_Direction_T Motor_GetDirectionForward(const Motor_State_T * p_motor) { return p_motor->Config.DirectionForward; }
static inline Motor_Direction_T Motor_GetDirectionReverse(const Motor_State_T * p_motor) { return (p_motor->Config.DirectionForward * -1); }
static inline bool Motor_IsDirectionForward(const Motor_State_T * p_motor) { return (p_motor->Direction == p_motor->Config.DirectionForward) && (p_motor->Direction != 0U); }
static inline bool Motor_IsDirectionReverse(const Motor_State_T * p_motor) { return (p_motor->Direction != p_motor->Config.DirectionForward) && (p_motor->Direction != 0U); }




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

// extern void Motor_SetAngleFeedforward(Motor_State_T * p_motor, angle16_t angle);

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
