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
    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_H
#define MOTOR_H

#include "Config.h"

#include "Math/math_speed.h"
#include "Math/FOC.h"
#include "MotorTimeRef.h"
#include "Transducer/MotorAnalog/MotorAnalog.h"
#include "Transducer/MotorAnalog/MotorAnalogRef.h"
#include "Transducer/Phase/Phase.h"
#include "Transducer/MotorSensor/MotorSensor.h"
#include "Transducer/Hall/Hall.h"
#include "Transducer/SinCos/SinCos.h"

#include "Transducer/Encoder/Encoder_ModeDT.h"
#include "Transducer/Encoder/Encoder_ISR.h"
#include "Transducer/Thermistor/Thermistor.h"

#include "Utility/StateMachine/StateMachine.h"
#include "Utility/Timer/Timer.h"
#include "Utility/LimitArray/LimitArray.h"

#include "Math/Fixed/fixed.h"
#include "Math/Linear/Linear_ADC.h"
#include "Math/Linear/Linear_Voltage.h"
#include "Math/Linear/Linear.h"
#include "Math/Ramp/Ramp.h"
#include "Math/PID/PID.h"
#include "Math/Filter/Filter_MovAvg.h"
#include "Math/Filter/Filter.h"

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>


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
        uint8_t OpenLoop   : 1U;   /* 0 -> Position feedback, 1 -> OpenLoop */ /* Not used for feedback run state, a flag is require to track transitions */
        uint8_t Speed      : 1U;   /* 0 -> Voltage or Current only, 1 -> Speed feedback */
        uint8_t Current    : 1U;   /* 0 -> Voltage, 1-> Current */
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

/*
    Effectively sync mailbox for async results
*/
typedef union Motor_StateFlags
{
    struct
    {
        uint16_t HeatWarning        : 1U;
        uint16_t ILimited           : 1U;
        uint16_t SpeedLimited       : 1U;
        // uint16_t RampDisable        : 1U;
        // uint16_t VarControlEnabled  : 1U; /*  */
    };
    uint16_t Value;
}
Motor_StateFlags_T;

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
    @brief Motor Config - Runtime variable configuration. Load from non volatile memory.
*/
typedef struct Motor_Config
{
    MotorSensorId_T             SensorMode;
    Motor_CommutationMode_T     CommutationMode;
    Motor_Direction_T           DirectionForward;

    /*
        Calibration parameters
    */
    uint8_t PolePairs;          /* Motor Pole Pairs. Use to derive Hall/Encoder speed calibration */
    uint16_t Kv;                /* Motor Constant. Use to derive SpeedVRef */

    /* Config stored in Electrical Degrees need to sync with pole pairs */
    uint16_t SpeedRated_DegControl; /* Speed at nominal VSource. Clamp or scale limits. >= VSource*Kv */

    uint16_t VSpeedScalar_Fract16;      /* Use to derive SpeedVMatchRef, ensure resume control at lower speed. */
                                        /* 9/10 scalar => 100% Speed => 90% V */
    // Motor_ResumeMode_T ResumeMode; // option Scale to VSpeed or VBemf on resume

    uint16_t IaZeroRef_Adcu;
    uint16_t IbZeroRef_Adcu;
    uint16_t IcZeroRef_Adcu;

    /* Persistent Base Limits. Same units as control vars */
    uint16_t SpeedLimitForward_Fract16;     /* of SpeedVRef_Rpm, alternatively use control angle */
    uint16_t SpeedLimitReverse_Fract16;
    uint16_t ILimitMotoring_Fract16;        /* of I_MAX_AMPS. */
    uint16_t ILimitGenerating_Fract16;

    // uint32_t RampAccel_Cycles;
    uint32_t SpeedRampTime_Cycles;      /* Time to reach SpeedVRef_Rpm */
    uint32_t TorqueRampTime_Cycles;     /* Time to reach I_RATED */

    /* Calibration and Jog Align. Adhere to OpenLoop Limits */
    uint16_t AlignPower_Fract16;        /* V or I */
    uint32_t AlignTime_Cycles;          /* Ramp time and step duration */

    /* OpenLoop Run/StartUp */
// #if defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
    uint16_t OpenLoopSpeedFinal_Fract16;    /*   */
    uint16_t OpenLoopIFinal_Fract16;
    uint32_t OpenLoopSpeedRamp_Cycles;      /* Time to reach OpenLoopSpeed */
    uint32_t OpenLoopIRamp_Cycles;          /* Time to reach OpenLoopI */
    // uint16_t OpenLoopGain_VHz;
// #endif

#if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
    Phase_Mode_T PhasePwmMode;     /* Only 1 nvm param for phase module. */
#endif

#if defined(CONFIG_MOTOR_UNIT_CONVERSION_LOCAL) && defined(CONFIG_MOTOR_SURFACE_SPEED_ENABLE)
    uint16_t SurfaceDiameter;
    uint16_t GearRatioOutput;
    uint16_t GearRatioInput;
#endif
}
Motor_Config_T;

/*!
    @brief Motor Const - Compile time const configuration. Per instance.
    program, meta, parameters. unrelated to end user.
*/
typedef const struct Motor_Const
{
    const MotorAnalog_Conversions_T ANALOG_CONVERSIONS;
    // const MotorAnalog_Conversions_T * P_ANALOG_CONVERSIONS; /* change to store in flash */
    const Motor_Config_T * const P_NVM_CONFIG;
}
Motor_Const_T;

typedef struct Motor
{
    const Motor_Const_T CONST;
    Motor_Config_T Config;

    // volatile MotorAnalog_Results_T AnalogResults;

    Encoder_T Encoder;
    Hall_T Hall;
    // MotorSensor_T Sensor;
    Phase_T Phase;
    Thermistor_T Thermistor;

    /*
        State and SubStates
    */
    StateMachine_T StateMachine;
    Timer_T ControlTimer;                       /* State Timer */
    uint32_t ControlTimerBase;                  /* Control Freq ~ 20kHz, calibration, commutation, angle control. overflow 20Khz: 59 hours*/
    Motor_Direction_T Direction;                /* Applied spin direction. */
    Motor_FeedbackMode_T FeedbackMode;          /* Active FeedbackMode, Control/Run SubState */
    Motor_FaultFlags_T FaultFlags;              /* Fault SubState */
    Motor_StateFlags_T StateFlags;              /* Can be pushed to submodules */

    bool IsVarControlEnabled;                   /* Direct Var Set Enable. */

    uint8_t CalibrationStateIndex;
    volatile Phase_Bits_T IBatch;
    volatile Phase_Bits_T VBatch;

    /*
        User Input Ramps
        UserCmdValue as RampTarget. Updated without StateMachine check
    */
    Ramp_T SpeedRamp;               /* Output [-32767:32767] SetPoint => SpeedReq */
    Ramp_T TorqueRamp;              /* Output [-32767:32767] SetPoint => IReq */

    /* Openloop Preset, StartUp */
    Ramp_T OpenLoopSpeedRamp;       /* OpenLoopSpeed Ramp */
    Ramp_T OpenLoopIRamp;           /* OpenLoopI Ramp */
    // Ramp_T * p_ActiveRamp;       /* alternatively, reset values on switch */

    /*
        Position Sensor,
    */
    angle16_t MechanicalAngle;
    angle16_t ElectricalAngle;          /* Angle Feedback */

    /*
        Speed Feedback
    */
    angle16_t AngularSpeed_DegControl;  /* ElectricalAngleDelta */
    // uint16_t SpeedUnitScalar;
    int32_t Speed_Fract16;              /* Speed Feedback Variable. Can over saturate. - is virtual CW, + => CCW */
    Timer_T SpeedTimer;                 /* Outer Speed Loop Timer */
    PID_T PidSpeed;                     /* Input PidSpeed(RampCmd - Speed_Fract16), Output => VPwm, Vq, Iq. */
    // PID_T PidPosition;    /* todo */

    /*
        Active Limits
    */
    /* Non directional */
    /* Use to derive directional Feedback */
    uint16_t SpeedLimitForward_Fract16; /* May over saturate */
    uint16_t SpeedLimitReverse_Fract16;
    uint16_t ILimitMotoring_Fract16;
    uint16_t ILimitGenerating_Fract16;

    /* Alternatively, store as */
    /* Cached directional limits - on feedback */
    // int16_t SpeedLimitCcw_Fract16;  /* + */
    // int16_t SpeedLimitCw_Fract16;   /* - */
    // int16_t ILimitCcw_Fract16;      /* + */
    // int16_t ILimitCw_Fract16;       /* - */

    /*
        FOC
    */
    FOC_T Foc;
    PID_T PidIq;                /* Input (IqReq - IqFeedback), Output Vq. Sign indicates ccw/cw direction */
    PID_T PidId;

    /*
        Unit Conversions
    */
    Linear_T UnitsIa;           /* Fract16 and Amps */
    Linear_T UnitsIb;
    Linear_T UnitsIc;
    Linear_T UnitsVabc;         /* Vbemf/VPhase Fract16 conversion of adcu */
    Linear_T UnitsVSpeed;       /* VbemfSpeed. Vbemf_Fract16 of Speed_Fract16. Resume voltage, calculated Vbemf */

    Filter_T FilterA;           /* Calibration use */
    Filter_T FilterB;
    Filter_T FilterC;

    /* Jog */
    // uint32_t JogIndex;

    /* Sensorless and SinCos. Non square wave Encoder module */
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE) || defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
    Linear_T UnitsAngleSpeed;           /*  */
    Linear_T UnitsSurfaceSpeed;         /*  */
    angle16_t SpeedAngle;              /* Save for reference, MechanicalDelta */
#endif

    /*
        Six-Step
    */
#if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
    PID_T PidIBus;
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

#if  defined(CONFIG_MOTOR_DEBUG_ENABLE)
    volatile uint32_t MicrosRef;
    // volatile bool DebugFlag;
    // volatile uint32_t DebugError;
    volatile uint32_t DebugTime[10U];
    volatile uint32_t DebugCounter;
#endif
}
Motor_T, * MotorPtr_T;


/*
    Types
*/
typedef int motor_value_t;
// typedef register_t motor_value_t;

typedef void(*Motor_Proc_T)(Motor_T * p_motor);
typedef motor_value_t(*Motor_Get_T)(const Motor_T * p_motor);
typedef void(*Motor_Set_T)(Motor_T * p_motor, motor_value_t value);
typedef bool(*Motor_Test_T)(const Motor_T * p_motor);
typedef bool(*Motor_TryProc_T)(Motor_T * p_motor);
typedef bool(*Motor_TrySet_T)(Motor_T * p_motor, motor_value_t value);
typedef bool(*Motor_TryValue_T)(const Motor_T * p_motor, motor_value_t value);

/******************************************************************************/
/*
    Static Reference
*/
/******************************************************************************/

/******************************************************************************/
/*
    Number formats

    [Fract16]     [-1:1) <=> [-32768:32767] in Q1.15
    [UFract16]    [0:2) <=> [0:65535] in Q1.15
    [Accum32]     [-2:2] <=> [-65536:65536] in Q1.15      Max [INT32_MIN:INT32_MAX]
    [Percent16]   [0:1) <=> [0:65535] in Q0.16
    [Fixed16]     [-1:1] <=> [-256:256] in Q8.8           Max [-32768:32767]
    [Fixed32]     [-1:1] <=> [-65536:65536] in Q16.16     Max [INT32_MIN:INT32_MAX]
*/
/******************************************************************************/
/*
    Local Conversion Units
*/
static inline uint32_t _Motor_MillisOf(uint32_t controlCycles) { return MOTOR_CONTROL_TIME_MS(controlCycles); }
static inline uint32_t _Motor_ControlCyclesOf(uint32_t millis) { return MOTOR_CONTROL_CYCLES(millis); }

static inline accum32_t Motor_I_Fract16OfAmps(int16_t amps)             { return amps * INT16_MAX / MotorAnalogRef_GetIMaxAmps(); }
static inline accum32_t Motor_V_Fract16OfVolts(int16_t volts)           { return volts * INT16_MAX / MotorAnalogRef_GetVMaxVolts(); }
static inline int16_t   Motor_I_AmpsOfFract16(accum32_t fract16)        { return fract16 * MotorAnalogRef_GetIMaxAmps() / 32768; }
static inline int16_t   Motor_V_VoltsOfFract16(accum32_t fract16)       { return fract16 * MotorAnalogRef_GetVMaxVolts() / 32768; }
static inline accum32_t Motor_Power_VAOfFract16(accum32_t fract16)      { return fract16 * MotorAnalogRef_GetIMaxAmps() / 32768 * MotorAnalogRef_GetVMaxVolts() / 32768; }

// static inline accum32_t Motor_Speed_Fract16OfRpm(const Motor_T * p_motor, int16_t speed_rpm)        { return speed_rpm * INT16_MAX / Motor_GetSpeedRatedRef_Rpm(p_motor); }
// static inline int16_t   Motor_Speed_RpmOfFract16(const Motor_T * p_motor, accum32_t speed_fract16)  { return speed_fract16 * Motor_GetSpeedRatedRef_Rpm(p_motor) / 32768; }


/******************************************************************************/
/*
    Simplify CommutationMode Check
    This function should optimize away select if only 1 mode is enabled
*/
/******************************************************************************/
static inline const void * _Motor_CommutationModeFn(const Motor_T * p_motor, const void * focFunction, const void * sixStepFunction)
{
    const void * fn;
    switch (p_motor->Config.CommutationMode)
    {
#if defined(CONFIG_MOTOR_FOC_ENABLE)
        case MOTOR_COMMUTATION_MODE_FOC: fn = focFunction; break;
#endif
#if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
        case MOTOR_COMMUTATION_MODE_SIX_STEP: fn = sixStepFunction; break;
#endif
        // default: assert(false); break;
    }
    return fn;
}

// c23
// #define Motor_CommutationModeFn(p_motor, focFunction, sixStepFunction) ((typeof(focFunction))(_Motor_CommutationModeFn(p_motor, focFunction, sixStepFunction)))

#define Motor_CommutationModeFn(p_motor, focSet, sixStepSet) \
    _Generic((focSet), \
        Motor_Set_T:    (Motor_Set_T)(_Motor_CommutationModeFn(p_motor, focSet, sixStepSet)), \
        Motor_Get_T:    (Motor_Get_T)(_Motor_CommutationModeFn(p_motor, focSet, sixStepSet)),  \
        Motor_Proc_T:   (Motor_Proc_T)(_Motor_CommutationModeFn(p_motor, focSet, sixStepSet))   \
    )

#define Motor_CommutationModeFn_Call(p_motor, focSet, sixStepSet, ...) (Motor_CommutationModeFn(p_motor, focSet, sixStepSet)(p_motor __VA_OPT__(,) __VA_ARGS__))



/******************************************************************************/
/*
    Interrupts
*/
/******************************************************************************/
/* Controls StateMachine Proc. Local Critical */
static inline void Motor_ClearInterrupt(Motor_T * p_motor)     { Phase_ClearInterrupt(&p_motor->Phase); }
static inline void Motor_DisableInterrupt(Motor_T * p_motor)   { Phase_DisableInterrupt(&p_motor->Phase); }
static inline void Motor_EnableInterrupt(Motor_T * p_motor)    { Phase_EnableInterrupt(&p_motor->Phase); }


/******************************************************************************/
/*
    Speed Ref / Conversion
*/
/******************************************************************************/
/*
    V of Speed
*/
static inline uint16_t Motor_RpmOfKvVFract16(const Motor_T * p_motor, uint16_t v_fract16) { return (int32_t)p_motor->Config.Kv * v_fract16 * MotorAnalogRef_GetVMaxVolts() >> 15U; }

/* VSource may update during run-time */
static inline uint16_t Motor_GetSpeedVRef_Rpm(const Motor_T * p_motor) { return Motor_RpmOfKvVFract16(p_motor, MotorAnalogRef_GetVSource_Fract16()); }
static inline uint16_t Motor_GetSpeedVRef_DegControl(const Motor_T * p_motor) { return speed_angle16_of_rpm(MOTOR_CONTROL_FREQ / p_motor->Config.PolePairs, Motor_GetSpeedVRef_Rpm(p_motor)); }

static inline uint16_t Motor_GetSpeedVSvpwmRef_Rpm(const Motor_T * p_motor) { return fract16_mul(Motor_GetSpeedVRef_Rpm(p_motor), FRACT16_2_DIV_SQRT3); }
static inline uint16_t Motor_GetSpeedVSvpwmRef_DegControl(const Motor_T * p_motor) { return fract16_mul(Motor_GetSpeedVRef_DegControl(p_motor), FRACT16_2_DIV_SQRT3); }

/* Speed is scaled to VSource */
// static inline accum32_t Motor_VOfSpeed_Fract16(accum32_t speed_fract16) { return fract16_mul(speed_fract16, MotorAnalogRef_GetVSource_Fract16()); }
// static inline accum32_t Motor_SpeedOfV_Fract16(accum32_t v_fract16)     { return fract16_div(v_fract16, MotorAnalogRef_GetVSource_Fract16()); }

/* As VBus using Kv */
static inline accum32_t Motor_VSpeed_Fract16OfDeg(const Motor_T * p_motor, int32_t speed_degControl)
    { return speed_degControl * MotorAnalogRef_GetVSource_Fract16() / Motor_GetSpeedVRef_DegControl(p_motor); }

static inline int32_t Motor_SpeedV_DegOfFract16(const Motor_T * p_motor, accum32_t v_fract16)
    { return v_fract16 * Motor_GetSpeedVRef_DegControl(p_motor) / MotorAnalogRef_GetVSource_Fract16(); }

/*!
    VPhase approximation via Speed
    @return
*/
static inline int32_t Motor_GetVSpeed_Fract16(const Motor_T * p_motor)
{
    int32_t result = Motor_VSpeed_Fract16OfDeg(p_motor, p_motor->AngularSpeed_DegControl);
    // int32_t result = p_motor->AngularSpeed_DegControl * MotorAnalogRef_GetVSource_Fract16() / p_motor->Config.SpeedRated_DegControl; /* assuming SpeedRated ~= SpeedVRef */
    return fract16_mul(result, p_motor->Config.VSpeedScalar_Fract16);
}

/*
    Per Control Cycle
*/
static inline int32_t Motor_GetSpeedRatedRef_ERpm(const Motor_T * p_motor) { return speed_rpm_of_angle16(MOTOR_CONTROL_FREQ, p_motor->Config.SpeedRated_DegControl); }
static inline int32_t Motor_GetSpeedRatedRef_Rpm(const Motor_T * p_motor) { return speed_rpm_of_angle16(MOTOR_CONTROL_FREQ, p_motor->Config.SpeedRated_DegControl) / p_motor->Config.PolePairs; }

/*
    SpeedFract16 via RPM Ref
*/
static inline int16_t Motor_MechAngleOfSpeed_Fract16(const Motor_T * p_motor, int16_t speed_fract16)
    { return ((int32_t)speed_fract16 * Motor_GetSpeedRatedRef_Rpm(p_motor)) / ((uint32_t)60 * MOTOR_CONTROL_FREQ / 2); }

static inline int32_t Motor_MechSpeedOfAngle_Fract16(const Motor_T * p_motor, int16_t angle16)
    { return ((uint32_t)angle16 * MOTOR_CONTROL_FREQ * 60) / ((uint32_t)2 * Motor_GetSpeedRatedRef_Rpm(p_motor)); }

static inline int16_t Motor_ElecAngleOfSpeed_Fract16(const Motor_T * p_motor, int16_t speed_fract16)
    { return ((int32_t)speed_fract16 * Motor_GetSpeedRatedRef_Rpm(p_motor)) / ((uint32_t)60 * MOTOR_CONTROL_FREQ / 2 / p_motor->Config.PolePairs); }




/******************************************************************************/
/*
    Direction
*/
/******************************************************************************/
/*!
    Convert between a user reference direction or non directional, to CCW/CW direction
    @param[in] userCmd int32_t[-65536:65536] fract16 or percent16. positive is forward relative to the user.
    @return int32_t[-65536:65536],
    @note Over saturated if input is -32768, caller call clamp. cast may resilt in overflow.

*/
static inline int32_t Motor_DirectionalValueOf(const Motor_T * p_motor, int32_t userCmd) { return (p_motor->Direction * userCmd); }

/*
*/
static inline Motor_Direction_T Motor_GetDirectionForward(const Motor_T * p_motor) { return p_motor->Config.DirectionForward; }
static inline Motor_Direction_T Motor_GetDirectionReverse(const Motor_T * p_motor) { return (p_motor->Config.DirectionForward * -1); }
static inline bool Motor_IsDirectionForward(const Motor_T * p_motor) { return (p_motor->Direction == p_motor->Config.DirectionForward) && (p_motor->Direction != 0U); }
static inline bool Motor_IsDirectionReverse(const Motor_T * p_motor) { return (p_motor->Direction != p_motor->Config.DirectionForward) && (p_motor->Direction != 0U); }

static inline int Motor_GetUserDirection(const Motor_T * p_motor) { return p_motor->Config.DirectionForward * p_motor->Direction; }


/******************************************************************************/
/*
    Signed Limits
    Call from Feedback loop
*/
/******************************************************************************/
/*
    Limits getters as abstraction
*/
/* Null returns 0 or lower value */

/* No plugging limit */
static inline ufract16_t _Motor_GetVLimitOf(const Motor_T * p_motor, Motor_Direction_T select) { return (p_motor->Direction == select) ? MotorAnalogRef_GetVSource_Fract16() : 0; }
static inline int16_t _Motor_GetVLimitCcw(const Motor_T * p_motor) { return _Motor_GetVLimitOf(p_motor, MOTOR_DIRECTION_CCW); }
static inline int16_t _Motor_GetVLimitCw(const Motor_T * p_motor) { return ((int32_t)0 - _Motor_GetVLimitOf(p_motor, MOTOR_DIRECTION_CW)); }

/* Limits of [Iq]. */
static inline ufract16_t _Motor_ILimitOf(const Motor_T * p_motor, Motor_Direction_T select) { return (p_motor->Direction == select) ? p_motor->ILimitMotoring_Fract16 : p_motor->ILimitGenerating_Fract16; }
static inline fract16_t _Motor_GetILimitCcw(const Motor_T * p_motor) { return _Motor_ILimitOf(p_motor, MOTOR_DIRECTION_CCW); }
static inline fract16_t _Motor_GetILimitCw(const Motor_T * p_motor) { return ((int32_t)0 - _Motor_ILimitOf(p_motor, MOTOR_DIRECTION_CW)); }

/* Limits of [Speed_Fract16] */
static inline ufract16_t _Motor_SpeedLimitOf(const Motor_T * p_motor, Motor_Direction_T select) { return (p_motor->Config.DirectionForward == select) ? p_motor->SpeedLimitForward_Fract16 : p_motor->SpeedLimitReverse_Fract16; }
static inline fract16_t _Motor_GetSpeedLimitCcw(const Motor_T * p_motor) { return _Motor_SpeedLimitOf(p_motor, MOTOR_DIRECTION_CCW); }
static inline fract16_t _Motor_GetSpeedLimitCw(const Motor_T * p_motor) { return ((int32_t)0 - _Motor_SpeedLimitOf(p_motor, MOTOR_DIRECTION_CW)); }

/* clamp with limit. On Req, same units */
static inline fract16_t Motor_VReqLimitOf(const Motor_T * p_motor, int16_t vReq) { return math_clamp(vReq, _Motor_GetVLimitCw(p_motor), _Motor_GetVLimitCcw(p_motor)); }
static inline fract16_t Motor_IReqLimitOf(const Motor_T * p_motor, int16_t iReq) { return math_clamp(iReq, _Motor_GetILimitCw(p_motor), _Motor_GetILimitCcw(p_motor)); }
static inline fract16_t Motor_SpeedReqLimitOf(const Motor_T * p_motor, int16_t speedReq) { return math_clamp(speedReq, _Motor_GetSpeedLimitCw(p_motor), _Motor_GetSpeedLimitCcw(p_motor)); }

/* as abs scalar */
/* UserSpeedLimit */
static inline uint16_t Motor_GetSpeedLimit(const Motor_T * p_motor) { return (Motor_IsDirectionForward(p_motor) ? p_motor->SpeedLimitForward_Fract16 : p_motor->SpeedLimitReverse_Fract16); }
static inline bool Motor_IsSpeedLimitReached(const Motor_T * p_motor) { return (math_abs(p_motor->Speed_Fract16) > Motor_GetSpeedLimit(p_motor)); }


/******************************************************************************/
/*

*/
/******************************************************************************/
static inline void Motor_EnableRamp(Motor_T * p_motor) { Ramp_SetSlope(&p_motor->SpeedRamp, p_motor->Config.SpeedRampTime_Cycles, INT16_MAX); }
static inline void Motor_DisableRamp(Motor_T * p_motor) { _Ramp_Disable(&p_motor->SpeedRamp); }

static inline void Motor_DisableVarControl(Motor_T * p_motor)   { p_motor->IsVarControlEnabled = false; }
static inline void Motor_EnableVarControl(Motor_T * p_motor)    { p_motor->IsVarControlEnabled = true; }


/******************************************************************************/
/*!
    Extern
*/
/******************************************************************************/
extern void Motor_Init(Motor_T * p_motor);
extern void Motor_InitReboot(Motor_T * p_motor);

extern void Motor_SetMechAngleFeedforward(Motor_T * p_motor, angle16_t angle);
extern void Motor_SetElecAngleFeedforward(Motor_T * p_motor, angle16_t angle);

extern fract16_t Motor_ProcTorqueRamp(Motor_T * p_motor);
extern fract16_t Motor_ProcSpeedRamp(Motor_T * p_motor);
extern fract16_t Motor_ProcOuterFeedback(Motor_T * p_motor);
extern void Motor_MatchSpeedTorqueState(Motor_T * p_motor, int32_t torqueReq);

extern void Motor_SetFeedbackMode(Motor_T * p_motor, Motor_FeedbackMode_T mode);
extern void Motor_SetFeedbackMode_Cast(Motor_T * p_motor, int modeValue);

extern void Motor_SetDirection(Motor_T * p_motor, Motor_Direction_T direction);
extern void Motor_SetDirection_Cast(Motor_T * p_motor, int direction);
extern void Motor_SetDirectionForward(Motor_T * p_motor);
extern void Motor_SetDirectionReverse(Motor_T * p_motor);

extern void Motor_ResetSpeedLimitActive(Motor_T * p_motor);
extern void Motor_ResetILimitActive(Motor_T * p_motor);
extern void Motor_SetSpeedLimitForward(Motor_T * p_motor, uint16_t speed_ufract16);
extern void Motor_SetSpeedLimitReverse(Motor_T * p_motor, uint16_t speed_ufract16);
extern void Motor_SetSpeedLimit_Scalar(Motor_T * p_motor, uint16_t scalar_ufract16);
extern void Motor_ClearSpeedLimit(Motor_T * p_motor);
extern void Motor_SetILimitMotoring(Motor_T * p_motor, uint16_t i_ufract16);
extern void Motor_SetILimitGenerating(Motor_T * p_motor, uint16_t i_ufract16);
extern void Motor_SetILimit_Scalar(Motor_T * p_motor, uint16_t scalar_ufract16);
extern void Motor_ClearILimit(Motor_T * p_motor);
extern bool Motor_TryILimit(Motor_T * p_motor, uint16_t i_Fract16);
extern bool Motor_TrySpeedLimit(Motor_T * p_motor, uint16_t speed_Fract16);
// extern void Motor_SetSpeedLimit(Motor_T * p_motor, uint16_t speed_ufract16);
// extern void Motor_SetILimit(Motor_T * p_motor, uint16_t i_ufract16);
extern void Motor_SetSpeedLimitWith(Motor_T * p_motor, LimitArray_T * p_limit);
extern void Motor_SetILimitWith(Motor_T * p_motor, LimitArray_T * p_limit);


// extern void Motor_ResetUnitsVabc(Motor_T * p_motor);
// extern void Motor_ResetUnitsIabc(Motor_T * p_motor);
// extern void Motor_ResetUnitsIa(Motor_T * p_motor);
// extern void Motor_ResetUnitsIb(Motor_T * p_motor);
// extern void Motor_ResetUnitsIc(Motor_T * p_motor);\

extern void Motor_InitSensor(Motor_T * p_motor);

extern angle16_t Motor_PollSensorAngle(Motor_T * p_motor);
extern angle16_t Motor_CaptureAngle(Motor_T * p_motor);

extern bool Motor_PollCaptureSpeed(Motor_T * p_motor);
extern int32_t Motor_PollSensorSpeed(Motor_T * p_motor);
extern angle16_t Motor_GetMechanicalAngle(const Motor_T * p_motor);
extern void Motor_ZeroSensor(Motor_T * p_motor);
extern bool Motor_IsClosedLoop(const Motor_T * p_motor);
extern bool Motor_VerifySensorCalibration(Motor_T * p_motor);

extern void Motor_SetSensorDirection(Motor_T * p_motor, Motor_Direction_T direction);
extern void Motor_ResetUnitsSensor(Motor_T * p_motor);

extern void Motor_ResetUnitsHallEncoder(Motor_T * p_motor);
extern void Motor_ResetUnitsEncoder(Motor_T * p_motor);
extern void Motor_ResetUnitsSinCos(Motor_T * p_motor);
// extern void Motor_ResetUnitsAngleSpeed_Mech(Motor_T * p_motor);
// extern void Motor_ResetUnitsAngleSpeed_ElecControl(Motor_T * p_motor);
#endif
