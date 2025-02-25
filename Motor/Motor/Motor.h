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

#include "MotorAnalog.h"

#include "Math/FOC.h"
#include "Transducer/Phase/Phase.h"
#include "Transducer/Hall/Hall.h"
#include "Transducer/SinCos/SinCos.h"
#include "Transducer/Encoder/Encoder_ModeDT.h"
#include "Transducer/Encoder/Encoder_ISR.h"
#include "Transducer/Thermistor/Thermistor.h"

#include "Utility/StateMachine/StateMachine.h"
#include "Utility/Timer/Timer.h"
#include "Utility/Limit/Limit.h"

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
#define MOTOR_CONTROL_CYCLES(Milliseconds, Freq)    ((uint32_t)((uint64_t)Milliseconds * Freq / 1000U))
#define MOTOR_CONTROL_TIME_MS(Cycles, Freq)         ((uint32_t)((uint64_t)Cycles * 1000U / Freq))
#define MOTOR_CONTROL_DIVIDER(Pow2)                 ((uint32_t)Pow2 - 1U)

/******************************************************************************/
/*
    Global "Static" Const
    These could be implemented as Preprocessor Macros.
*/
/******************************************************************************/
typedef const struct Motor_Static
{
    const uint16_t CONTROL_FREQ;
    // const uint16_t SPEED_FREQ;
    const uint32_t CONTROL_ANALOG_DIVIDER;  /* In Pow2 - 1 */
    const uint16_t V_MAX_VOLTS;             /* VSource Limit */
    const uint32_t V_ABC_R1;                /* VSensor calibration */
    const uint32_t V_ABC_R2;
    const uint16_t I_MAX_AMPS;              /* Motor Controller I rating. Unit conversion reference. User/Config get/set */
    const uint16_t I_PEAK_ADCU;             /* ISensor calibration. I_MAX_ZTP, Zero-To-Peak, derived from sensor hardware */
    const uint16_t ALIGN_VPWM_MAX;
    const uint16_t INIT_WAIT;
    // OpenLoopZcdTransition
}
Motor_Static_T;

/* Define in Main App */
extern const Motor_Static_T MOTOR_STATIC;

extern void Motor_Static_InitVSourceRef_V(uint16_t vSource);
extern void Motor_Static_InitVSourceRef_Adcu(uint16_t vSource_Adcu);
extern uint16_t Motor_Static_GetVSource_V(void);
extern uint16_t Motor_Static_GetVSource_V10(void);
extern uint16_t Motor_Static_GetVSource_Adcu(void);

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
    Sensor Mode Param at start up.
*/
typedef enum Motor_SensorMode
{
    MOTOR_SENSOR_MODE_HALL,
    MOTOR_SENSOR_MODE_ENCODER,
    MOTOR_SENSOR_MODE_SIN_COS,
    MOTOR_SENSOR_MODE_SENSORLESS,
    // MOTOR_SENSOR_MODE_EXTERN,
    // MOTOR_SENSOR_MODE_VTABLE,
}
Motor_SensorMode_T;

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


static const Motor_FeedbackMode_T MOTOR_FEEDBACK_MODE_OPEN_LOOP_SCALAR     = { .OpenLoop = 1U, .Speed = 0U, .Current = 0U, };
static const Motor_FeedbackMode_T MOTOR_FEEDBACK_MODE_OPEN_LOOP_CURRENT    = { .OpenLoop = 1U, .Speed = 0U, .Current = 1U, };
static const Motor_FeedbackMode_T MOTOR_FEEDBACK_MODE_VOLTAGE              = { .OpenLoop = 0U, .Speed = 0U, .Current = 0U, };
static const Motor_FeedbackMode_T MOTOR_FEEDBACK_MODE_CURRENT              = { .OpenLoop = 0U, .Speed = 0U, .Current = 1U, };
static const Motor_FeedbackMode_T MOTOR_FEEDBACK_MODE_SPEED_VOLTAGE        = { .OpenLoop = 0U, .Speed = 1U, .Current = 0U, };
static const Motor_FeedbackMode_T MOTOR_FEEDBACK_MODE_SPEED_CURRENT        = { .OpenLoop = 0U, .Speed = 1U, .Current = 1U, };

static inline Motor_FeedbackMode_T Motor_FeedbackMode_Cast(uint8_t value) { return ((Motor_FeedbackMode_T) { .Value = value }); }


/*
    Effectively sync mailbox for async calculations
*/
typedef union Motor_StateFlags
{
    struct
    {
        uint16_t HeatWarning        : 1U;
        uint16_t ILimited           : 1U;
        uint16_t SpeedLimited       : 1U;
        // uint16_t RampDisable        : 1U;
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
    };
    uint16_t Value;
}
Motor_FaultFlags_T;


/*
    Direction Run SubState
*/
typedef enum Motor_Direction
{
    MOTOR_DIRECTION_CCW,
    MOTOR_DIRECTION_CW,
}
Motor_Direction_T;



/*
    Sub-modules independently conform to same ID
*/
typedef enum Motor_SectorId
{
    MOTOR_SECTOR_ID_0 = 0U,
    MOTOR_SECTOR_ID_1 = 1U,
    MOTOR_SECTOR_ID_2 = 2U,
    MOTOR_SECTOR_ID_3 = 3U,
    MOTOR_SECTOR_ID_4 = 4U,
    MOTOR_SECTOR_ID_5 = 5U,
    MOTOR_SECTOR_ID_6 = 6U,
    MOTOR_SECTOR_ID_7 = 7U,
    MOTOR_SECTOR_ERROR_000 = 0U,
    MOTOR_SECTOR_ERROR_111 = 7U,

    MOTOR_PHASE_ERROR_0 = PHASE_ID_0,
    MOTOR_PHASE_AC = PHASE_ID_1_AC,
    MOTOR_PHASE_BC = PHASE_ID_2_BC,
    MOTOR_PHASE_BA = PHASE_ID_3_BA,
    MOTOR_PHASE_CA = PHASE_ID_4_CA,
    MOTOR_PHASE_CB = PHASE_ID_5_CB,
    MOTOR_PHASE_AB = PHASE_ID_6_AB,
    MOTOR_PHASE_ERROR_7 = PHASE_ID_7,
}
Motor_SectorId_T;

/******************************************************************************/
/*

*/
/******************************************************************************/
#define SPEED_LIMIT_ID_COUNT (2U)
#define I_LIMIT_ID_COUNT (4U)

typedef enum Motor_ILimitId
{
    /* Heat Limits are polling, optionally share 1 Id */
    MOTOR_I_LIMIT_HEAT_THIS,     /* MotorHeat */
    MOTOR_I_LIMIT_HEAT_MC,       /* From upper module */
    MOTOR_I_LIMIT_V_LOW,
    MOTOR_I_LIMIT_USER,
}
Motor_ILimitId_T;

typedef enum Motor_SpeedLimitId
{
    MOTOR_SPEED_LIMIT_MC,    /* From upper module */
    MOTOR_SPEED_LIMIT_USER,
}
Motor_SpeedLimitId_T;

#define MOTOR_SPEED_LIMIT_INIT(p_Motor) LIMIT_INIT((p_Motor)->SpeedLimitBuffer, SPEED_LIMIT_ID_COUNT)
#define MOTOR_I_LIMIT_INIT(p_Motor)     LIMIT_INIT((p_Motor)->ILimitBuffer, I_LIMIT_ID_COUNT)

/*!
    @brief Motor Config - Runtime variable configuration. Load from non volatile memory.
*/
typedef struct Motor_Config
{
    Motor_CommutationMode_T     CommutationMode;
    Motor_SensorMode_T          SensorMode;
    Motor_Direction_T           DirectionForward;

    /*
        Calibration parameters
    */
    uint8_t PolePairs;          /* Motor Pole Pairs. Use to derive Hall/Encoder speed calibration */
    uint16_t Kv;                /* Motor Constant. Use to derive SpeedVRef */
    uint16_t VSpeedScalar_UFract16;     /* Use to derive SpeedVMatchRef, < SpeedFeedbackRef_Rpm to ensure resume control at lower speed. */
                                        /* 9/10 scalar => 100% Speed => 90% V */
                                        /* Scalar in Fract16, speed may exceed 32767 */

    /* replace with getter */
    // uint16_t SpeedFeedbackRef_Rpm;   /* VSource*Kv. Feedback / PID Regulator Limits Ref. Sensor/User Fract16, unit conversion. */
                                        /* A value greater than achievable will cause integral windup. too low will lose PID control range */

    // Motor_ResumeMode_T ResumeMode; // option Scale to VSpeed or VBemf on resume
    uint16_t IaZeroRef_Adcu;
    uint16_t IbZeroRef_Adcu;
    uint16_t IcZeroRef_Adcu;
// #if defined(CONFIG_MOTOR_DEBUG_ENABLE)
    uint16_t IPeakRef_Adcu;
// #endif
    // uint16_t VabcRef_Adcu;

    /* Persistent Base Limits. Same units as control vars */
    uint16_t SpeedLimitForward_Fract16;     /* of SpeedVRef_Rpm */
    uint16_t SpeedLimitReverse_Fract16;
    uint16_t ILimitMotoring_Fract16;        /* of I_MAX_AMPS. */
    uint16_t ILimitGenerating_Fract16;

    uint32_t RampAccel_Cycles;
    uint16_t AlignPower_UFract16;          /* V or I */
    uint32_t AlignTime_Cycles;
// #if defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
    uint16_t OpenLoopSpeed_UFract16;    /*   */
    uint16_t OpenLoopPower_UFract16;    /* V or I */
    uint32_t OpenLoopAccel_Cycles;      /* Time to reach OpenLoopSpeed */
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
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
    SinCos_T SinCos;
#endif
    Phase_T Phase;
    Thermistor_T Thermistor;

    /*
        State and SubStates
    */
    StateMachine_T StateMachine;
    Timer_T ControlTimer;                       /* State Timer */
    uint32_t ControlTimerBase;                  /* Control Freq ~ 20kHz, calibration, commutation, angle control. overflow 20Khz: 59 hours*/
    Motor_Direction_T Direction;                /* Active spin direction. SubState of Run State */
    Motor_FeedbackMode_T FeedbackMode;          /* Active FeedbackMode, Control/Run SubState */
    Motor_FaultFlags_T FaultFlags;              /* Fault SubState */
    Motor_StateFlags_T StateFlags;              /* Can be pushed to modules */

    uint32_t OpenLoopState; /* depreciate */
    uint32_t CalibrationState;  /* SubState, selection for calibration */

    uint8_t CalibrationStateIndex;
    volatile Phase_Bits_T IFlags;
    volatile Phase_Bits_T VFlags;

    /*
        Active Limits
    */
    /* Non directional */
    uint16_t SpeedLimitForward_Fract16; /* May over saturate */
    uint16_t SpeedLimitReverse_Fract16;
    uint16_t ILimitMotoring_Fract16;    /* 32767 Max */
    uint16_t ILimitGenerating_Fract16;

    /* Cached directional limits - on feedback */
    // int16_t SpeedLimitCcw_Fract16;  /* + */
    // int16_t SpeedLimitCw_Fract16;   /* - */
    // int16_t ILimitCcw_Fract16;      /* + */
    // int16_t ILimitCw_Fract16;       /* - */

    /* Sentinel values buffers */
    Limit_T SpeedLimit;
    Limit_T ILimit;
    uint16_t SpeedLimitBuffer[SPEED_LIMIT_ID_COUNT];
    uint16_t ILimitBuffer[I_LIMIT_ID_COUNT];

    /*
        Input
    */
    Ramp_T Ramp;            /* User Input Ramp, UserCmdValue as RampTarget - Speed, Current, or Voltage. Updated without StateMachine check */
                            /* Output [-32767:32767] SetPoint => SpeedReq, IReq, VReq. */

    /*  */
    Ramp_T AuxRamp;             /* OpenLoop and Align,  */
    Ramp_T OpenLoopSpeedRamp;   /* OpenLoopSpeed Ramp */
    // Ramp_T Generating;

    /*

    */
    angle16_t MechanicalAngle;
    angle16_t ElectricalAngle;  /* Angle Feedback. Shared E-Cycle edge detect, User output */

    PID_T PidPosition;    /* todo */

    /*
        Speed Feedback
    */
    int32_t Speed_Fract16;              /* [-32765*2:32765*2] Speed Feedback Variable. Can over saturate. - is virtual CW, + => CCW */
    Timer_T SpeedTimer;                 /* Speed Calc Timer */
    PID_T PidSpeed;                     /* Input PidSpeed(RampCmd - Speed_Fract16), Output [-32768:32767] => VPwm, Vq, Iq. Updated once per ms */

    /*
        FOC
    */
    FOC_T Foc;
    PID_T PidIq;                        /* Input (IqReq - IqFeedback), Output Vq. Sign indicates ccw/cw direction */
    PID_T PidId;

    /*
        Unit Conversions
    */
    Linear_T UnitsIa;           /* Fract16 and Amps */
    Linear_T UnitsIb;
    Linear_T UnitsIc;
    Linear_T UnitsVabc;         /* Vbemf/VPhase Fract16 conversion of adcu */
    Linear_T UnitsVSpeed;       /* VbemfSpeed. Vbemf_Fract16 of Speed_Fract16. Resume voltage, alternative to ADC Vbemf sampling */

    Filter_T FilterA;           /* Calibration use */
    Filter_T FilterB;
    Filter_T FilterC;

    /* Jog */
    uint32_t JogIndex;

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
#endif

    #if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
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

// static const fract16_t MOTOR_V_LIMIT_CCW = INT16_MAX;
// static const fract16_t MOTOR_V_LIMIT_CW = -INT16_MAX;

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
#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
/*
    Internal unit numeric representation conversions only
    Conversion of units handle by sub modules
*/
static inline int32_t _Motor_Speed_Fract16OfRpm(const Motor_T * p_motor, int32_t speed_rpm)     { return speed_rpm * INT16_MAX / Motor_GetSpeedVRef_Rpm(p_motor); }
static inline int16_t _Motor_Speed_RpmOfFract16(const Motor_T * p_motor, int32_t speed_fract16) { return speed_fract16 * Motor_GetSpeedVRef_Rpm(p_motor) / 32768; }
static inline int32_t _Motor_I_Fract16OfAmps(int32_t i_amp)             { return i_amp * INT16_MAX / MOTOR_STATIC.I_MAX_AMPS; }
static inline int16_t _Motor_I_AmpsOfFract16(int32_t i_fract16)         { return i_fract16 * MOTOR_STATIC.I_MAX_AMPS / 32768; }
static inline int32_t _Motor_V_Fract16OfVolts(int32_t v_volts)          { return v_volts * INT16_MAX / Motor_Static_GetVSource_V(); }
static inline int16_t _Motor_V_VoltsOfFract16(int32_t v_fract16)        { return v_fract16 * Motor_Static_GetVSource_V() / 32768; }
static inline int32_t _Motor_Power_WattsOfFract16(int32_t vi_fract16)   { return vi_fract16 * MOTOR_STATIC.I_MAX_AMPS * Motor_Static_GetVSource_V() / 32768; }
#endif


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
        // default: assert(false); break; // error
    }
    return fn;
}

typedef void(*Motor_ProcVoid_T)(Motor_T * p_motor);
typedef void(*Motor_SetUInt32_T)(Motor_T * p_motor, uint32_t value);
typedef void(*Motor_SetUInt16_T)(Motor_T * p_motor, uint16_t value);
typedef void(*Motor_SetUInt8_T)(Motor_T * p_motor, uint8_t value);
typedef int32_t(*Motor_GetInt32_T)(const Motor_T * p_motor);

#define Motor_CommutationModeFn(p_motor, focSet, sixStepSet) \
    _Generic((focSet), \
        Motor_GetInt32_T:   (Motor_GetInt32_T)(_Motor_CommutationModeFn(p_motor, focSet, sixStepSet)),  \
        Motor_SetUInt32_T:  (Motor_SetUInt32_T)(_Motor_CommutationModeFn(p_motor, focSet, sixStepSet)), \
        Motor_SetUInt16_T:  (Motor_SetUInt16_T)(_Motor_CommutationModeFn(p_motor, focSet, sixStepSet)), \
        Motor_SetUInt8_T:   (Motor_SetUInt8_T)(_Motor_CommutationModeFn(p_motor, focSet, sixStepSet)),  \
        Motor_ProcVoid_T:   (Motor_ProcVoid_T)(_Motor_CommutationModeFn(p_motor, focSet, sixStepSet))   \
    )

// c23
// #define Motor_CommutationModeFn(p_motor, focFunction, sixStepFunction) ((typeof(focFunction))(_Motor_CommutationModeFn(p_motor, focFunction, sixStepFunction)))

#define Motor_CommutationModeFn_Call(p_motor, focSet, sixStepSet, ...) (Motor_CommutationModeFn(p_motor, focSet, sixStepSet)(p_motor __VA_OPT__(,) __VA_ARGS__))

/* todo depreciate */
static inline void Motor_ProcCommutationMode(Motor_T * p_motor, Motor_ProcVoid_T focProc, Motor_ProcVoid_T sixStepProc)
    { ((Motor_ProcVoid_T)_Motor_CommutationModeFn(p_motor, focProc, sixStepProc))(p_motor); }

static inline void Motor_SetCommutationModeUInt32(Motor_T * p_motor, Motor_SetUInt32_T focSet, Motor_SetUInt32_T sixStepSet, uint32_t value)
    { ((Motor_SetUInt32_T)_Motor_CommutationModeFn(p_motor, focSet, sixStepSet))(p_motor, value); }

static inline void Motor_SetCommutationModeUInt16(Motor_T * p_motor, Motor_SetUInt16_T focSet, Motor_SetUInt16_T sixStepSet, uint16_t value)
    { ((Motor_SetUInt16_T)_Motor_CommutationModeFn(p_motor, focSet, sixStepSet))(p_motor, value); }

static inline void Motor_SetCommutationModeUInt8(Motor_T * p_motor, Motor_SetUInt8_T focSet, Motor_SetUInt8_T sixStepSet, uint8_t value)
    { ((Motor_SetUInt8_T)_Motor_CommutationModeFn(p_motor, focSet, sixStepSet))(p_motor, value); }

static inline int32_t Motor_GetCommutationModeInt32(const Motor_T * p_motor, Motor_GetInt32_T focGet, Motor_GetInt32_T sixStepGet)
    { return ((Motor_GetInt32_T)_Motor_CommutationModeFn(p_motor, focGet, sixStepGet))(p_motor); }



/******************************************************************************/
/*
    Interrupts
*/
/******************************************************************************/
static inline void Motor_ClearInterrupt(Motor_T * p_motor)     { Phase_ClearInterrupt(&p_motor->Phase); }
/* Controls StateMachine Proc. Local Critical */
static inline void Motor_DisableInterrupt(Motor_T * p_motor)   { Phase_DisableInterrupt(&p_motor->Phase); }
static inline void Motor_EnableInterrupt(Motor_T * p_motor)    { Phase_EnableInterrupt(&p_motor->Phase); }


/******************************************************************************/
/*
    MOTOR_STATIC
*/
/******************************************************************************/
static inline uint32_t _Motor_MillisOf(uint32_t controlCycles) { return controlCycles * 1000U / MOTOR_STATIC.CONTROL_FREQ; }
static inline uint32_t _Motor_ControlCyclesOf(uint32_t millis) { return millis * MOTOR_STATIC.CONTROL_FREQ / 1000U; }

static uint16_t Motor_VSourceLimitOf(uint16_t vSource) { return (vSource < MOTOR_STATIC.V_MAX_VOLTS) ? vSource : MOTOR_STATIC.V_MAX_VOLTS; }
static inline bool Motor_IsAnalogCycle(const Motor_T * p_motor) { return ((p_motor->ControlTimerBase & MOTOR_STATIC.CONTROL_ANALOG_DIVIDER) == 0UL); }

// alternatively cache results SpeedFeedbackRef_Rpm
static inline uint16_t Motor_GetSpeedVRef_Rpm(const Motor_T * p_motor) { return p_motor->Config.Kv * Motor_Static_GetVSource_V(); }
// static inline uint16_t Motor_GetSpeedVRef_Rpm(const Motor_T * p_motor) { return Motor_Static_RpmOfKv(p_motor->Config.Kv); }

static inline uint16_t Motor_GetIPeakRef_Adcu(const Motor_T * p_motor)
{
#if defined(CONFIG_MOTOR_DEBUG_ENABLE)
    return p_motor->Config.IPeakRef_Adcu;
#else
    return MOTOR_STATIC.I_PEAK_ADCU;
#endif
}

/******************************************************************************/
/*

*/
/******************************************************************************/
// for sensor which return a delta angle instead of speed.
static inline uint32_t speed_rpm_of_angle16(uint32_t sampleFreq, uint16_t angle16)      { return (angle16 * sampleFreq >> 16U) * 60U; }
static inline uint32_t speed_angle16_of_rpm(uint32_t sampleFreq, uint16_t rpm)          { return ((uint32_t)rpm << 16U) / (60U * sampleFreq); }

// static inline uint32_t _Motor_RpmOfAngle16(uint16_t angle16)                         { return speed_rpm_of_angle16(MOTOR_STATIC.SPEED_FREQ, angle16); }
// static inline uint32_t _Motor_Angle16OfRpm(uint16_t rpm)                             { return speed_angle16_of_rpm(MOTOR_STATIC.SPEED_FREQ, rpm); }

static inline uint16_t Motor_MechAngleOfSpeed_Fract16(Motor_T * p_motor, int16_t speed_fract16)
    { return ((int32_t)speed_fract16 * 2 * Motor_GetSpeedVRef_Rpm(p_motor)) / (60 * MOTOR_STATIC.CONTROL_FREQ); }

static inline uint32_t Motor_MechSpeedOfAngle_Fract16(Motor_T * p_motor, uint16_t angle16)
    { return ((uint32_t)angle16 * MOTOR_STATIC.CONTROL_FREQ * 60) / (2 * Motor_GetSpeedVRef_Rpm(p_motor)); }

static inline uint16_t Motor_ElectricalAngleOfSpeed(Motor_T * p_motor, int16_t speed_fract16)
    { return ((int32_t)speed_fract16 * 2 * Motor_GetSpeedVRef_Rpm(p_motor) * p_motor->Config.PolePairs) / (60 * MOTOR_STATIC.CONTROL_FREQ); }


/******************************************************************************/
/*

*/
/******************************************************************************/
/*!
    Convert between a user reference direction or non directional, to CCW/CW direction
    @param[in] userCmd int32_t[-65536:65536] fract16 or percent16. positive is forward relative to the user.
    @return int32_t[-65536:65536], Over saturated if input is -32768
    *DirectionComp
*/
static inline int32_t Motor_DirectionalValueOf(const Motor_T * p_motor, int32_t userCmd) { return (p_motor->Direction == MOTOR_DIRECTION_CCW) ? userCmd : (int32_t)0 - userCmd; }
// static inline int32_t Motor_DirectionalValueOf(const Motor_T * p_motor, int32_t userCmd) { return (p_motor->Direction * userCmd); }


/*

*/
static inline Motor_Direction_T Motor_GetDirectionForward(const Motor_T * p_motor) { return p_motor->Config.DirectionForward; }
static inline Motor_Direction_T Motor_GetDirectionReverse(const Motor_T * p_motor) { return ((p_motor->Config.DirectionForward == MOTOR_DIRECTION_CCW) ? MOTOR_DIRECTION_CW : MOTOR_DIRECTION_CCW); }
static inline bool Motor_IsDirectionForward(const Motor_T * p_motor) { return (p_motor->Direction == p_motor->Config.DirectionForward); }
static inline bool Motor_IsDirectionReverse(const Motor_T * p_motor) { return !Motor_IsDirectionForward(p_motor); }
// static inline bool Motor_IsZeroSpeed(const Motor_T * p_motor) { return (p_motor->Speed_Fract16 == 0); }


/******************************************************************************/
/*
    Apply Limits
    Call from Feedback loop
*/
/******************************************************************************/
/*
    Limits getters as abstraction
*/
static inline int32_t _Motor_GetVLimitCcw(const Motor_T * p_motor) { return INT16_MAX; }
static inline int32_t _Motor_GetVLimitCw(const Motor_T * p_motor)  { return (-INT16_MAX); }

static inline int32_t _Motor_GetILimitCcw(const Motor_T * p_motor) { return (p_motor->Direction == MOTOR_DIRECTION_CCW) ? p_motor->ILimitMotoring_Fract16 : p_motor->ILimitGenerating_Fract16; }
static inline int32_t _Motor_GetILimitCw(const Motor_T * p_motor)  { return ((int32_t)0 - ((p_motor->Direction == MOTOR_DIRECTION_CCW) ? p_motor->ILimitGenerating_Fract16 : p_motor->ILimitMotoring_Fract16)); }

static inline int32_t _Motor_GetSpeedLimitCcw(const Motor_T * p_motor)  { return (p_motor->Config.DirectionForward == MOTOR_DIRECTION_CCW) ? p_motor->SpeedLimitForward_Fract16 : p_motor->SpeedLimitReverse_Fract16; }
static inline int32_t _Motor_GetSpeedLimitCw(const Motor_T * p_motor)   { return ((int32_t)0 - ((p_motor->Config.DirectionForward == MOTOR_DIRECTION_CCW) ? p_motor->SpeedLimitReverse_Fract16 : p_motor->SpeedLimitForward_Fract16)); }

//abs
static inline uint16_t Motor_GetSpeedLimit(const Motor_T * p_motor) { return (Motor_IsDirectionForward(p_motor) ? p_motor->SpeedLimitForward_Fract16 : p_motor->SpeedLimitReverse_Fract16); }


/* same units */
static inline int16_t Motor_SpeedReqLimitOf(const Motor_T * p_motor, int16_t req)   { return math_clamp(req, _Motor_GetSpeedLimitCw(p_motor), _Motor_GetSpeedLimitCcw(p_motor)); }
static inline int16_t Motor_IReqLimitOf(const Motor_T * p_motor, int16_t req)       { return math_clamp(req, _Motor_GetILimitCw(p_motor), _Motor_GetILimitCcw(p_motor)); }


/*
    Apply limit as scalar, req = limit_scalar/feedback
*/
/*
    Req may be I, or V
*/
// SpeedLimit with SpeedPid disabled
// alternatively enable PID // MatchOutput
// ScaleWithSpeedLimit
static inline int16_t Motor_ReqOfSpeedLimit(const Motor_T * p_motor, int16_t req)
{
    /* fract16_div always return positive < 1 */
    // if      (p_motor->Speed_Fract16 < p_motor->SpeedLimitCw_Fract16)  { limitedReq = 0 - fract16_div(p_motor->SpeedLimitCw_Fract16, p_motor->Speed_Fract16); } /* Speed is more negative */
    // else if (p_motor->Speed_Fract16 > p_motor->SpeedLimitCcw_Fract16) { limitedReq = fract16_div(p_motor->SpeedLimitCcw_Fract16, p_motor->Speed_Fract16); }

    uint32_t speedLimit = Motor_GetSpeedLimit(p_motor);
    uint32_t speed = math_abs(p_motor->Speed_Fract16);
    int16_t limitedReq = req;

    if (speed > speedLimit) { limitedReq = (int32_t)req * speedLimit / speed; }

    return limitedReq;
}


/******************************************************************************/
/*

*/
/******************************************************************************/
/*!
    V of Speed
    VPhase approximation via Speed
        => Speed_Fract16 * SpeedMatchRef_Rpm / SpeedFeedbackRef_Rpm

    @return [0:65535] <=> [0:2) Unsaturated output
*/
static inline int32_t Motor_GetVSpeed_Fract16(const Motor_T * p_motor) { return p_motor->Speed_Fract16 * p_motor->Config.VSpeedScalar_UFract16 >> 15; }


/******************************************************************************/
/*

*/
/******************************************************************************/
static inline void Motor_EnableRamp(Motor_T * p_motor) { Ramp_SetSlope(&p_motor->Ramp, p_motor->Config.RampAccel_Cycles, INT16_MAX); }
static inline void Motor_DisableRamp(Motor_T * p_motor) { _Ramp_Disable(&p_motor->Ramp); }

/******************************************************************************/
/*!
    Extern
*/
/******************************************************************************/
extern void Motor_Init(Motor_T * p_motor);
extern void Motor_InitReboot(Motor_T * p_motor);
extern void Motor_InitSensor(Motor_T * p_motor);

extern bool Motor_VerifySensorCalibration(Motor_T * p_motor);

extern angle16_t Motor_PollSensorAngle(Motor_T * p_motor);
extern bool Motor_PollCaptureSpeed(Motor_T * p_motor);
extern void Motor_ProcSpeedFeedback(Motor_T * p_motor);
extern int32_t Motor_PollSensorSpeed(Motor_T * p_motor);
extern angle16_t Motor_GetMechanicalAngle(const Motor_T * p_motor);

extern void Motor_ZeroSensor(Motor_T * p_motor);
extern bool Motor_IsClosedLoopStart(const Motor_T * p_motor);

extern void Motor_UpdateSpeedControlLimits(Motor_T * p_motor);
extern void Motor_SetFeedbackMode(Motor_T * p_motor, Motor_FeedbackMode_T mode);
extern void Motor_SetFeedbackMode_Cast(Motor_T * p_motor, uint8_t modeValue);

extern void Motor_SetSpeedLimit(Motor_T * p_motor, uint16_t speed_ufract16);
extern void Motor_ClearSpeedLimit(Motor_T * p_motor);
extern void Motor_SetILimit(Motor_T * p_motor, uint16_t i_ufract16);
extern void Motor_ClearILimit(Motor_T * p_motor);

extern bool Motor_SetSpeedLimitEntry(Motor_T * p_motor, uint8_t id, uint16_t speed_ufract16);
extern bool Motor_SetSpeedLimitEntry_Scalar(Motor_T * p_motor, uint8_t id, uint16_t scalar_ufract16);
extern bool Motor_ClearSpeedLimitEntry(Motor_T * p_motor, uint8_t id);

extern bool Motor_SetILimitMotoringEntry(Motor_T * p_motor, uint8_t id, uint16_t i_ufract16);
extern bool Motor_SetILimitMotoringEntry_Scalar(Motor_T * p_motor, uint8_t id, uint16_t scalar_ufract16);
extern bool Motor_ClearILimitMotoringEntry(Motor_T * p_motor, uint8_t id);

extern bool Motor_TrySystemSpeedLimit(Motor_T * p_motor, uint16_t fract16);
extern bool Motor_ClearSystemSpeedLimit(Motor_T * p_motor);
extern bool Motor_TrySystemILimit(Motor_T * p_motor, uint16_t fract16);
extern bool Motor_ClearSystemILimit(Motor_T * p_motor);

extern void Motor_SetDirectionCcw(Motor_T * p_motor);
extern void Motor_SetDirectionCw(Motor_T * p_motor);
extern void Motor_SetDirection(Motor_T * p_motor, Motor_Direction_T direction);
extern void Motor_SetDirection_Cast(Motor_T * p_motor, uint8_t direction);
extern void Motor_SetDirectionForward(Motor_T * p_motor);
extern void Motor_SetDirectionReverse(Motor_T * p_motor);

extern void Motor_ResetSpeedLimitActive(Motor_T * p_motor);
extern void Motor_ResetILimitActive(Motor_T * p_motor);
extern void Motor_ResetUnitsSensor(Motor_T * p_motor);
extern void Motor_ResetUnitsVabc(Motor_T * p_motor);
extern void Motor_ResetUnitsIabc(Motor_T * p_motor);
extern void Motor_ResetUnitsIa(Motor_T * p_motor);
extern void Motor_ResetUnitsIb(Motor_T * p_motor);
extern void Motor_ResetUnitsIc(Motor_T * p_motor);
extern void Motor_ResetUnitsHallEncoder(Motor_T * p_motor);
extern void Motor_ResetUnitsEncoder(Motor_T * p_motor);
extern void Motor_ResetUnitsSinCos(Motor_T * p_motor);
extern void Motor_ResetUnitsAngleSpeed_Mech(Motor_T * p_motor);
extern void Motor_ResetUnitsAngleSpeed_ElecControl(Motor_T * p_motor);

extern void Motor_Jog12Step(Motor_T * p_motor, uint8_t step);
extern void Motor_Jog6PhaseStep(Motor_T * p_motor, uint8_t step);
extern void Motor_Jog6Step(Motor_T * p_motor, uint8_t step);
extern void Motor_Jog12(Motor_T * p_motor);
extern void Motor_Jog6Phase(Motor_T * p_motor);
extern void Motor_Jog6(Motor_T * p_motor);

#endif

// alternatively derived values as base
// static inline uint16_t Motor_User_GetSpeedLimitForward(const Motor_T * p_motor) { return (p_motor->Config.DirectionForward == MOTOR_DIRECTION_CCW) ? p_motor->SpeedLimitCcw_Fract16 : (0 - p_motor->SpeedLimitCw_Fract16); }
// static inline uint16_t Motor_User_GetSpeedLimitReverse(const Motor_T * p_motor) { return (p_motor->Config.DirectionForward == MOTOR_DIRECTION_CCW) ? (0 - p_motor->SpeedLimitCw_Fract16) : p_motor->SpeedLimitCcw_Fract16; }
// static inline uint16_t Motor_User_GetILimitMotoring(const Motor_T * p_motor)    { return (p_motor->Direction == MOTOR_DIRECTION_CCW) ? p_motor->ILimitCcw_Fract16 : (0 - p_motor->ILimitCw_Fract16); }
// static inline uint16_t Motor_User_GetILimitGenerating(const Motor_T * p_motor)  { return (p_motor->Direction == MOTOR_DIRECTION_CCW) ? (0 - p_motor->ILimitCw_Fract16) : p_motor->ILimitCcw_Fract16; }

// static inline int16_t Motor_User_SpeedCmdLimitOf(const Motor_T * p_motor, int32_t speed_Fract16)
// {
//     return math_clamp(speed_Fract16, 0, Motor_User_GetSpeedLimit(p_motor));
// };

// static inline int16_t Motor_User_ICmdLimitOf(const Motor_T * p_motor, int16_t i_Fract16)
// {
//     return math_clamp(i_Fract16, (int32_t)0 - Motor_User_GetILimitGenerating(p_motor), Motor_User_GetILimitMotoring(p_motor));
// };



// static inline int16_t Motor_SpeedReqLimitOf(const Motor_T * p_motor, int16_t req)   { return math_clamp(req, p_motor->SpeedLimitCw_Fract16, p_motor->SpeedLimitCcw_Fract16); };
// static inline int16_t Motor_IReqLimitOf(const Motor_T * p_motor, int16_t req)       { return math_clamp(req, p_motor->ILimitCw_Fract16, p_motor->ILimitCcw_Fract16); };
