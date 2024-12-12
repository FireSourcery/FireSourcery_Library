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

#include "Math/Q/Q.h"
#include "Math/Linear/Linear_ADC.h"
#include "Math/Linear/Linear_Voltage.h"
#include "Math/Linear/Linear_Ramp.h"
#include "Math/Linear/Linear.h"
#include "Math/PID/PID.h"
#include "Math/Filter/Filter_MovAvg.h"
#include "Math/Filter/Filter.h"

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>


// #define MOTOR_CONTROL_DIVIDER(Pow2) ((uint32_t)Pow2 - 1U)
#define MOTOR_CONTROL_CYCLES(MilliSeconds, Freq)    ((uint32_t)((uint64_t)MilliSeconds * Freq / 1000U))
#define MOTOR_CONTROL_TIME_MS(Cycles, Freq)         ((uint32_t)((uint64_t)Cycles * 1000U / Freq))

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
    const uint16_t V_MAX_VOLTS;         /* VSource Limit */
    const uint32_t V_ABC_R1;            /* VSensor calibration */
    const uint32_t V_ABC_R2;
    const uint16_t I_MAX_AMPS;          /* Motor Controller I rating. Unit conversion reference. User/Config get/set */
    const uint16_t I_MAX_ADCU;          /* ISensor calibration. I_MAX_ZTP, Zero-To-Peak, derived from sensor hardware */
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
    MOTOR_SENSOR_MODE_EXTERN,
    MOTOR_SENSOR_MODE_VTABLE,
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
        uint8_t OpenLoop   : 1U;   /* 0 -> Position feedback, 1 -> OpenLoop */
        uint8_t Speed      : 1U;   /* 0 -> Voltage or Current only, 1 -> Speed feedback */
        uint8_t Current    : 1U;   /* 0 -> Voltage, 1-> Current */
        uint8_t Position   : 1U;
    };
    uint8_t Word; /* Id */
}
Motor_FeedbackMode_T;

static const Motor_FeedbackMode_T MOTOR_FEEDBACK_MODE_OPEN_LOOP_SCALAR     = { .OpenLoop = 1U, .Speed = 0U, .Current = 0U, };
static const Motor_FeedbackMode_T MOTOR_FEEDBACK_MODE_OPEN_LOOP_CURRENT    = { .OpenLoop = 1U, .Speed = 0U, .Current = 1U, };
static const Motor_FeedbackMode_T MOTOR_FEEDBACK_MODE_VOLTAGE              = { .OpenLoop = 0U, .Speed = 0U, .Current = 0U, };
static const Motor_FeedbackMode_T MOTOR_FEEDBACK_MODE_CURRENT              = { .OpenLoop = 0U, .Speed = 0U, .Current = 1U, };
static const Motor_FeedbackMode_T MOTOR_FEEDBACK_MODE_SPEED_VOLTAGE        = { .OpenLoop = 0U, .Speed = 1U, .Current = 0U, };
static const Motor_FeedbackMode_T MOTOR_FEEDBACK_MODE_SPEED_CURRENT        = { .OpenLoop = 0U, .Speed = 1U, .Current = 1U, };

// static inline Motor_FeedbackMode_T Motor_FeedbackMode_Cast(uint8_t word) { Motor_FeedbackMode_T flags = { .Word = word }; return flags; }
static inline Motor_FeedbackMode_T Motor_FeedbackMode_Cast(uint8_t word) { return ((Motor_FeedbackMode_T) { .Word = word }); }

typedef union Motor_ABC
{
    struct
    {
        uint8_t A : 1U;
        uint8_t B : 1U;
        uint8_t C : 1U;
    };
    uint8_t Value;
}
Motor_ABC_T;

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
        // uint16_t SensorFeedback  : 1U;
        // uint16_t Hold        : 1U;
        // uint16_t Control     : 1U;
        // uint16_t Release     : 1U;
    };
    uint16_t Word;
}
Motor_StateFlags_T;

typedef union
{
    struct
    {
        uint16_t Overheat       : 1U;
        uint16_t AlignStartUp   : 1U;
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

*/
typedef enum Motor_OpenLoopState
{
    MOTOR_OPEN_LOOP_STATE_ALIGN,
    MOTOR_OPEN_LOOP_STATE_VALIDATE_ALIGN,
    MOTOR_OPEN_LOOP_STATE_RUN,
}
Motor_OpenLoopState_T;

/*
    Calibration SubState
*/
typedef enum Motor_CalibrationState
{
    MOTOR_CALIBRATION_STATE_DISABLE,
    MOTOR_CALIBRATION_STATE_ADC,
    MOTOR_CALIBRATION_STATE_HALL,
    MOTOR_CALIBRATION_STATE_ENCODER,
    MOTOR_CALIBRATION_STATE_SIN_COS,
    MOTOR_CALIBRATION_STATE_POSITION_SENSOR,
}
Motor_CalibrationState_T;

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
#define I_LIMIT_ID_COUNT (3U)

typedef enum Motor_ILimitActiveId
{
    MOTOR_I_LIMIT_ACTIVE_HEAT_THIS,     /* MotorHeat */
    MOTOR_I_LIMIT_ACTIVE_MC,            /* From upper module */
    MOTOR_I_LIMIT_ACTIVE_USER,
}
Motor_ILimitActiveId_T;

typedef enum Motor_SpeedLimitActiveId
{
    MOTOR_SPEED_LIMIT_ACTIVE_MC, /* From upper module */
    MOTOR_SPEED_LIMIT_ACTIVE_USER,
}
Motor_SpeedLimitActiveId_T;

#define MOTOR_SPEED_LIMIT_INIT(p_Motor) LIMIT_INIT((p_Motor)->SpeedLimitBuffer, SPEED_LIMIT_ID_COUNT)
#define MOTOR_I_LIMIT_INIT(p_Motor)     LIMIT_INIT((p_Motor)->ILimitBuffer, I_LIMIT_ID_COUNT)

/*!
    @brief Motor Config - Runtime variable configuration. Load from non volatile memory.
*/
typedef struct Motor_Config
{
    Motor_CommutationMode_T     CommutationMode;
    Motor_SensorMode_T          SensorMode;
    Motor_FeedbackMode_T        FeedbackModeDefault;     /* Default FeedbackMode */
    Motor_Direction_T           DirectionForward;
    uint8_t PolePairs;          /* Motor Pole Pairs. Use to derive Hall/Encoder speed calibration */
    uint16_t Kv;                /* Motor Constant. Use to derive SpeedRef */

    /*
        Ref values, calibration parameters
    */
    uint16_t SpeedFeedbackRef_Rpm;  /* VSource*Kv. Feedback / PID Regulator Limits Ref. Sensor/User Frac16, unit conversion. */
                                        /* A value greater than achievable will cause integral windup. too low will lose PID control range */
    uint16_t SpeedMatchRef_Rpm;     /* Must be < SpeedFeedbackRef_Rpm to ensure resume control at lower speed. */
    // Motor_ResumeMode_T    ResumeMode; // option Scale to VSpeed or VBemf on resume
    uint16_t IaZeroRef_Adcu;
    uint16_t IbZeroRef_Adcu;
    uint16_t IcZeroRef_Adcu;
// #if defined(CONFIG_MOTOR_DEBUG_ENABLE)
    uint16_t IPeakRef_Adcu;
// #endif

    /* Base Limits. Persistent User Param. */
    uint16_t SpeedLimitForward_Scalar16;   /* Frac16 of SpeedFeedbackRef_Rpm */
    uint16_t SpeedLimitReverse_Scalar16;
    uint16_t ILimitMotoring_Scalar16;      /* Frac16 of RefMax I_MAX_AMPS. ILimit must be < 100% to account for I_Frac saturation. */
    uint16_t ILimitGenerating_Scalar16;

    uint32_t RampAccel_Cycles;

    uint16_t AlignPower_Scalar16;          /* V or I */
    uint32_t AlignTime_Cycles;

// #if defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
    uint16_t OpenLoopSpeed_Scalar16;    /* Max, 65536 will overflow */
    uint16_t OpenLoopPower_Scalar16;    /* V or I */
    uint32_t OpenLoopAccel_Cycles;      /* Time to reach OpenLoopSpeed */
    // uint16_t OpenLoopGain_VHz;
// #endif

#if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
    Phase_Mode_T PhasePwmMode;     /* Only 1 nvm param for phase module. */
#endif

#if defined(CONFIG_MOTOR_UNIT_CONVERSION_LOCAL) && defined(CONFIG_MOTOR_SURFACE_SPEED_ENABLE)
    uint16_t SurfaceDiameter;
    uint16_t GearRatio_Factor;
    uint16_t GearRatio_Divisor;
#endif
}
Motor_Config_T;

/*!
    @brief Motor Const - Compile time const configuration. Per instance.
    program, meta, parameters. unrelated to end user.
*/
typedef const struct Motor_Const
{
    Analog_T * const P_ANALOG;
    const MotorAnalog_Conversions_T ANALOG_CONVERSIONS;
    const Motor_Config_T * const P_NVM_CONFIG;
}
Motor_Const_T;

typedef struct Motor
{
    const Motor_Const_T CONST;
    Motor_Config_T Config;

    volatile MotorAnalog_Results_T AnalogResults;
    // MotorAnalog_ChannelGroup_T AnalogGroup;

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
    Motor_StateFlags_T StateFlags;            /* Can be pushed to modules */
    Motor_OpenLoopState_T OpenLoopState;
    Motor_CalibrationState_T CalibrationState;  /* SubState, selection for calibration */
    uint8_t CalibrationStateIndex;

    /*
        Active Limits
    */
    Limit_T SpeedLimit;
    Limit_T ILimit;
    uint16_t SpeedLimitBuffer[SPEED_LIMIT_ID_COUNT];
    uint16_t ILimitBuffer[I_LIMIT_ID_COUNT];            /* Fixed buffer for 3 values, internal thermistor, user, upper layer */
    // // Motor_SpeedLimitActiveId_T SpeedLimitActiveId;
    // uint16_t SpeedLimitDirect_Scalar16;             /* Frac16 of SpeedRefMax Param. Compare on set */
    // // Motor_ILimitActiveId_T ILimitActiveId;
    // uint16_t ILimitActiveSentinel_Scalar16;         /* Compare on set */

    /* Non directional scalars */
    uint16_t SpeedLimitForward_Scalar16;
    uint16_t SpeedLimitReverse_Scalar16;
    uint16_t ILimitMotoring_Scalar16;
    uint16_t ILimitGenerating_Scalar16;

    /* Active directional limits - on feedback */
    int16_t SpeedLimitCcw_Frac16;
    int16_t SpeedLimitCw_Frac16;
    int16_t ILimitCcw_Frac16;
    int16_t ILimitCw_Frac16;

    /*
        Input
    */
    Linear_T Ramp;          /* User Input Ramp, UserCmdValue as RampTarget - Speed, Current, or Voltage. Updated without StateMachine check */
                            /* Output [-32767:32767] SetPoint => SpeedReq, IReq, VReq. */
    Linear_T AuxRamp;       /* OpenLoop and Align */

    /*

    */
    qangle16_t MechanicalAngle;
    qangle16_t ElectricalAngle;         /* Angle Feedback. Shared E-Cycle edge detect, User output */

    PID_T PidPosition;    /* todo */

    /*
        Speed Feedback
    */
    int32_t Speed_Frac16;              /* [-32765*2:32765*2] Speed Feedback Variable. Can over saturate. - is virtual CW, + => CCW */
    Timer_T SpeedTimer;                 /* Speed Calc Timer */
    PID_T PidSpeed;                     /* Input PidSpeed(RampCmd - Speed_Frac16), Output [-32768:32767] => VPwm, Vq, Iq. Updated once per ms */

    /*
        FOC
    */
    FOC_T Foc;
    PID_T PidIq;                        /* Input (IqReq - IqFeedback), Output Vq. Sign indicates ccw/cw direction */
    PID_T PidId;

    /*
        Unit Conversions
    */
    Linear_T UnitsIa;           /* Frac16 and Amps */
    Linear_T UnitsIb;
    Linear_T UnitsIc;
    Linear_T UnitsVabc;         /* Vbemf/VPhase Frac16 conversion of adcu */
    // Linear_T UnitsVSource_V;    /* VSource Volts of ADCU */
    Linear_T UnitsVSpeed;       /* VbemfSpeed. Vbemf_Frac16 of Speed_Frac16. Resume voltage, alternative to ADC Vbemf sampling */

    Filter_T FilterA;           /* Calibration use */
    Filter_T FilterB;
    Filter_T FilterC;

    /* Jog */
    uint32_t JogIndex;



    /* Sensorless and SinCos. Non square wave Encoder module */
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE) || defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
    Linear_T UnitsAngleSpeed;           /*  */
    Linear_T UnitsSurfaceSpeed;         /*  */
    qangle16_t SpeedAngle;              /* Save for reference, MechanicalDelta */
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
    uint32_t IBus_Frac16;
    uint16_t VPwm;     /* Six-Step Control Variable */
    uint16_t VBemfPeak_Adcu;
    uint16_t VBemfPeakTemp_Adcu;
    uint16_t IPhasePeak_Adcu;
    uint16_t IPhasePeakTemp_Adcu;
#endif

    /*
        OpenLoop speed ramp
    */
// #if defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE) || defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
    Linear_T OpenLoopSpeedRamp;            /* OpenLoopSpeed Ramp */
    #if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
    uint32_t OpenLoopCommutationPeriod;
    #endif
// #endif

#if  defined(CONFIG_MOTOR_DEBUG_ENABLE)
    volatile uint32_t MicrosRef;
    // volatile bool DebugFlag;
    // volatile uint32_t DebugError;
    volatile uint32_t DebugTime[10U];
    volatile uint32_t DebugCounter;
    volatile Motor_ABC_T PhaseFlags;
#endif
}
Motor_T, * Motor_Ptr;



/******************************************************************************/
/*
    Number formats

    Frac16      [-1:1) <=> [-32768:32767] in Q1.15
    Accum32/Frac16 => may over saturate to [-2:2) <=> [-65536:65536]
    UFrac16     [0:2) <=> [0:65535] in Q1.15
    Percent16   [0:1) <=> [0:65535] in Q0.16
    Fixed32     [-1:1] <=> [-65536:65536] in Q16.16     Max [INT32_MIN:INT32_MAX],
    Fixed16     [-1:1] <=> [-256:256] in Q8.8           Max [-32768:32767],
*/
/******************************************************************************/
#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
/* Internal unit representations only */
static inline int32_t _Motor_ConvertSpeed_RpmToFrac16(const Motor_T * p_motor, int32_t speed_rpm)        { return speed_rpm * INT16_MAX / p_motor->Config.SpeedFeedbackRef_Rpm; }
static inline int16_t _Motor_ConvertSpeed_Frac16ToRpm(const Motor_T * p_motor, int32_t speed_frac16)     { return speed_frac16 * p_motor->Config.SpeedFeedbackRef_Rpm / 32768; }
static inline int32_t _Motor_ConvertI_AmpsToIFrac16(int32_t i_amp)                                 { return i_amp * INT16_MAX / MOTOR_STATIC.I_MAX_AMPS; }
static inline int16_t _Motor_ConvertI_Frac16ToAmps(int32_t i_frac16)                               { return i_frac16 * MOTOR_STATIC.I_MAX_AMPS / 32768; }
static inline int32_t _Motor_ConvertV_VoltsToFrac16(int32_t v_volts)                               { return v_volts * INT16_MAX / Motor_Static_GetVSource_V(); }
static inline int16_t _Motor_ConvertV_Frac16ToVolts(int32_t v_frac16)                              { return v_frac16 * Motor_Static_GetVSource_V() / 32768; }
static inline int32_t _Motor_ConvertPower_Frac16ToWatts(int32_t vi_frac16)                         { return vi_frac16 * MOTOR_STATIC.I_MAX_AMPS * Motor_Static_GetVSource_V() / 32768; }

// for sensor which report an delta angle instead of speed.
// static inline uint32_t speed_rpm_of_angle16(uint32_t sampleFreq, uint16_t angle16)      { return (angle16 * sampleFreq >> 16U) * 60U; }
// static inline uint32_t speed_angle16_of_rpm(uint32_t sampleFreq, uint16_t rpm)          { return ((uint32_t)rpm << 16U) / (60U * sampleFreq); }
// static inline uint32_t _Motor_RpmOfMechAngle16(uint16_t angle16)                        { return speed_rpm_of_angle16(MOTOR_STATIC.SPEED_FREQ, angle16); }
// static inline uint32_t _Motor_Angle16OfRpm(uint16_t rpm)                                { return speed_angle16_of_rpm(MOTOR_STATIC.SPEED_FREQ, rpm); }
#endif



/******************************************************************************/
/*
    Simplify CommutationMode Check
    These function should optimize away select if only 1 mode is enabled
*/
/******************************************************************************/
#if     defined(CONFIG_MOTOR_FOC_ENABLE) && !defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
#define _Motor_CommutationModeFn(p_motor, focFunction, sixStepFunction) (focFunction)
#elif   !defined(CONFIG_MOTOR_FOC_ENABLE) && defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
#define _Motor_CommutationModeFn(p_motor, focFunction, sixStepFunction) (sixStepFunction)
#else
static inline const void * _Motor_CommutationModeFn(const Motor_T * p_motor, const void * focFunction, const void * sixStepFunction)
{
    const void * fn;
    switch(p_motor->Config.CommutationMode)
    {
#if     defined(CONFIG_MOTOR_FOC_ENABLE)
        case MOTOR_COMMUTATION_MODE_FOC: fn = focFunction; break;
#endif
#if     defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
        case MOTOR_COMMUTATION_MODE_SIX_STEP: fn = sixStepFunction; break;
#endif
        default: assert(false); // error
    }
    return fn;
}
#endif

typedef void(*Motor_ProcVoid_T)(Motor_T * p_motor);
typedef int32_t(*Motor_GetInt32_T)(const Motor_T * p_motor);
typedef void(*Motor_SetUInt32_T)(Motor_T * p_motor, uint32_t value);
typedef void(*Motor_SetUInt8_T)(Motor_T * p_motor, uint8_t value);
typedef void(*Motor_SetUInt16_T)(Motor_T * p_motor, uint16_t value);

static inline void Motor_ProcCommutationMode(Motor_T * p_motor, Motor_ProcVoid_T focProc, Motor_ProcVoid_T sixStepProc)
    { ((Motor_ProcVoid_T)_Motor_CommutationModeFn(p_motor, focProc, sixStepProc))(p_motor); }

static inline int32_t Motor_GetCommutationModeInt32(const Motor_T * p_motor, Motor_GetInt32_T focGet, Motor_GetInt32_T sixStepGet)
    { return ((Motor_GetInt32_T)_Motor_CommutationModeFn(p_motor, focGet, sixStepGet))(p_motor); }

static inline void Motor_SetCommutationModeUInt32(Motor_T * p_motor, Motor_SetUInt32_T focSet, Motor_SetUInt32_T sixStepSet, uint32_t value)
    { ((Motor_SetUInt32_T)_Motor_CommutationModeFn(p_motor, focSet, sixStepSet))(p_motor, value); }

static inline void Motor_SetCommutationModeUInt8(Motor_T * p_motor, Motor_SetUInt8_T focSet, Motor_SetUInt8_T sixStepSet, uint8_t value)
    { ((Motor_SetUInt8_T)_Motor_CommutationModeFn(p_motor, focSet, sixStepSet))(p_motor, value); }

static inline void Motor_SetCommutationModeUInt16(Motor_T * p_motor, Motor_SetUInt16_T focSet, Motor_SetUInt16_T sixStepSet, uint16_t value)
    { ((Motor_SetUInt16_T)_Motor_CommutationModeFn(p_motor, focSet, sixStepSet))(p_motor, value); }

static inline uint16_t Motor_GetIPeakRef_Adcu(Motor_T * p_motor)
{
#if defined(CONFIG_MOTOR_DEBUG_ENABLE)
    return p_motor->Config.IPeakRef_Adcu;
#else
    return MOTOR_STATIC.I_MAX_ADCU;
#endif
}


/******************************************************************************/
/*
    Interrupts
*/
/******************************************************************************/
static inline void Motor_ClearInterrupt(Motor_T * p_motor)     { Phase_ClearInterrupt(&p_motor->Phase); }
/* Controls StateMachine Proc. Local Critical */
static inline void Motor_DisableInterrupt(Motor_T * p_motor)   { Phase_DisableInterrupt(&p_motor->Phase); }
static inline void Motor_EnableInterrupt(Motor_T * p_motor)    { Phase_EnableInterrupt(&p_motor->Phase); }

static inline void Motor_DisableOutput(Motor_T * p_motor)      { Phase_Float(&p_motor->Phase); }


/******************************************************************************/
/*
*/
/******************************************************************************/
static inline uint32_t _Motor_MillisOf(uint32_t controlCycles) { return controlCycles * 1000U / MOTOR_STATIC.CONTROL_FREQ; }
static inline uint32_t _Motor_ControlCyclesOf(uint32_t millis) { return millis * MOTOR_STATIC.CONTROL_FREQ / 1000U; }
static uint16_t Motor_VSourceLimitOf(uint16_t vSource) { return math_clamp(vSource, 0U, MOTOR_STATIC.V_MAX_VOLTS); }
static inline bool Motor_IsAnalogCycle(const Motor_T * p_motor) { return ((p_motor->ControlTimerBase & MOTOR_STATIC.CONTROL_ANALOG_DIVIDER) == 0UL); }

/* clears fault of type adc reading */
static inline void Motor_PollAdcFaultFlags(Motor_T * p_motor)  { p_motor->FaultFlags.Overheat = Thermistor_IsFault(&p_motor->Thermistor); }
static inline Motor_Direction_T Motor_DirectionForward(const Motor_T * p_motor) { return p_motor->Config.DirectionForward; }
static inline Motor_Direction_T Motor_DirectionReverse(const Motor_T * p_motor) { return ((p_motor->Config.DirectionForward == MOTOR_DIRECTION_CCW) ? MOTOR_DIRECTION_CW : MOTOR_DIRECTION_CCW); }

/******************************************************************************/
/*
    Main Cmd value
*/
/******************************************************************************/
/*!
    Convert between user reference direction to CCW/CW direction
    @param[in] userCmd int32_t[-65536:65536] frac16 or scalar16. positive is forward relative to the user.
    @return int32_t[-65536:65536], Over saturated if input is -32768
*/
static inline int32_t Motor_DirectionalValueOf(const Motor_T * p_motor, int32_t userCmd) { return (p_motor->Direction == MOTOR_DIRECTION_CCW) ? userCmd : (int32_t)0 - userCmd; }

/* Getters satisfy generic use. Setters are specific to control mode. */
static inline int32_t Motor_GetCmd(const Motor_T * p_motor)         { return Motor_DirectionalValueOf(p_motor, Linear_Ramp_GetTarget(&p_motor->Ramp)); }
static inline int32_t Motor_GetSetPoint(const Motor_T * p_motor)    { return Motor_DirectionalValueOf(p_motor, Linear_Ramp_GetOutput(&p_motor->Ramp)); }

/*
    Bypasses User Mode restrictions, bounds
    Concurrency note: only 1 thread updates RampTarget. StateMachine Proc thread only updates OutputState
*/
static inline void Motor_SetCmd(Motor_T * p_motor, int16_t userCmd) { Linear_Ramp_SetTarget(&p_motor->Ramp, Motor_DirectionalValueOf(p_motor, userCmd)); }
// static inline void Motor_ClearCmdState(Motor_T * p_motor) { Linear_Ramp_SetOutputState(&p_motor->Ramp, 0); }

/******************************************************************************/
/*
    Apply Limits
    Call from Feedback loop
*/
/******************************************************************************/
static inline int16_t Motor_SpeedReqLimitOf(const Motor_T * p_motor, int16_t req)   { return math_clamp(req, p_motor->SpeedLimitCw_Frac16, p_motor->SpeedLimitCcw_Frac16); };
static inline int16_t Motor_IReqLimitOf(const Motor_T * p_motor, int16_t req)       { return math_clamp(req, p_motor->ILimitCw_Frac16, p_motor->ILimitCcw_Frac16); };

/*
    Apply limit scalar,
    or replace request with scalar, req = limit/feedback
    todo set is active flag
*/
/*
    Req may be I, or V
*/
// alternatively enable PID // MatchOutput
// p_motor->FeedbackMode.Speed = 1U;
// ScaleWithSpeedLimit
static inline int16_t Motor_ReqOfSpeedLimit(const Motor_T * p_motor, int16_t req)
{
    int16_t limitedReq;
    /* qfrac16_div always return positive < 1 */
    if      (p_motor->Speed_Frac16 < p_motor->SpeedLimitCw_Frac16)  { limitedReq = 0 - qfrac16_div(p_motor->SpeedLimitCw_Frac16, p_motor->Speed_Frac16); } /* Speed is more negative */
    else if (p_motor->Speed_Frac16 > p_motor->SpeedLimitCcw_Frac16) { limitedReq = qfrac16_div(p_motor->SpeedLimitCcw_Frac16, p_motor->Speed_Frac16); }
    else                                                            { limitedReq = req; }
    return limitedReq;
}

// alternatively enable PID // MatchOutput
// p_motor->FeedbackMode.Current = 1U;
// PID_SetOutputState(&p_motor->PidIq, FOC_GetVq(&p_motor->Foc));
// PID_SetOutputState(&p_motor->PidId, FOC_GetVd(&p_motor->Foc));
static inline int16_t Motor_VReqOfILimit(const Motor_T * p_motor, int32_t feedback, int16_t req)
{
    int16_t limitedReq;
    if      (feedback < p_motor->ILimitCw_Frac16)   { limitedReq = 0 - qfrac16_div(p_motor->ILimitCw_Frac16, feedback); }
    else if (feedback > p_motor->ILimitCcw_Frac16)  { limitedReq = qfrac16_div(p_motor->ILimitCcw_Frac16, feedback); }
    else                                            { limitedReq = req; }
    return limitedReq;
};


/******************************************************************************/
/* */
/******************************************************************************/
/* Call from StateMachine only */
static inline void Motor_SetFeedbackMode(Motor_T * p_motor, Motor_FeedbackMode_T mode)  { p_motor->FeedbackMode.Word = mode.Word; }
static inline void Motor_SetFeedbackMode_Cast(Motor_T * p_motor, uint8_t modeValue)     { p_motor->FeedbackMode.Word = modeValue; }

/******************************************************************************/
/*

*/
/******************************************************************************/
static inline uint16_t Motor_GetSpeedVRef_Rpm(Motor_T * p_motor) { return p_motor->Config.Kv * Motor_Static_GetVSource_V(); }

/*
    V of Speed
    VPhase approximation using Kv and Speed
        = Speed_Frac16 * SpeedMatchRef_Rpm / SpeedFeedbackRef_Rpm
        SpeedRef_Rpm ~= Kv * VSource
    User sets lower SpeedMatchRef_Rpm to ensure not match to higher speed
    Saturated output for use as input into next operation
    @return V Frac16
*/
static inline int16_t Motor_GetVSpeed_Frac16(const Motor_T * p_motor) { return Linear_Q16_Frac(&p_motor->UnitsVSpeed, p_motor->Speed_Frac16); }
// static inline int32_t _Motor_VSpeed_VFrac16OfSpeedFrac16(Motor_T * p_motor, int32_t speed_frac16)   { return speed_frac16 * p_motor->Config.SpeedMatchRef_Rpm / p_motor->Config.SpeedFeedbackRef_Rpm; }

/******************************************************************************/
/*!
    Extern
*/
/******************************************************************************/
extern void Motor_Init(Motor_T * p_motor);
extern void Motor_InitReboot(Motor_T * p_motor);
extern void Motor_InitSensor(Motor_T * p_motor);

extern qangle16_t Motor_PollSensorAngle(Motor_T * p_motor);
extern bool Motor_ProcSensorSpeed(Motor_T * p_motor);
extern void Motor_ProcSpeedFeedback(Motor_T * p_motor);
extern qangle16_t Motor_GetMechanicalAngle(const Motor_T * p_motor);

extern void Motor_ZeroSensor(Motor_T * p_motor);
extern void Motor_CalibrateSensorZero(Motor_T * p_motor);
extern void Motor_ValidateSensorAlign(Motor_T * p_motor);
extern bool Motor_IsSensorFeedbackAvailable(const Motor_T * p_motor);
extern bool Motor_IsFeedbackAvailable(const Motor_T * p_motor);
extern bool Motor_PollAlignFault(Motor_T * p_motor);

// extern int32_t Motor_GetSpeedLimitReq(const Motor_T * p_motor);
// extern int32_t Motor_GetILimitReq(const Motor_T * p_motor, int32_t req, int32_t feedback);

extern void Motor_UpdateFeedbackILimits(Motor_T * p_motor);
extern void Motor_UpdateFeedbackSpeedLimits(Motor_T * p_motor);

extern void Motor_SetILimitActive(Motor_T * p_motor, uint16_t scalar16);
extern void Motor_ClearILimitActive(Motor_T * p_motor);
extern void Motor_SetSpeedLimitActive(Motor_T * p_motor, uint16_t scalar_frac16);
extern void Motor_ClearSpeedLimitActive(Motor_T * p_motor);

extern bool Motor_SetSpeedLimitEntry(Motor_T * p_motor, uint8_t id, uint16_t scalar_frac16);
extern bool Motor_ClearSpeedLimitEntry(Motor_T * p_motor, uint8_t id);
extern bool Motor_SetILimitEntry(Motor_T * p_motor, uint8_t id, uint16_t scalar_frac16);
extern bool Motor_ClearILimitEntry(Motor_T * p_motor, uint8_t id);

extern bool Motor_TrySystemSpeedLimit(Motor_T * p_motor, uint16_t scalar16);
extern bool Motor_ClearSystemSpeedLimit(Motor_T * p_motor);
extern bool Motor_TrySystemILimit(Motor_T * p_motor, uint16_t scalar16);
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
extern void Motor_ResetUnitsVSpeed(Motor_T * p_motor);

extern void Motor_Jog12Step(Motor_T * p_motor, uint8_t step);
extern void Motor_Jog6PhaseStep(Motor_T * p_motor, uint8_t step);
extern void Motor_Jog6Step(Motor_T * p_motor, uint8_t step);
extern void Motor_Jog12(Motor_T * p_motor);
extern void Motor_Jog6Phase(Motor_T * p_motor);
extern void Motor_Jog6(Motor_T * p_motor);

#endif

