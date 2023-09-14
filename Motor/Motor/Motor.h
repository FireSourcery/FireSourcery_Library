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

#include "Global_Motor.h"
#include "MotorAnalog.h"
#include "Peripheral/Analog/AnalogN/AnalogN.h"

#include "Transducer/Phase/Phase.h"
#include "Transducer/Hall/Hall.h"
#include "Transducer/SinCos/SinCos.h"
#include "Math/FOC.h"

#include "Transducer/Encoder/Encoder_ModeDT.h"
#include "Transducer/Encoder/Encoder_ISR.h"
#include "Transducer/Thermistor/Thermistor.h"

#include "Utility/StateMachine/StateMachine.h"
#include "Utility/Timer/Timer.h"

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

typedef enum Motor_CommutationMode_Tag
{
    MOTOR_COMMUTATION_MODE_FOC,
    MOTOR_COMMUTATION_MODE_SIX_STEP,
}
Motor_CommutationMode_T;

/*
    Sensor Mode Param at start up.
*/
typedef enum Motor_SensorMode_Tag
{
    MOTOR_SENSOR_MODE_HALL,
    MOTOR_SENSOR_MODE_ENCODER,
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
    MOTOR_SENSOR_MODE_SIN_COS,
#endif
#if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
    MOTOR_SENSOR_MODE_SENSORLESS,
#endif
#if defined(CONFIG_MOTOR_SENSORS_EXTERN_ENABLE)
    MOTOR_SENSOR_MODE_EXTERN,
#endif
#if defined(CONFIG_MOTOR_SENSORS_VTABLE_ENABLE)
    MOTOR_SENSOR_MODE_VTABLE,
#endif
}
Motor_SensorMode_T;

/******************************************************************************/
/*
    Feedback Mode
*/
/******************************************************************************/
typedef union Motor_FeedbackMode_Tag
{
    struct
    {
        uint8_t OpenLoop   : 1U;   /* 0 -> Position feedback, 1 -> OpenLoop */
        uint8_t Speed      : 1U;   /* 0 -> Voltage or Current only, 1 -> Speed feedback */
        uint8_t Current    : 1U;   /* 0 -> Voltage, 1-> Current */
        uint8_t Scalar     : 1U;   /* 0 -> Voltage, 1-> Use Scalar */
        uint8_t Position   : 1U;
    };
    uint8_t Word;
}
Motor_FeedbackMode_T;

static const Motor_FeedbackMode_T MOTOR_FEEDBACK_MODE_OPEN_LOOP            = { .OpenLoop = 1U, .Speed = 0U, .Current = 0U, .Scalar = 0U, };
static const Motor_FeedbackMode_T MOTOR_FEEDBACK_MODE_OPEN_LOOP_CURRENT    = { .OpenLoop = 1U, .Speed = 0U, .Current = 1U, .Scalar = 0U, };
static const Motor_FeedbackMode_T MOTOR_FEEDBACK_MODE_VOLTAGE              = { .OpenLoop = 0U, .Speed = 0U, .Current = 0U, .Scalar = 0U, };
static const Motor_FeedbackMode_T MOTOR_FEEDBACK_MODE_VOLTAGE_FREQ         = { .OpenLoop = 0U, .Speed = 0U, .Current = 0U, .Scalar = 1U, };
static const Motor_FeedbackMode_T MOTOR_FEEDBACK_MODE_CURRENT              = { .OpenLoop = 0U, .Speed = 0U, .Current = 1U, .Scalar = 0U, };
static const Motor_FeedbackMode_T MOTOR_FEEDBACK_MODE_SPEED_VOLTAGE        = { .OpenLoop = 0U, .Speed = 1U, .Current = 0U, .Scalar = 0U, };
static const Motor_FeedbackMode_T MOTOR_FEEDBACK_MODE_SPEED_CURRENT        = { .OpenLoop = 0U, .Speed = 1U, .Current = 1U, .Scalar = 0U, };

static inline Motor_FeedbackMode_T Motor_FeedbackMode(uint8_t word) { Motor_FeedbackMode_T flags = { .Word = word }; return flags; }

// consolidate state machine inputs
// typedef union
// {
//     struct
//     {
//         Motor_FeedbackMode_T FeedbackMode : 5U;
//         uint16_t Hold        : 1U;
//         uint16_t Control     : 1U;
//         uint16_t Release     : 1U;
//     };
//     uint16_t Word;
// }
// Motor_ControlReq_T;

/*
    Effectively sync mailbox for async calculations
*/
typedef union Motor_StatusFlags_Tag
{
    struct
    {
        uint16_t HeatWarning        : 1U;
        uint16_t ILimitActive       : 1U;
        uint16_t SpeedLimitActive   : 1U;
        uint16_t ILimitReached      : 1U;
        uint16_t SpeedLimitReached  : 1U;
        // uint16_t SensorFeedback  : 1U;
        uint16_t Hold        : 1U;
        uint16_t Control     : 1U;
        uint16_t Release     : 1U;
    };
    uint16_t Word;
}
Motor_StatusFlags_T;

typedef union
{
    struct
    {
        uint16_t OverHeat       : 1U;
        uint16_t AlignStartUp   : 1U;
    };
    uint16_t Word;
}
Motor_FaultFlags_T;

typedef enum Motor_ILimitActiveId_Tag
{
    MOTOR_I_LIMIT_ACTIVE_HEAT_THIS,     /* MotorHeat */
    MOTOR_I_LIMIT_ACTIVE_UPPER,         /* From upper module */
    MOTOR_I_LIMIT_ACTIVE_USER,
}
Motor_ILimitActiveId_T;

typedef enum Motor_SpeedLimitActiveId_Tag
{
    MOTOR_SPEED_LIMIT_ACTIVE_UPPER, /* From upper module */
    MOTOR_SPEED_LIMIT_ACTIVE_USER,
}
Motor_SpeedLimitActiveId_T;

/*
    Direction Run SubState
*/
typedef enum Motor_Direction_Tag
{
    MOTOR_DIRECTION_CCW,
    MOTOR_DIRECTION_CW,
}
Motor_Direction_T;

/*

*/
typedef enum Motor_OpenLoopState_Tag
{
    MOTOR_OPEN_LOOP_STATE_ALIGN,
    MOTOR_OPEN_LOOP_STATE_VALIDATE_ALIGN,
    MOTOR_OPEN_LOOP_STATE_RUN,
}
Motor_OpenLoopState_T;

/*
    Calibration SubState
*/
typedef enum Motor_CalibrationState_Tag
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
    All modules independently conform to same ID
*/
typedef enum Motor_SectorId_Tag
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

/*!
    @brief Motor Parameters - Runtime variable configuration. Load from non volatile memory.
*/
typedef struct __attribute__((aligned(2U))) Motor_Params_Tag
{
    Motor_CommutationMode_T     CommutationMode;
    Motor_SensorMode_T          SensorMode;
    Motor_FeedbackMode_T        FeedbackModeDefault;     /* Default FeedbackMode  */
    Motor_Direction_T           DirectionForward;

    /*
        Ref values, known calibration parameter provide by user
    */
    uint8_t PolePairs;
    uint16_t Kv;
    uint16_t SpeedFeedbackRef_Rpm;  /* Feedback / PID Regulator Limits Ref, User IO units conversion, Encoder speed calc ref. */
                                    /* A value greater than achievable will cause integral windup */
    //SpeedVOut
    uint16_t VSpeedRef_Rpm;         /* Derive from Kv + offset. Use < SpeedFeedbackRef_Rpm to begin at lower speed. */
                                    //option Scale to KvSpeed or Bemf on resume
#if defined(CONFIG_MOTOR_DEBUG_ENABLE)
    uint16_t IPeakRef_Adcu;
#endif
    uint16_t IaZeroRef_Adcu;
    uint16_t IbZeroRef_Adcu;
    uint16_t IcZeroRef_Adcu;

    uint16_t SpeedLimitForward_Scalar16;   /* "Root" Limits. Persistent User Param. Frac16 of SpeedFeedbackRef_Rpm */
    uint16_t SpeedLimitReverse_Scalar16;
    uint16_t ILimitMotoring_Scalar16;      /* Persistent User Param. Frac16 of RefMax I_MAX_AMPS. ILimit must be < 100% to account for I_Frac saturation. */
    uint16_t ILimitGenerating_Scalar16;

    uint16_t RampAccel_Cycles;
    uint16_t AlignPower_Scalar16;          /* V or I */
    uint32_t AlignTime_Cycles;

#if defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
    uint16_t OpenLoopSpeed_Scalar16;    /* Max, 65536 will overflow */
    uint16_t OpenLoopPower_Scalar16;    /* V or I */
    uint16_t OpenLoopAccel_Cycles;      /* Time to reach OpenLoopSpeed */
    // uint16_t OpenLoopVHzGain;
#endif

#if defined(CONFIG_MOTOR_SURFACE_SPEED_ENABLE)
    uint16_t SurfaceDiameter;
    uint16_t GearRatio_Factor;
    uint16_t GearRatio_Divisor;
#endif

#if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
    Phase_Mode_T PhasePwmMode;     /* Only 1 nvm param for phase module. */
#endif
}
Motor_Params_T;

/*!
    @brief Motor Config - Compile time const configuration. Unique per Motor
*/
typedef const struct Motor_Init_Tag
{
    AnalogN_T * const P_ANALOG_N;
    const MotorAnalog_Conversions_T ANALOG_CONVERSIONS;
    const Motor_Params_T * const P_PARAMS_NVM;
    void (*INIT_SENSOR_HALL)(void);
    void (*INIT_SENSOR_ENCODER)(void);
}
Motor_Config_T;

typedef struct Motor_Tag
{
    // static uint16_t VSourceRef_V;

    const Motor_Config_T CONFIG;
    Motor_Params_T Parameters;

    volatile MotorAnalog_Results_T AnalogResults;
    // MotorAnalog_ChannelGroup_T AnalogCmd;

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
    uint32_t ControlTimerBase;                  /* Control Freq ~ 20kHz, calibration, commutation, angle control. overflow at 20Khz, 59 hours*/
    Timer_T ControlTimer;                       /* State Timer */
    Motor_Direction_T Direction;                /* Active spin direction */
    Motor_FeedbackMode_T FeedbackMode;          /* Active FeedbackMode */
    Motor_StatusFlags_T StatusFlags;
    /* SubStates */
    Motor_FaultFlags_T FaultFlags;
    Motor_OpenLoopState_T OpenLoopState;
    Motor_CalibrationState_T CalibrationState;     /* SubState, selection for calibration */
    uint8_t CalibrationStateIndex;

    /*

    */
    Linear_T Ramp;          /* User Input Ramp, UserCmdValue as RampTarget - Speed, Current, or Voltage. Updated without StateMachine check */
                            /* Output [-32767:32767] SetPoint => SpeedReq, IReq, VReq. */
    Linear_T AuxRamp;       /* OpenLoop and Align */

    /*
        Active Limits
    */
    /* Non directional scalars */
    uint16_t SpeedLimitForward_Scalar16;
    uint16_t SpeedLimitReverse_Scalar16;
    uint16_t ILimitMotoring_Scalar16;
    uint16_t ILimitGenerating_Scalar16;

    uint16_t SpeedLimitDirect_Scalar16;     /* Frac16 of SpeedRefMax Param. Check on set */
    uint16_t SpeedLimitBuffer[2U];
    // Motor_SpeedLimitActiveId_T SpeedLimitActiveId;
    // uint16_t SpeedLimitActiveScalar;

    uint16_t ILimitBuffer[3U];                      /* Fixed buffer for 3 values, internal thermistor, user, upper layer */
    // Motor_ILimitActiveId_T ILimitActiveId;
    uint16_t ILimitActiveSentinel_Scalar16;         /* Store for comparison */

    /* Active directional limits   */
    int16_t SpeedLimitCcw_FracS16;
    int16_t SpeedLimitCw_FracS16;
    int16_t ILimitCcw_FracS16;
    int16_t ILimitCw_FracS16;

    PID_T PidPosition;    /* todo */

    /*
        Speed Feedback
    */
    int32_t Speed_FracS16;             /* [-32765*2:32765*2] Speed Feedback Variable. Can over saturate. - is virtual CW, + => CCW */
    Timer_T SpeedTimer;                /* Speed Calc Timer */
    PID_T PidSpeed;                    /* Input PidSpeed(RampCmd - Speed_FracS16), Output [-32768:32767] => VPwm, Vq, Iq. Updated once per ms */

    qangle16_t MechanicalAngle;
    qangle16_t ElectricalAngle;         /* Angle Feedback. Shared E-Cycle edge detect, User output */

    /*
        FOC
    */
    FOC_T Foc;
    PID_T PidIq;                    /* Input (IqReq - IqFeedback), Output Vq. Sign indicates ccw/cw direction */
    PID_T PidId;

    /*
        Unit Conversions
    */
    Linear_T UnitsIa;           /* Frac16 and Amps */
    Linear_T UnitsIb;
    Linear_T UnitsIc;
    Linear_T UnitsVabc;         /* Bemf V,mV, and Frac16 conversion */
    Linear_T UnitsVSpeed;       /* Speed_Frac16 => VBemf_Frac16 */

    Filter_T FilterA;           /* Calibration use */
    Filter_T FilterB;
    Filter_T FilterC;

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
        OpenLoops speed ramp
    */
#if defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE) || defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
    Linear_T OpenLoopSpeedRamp;            /* OpenLoopSpeed Ramp */
    #if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
    uint32_t OpenLoopCommutationPeriod;
    #endif
#endif

    /* Jog */
    uint32_t JogIndex;


#if  defined(CONFIG_MOTOR_DEBUG_ENABLE)
    uint32_t MicrosRef;
    volatile bool DebugFlag;
    volatile uint32_t DebugError;
    volatile uint32_t DebugTime[10U];
    volatile uint32_t DebugTimeABC[3U];
    volatile uint32_t DebugCounter;
    volatile uint32_t DebugCounter2;
#endif
}
Motor_T;

#if defined(CONFIG_MOTOR_DEBUG_ENABLE)
#include "Motor_Debug.h"
#endif

#define MOTOR_CONTROL_CYCLES(MilliSeconds, Freq) ((uint32_t)MilliSeconds * (uint32_t)Freq / 1000UL)
#define MOTOR_CONTROL_TIME_MS(Cycles, Freq) ((uint32_t)Cycles * 1000UL / (uint32_t)Freq)

/******************************************************************************/
/*
    Number formats

    Frac16      [-1:1) <=> [-32768:32767] in Q1.15
        => may over saturate to [-2:2) <=> [-65536:65536]
    UFrac16     [0:2) <=> [0:65535] in Q1.15
    Scalar16    [0:1) <=> [0:65535] in Q0.16
    Fixed32     [-1:1] <=> [-65536:65536] in Q16.16     Max [INT32_MIN:INT32_MAX],
    Fixed16     [-1:1] <=> [-256:256]                   Max [-32768:32767],
*/
/******************************************************************************/
#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
static inline int32_t _Motor_ConvertSpeed_RpmToFracS16(const Motor_T * p_motor, int32_t speed_rpm)        { return speed_rpm * INT16_MAX / p_motor->Parameters.SpeedFeedbackRef_Rpm; }
static inline int16_t _Motor_ConvertSpeed_FracS16ToRpm(const Motor_T * p_motor, int32_t speed_frac16)     { return speed_frac16 * p_motor->Parameters.SpeedFeedbackRef_Rpm / 32768; }
static inline int32_t _Motor_ConvertI_AmpsToIFracS16(int32_t i_amp)                                 { return i_amp * INT16_MAX / GLOBAL_MOTOR.I_MAX_AMPS; }
static inline int16_t _Motor_ConvertI_FracS16ToAmps(int32_t i_frac16)                               { return i_frac16 * GLOBAL_MOTOR.I_MAX_AMPS / 32768; }
static inline int32_t _Motor_ConvertV_VoltsToFracS16(int32_t v_volts)                               { return v_volts * INT16_MAX / Global_Motor_GetVSource_V(); }
static inline int16_t _Motor_ConvertV_FracS16ToVolts(int32_t v_frac16)                              { return v_frac16 * Global_Motor_GetVSource_V() / 32768; }
static inline int32_t _Motor_ConvertPower_FracS16ToWatts(int32_t vi_frac16)                         { return vi_frac16 * GLOBAL_MOTOR.I_MAX_AMPS * Global_Motor_GetVSource_V() / 32768; }

static inline uint32_t _Motor_ConvertToMillis(uint32_t controlCycles) { return controlCycles * 1000 / GLOBAL_MOTOR.CONTROL_FREQ; }
static inline uint32_t _Motor_ConvertToControlCycles(uint32_t millis) { return millis * GLOBAL_MOTOR.CONTROL_FREQ / 1000; }

static inline int32_t _Motor_ConvertSpeed_RpmToScalar16(Motor_T * p_motor, int32_t speed_rpm)       { return speed_rpm * UINT16_MAX / p_motor->Parameters.SpeedFeedbackRef_Rpm; }
static inline int16_t _Motor_ConvertSpeed_Scalar16ToRpm(Motor_T * p_motor, int32_t speed_scalar16)  { return speed_scalar16 * p_motor->Parameters.SpeedFeedbackRef_Rpm / 65536; }
static inline int32_t _Motor_ConvertI_AmpToScalar16(int32_t i_amp)                                  { return i_amp * UINT16_MAX / GLOBAL_MOTOR.I_MAX_AMPS; }
static inline int16_t _Motor_ConvertI_Scalar16ToAmp(int32_t i_scalar16)                             { return i_scalar16 * GLOBAL_MOTOR.I_MAX_AMPS / 65536; }
static inline int32_t _Motor_ConvertV_VoltsToScalar16(int32_t v_volts)                              { return v_volts * UINT16_MAX / Global_Motor_GetVSource_V(); }
static inline int16_t _Motor_ConvertV_Scalar16ToVolts(int32_t v_scalar16)                           { return v_scalar16 * Global_Motor_GetVSource_V() / 65536; }
static inline int32_t _Motor_ConvertPower_Scalar16ToWatts(int32_t vi_scalar16)                      { return vi_scalar16 * GLOBAL_MOTOR.I_MAX_AMPS * Global_Motor_GetVSource_V() / 65536; }

/* todo */
// static inline uint32_t speed_angle16torpm(uint16_t angle16, uint32_t sampleFreq)                     { return  (angle16 * sampleFreq >> 16U) * 60U; }
// static inline uint32_t speed_rpmtoangle16(uint16_t rpm, uint32_t sampleFreq)                         { return (rpm << 16U) / (60U * sampleFreq); }
// static inline uint32_t _Motor_ConvertMechAngleToRpm(Motor_T * p_motor, uint16_t angle16 )            { return (angle16 * GLOBAL_MOTOR.SPEED_FREQ >> 16U) * 60U; }
// static inline uint32_t _Motor_ConvertMechRpmToAngle(Motor_T * p_motor, uint16_t rpm )                { return (rpm << 16U) / (60U * GLOBAL_MOTOR.SPEED_FREQ); }

// static inline int32_t _Motor_VSpeed_ConvertSpeedToVFrac16(Motor_T * p_motor, int32_t speed_frac16)   { return speed_frac16 * p_motor->Parameters.VSpeedRef_Rpm / p_motor->Parameters.SpeedFeedbackRef_Rpm; }
// static inline int32_t _Motor_VSpeed_ConvertRpmToVFrac16(Motor_T * p_motor, int32_t speed_rpm)        { return _Motor_VSpeed_ConvertSpeedToVFrac16(p_motor, _Motor_ConvertSpeed_RpmToScalar16(p_motor, speed_rpm) ); }
// static inline uint32_t _Motor_VSpeed_ConvertToVSpeed(Motor_T * p_motor, uint16_t rpm)                { return Linear_Function(&p_motor->UnitsVSpeed, _Motor_ConvertSpeed_RpmToScalar16(p_motor, rpm)); }
#endif


/******************************************************************************/
/*
    Simplify CommutationMode Check
*/
/******************************************************************************/
typedef void(*Motor_Function_T)(Motor_T * p_motor);

static inline void Motor_ProcCommutationMode(Motor_T * p_motor, Motor_Function_T focFunction, Motor_Function_T sixStepFunction)
{
#if     defined(CONFIG_MOTOR_SIX_STEP_ENABLE) && defined(CONFIG_MOTOR_FOC_ENABLE)
    if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)               { focFunction(p_motor); }
    else /* p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP */   { sixStepFunction(p_motor); }
#elif   defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
    (void)focFunction; sixStepFunction(p_motor);
#elif   defined(CONFIG_MOTOR_FOC_ENABLE)
    (void)sixStepFunction; focFunction(p_motor);
#endif
}

typedef int32_t(*Motor_FunctionGetInt32_T)(const Motor_T * p_motor);

static inline int32_t Motor_GetCommutationModeInt32(const Motor_T * p_motor, Motor_FunctionGetInt32_T focFunction, Motor_FunctionGetInt32_T sixStepFunction)
{
#if     defined(CONFIG_MOTOR_SIX_STEP_ENABLE) && defined(CONFIG_MOTOR_FOC_ENABLE)
    int32_t value32;
    if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)                 { value32 = focFunction(p_motor); }
    else /* p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP */     { value32 = sixStepFunction(p_motor); }
    return value32;
#elif   defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
    (void)focFunction; return sixStepFunction(p_motor);
#else /* defined(CONFIG_MOTOR_FOC_ENABLE) */
    (void)sixStepFunction; return focFunction(p_motor);
#endif
}

/* Type cast warning for enums */
typedef void(*Motor_FunctionSetUInt32_T)(Motor_T * p_motor, uint32_t var);

static inline void Motor_SetCommutationModeUInt32(Motor_T * p_motor, Motor_FunctionSetUInt32_T focFunction, Motor_FunctionSetUInt32_T sixStepFunction, uint32_t var)
{
#if     defined(CONFIG_MOTOR_SIX_STEP_ENABLE) && defined(CONFIG_MOTOR_FOC_ENABLE)
    if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)                 { focFunction(p_motor, var); }
    else /* p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP */     { sixStepFunction(p_motor, var); }
#elif     defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
    (void)focFunction; sixStepFunction(p_motor, var);
#else /* defined(CONFIG_MOTOR_FOC_ENABLE) */
    (void)sixStepFunction; focFunction(p_motor, var);
#endif
}

/******************************************************************************/
/*
    Interrupts
*/
/******************************************************************************/
static inline void Motor_ClearPwmInterrupt(Motor_T * p_motor) { Phase_ClearInterrupt(&p_motor->Phase); }
static inline void Motor_DisablePwm(Motor_T * p_motor) { Phase_DisableInterrupt(&p_motor->Phase); }
static inline void Motor_EnablePwm(Motor_T * p_motor) { Phase_EnableInterrupt(&p_motor->Phase); }

/******************************************************************************/
/*
    Direction Cmd
*/
/******************************************************************************/
static inline Motor_Direction_T Motor_DirectionForward(const Motor_T * p_motor) { return p_motor->Parameters.DirectionForward; }
static inline Motor_Direction_T Motor_DirectionReverse(const Motor_T * p_motor) { return ((p_motor->Parameters.DirectionForward == MOTOR_DIRECTION_CCW) ? MOTOR_DIRECTION_CW : MOTOR_DIRECTION_CCW); }

/*!
    Convert user reference direction to CCW/CW direction
    @param[in] userCmd int32_t[-65536:65536] frac16 or scalar16. positive is forward relative to the user.
    @return int32_t[-65536:65536], Over saturated if input is -32768
*/
static inline int32_t Motor_LogicalDirectionCmd(const Motor_T * p_motor, int32_t userCmd) { return (p_motor->Direction == MOTOR_DIRECTION_CCW) ? userCmd : (int32_t)0 - userCmd; }

/* Call from StateMachine only */
static inline void Motor_SetFeedbackMode_Cast(Motor_T * p_motor, uint8_t modeWord) { p_motor->FeedbackMode.Word = modeWord; }
static inline void Motor_SetFeedbackMode(Motor_T * p_motor, Motor_FeedbackMode_T mode) { p_motor->FeedbackMode.Word = mode.Word; }

/******************************************************************************/
/*

*/
/******************************************************************************/
/*
    VPhase approximation using Kv and Speed
        = Speed_FracS16 * VSpeedRef_Rpm / SpeedFeedbackRef_Rpm
    User sets lower VSpeedRef_Rpm to ensure not match to higher speed
    Saturated output for use as input into next operation
*/
static inline int16_t Motor_GetVSpeed_Frac16(Motor_T * p_motor)
{
    return Linear_Frac16_Signed(&p_motor->UnitsVSpeed, p_motor->Speed_FracS16);
    // return Linear_Function_Sat(&p_motor->UnitsVSpeed, p_motor->Speed_FracS16);
    // return Linear_Function_FracS16(&p_motor->UnitsVSpeed, p_motor->Speed_FracS16);
}



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
extern bool Motor_CheckSensorFeedback(const Motor_T * p_motor);
extern bool Motor_CheckOpenLoop(const Motor_T * p_motor);
extern bool Motor_PollAlignFault(Motor_T * p_motor);

extern int32_t Motor_GetSpeedLimitReq(const Motor_T * p_motor);
extern int32_t Motor_GetILimitReq(const Motor_T * p_motor,  int32_t req, int32_t feedback);
extern void Motor_UpdateFeedbackILimits(Motor_T * p_motor);
extern void Motor_UpdateFeedbackSpeedLimits(Motor_T * p_motor);

extern void Motor_SetDirectionCcw(Motor_T * p_motor);
extern void Motor_SetDirectionCw(Motor_T * p_motor);
extern void Motor_SetDirection(Motor_T * p_motor, Motor_Direction_T direction);
extern void Motor_SetDirectionForward(Motor_T * p_motor);
extern void Motor_SetDirectionReverse(Motor_T * p_motor);

extern void Motor_ResetSpeedLimitActive(Motor_T * p_motor);
extern void Motor_ResetILimitActive(Motor_T * p_motor);
extern void Motor_ResetUnitsSensor(Motor_T * p_motor);
extern void Motor_ResetUnitsVabc(Motor_T * p_motor);
extern void Motor_ResetUnitsIabc(Motor_T * p_motor);
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

/* Feedback Control Variable Mode  */
// typedef enum Motor_FeedbackModeId_Tag
// {
//     MOTOR_FEEDBACK_MODE_OPEN_LOOP,
//     MOTOR_FEEDBACK_MODE_OPEN_LOOP_CURRENT,
//     MOTOR_FEEDBACK_MODE_CONSTANT_VOLTAGE,
//     MOTOR_FEEDBACK_MODE_SCALAR_VOLTAGE_FREQ,
//     MOTOR_FEEDBACK_MODE_CONSTANT_SPEED_VOLTAGE,
//     MOTOR_FEEDBACK_MODE_CONSTANT_CURRENT,
//     MOTOR_FEEDBACK_MODE_CONSTANT_SPEED_CURRENT,
//     MOTOR_FEEDBACK_MODE_POSITION_SPEED_CURRENT,
// }
// Motor_FeedbackModeId_T;

/* SuperFunction AntiPattern rationale, single variable alias, should be compiler optimizable */
// static inline Motor_FeedbackMode_T Motor_FeedbackModeFlags(Motor_FeedbackModeId_T mode)
// {
//     Motor_FeedbackMode_T flags;

//     switch(mode)
//     {
//         case MOTOR_FEEDBACK_MODE_OPEN_LOOP:                 flags.State = MODE_OPEN_LOOP.State;             break;
//         case MOTOR_FEEDBACK_MODE_OPEN_LOOP_CURRENT:         flags.State = MODE_OPEN_LOOP_CURRENT.State;     break;
//         case MOTOR_FEEDBACK_MODE_CONSTANT_VOLTAGE:          flags.State = MODE_VOLTAGE.State;               break;
//         case MOTOR_FEEDBACK_MODE_SCALAR_VOLTAGE_FREQ:       flags.State = MODE_VOLTAGE_FREQ.State;          break;
//         case MOTOR_FEEDBACK_MODE_CONSTANT_CURRENT:          flags.State = MODE_CURRENT.State;               break;
//         case MOTOR_FEEDBACK_MODE_CONSTANT_SPEED_VOLTAGE:    flags.State = MODE_SPEED_VOLTAGE.State;         break;
//         case MOTOR_FEEDBACK_MODE_CONSTANT_SPEED_CURRENT:    flags.State = MODE_SPEED_CURRENT.State;         break;
//         default: flags.State = 0; break;
//     }

//     return flags;
// }

// static inline Motor_FeedbackModeId_T Motor_FeedbackModeId(Motor_FeedbackMode_T mode)
// {
//     Motor_FeedbackModeId_T id;

//     switch((uint32_t)mode.State)
//     {
//         case (uint32_t)MODE_OPEN_LOOP.State:          id = MOTOR_FEEDBACK_MODE_OPEN_LOOP;                 break;
//         case (uint32_t)MODE_OPEN_LOOP_CURRENT.State:  id = MOTOR_FEEDBACK_MODE_OPEN_LOOP_CURRENT;         break;
//         case (uint32_t)MODE_VOLTAGE.State:            id = MOTOR_FEEDBACK_MODE_CONSTANT_VOLTAGE;          break;
//         case (uint32_t)MODE_VOLTAGE_FREQ.State:       id = MOTOR_FEEDBACK_MODE_SCALAR_VOLTAGE_FREQ;       break;
//         case (uint32_t)MODE_CURRENT.State:            id = MOTOR_FEEDBACK_MODE_CONSTANT_CURRENT;          break;
//         case (uint32_t)MODE_SPEED_VOLTAGE.State:      id = MOTOR_FEEDBACK_MODE_CONSTANT_SPEED_VOLTAGE;    break;
//         case (uint32_t)MODE_SPEED_CURRENT.State:      id = MOTOR_FEEDBACK_MODE_CONSTANT_SPEED_CURRENT;    break;
//         default: id = 0; break;
//     }

//     return id;
// }