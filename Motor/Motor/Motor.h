/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2021 FireSourcery / The Firebrand Forge Inc

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
    @file     Motor.h
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
// #include "Math/Linear/Linear_Speed.h"
#include "Math/Linear/Linear.h"
#include "Math/PID/PID.h"
#include "Math/Filter/Filter_MovAvg.h"
#include "Math/Filter/Filter.h"

#include <stdint.h>
#include <stdbool.h>

#define MOTOR_LIB_VERSION_OPT        0U
#define MOTOR_LIB_VERSION_MAJOR     0U
#define MOTOR_LIB_VERSION_MINOR     1U
#define MOTOR_LIB_VERSION_BUGFIX     0U

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
}
Motor_SensorMode_T;

/*
    Feedback Control Variable Mode
*/
typedef enum Motor_FeedbackModeId_Tag
{
    MOTOR_FEEDBACK_MODE_OPEN_LOOP,
    MOTOR_FEEDBACK_MODE_OPEN_LOOP_CURRENT,
    MOTOR_FEEDBACK_MODE_CONSTANT_VOLTAGE,
    MOTOR_FEEDBACK_MODE_SCALAR_VOLTAGE_FREQ,
    MOTOR_FEEDBACK_MODE_CONSTANT_SPEED_VOLTAGE,
    MOTOR_FEEDBACK_MODE_CONSTANT_CURRENT,
    MOTOR_FEEDBACK_MODE_CONSTANT_SPEED_CURRENT,
    MOTOR_FEEDBACK_MODE_POSITION_SPEED_CURRENT,
}
Motor_FeedbackModeId_T;

/*
    Bit representation
*/
typedef union Motor_FeedbackMode_Tag
{
    struct
    {
        uint32_t Position : 1U;
        uint32_t Speed : 1U;    /* 0 -> Voltage or Current only, 1 -> Speed feedback */
        uint32_t Current : 1U;    /* 0 -> Voltage, 1-> Current */
        uint32_t Scalar : 1U;     /* 1-> Use Scalar  */
        uint32_t OpenLoop : 1U;     /* 0 -> Position feedback, 1 -> Openloop */
        uint32_t IsDisable : 1U;    /* IsDisable, check for update */
    };
    uint32_t State;
}
Motor_FeedbackMode_T;

/*
    Effectively sync mailbox for async calculations
*/
typedef union Motor_StatusFlags_Tag
{
    struct
    {
        uint32_t HeatWarning : 1U;
        uint32_t ILimitActive : 1U;
        // uint32_t SpeedLimitActive         : 1U;
        // uint32_t SensorFeedback            : 1U;
        // uint32_t FieldWeakening             : 1U;
    };
    uint32_t State;
}
Motor_StatusFlags_T;

typedef union
{
    struct
    {
        uint32_t HeatShutdown : 1U;
        uint32_t AlignStartUp : 1U;
    };
    uint32_t State;
}
Motor_FaultFlags_T;

// typedef enum Motor_SpeedLimitActiveId_Tag
// {
//     MOTOR_SPEED_LIMIT_ACTIVE_DISABLE = 0U,
//     MOTOR_SPEED_LIMIT_ACTIVE_SYSTEM = 1U, /* From upper module */
//     MOTOR_SPEED_LIMIT_ACTIVE_USER = 2U,
// }
// Motor_SpeedLimitActiveId_T;

typedef enum Motor_ILimitActiveId_Tag
{
    MOTOR_I_LIMIT_ACTIVE_DISABLE = 0U,
    MOTOR_I_LIMIT_ACTIVE_HEAT = 1U,     /* MotorHeat */
    MOTOR_I_LIMIT_ACTIVE_SYSTEM = 20U,  /* From upper module */
    MOTOR_I_LIMIT_ACTIVE_USER = 30U,
}
Motor_ILimitActiveId_T;

/*
    Direction Run SubState
*/
typedef enum Motor_Direction_Tag
{
    MOTOR_DIRECTION_CW,
    MOTOR_DIRECTION_CCW,
}
Motor_Direction_T;

typedef enum Motor_DirectionCalibration_Tag
{
    MOTOR_FORWARD_IS_CW,
    MOTOR_FORWARD_IS_CCW,
}
Motor_DirectionCalibration_T;

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

/******************************************************************************/
/*!
    Frac16 [-1:1] <=> [-65536:65536] in q16.16 may over saturate
    FracU16 [0:1] <=> [0:65535] in q0.16
    FracS16 [-1:1] <=> [-32768:32767] in q1.15
*/
/******************************************************************************/
/*!
    @brief Motor Parameters - Runtime variable configuration. Load from non volatile memory.
*/
typedef struct __attribute__((aligned(2U))) Motor_Params_Tag
{
    Motor_CommutationMode_T     CommutationMode;
    Motor_SensorMode_T             SensorMode;
    Motor_FeedbackModeId_T         DefaultFeedbackMode;     /* Default ControlFeedbackMode, and ThrottleCmd */
    Motor_DirectionCalibration_T DirectionCalibration;

    /*
        Ref values, known calibration parameter provide by user
    */
    uint8_t PolePairs;
    uint16_t Kv;
    uint16_t SpeedFeedbackRef_Rpm;     /* Feedback / PID Regulator Limits Ref, User IO units conversion, Encoder speed calc ref. */
    uint16_t VSpeedRef_Rpm;         /* Derive from Kv + offset. Use < SpeedFeedbackRef_Rpm to begin at lower speed. */
    //option Scale to Kv or Bemf on resume
#if defined(CONFIG_MOTOR_DEBUG_ENABLE)
    uint16_t IPeakRef_Adcu;
#endif
    uint16_t IaZeroRef_Adcu;
    uint16_t IbZeroRef_Adcu;
    uint16_t IcZeroRef_Adcu;

    uint16_t SpeedLimitForward_ScalarU16;     /* "Root" Limits. Persistent User Param. Frac16 of SpeedFeedbackRef_Rpm */
    uint16_t SpeedLimitReverse_ScalarU16;
    uint16_t ILimitMotoring_ScalarU16;         /* Persistent User Param. Frac16 of RefMax I_UNITS_AMPS. ILimit must be < 100% to account for I_Frac saturation. */
    uint16_t ILimitGenerating_ScalarU16;

    uint16_t RampAccel_Cycles;
    // uint16_t SpeedRampAccel_Cycles;
    // uint16_t VIRampAccel_Cycles;
    uint16_t AlignPower_FracU16;     /* V or I Frac16 */
    uint32_t AlignTime_Cycles;
    // uint16_t VoltageBrakeScalar_Frac16; /* [0:65535], 0 is highest intensity */
#if defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE) || defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
    uint16_t OpenLoopSpeed_FracU16;         /* Max, 65536 will overflow */
    uint16_t OpenLoopPower_FracU16;         /* V or I Frac16 */
    uint16_t OpenLoopAccel_Cycles;             /* Time to reach OpenLoopSpeed_RPM */
    // uint16_t OpenLoopVHzGain;
#endif

    uint16_t SurfaceDiameter;
    uint16_t GearRatio_Factor;
    uint16_t GearRatio_Divisor;
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
    const Motor_Config_T CONFIG;
    Motor_Params_T Parameters;

    volatile MotorAnalog_Results_T AnalogResults;

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
    uint32_t ControlTimerBase;                 /* Control Freq ~ 20kHz, calibration, commutation, angle control. overflow at 20Khz, 59 hours*/
    Timer_T ControlTimer;                     /* State Timer */
    /* Run SubState */
    Motor_Direction_T Direction;             /* Active spin direction */
    Motor_FeedbackMode_T ControlFeedbackMode;
    Motor_StatusFlags_T StatusFlags;
    /* Calibration SubState */
    Motor_CalibrationState_T CalibrationState;     /* SubState, selection for calibration */
    uint8_t CalibrationStateIndex;
    Motor_FaultFlags_T FaultFlags;
    Motor_OpenLoopState_T OpenLoopState;

    /*
        UserCmd Input => Ramp
    */
    Linear_T Ramp;        /* User Input Ramp, Ramp Target is UserCmd -  Speed, Current, or Voltage. Updated without StateMachine Check */
    /* [-32767:32767] SetPoint after ramp => SpeedReq, IReq, VReq. [0:65535] VFreq Mode */
    Linear_T AuxRamp;     /* OpenLoop and Align */

    /*
        Active Limits
    */
    /* Non directional scalars */
    uint16_t SpeedLimitDirect_ScalarU16;     /* Frac16 of SpeedRefMax Param. Check on set */
    uint16_t SpeedLimitForward_ScalarU16;
    uint16_t SpeedLimitReverse_ScalarU16;
    // Motor_SpeedLimitActiveId_T SpeedLimitActiveId;
    // uint16_t SpeedLimitActiveScalar;
    int16_t SpeedLimitCcw_FracS16;    /* Checked when Speed feedback inactive */
    int16_t SpeedLimitCw_FracS16;
    uint16_t ILimitMotoring_ScalarU16;
    uint16_t ILimitGenerating_ScalarU16;
    Motor_ILimitActiveId_T ILimitActiveId;
    uint16_t ILimitActiveSentinel;        /* Store for comparison */
    int16_t ILimitCcw_FracS16;             /* Checked when Current feedback inactive, [-32767:32767] directional input into IqPid, Same as SpeedPid output limit during SPEED_CURRENT_MODE */
    int16_t ILimitCw_FracS16;

    /* todo */
    PID_T PidPosition;

    /*
        Speed Feedback
    */
    int32_t Speed_FracS16;             /* [-32765*2:32765*2] Speed Feedback Variable. Can over saturate */
    Timer_T SpeedTimer;                /* Speed Calc Timer */
    PID_T PidSpeed;                    /* Input PidSpeed(RampCmd - Speed_FracS16), Output [-32768:32767] => VPwm, Vq, Iq. Updated once per ms */

    /* Sensorless and SinCos. Non square wave Encoder module */
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE) || defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
    Linear_T UnitsAngleSpeed;             /*  */
    Linear_T UnitsSurfaceSpeed;         /*  */
    qangle16_t SpeedAngle;             /* Save for reference, MechanicalDelta */
#endif

    qangle16_t MechanicalAngle;
    qangle16_t ElectricalAngle;        /* Angle Feedback. Shared E-Cycle edge detect, User output */

    /*
        FOC
    */
    FOC_T Foc;
    PID_T PidIq;                    /* Input (IqReq - IqFeedback), Output Vq. Sign indicates ccw/cw direction */
    PID_T PidId;

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
    uint32_t IBusSum_Frac16;
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

    /*
        Unit Conversions
    */
    Linear_T UnitsIa;          /* Frac16 and Amps */
    Linear_T UnitsIb;
    Linear_T UnitsIc;
    Linear_T UnitsVabc;        /* Bemf V,mV, and Frac16 conversion */
    Linear_T UnitsVSpeed;     /* Frac16 Speed => V */

    Filter_T FilterA;         /* Calibration use */
    Filter_T FilterB;
    Filter_T FilterC;

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

/******************************************************************************/
/*
    Simplify CommutationMode Check
*/
/******************************************************************************/
typedef void(*Motor_CommutationModeFunction_T)(Motor_T * p_motor);

static inline void Motor_ProcCommutationMode(Motor_T * p_motor, Motor_CommutationModeFunction_T focFunction, Motor_CommutationModeFunction_T sixStepFunction)
{
#if     defined(CONFIG_MOTOR_SIX_STEP_ENABLE) && defined(CONFIG_MOTOR_FOC_ENABLE)
    if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC) { focFunction(p_motor); }
    else /* p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP */ { sixStepFunction(p_motor); }
#elif     defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
    (void)focFunction; sixStepFunction(p_motor);
#elif     defined(CONFIG_MOTOR_FOC_ENABLE)
    (void)sixStepFunction; focFunction(p_motor);
#endif
}

// uint32_t type cast warning
// typedef void(*Motor_CommutationModeFunction1_T)(Motor_T * p_motor, uint32_t var);

// static inline void Motor_ProcCommutationMode1(Motor_T * p_motor, Motor_CommutationModeFunction1_T focFunction, Motor_CommutationModeFunction1_T sixStepFunction, uint32_t var)
// {
// #if     defined(CONFIG_MOTOR_SIX_STEP_ENABLE) && defined(CONFIG_MOTOR_FOC_ENABLE)
//     if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)                 { focFunction(p_motor, var); }
//     else /* p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP */     { sixStepFunction(p_motor, var); }
// #elif     defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
//     (void)focFunction;    sixStepFunction(p_motor, var);
// #else /* defined(CONFIG_MOTOR_FOC_ENABLE) */
//     (void)sixStepFunction;    focFunction(p_motor, var);
// #endif
// }

/******************************************************************************/
/*

*/
/******************************************************************************/
#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
static inline int32_t _Motor_ConvertSpeed_RpmToFracS16(Motor_T * p_motor, int32_t speed_rpm)        { return speed_rpm * 32767 / p_motor->Parameters.SpeedFeedbackRef_Rpm; }
static inline int16_t _Motor_ConvertSpeed_FracS16ToRpm(Motor_T * p_motor, int32_t speed_frac16)     { return speed_frac16 * p_motor->Parameters.SpeedFeedbackRef_Rpm / 32768; }

static inline int32_t _Motor_ConvertI_AmpsToIFracS16(int32_t i_amp)             { return i_amp * 32767 / GLOBAL_MOTOR.I_UNITS_AMPS; }
static inline int16_t _Motor_ConvertI_FracS16ToAmps(int32_t i_frac16)           { return i_frac16 * GLOBAL_MOTOR.I_UNITS_AMPS / 32768; }
static inline int32_t _Motor_ConvertV_VoltsToFracS16(int32_t v_volts)           { return v_volts * 32767 / Global_Motor_GetVSource_V(); }
static inline int16_t _Motor_ConvertV_FracS16ToVolts(int32_t v_frac16)          { return v_frac16 * Global_Motor_GetVSource_V() / 32768; }
static inline int32_t _Motor_ConvertPower_FracS16ToWatts(int32_t vi_frac16)     { return vi_frac16 * GLOBAL_MOTOR.I_UNITS_AMPS * Global_Motor_GetVSource_V() / 32768; }

static inline uint32_t _Motor_ConvertToMillis(Motor_T * p_motor, uint32_t controlCycles) { (void)p_motor; return controlCycles * 1000 / GLOBAL_MOTOR.CONTROL_FREQ; }
static inline uint32_t _Motor_ConvertToControlCycles(Motor_T * p_motor, uint32_t millis) { (void)p_motor; return millis * GLOBAL_MOTOR.CONTROL_FREQ / 1000; }

//todo rename Scalar
static inline int32_t _Motor_ConvertSpeed_RpmToScalar16(Motor_T * p_motor, int32_t speed_rpm)       { return speed_rpm * 65535 / p_motor->Parameters.SpeedFeedbackRef_Rpm; }
static inline int16_t _Motor_ConvertSpeed_Scalar16ToRpm(Motor_T * p_motor, int32_t speed_frac16)    { return speed_frac16 * p_motor->Parameters.SpeedFeedbackRef_Rpm / 65536; }
static inline int32_t _Motor_ConvertToIFrac16(Motor_T * p_motor, int32_t i_amp)             { (void)p_motor; return i_amp * 65535 / GLOBAL_MOTOR.I_UNITS_AMPS; }
static inline int16_t _Motor_ConvertToAmps(Motor_T * p_motor, int32_t i_frac16)             { (void)p_motor; return i_frac16 * GLOBAL_MOTOR.I_UNITS_AMPS / 65536; }
static inline int32_t _Motor_ConvertToVFrac16(Motor_T * p_motor, int32_t v_volts)           { (void)p_motor; return v_volts * 65535 / Global_Motor_GetVSource_V(); }
static inline int16_t _Motor_ConvertToVolts(Motor_T * p_motor, int32_t v_frac16)            { (void)p_motor; return v_frac16 * Global_Motor_GetVSource_V() / 65536; }
static inline int32_t _Motor_ConvertToWatts(Motor_T * p_motor, int32_t vi_frac16)           { (void)p_motor; return vi_frac16 * GLOBAL_MOTOR.I_UNITS_AMPS * Global_Motor_GetVSource_V() / 65536; }

// static inline uint32_t speed_angle16torpm(uint16_t angle16, uint32_t sampleFreq)         { return  (angle16 * sampleFreq >> 16U) * 60U; }
// static inline uint32_t speed_rpmtoangle16(uint16_t rpm, uint32_t sampleFreq)             { return (rpm << 16U) / (60U * sampleFreq); }
// static inline uint32_t _Motor_ConvertMechAngleToRpm(Motor_T * p_motor, uint16_t angle16 )         { return (angle16 * GLOBAL_MOTOR.SPEED_FREQ >> 16U) * 60U; }
// static inline uint32_t _Motor_ConvertMechRpmToAngle(Motor_T * p_motor, uint16_t rpm )             { return (rpm << 16U) / (60U * GLOBAL_MOTOR.SPEED_FREQ); }
// static inline int32_t _Motor_VSpeed_ConvertSpeedToVFrac16(Motor_T * p_motor, int32_t speed_frac16)     { return speed_frac16 * p_motor->Parameters.VSpeedRef_Rpm / p_motor->Parameters.SpeedFeedbackRef_Rpm; }
// static inline int32_t _Motor_VSpeed_ConvertRpmToVFrac16(Motor_T * p_motor, int32_t speed_rpm)     { return _Motor_VSpeed_ConvertSpeedToVFrac16(p_motor, _Motor_ConvertSpeed_RpmToScalar16(p_motor, speed_rpm) ); }
// static inline uint32_t _Motor_VSpeed_ConvertToVSpeed(Motor_T * p_motor, uint16_t rpm)         { return Linear_Function(&p_motor->UnitsVSpeed, _Motor_ConvertSpeed_RpmToScalar16(p_motor, rpm)); }
#endif

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
    Speed
*/
/******************************************************************************/
/*
    V in Frac16 using Kv and Speed
        = Speed_FracS16 * VSpeedRef_Rpm / SpeedFeedbackRef_Rpm
    User sets lower VSpeedRef_Rpm to ensure not match to higher speed
    Output must be saturated. Calling function /2 does not clear over saturation
*/
static inline int32_t Motor_GetKvSpeed_Frac16(Motor_T * p_motor)
{
    return Linear_Function_Frac16(&p_motor->UnitsVSpeed, p_motor->Speed_FracS16);
}

static inline bool Motor_CheckSpeedOverThreshold(Motor_T * p_motor)
{
    return !math_isbound(p_motor->Speed_FracS16, 0 - ((int32_t)p_motor->SpeedLimitCw_FracS16 * 7 / 8), (int32_t)p_motor->SpeedLimitCcw_FracS16 * 7 / 8);
}

/*!
    Convert user reference direction to CCW/CW direction
    @param[in] userInput int16_t[-32768:32767] userCmd or readValue
    @return int32_t[-32768:32768], Over saturated if input is -32768
*/
static inline int32_t Motor_ConvertUserDirection(Motor_T * p_motor, int32_t userCmd)
{
    return (p_motor->Direction == MOTOR_DIRECTION_CCW) ? userCmd : (int32_t)0 - userCmd;
}

// static inline void Motor_SetDirectionalCmd(Motor_T * p_motor, int16_t userCmd)
// {
//     Linear_Ramp_SetTarget(&p_motor->Ramp, _Motor_ConvertDirectionalCmd(p_motor, userCmd));
// }

/******************************************************************************/
/*
    Feedback Mode
*/
/******************************************************************************/
/* SuperFunction AntiPattern rationale there is only 1 state variable */
static inline Motor_FeedbackMode_T Motor_ConvertFeedbackModeId(Motor_FeedbackModeId_T mode)
{
    static const Motor_FeedbackMode_T MODE_OPEN_LOOP = { .OpenLoop = 1U, .Speed = 0U, .Current = 0U, .Scalar = 0U, .IsDisable = 0U, };
    static const Motor_FeedbackMode_T MODE_OPEN_LOOP_CURRENT = { .OpenLoop = 1U, .Speed = 0U, .Current = 1U, .Scalar = 0U, .IsDisable = 0U, };
    static const Motor_FeedbackMode_T MODE_VOLTAGE = { .OpenLoop = 0U, .Speed = 0U, .Current = 0U, .Scalar = 0U, .IsDisable = 0U, };
    static const Motor_FeedbackMode_T MODE_VOLTAGE_FREQ = { .OpenLoop = 0U, .Speed = 0U, .Current = 0U, .Scalar = 1U, .IsDisable = 0U, };
    static const Motor_FeedbackMode_T MODE_CURRENT = { .OpenLoop = 0U, .Speed = 0U, .Current = 1U, .Scalar = 0U, .IsDisable = 0U, };
    static const Motor_FeedbackMode_T MODE_SPEED_VOLTAGE = { .OpenLoop = 0U, .Speed = 1U, .Current = 0U, .Scalar = 0U, .IsDisable = 0U, };
    static const Motor_FeedbackMode_T MODE_SPEED_CURRENT = { .OpenLoop = 0U, .Speed = 1U, .Current = 1U, .Scalar = 0U, .IsDisable = 0U, };

    Motor_FeedbackMode_T flags;

    switch(mode)
    {
        case MOTOR_FEEDBACK_MODE_OPEN_LOOP:                    flags.State = MODE_OPEN_LOOP.State;             break;
        case MOTOR_FEEDBACK_MODE_OPEN_LOOP_CURRENT:            flags.State = MODE_OPEN_LOOP_CURRENT.State;     break;
        case MOTOR_FEEDBACK_MODE_CONSTANT_VOLTAGE:            flags.State = MODE_VOLTAGE.State;                break;
        case MOTOR_FEEDBACK_MODE_SCALAR_VOLTAGE_FREQ:        flags.State = MODE_VOLTAGE_FREQ.State;            break;
        case MOTOR_FEEDBACK_MODE_CONSTANT_CURRENT:            flags.State = MODE_CURRENT.State;                break;
        case MOTOR_FEEDBACK_MODE_CONSTANT_SPEED_VOLTAGE:    flags.State = MODE_SPEED_VOLTAGE.State;            break;
        case MOTOR_FEEDBACK_MODE_CONSTANT_SPEED_CURRENT:    flags.State = MODE_SPEED_CURRENT.State;            break;
        default: flags.State = 0; break;
    }

    return flags;
}

static inline bool Motor_CheckOpenLoop(Motor_T * p_motor)
{
#if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE) || defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE)  || defined(CONFIG_MOTOR_DEBUG_ENABLE)
    return (p_motor->ControlFeedbackMode.OpenLoop == 1U);
#else
    (void)p_motor; return false;
#endif
}

/******************************************************************************/
/*
    Common Sets
*/
/******************************************************************************/
static inline void Motor_ResetActiveSpeedLimits(Motor_T * p_motor)
{
    p_motor->SpeedLimitForward_ScalarU16 = p_motor->Parameters.SpeedLimitForward_ScalarU16;
    p_motor->SpeedLimitReverse_ScalarU16 = p_motor->Parameters.SpeedLimitReverse_ScalarU16;
    // p_motor->SpeedLimitActiveScalar = 0xFFFF;
}

static inline void Motor_ResetActiveILimits(Motor_T * p_motor)
{
    p_motor->ILimitMotoring_ScalarU16 = p_motor->Parameters.ILimitMotoring_ScalarU16;
    p_motor->ILimitGenerating_ScalarU16 = p_motor->Parameters.ILimitGenerating_ScalarU16;
    p_motor->ILimitActiveSentinel = 0xFFFFU; /* Use for comparison on set */
}
/******************************************************************************/
/*

*/
/******************************************************************************/

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
extern qangle16_t Motor_GetMechanicalAngle(Motor_T * p_motor);

extern void Motor_ZeroSensor(Motor_T * p_motor);
extern void Motor_CalibrateSensorZero(Motor_T * p_motor);
extern void Motor_ValidateSensorAlign(Motor_T * p_motor);
extern bool Motor_CheckSensorFeedback(Motor_T * p_motor);
extern bool Motor_CheckAlignFault(Motor_T * p_motor);

extern void Motor_SetFeedbackILimits(Motor_T * p_motor);
extern void Motor_SetFeedbackSpeedLimits(Motor_T * p_motor);
// extern void Motor_SetFeedbackLimitsCcw(Motor_T * p_motor);
// extern void Motor_SetFeedbackLimitsCw(Motor_T * p_motor);
extern void Motor_SetDirectionCcw(Motor_T * p_motor);
extern void Motor_SetDirectionCw(Motor_T * p_motor);
extern void Motor_SetDirection(Motor_T * p_motor, Motor_Direction_T direction);
extern void Motor_SetDirectionForward(Motor_T * p_motor);
extern void Motor_SetDirectionReverse(Motor_T * p_motor);

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
